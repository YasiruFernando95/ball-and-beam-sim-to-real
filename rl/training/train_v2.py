from __future__ import annotations

import math
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import gymnasium as gym
from gymnasium.wrappers import TimeLimit

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback
from stable_baselines3.common.callbacks import StopTrainingOnNoModelImprovement
from stable_baselines3.common.vec_env import VecFrameStack

import torch as th

from ball_beam_env_v2 import BallBeamCleanEnv

# ----------------------------
# Domain Randomization Wrapper
# ----------------------------


@dataclass
class DRRanges:
    delay_steps: Tuple[int, int] = (10, 16)
    tau_s: Tuple[float, float] = (0.05, 0.11)
    rate_limit_degps: Tuple[float, float] = (120.0, 220.0)

    # IMPORTANT: cover your real mass ~0.065 kg
    ball_mass: Tuple[float, float] = (0.050, 0.080)

    lat_fric: Tuple[float, float] = (0.02, 0.12)
    roll_fric: Tuple[float, float] = (0.0005, 0.006)
    spin_fric: Tuple[float, float] = (0.0005, 0.006)

    x_bias_m: Tuple[float, float] = (-0.005, 0.005)
    theta_bias_deg: Tuple[float, float] = (-1.0, 1.0)

    x_noise_m: Tuple[float, float] = (0.0, 0.002)
    theta_noise_deg: Tuple[float, float] = (0.0, 0.20)
    xdot_noise_mps: Tuple[float, float] = (0.0, 0.04)
    thdot_noise_degps: Tuple[float, float] = (0.0, 10.0)


class DomainRandomizationWrapper(gym.Wrapper):
    def __init__(self, env: gym.Env, enable: bool = True, ranges: DRRanges = DRRanges(), seed: Optional[int] = None):
        super().__init__(env)
        self.enable = enable
        self.r = ranges
        self.rng = np.random.default_rng(seed)

        self._x_bias = 0.0
        self._th_bias = 0.0
        self._x_noise = 0.0
        self._th_noise = 0.0
        self._xd_noise = 0.0
        self._thd_noise = 0.0

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        if self.enable:
            self._apply_domain_randomization()
        return self._observe(obs), info

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        return self._observe(obs), reward, terminated, truncated, info

    def _apply_domain_randomization(self):
        e: BallBeamCleanEnv = self.env.unwrapped  # type: ignore

        delay = int(self.rng.integers(self.r.delay_steps[0], self.r.delay_steps[1] + 1))
        tau = float(self.rng.uniform(self.r.tau_s[0], self.r.tau_s[1]))
        rate_lim_degps = float(self.rng.uniform(self.r.rate_limit_degps[0], self.r.rate_limit_degps[1]))

        e.servo_delay_steps = delay
        e.servo_tau_s = max(1e-4, tau)
        e._sp_buf.clear()
        e._sp_buf.extend([0.0] * (e.servo_delay_steps + 1))
        e.setpoint_rate_limit = math.radians(rate_lim_degps)

        m = float(self.rng.uniform(self.r.ball_mass[0], self.r.ball_mass[1]))
        lat = float(self.rng.uniform(self.r.lat_fric[0], self.r.lat_fric[1]))
        roll = float(self.rng.uniform(self.r.roll_fric[0], self.r.roll_fric[1]))
        spin = float(self.rng.uniform(self.r.spin_fric[0], self.r.spin_fric[1]))

        e.p.changeDynamics(
            e.ball_id, -1,
            mass=m,
            lateralFriction=lat,
            rollingFriction=roll,
            spinningFriction=spin
        )

        self._x_bias = float(self.rng.uniform(*self.r.x_bias_m))
        self._th_bias = math.radians(float(self.rng.uniform(*self.r.theta_bias_deg)))

        self._x_noise = float(self.rng.uniform(*self.r.x_noise_m))
        self._th_noise = math.radians(float(self.rng.uniform(*self.r.theta_noise_deg)))

        self._xd_noise = float(self.rng.uniform(*self.r.xdot_noise_mps))
        self._thd_noise = math.radians(float(self.rng.uniform(*self.r.thdot_noise_degps)))

    def _observe(self, obs):
        if not self.enable:
            return obs

        obs = np.asarray(obs, dtype=np.float32).ravel()
        n = int(obs.shape[0])

        # Parse base state + pass-through extras (no noise by default)
        if n == 4:
            x, xd, th, thd = map(float, obs)
            extras = []
        elif n == 5:
            x, xd, th, thd, th_cmd = map(float, obs)
            extras = [th_cmd]
        elif n == 6:
            # Example 6D: [x, xdot, theta, thdot, theta_sp_delayed, theta_cmd]
            x, xd, th, thd, th_sp_del, th_cmd = map(float, obs)
            extras = [th_sp_del, th_cmd]
        else:
            raise ValueError(f"Unexpected obs size: {obs.shape}")

        # biases + noise (physical channels)
        x  = x  + self._x_bias  + (float(self.rng.normal(0.0, self._x_noise))  if self._x_noise  > 0 else 0.0)
        th = th + self._th_bias + (float(self.rng.normal(0.0, self._th_noise)) if self._th_noise > 0 else 0.0)

        if self._xd_noise > 0:
            xd = xd + float(self.rng.normal(0.0, self._xd_noise))
        if self._thd_noise > 0:
            thd = thd + float(self.rng.normal(0.0, self._thd_noise))

        out = [x, xd, th, thd] + extras
        return np.array(out, dtype=np.float32)



class ActionRepeatWrapper(gym.Wrapper):
    """
    Repeat the same action for `repeat` env-steps.
    This effectively lowers the policy rate: ctrl_hz / repeat.
    """
    def __init__(self, env: gym.Env, repeat: int = 5):
        super().__init__(env)
        assert repeat >= 1
        self.repeat = int(repeat)

    def step(self, action):
        total_reward = 0.0
        terminated = False
        truncated = False
        info = {}

        obs = None
        for k in range(self.repeat):
            obs, r, terminated, truncated, info = self.env.step(action)
            total_reward += float(r)
            if terminated or truncated:
                break

        # Helpful debug: how many internal steps were executed
        info = dict(info)
        info["action_repeat"] = self.repeat
        info["repeat_steps_executed"] = k + 1

        return obs, total_reward, terminated, truncated, info


# ----------------------------
# Factories
# ----------------------------

ACTION_REPEAT = 5  # RL at ~20Hz if ctrl_hz=100

def make_env(
    render: bool,
    dr_enable: bool,          # keep the param for later, but we will pass False for now
    reset_x: float,
    reset_th_deg: float,
    seed: int,
    max_episode_steps: int = 900
):
    env = BallBeamCleanEnv(
        render_mode="human" if render else None,
        ctrl_hz=100.0,
        time_step=1.0 / 240.0,
        theta_limit_deg=25.0,

        reset_x_range_m=reset_x,
        reset_theta_range_deg=reset_th_deg,

        servo_delay_steps=13,
        servo_tau_s=0.070,
        setpoint_rate_limit_degps=160.0,
        seed=seed,
    )

    # --- KEY CHANGE 1: slow down the policy ---
    env = ActionRepeatWrapper(env, repeat=ACTION_REPEAT)

    # --- KEY CHANGE 2: DR/Noise OFF for now ---
    # DomainRandomizationWrapper is where bias/noise is injected :contentReference
    if dr_enable:
        env = DomainRandomizationWrapper(env, enable=True, seed=seed)

    # --- Keep episode time consistent in REAL control-steps ---
    # Original was 900 steps at 100Hz ~= 9s.
    # Now each RL "step" executes ACTION_REPEAT control-steps.
    max_steps_rl = max(1, int(max_episode_steps // ACTION_REPEAT))
    env = TimeLimit(env, max_episode_steps=max_steps_rl)

    return env



def build_vec_env(n_envs: int, dr_enable: bool, reset_x: float, reset_th_deg: float, base_seed: int):
    def thunk(i: int):
        return lambda: make_env(False, dr_enable, reset_x, reset_th_deg, base_seed + 1000 * i)
    venv = DummyVecEnv([thunk(i) for i in range(n_envs)])
    venv = VecMonitor(venv)
    return venv


def build_eval_env(seed: int):
    # Eval with DR OFF so you can track true skill cleanly
    env = DummyVecEnv([lambda: make_env(False, False, 0.10, 4.0, seed + 999)])
    env = VecMonitor(env)
    return env


# ----------------------------
# Main training
# ----------------------------

def main():
    BASE_DIR = Path(__file__).resolve().parent
    LOG_DIR = BASE_DIR / "logs"
    MODEL_DIR = BASE_DIR / "models"
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    MODEL_DIR.mkdir(parents=True, exist_ok=True)

    RUN_NAME = "sac_ballbeam_simple_32"
    SEED = 42
    TOTAL_STEPS = 500_000

    N_ENVS = 4  # keep simple like your current script

    # Train DR ON from the start (single-phase)
    train_env = build_vec_env(N_ENVS, dr_enable=True, reset_x=0.10, reset_th_deg=4.0, base_seed=SEED)
    eval_env = build_eval_env(SEED)

    checkpoint_cb = CheckpointCallback(
        save_freq=25_000,  # was 100k
        save_path=str(MODEL_DIR),
        name_prefix=RUN_NAME + "_ckpt",
        save_replay_buffer=False,    # speeds things up a lot
    )


    stop_cb = StopTrainingOnNoModelImprovement(
        max_no_improvement_evals=6,  # 6 evals with no improvement
        min_evals=8,                 # give it some time first
        verbose=1
    )

    eval_cb = EvalCallback(
        eval_env=eval_env,
        best_model_save_path=str(MODEL_DIR),
        log_path=str(LOG_DIR),
        eval_freq=20_000,            # more frequent feedback than 50k
        n_eval_episodes=8,           # much faster than 10
        deterministic=True,
        render=False,
        callback_after_eval=stop_cb
    )

    model = SAC(
        "MlpPolicy",
        train_env,
        verbose=1,
        tensorboard_log=str(LOG_DIR),
        seed=SEED,

        learning_rate=3e-4,
        gamma=0.995,
        tau=0.01,

        buffer_size=500_000,
        learning_starts=20_000,
        batch_size=256,
        train_freq=(1, "step"),
        gradient_steps=4,
        target_entropy= -0.3,
        ent_coef="auto_0.1",
        use_sde=False,
        sde_sample_freq=16,
        # keep SB3 defaults for target entropy & target updates
        policy_kwargs=dict(net_arch=[256, 256]),
    )

    print(f"\n[Single-phase] DR=OFF  steps={TOTAL_STEPS:,}")
    model.learn(total_timesteps=TOTAL_STEPS, tb_log_name=RUN_NAME, callback=[checkpoint_cb, eval_cb])

    final_path = MODEL_DIR / RUN_NAME
    model.save(str(final_path))
    model.save_replay_buffer(str(MODEL_DIR / (RUN_NAME + "_replay_final.pkl")))

    print("\n[Done]")
    print(f"Final model: {final_path}.zip")
    print(f"Best model : saved under {MODEL_DIR} by EvalCallback")

    train_env.close()
    eval_env.close()


if __name__ == "__main__":
    main()
