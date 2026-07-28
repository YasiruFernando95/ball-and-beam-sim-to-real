import time
import math
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

from ball_beam_env_v2 import BallBeamCleanEnv


# -----------------------
# Configuration
# -----------------------
BASE_DIR = Path(__file__).resolve().parent

MODEL_DIR = BASE_DIR / "models"   # from train_v3_no_vecnorm.py
MODEL_NAME = "sac_ballbeam_simple_32"    # final model name (without .zip)
MODEL_PATH = MODEL_DIR / MODEL_NAME          # SB3 loads .zip automatically

RENDER = True
TEST_SECONDS = 60.0

CTRL_HZ = 100.0
DT = 1.0 / CTRL_HZ

PRINT_EVERY_WALL_S = 0.10  # print paced like hardware telemetry


def make_env():
    env = BallBeamCleanEnv(
        render_mode="human" if RENDER else None,
        ctrl_hz=CTRL_HZ,
        time_step=1.0 / 240.0,
        theta_limit_deg=25.0,
        servo_delay_steps=13,
        servo_tau_s=0.070,
        setpoint_rate_limit_degps=160.0,
    )


    return env


def main():
    # Vec env wrapper required by SB3
    vec_env = DummyVecEnv([lambda: make_env()])
    vec_env = VecMonitor(vec_env)

    model = SAC.load(str(MODEL_PATH), env=vec_env)

    obs = vec_env.reset()

    # Base env handle
    base_env: BallBeamCleanEnv = vec_env.envs[0].unwrapped  # type: ignore
    theta_limit_rad = float(getattr(base_env, "theta_limit", math.radians(25.0)))

    n_steps = int(TEST_SECONDS / DT)

    # Logs (physical units)
    t_log, x_log, xdot_log, th_deg_log, thdot_degps_log = [], [], [], [], []
    sp_req_deg_log, sp_cmd_deg_log, reward_log = [], [], []

    wall_t0 = time.time()
    last_print_wall = -1e9

    print(f"[TEST] model : {MODEL_PATH}.zip")
    print(f"[TEST] ctrl_hz: {CTRL_HZ:.1f} Hz (dt={DT:.4f}s), steps={n_steps}")
    print(f"[TEST] limit : ±{math.degrees(theta_limit_rad):.1f} deg\n")

    t0 = time.time()

    for k in range(n_steps):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = vec_env.step(action)

        # vec_env gives arrays of shape (n_envs, ...)
        obs0 = obs[0]
        # x, xdot, th, thdot = map(float, obs0)
        x, xdot, th, thdot, _theta_sp_delayed_last, _theta_cmd = map(float, obs0)

        theta_sp_req_rad = float(np.asarray(action).ravel()[0])
        theta_sp_req_rad = float(np.clip(theta_sp_req_rad, -theta_limit_rad, +theta_limit_rad))

        # internal actuator state (if env uses it)
        theta_cmd_rad = float(getattr(base_env, "_theta_cmd", theta_sp_req_rad))

        sim_t = k * DT
        r = float(np.asarray(reward).ravel()[0])

        # Log
        t_log.append(sim_t)
        x_log.append(x)
        xdot_log.append(xdot)
        th_deg_log.append(math.degrees(th))
        thdot_degps_log.append(math.degrees(thdot))
        sp_req_deg_log.append(math.degrees(theta_sp_req_rad))
        sp_cmd_deg_log.append(math.degrees(theta_cmd_rad))
        reward_log.append(r)

        # Real-time pacing:
        target = t0 + (k + 1) * DT
        sleep_s = target - time.time()
        if sleep_s > 0:
            time.sleep(sleep_s)

        # Print paced by wall clock
        now = time.time()
        if (now - last_print_wall) >= PRINT_EVERY_WALL_S:
            last_print_wall = now
            print(
                f"t={sim_t:6.2f}s | "
                f"x={x:+.4f} m | xdot={xdot:+.3f} m/s | "
                f"θ={math.degrees(th):+6.2f}° | θdot={math.degrees(thdot):+7.1f}°/s | "
                f"θsp_req={math.degrees(theta_sp_req_rad):+6.2f}° | "
                f"θ_cmd={math.degrees(theta_cmd_rad):+6.2f}° | r={r:+.3f}"
            )
            

        # Reset when episode ends
        if bool(done[0]):
            # print(info)
            obs = vec_env.reset()

    vec_env.close()
    wall = time.time() - wall_t0
    print(f"\n[TEST] Finished. Wall time: {wall:.1f}s")

    # -----------------------
    # Plots
    # -----------------------
    t = np.array(t_log)

    plt.figure()
    plt.plot(t, x_log)
    plt.xlabel("Time (s)")
    plt.ylabel("Ball position x (m)")
    plt.title("Ball position")
    plt.grid(True)

    plt.figure()
    plt.plot(t, th_deg_log, label="θ (deg)")
    plt.plot(t, sp_req_deg_log, "--", label="θsp requested (deg)")
    plt.plot(t, sp_cmd_deg_log, ":", label="θ_cmd (actuator state, deg)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("Beam angle + setpoints")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(t, xdot_log, label="xdot (m/s)")
    plt.plot(t, thdot_degps_log, label="theta_dot (deg/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed")
    plt.title("Speeds")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(t, reward_log)
    plt.xlabel("Time (s)")
    plt.ylabel("Reward")
    plt.title("Reward")
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()
