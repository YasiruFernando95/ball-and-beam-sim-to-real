"""

Meets requirements:
1) Balance around center with low jitter (reward + command smoothing penalties)
2) Avoid ends; terminate if "stuck" at an end zone / endstop contact persists
3) Avoid sudden extreme movements/speeds (rate limit + penalties)
4) Hard terminate if |beam angle| > 25 deg
5) Uses calibrated actuator identification as a simple delay + 1st-order lag model
6) Action is beam (motor) angle setpoint in radians (bounded to ±25 deg)

Observation (raw SI units): [x (m), xdot (m/s), theta (rad), thdot (rad/s)]
Action: theta_setpoint (rad) in [-theta_limit, +theta_limit]
"""

from __future__ import annotations

import os
import math
from collections import deque
from typing import Optional, Tuple, Dict, Any

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import pybullet as pybullet
import pybullet_data
from pybullet_utils.bullet_client import BulletClient


class BallBeamCleanEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 240}

    def __init__(
        self,
        render_mode: Optional[str] = None,
        urdf_name: str = "ball_beam.urdf",
        time_step: float = 1.0 / 240.0,     # Bullet step
        ctrl_hz: float = 100.0,             # controller/update rate target
        theta_limit_deg: float = 25.0,

        # --- Ball / beam geometry assumptions (match URDF) ---
        beam_length_m: float = 0.30,        # your URDF uses 0.30m beam
        ball_diameter_m: float = 0.025,

        # --- Actuator identification (your measured closed-loop) ---

        servo_delay_steps: int = 13,        # delay in CONTROL steps (not Bullet steps)
        servo_tau_s: float = 0.070,         # 1st-order lag time constant (s)

        setpoint_rate_limit_degps: float = 140.0,

        # --- Control application in Bullet ---
        joint_force_limit: float = 0.12,     # VELOCITY_CONTROL force cap
        joint_max_vel_degps: float = 250.0, # targetVelocity cap

        # --- Reset simplification (make learning easy) ---
        reset_x_range_m: float = 0.06,      # start near center (smaller = easier)
        reset_theta_range_deg: float = 2.0,

        # --- Termination logic ---
        end_stuck_steps: int = 300,          # consecutive control-steps stuck => done
        end_contact_steps: int = 200,        # consecutive control-steps contacting endstop => done

        seed: Optional[int] = None,
    ):
        super().__init__()
        self.render_mode = render_mode

        # --- time / rates ---
        self.time_step = float(time_step)
        self.ctrl_hz = float(ctrl_hz)
        self.ctrl_dt = 1.0 / self.ctrl_hz

        # frame skip to approximate ctrl_hz while stepping Bullet at time_step
        self.frame_skip = max(1, int(round(self.ctrl_dt / self.time_step)))

        # --- limits ---
        self.theta_limit = math.radians(float(theta_limit_deg))
        self.setpoint_rate_limit = math.radians(float(setpoint_rate_limit_degps))  # rad/s
        self.joint_max_vel = math.radians(float(joint_max_vel_degps))              # rad/s
        self.joint_force_limit = float(joint_force_limit)

        # --- geometry ---
        self.L = float(beam_length_m)
        self.half_L = 0.5 * self.L
        self.ball_r = 0.5 * float(ball_diameter_m)

        # --- reachable ball-center limit on the beam (geometry-consistent) ---
        # Clearance accounts for endstop thickness / contact slop / Bullet penetration.
        self.x_clearance_m = 0.004  # 4 mm is a good start; tune 2–8 mm
        self.x_limit = max(0.0, self.half_L - self.ball_r)

        # Define end-zone start relative to x_limit, not half_L.
        self.end_margin_m = 0.020
        self.end_zone_start = max(0.0, self.x_limit - self.end_margin_m)

        # --- obs / reward shaping toggles ---
        self.include_actuator_state = bool(getattr(self, "include_actuator_state", True))
        # if True: obs = [x, xdot, theta, thdot, theta_sp_delayed, theta_cmd]
        # if False: obs = [x, xdot, theta, thdot]
        self.use_progress_reward = bool(getattr(self, "use_progress_reward", True))
        self.use_endzone_penalty = bool(getattr(self, "use_endzone_penalty", True))

        self._last_obs = np.zeros((4,), dtype=np.float32)  # overwritten on reset



        self.reset_x_range = float(reset_x_range_m)
        self.reset_theta_range = math.radians(float(reset_theta_range_deg))

        # --- actuator ID model (delay + 1st-order lag) ---
        self.servo_delay_steps = int(max(0, servo_delay_steps))
        self.servo_tau_s = float(max(1e-4, servo_tau_s))
        self._sp_buf = deque([0.0] * (self.servo_delay_steps + 1),
                             maxlen=self.servo_delay_steps + 1)
        self._theta_cmd = 0.0           # internal actuator state after lag
        self._theta_sp_prev = 0.0       # for rate limiting

        # --- termination helpers ---
        self.end_stuck_steps_limit = int(end_stuck_steps)
        self.end_contact_steps_limit = int(end_contact_steps)
        self._end_stuck_steps = 0
        self._end_contact_steps = 0

        self.track_kp_w = 8.0   # 1/s  (start 6–12)
        self.track_kd_w = 1.5   # damping on thdot (start 1–3)

        self.gauss_sigma_m = 0.020  # 15 mm “good zone” (tune 0.01–0.03)
        self.w_cmd_penalty = 0.03   # penalty weight for beam command rate

        self.x_band = 0.010
        self.xd_band = 0.020
        self.th_band = np.deg2rad(1.0)
        self.thd_band = np.deg2rad(15.0)

        self.hold_steps = 0
        self.hold_bonus = 0.10      # per-step bonus cap (tune 0.05–0.2)
        self.hold_ramp_steps = 100  # ~1s if ctrl_hz=100

        self.end_recovery_rate_boost_degps = 220.0   # only active near ends + recovering
        self.end_recovery_delay_relief = True        # optional; set False to keep exact delay always



        # self.ax_prev = 0.0


        # --- RNG ---
        self.np_random = np.random.default_rng(seed)

        # --- spaces ---
        # Action is motor/beam angle setpoint in radians (bounded)
        self.action_space = spaces.Box(
            low=np.array([-self.theta_limit], dtype=np.float32),
            high=np.array([+self.theta_limit], dtype=np.float32),
            dtype=np.float32,
        )

        if self.include_actuator_state:
            # [x, xdot, theta, thdot, theta_sp_delayed, theta_cmd]
            obs_hi = np.array([self.x_limit, 3.0, self.theta_limit, 50.0, self.theta_limit, self.theta_limit], dtype=np.float32)
        else:
            # [x, xdot, theta, thdot]
            obs_hi = np.array([self.x_limit, 3.0, self.theta_limit, 50.0], dtype=np.float32)

        self.observation_space = spaces.Box(-obs_hi, obs_hi, dtype=np.float32)


        # --- connect Bullet (per-env client to avoid conflicts) ---
        mode = pybullet.GUI if self.render_mode == "human" else pybullet.DIRECT
        self.p = BulletClient(connection_mode=mode)
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.p.setTimeStep(self.time_step)
        self.p.setGravity(0, 0, -9.81)

        # URDF path
        self.urdf_path = os.path.join(os.path.dirname(__file__), urdf_name)

        # IDs
        self.robot_id = None
        self.ball_id = None
        self.hinge_joint = None          # joint index for hinge_y
        self.beam_link = None            # same index as hinge joint in Bullet
        self.endstop_links = set()       # link indices for endstops

        # A good spawn height in BEAM frame (from your URDF geometry)
        # Keep it constant for simplicity.
        self._valley_z_beamframe = 0.04914
        self._spawn_eps = 0.001

        # ---- episode metric config ----
        self.band_m = 0.010         # 10 mm deadband
        self.hold_s = 2.0           # must hold for 2 seconds
        self._hold_steps_req = int(round(self.hold_s * self.ctrl_hz))

        # ---- episode metric state ----
        self._ep_step = 0
        self._in_band_ctr = 0
        self._first_hold_step = None
        self._max_absx = 0.0

        # self._x_prev = 0.0  # for progress shaping

        self._build_world()

    # ---------------- world build ----------------
    def _build_world(self):
        self.p.resetSimulation()
        self.p.setGravity(0, 0, -9.81)
        self.p.setTimeStep(self.time_step)
        self.p.loadURDF("plane.urdf")

        self.robot_id = self.p.loadURDF(
            self.urdf_path,
            [0, 0, 0],
            useFixedBase=True,
            flags=self.p.URDF_USE_INERTIA_FROM_FILE,
        )

        # Find hinge joint by name (URDF has endstops too)
        hinge = None
        endstop_links = set()
        for ji in range(self.p.getNumJoints(self.robot_id)):
            info = self.p.getJointInfo(self.robot_id, ji)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            if joint_name == "hinge_y":
                hinge = ji
            if "endstop" in link_name:
                endstop_links.add(ji)

        if hinge is None:
            raise RuntimeError("Could not find hinge joint named 'hinge_y' in the URDF.")

        self.hinge_joint = hinge
        self.beam_link = hinge
        self.endstop_links = endstop_links

        # Disable default motor, we'll command explicitly
        self.p.setJointMotorControl2(
            self.robot_id,
            self.hinge_joint,
            controlMode=self.p.VELOCITY_CONTROL,
            force=0.0,
        )

        # Create ball
        col = self.p.createCollisionShape(self.p.GEOM_SPHERE, radius=self.ball_r)
        vis = self.p.createVisualShape(self.p.GEOM_SPHERE, radius=self.ball_r, rgbaColor=[0.8, 0.8, 0.8, 1.0])
        self.ball_id = self.p.createMultiBody(
            baseMass=0.065,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
        )

        # Conservative friction defaults (keep simple; randomize later if needed)
        self.p.changeDynamics(
            self.ball_id, -1,
            lateralFriction=0.25,
            rollingFriction=0.002,
            spinningFriction=0.002,
            restitution=0.0
        )


        # Mild damping on hinge
        self.p.changeDynamics(self.robot_id, self.hinge_joint, jointDamping=0.02)

    # ---------------- utility ----------------
    def _beam_frame(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        link = self.p.getLinkState(self.robot_id, self.beam_link, computeForwardKinematics=True)
        return link[4], link[5]  # world pos, world orn

    def _get_obs(self) -> np.ndarray:
        theta, thdot = self.p.getJointState(self.robot_id, self.hinge_joint)[:2]

        # World poses
        ball_pos_w, _ = self.p.getBasePositionAndOrientation(self.ball_id)
        ball_v_w, _ = self.p.getBaseVelocity(self.ball_id)

        beam_pos_w, beam_orn_w = self._beam_frame()  # origin + orientation of beam frame
        inv_pos, inv_orn = self.p.invertTransform(beam_pos_w, beam_orn_w)

        # Ball position in beam frame
        local_ball, _ = self.p.multiplyTransforms(inv_pos, inv_orn, ball_pos_w, [0, 0, 0, 1])
        x = float(local_ball[0])

        # Beam x-axis in world
        R = np.array(self.p.getMatrixFromQuaternion(beam_orn_w), dtype=np.float32).reshape(3, 3)
        beam_x_w = R[:, 0]

        # Beam angular velocity in world:
        # hinge rotates around local Y axis; in world it's R[:,1] scaled by thdot
        beam_y_w = R[:, 1]
        omega_w = beam_y_w * float(thdot)

        # r = ball position relative to beam origin (world)
        r_w = np.array(ball_pos_w, dtype=np.float32) - np.array(beam_pos_w, dtype=np.float32)

        # Relative linear velocity of ball w.r.t. rotating beam
        v_rel_w = np.array(ball_v_w, dtype=np.float32) - np.cross(omega_w, r_w)

        # xdot = component along beam x-axis
        xdot = float(np.dot(v_rel_w, beam_x_w))

        # Keep physical x, only clamp extreme penetration spikes slightly beyond limit
        x_raw = float(x)

        x = float(np.clip(x_raw, -self.x_limit, +self.x_limit))

        # Optional: clip velocity too (Bullet can spike during collisions)
        xdot = float(np.clip(xdot, -3.0, +3.0))


        base = np.array([x, xdot, float(theta), float(thdot)], dtype=np.float32)

        if self.include_actuator_state:
            # theta_sp_delayed isn't known inside _get_obs unless we store it in step()
            # so we cache the last used value (set in step()).
            theta_sp_delayed = float(getattr(self, "_theta_sp_delayed_last", 0.0))
            theta_cmd = float(getattr(self, "_theta_cmd", 0.0))
            extra = np.array([theta_sp_delayed, theta_cmd], dtype=np.float32)
            return np.concatenate([base, extra]).astype(np.float32)

        return base

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        if seed is not None:
            self.np_random = np.random.default_rng(seed)

        # ---------------- counters ----------------
        self._end_stuck_steps = 0
        self._end_contact_steps = 0
        self._ep_step = 0
        self._in_band_ctr = 0
        self._first_hold_step = None
        self._max_absx = 0.0
        self.hold_steps = 0

        # previous terms used in step()
        self._a_prev = 0.0
        self._u_prev = 0.0

        # ---------------- hinge reset ----------------
        init_theta = float(self.np_random.uniform(-self.reset_theta_range, +self.reset_theta_range))
        self.p.resetJointState(self.robot_id, self.hinge_joint, init_theta, 0.0)

        # ---------------- actuator internal state ----------------
        # Start actuator state consistent with hinge angle (reduces first-step transient)
        self._theta_sp_prev = init_theta
        self._theta_cmd = init_theta
        self._theta_cmd_prev = init_theta

        # Delay buffer: fill with init_theta instead of zeros
        self._sp_buf = deque([init_theta] * (self.servo_delay_steps + 1),
                            maxlen=self.servo_delay_steps + 1)

        # ---------------- ball reset ----------------
        x_range = self.reset_x_range
        if options is not None:
            x_range = float(options.get("reset_x_range_m", x_range))

        # margin to avoid spawning inside endstop/mesh penetration region
        x_margin = 1.5 * self.ball_r
        x_hi_safe = max(0.0, self.x_limit - x_margin)

        # keep center range within safe limits
        x_range = float(np.clip(x_range, 0.0, x_hi_safe))

        # mixture reset: center most of the time, ends sometimes
        p_end = float(getattr(self, "reset_p_end", 0.25))  # expose as param if you want
        if self.np_random.random() < p_end and x_hi_safe > self.end_zone_start:
            lo = float(np.clip(self.end_zone_start, 0.0, x_hi_safe))
            hi = float(x_hi_safe)  # safe cap
            sgn = 1.0 if self.np_random.random() < 0.5 else -1.0
            init_x = float(sgn * self.np_random.uniform(lo, hi))
        else:
            init_x = float(self.np_random.uniform(-x_range, +x_range))

        beam_pos, beam_orn = self._beam_frame()
        local_ball = [init_x, 0.0, self._valley_z_beamframe + self.ball_r + self._spawn_eps]
        world_ball, _ = self.p.multiplyTransforms(beam_pos, beam_orn, local_ball, [0, 0, 0, 1])

        self.p.resetBasePositionAndOrientation(self.ball_id, world_ball, [0, 0, 0, 1])
        self.p.resetBaseVelocity(self.ball_id, [0, 0, 0], [0, 0, 0])

        # Important: set ax_prev to current |x| so progress terms don't spike on step 1
        self._ax_prev = abs(init_x)

        # ---------------- settle ----------------
        # a few bullet steps to resolve contacts
        for _ in range(10):
            self.p.stepSimulation()

        obs = self._get_obs()
        # cache last obs for next step (prevents calling _get_obs() inside action preprocess)
        self._last_obs = obs.copy()
        return obs, {}

    # ---------------- step ----------------
    def step(self, action: np.ndarray):
        # ---------------- action -> theta_cmd (delay + lag) ----------------
        theta_sp = float(np.clip(action[0], -self.theta_limit, +self.theta_limit))

        # --- Recovery boost (near ends, only when action points toward center) ---
        x_now = self._last_obs[0]

        near_end = abs(x_now) > self.end_zone_start  # requires you defined end_zone_start properly
        toward_center = (x_now * theta_sp) < 0.0     # x>0 => want negative theta, x<0 => want positive theta

        # Default rate limit
        rate_limit = float(self.setpoint_rate_limit)  # rad/s

        # If near end AND trying to recover, allow faster setpoint change
        if near_end and toward_center:
            boost_degps = float(getattr(self, "end_recovery_rate_boost_degps", 220.0))  # try 180–280
            rate_limit = max(rate_limit, math.radians(boost_degps))

        # Rate limit (rad/s) using possibly boosted rate
        max_step = rate_limit * self.ctrl_dt
        theta_sp = float(np.clip(theta_sp, self._theta_sp_prev - max_step, self._theta_sp_prev + max_step))
        self._theta_sp_prev = theta_sp

        # Delay buffer
        self._sp_buf.append(theta_sp)

        use_delay_relief = bool(getattr(self, "end_recovery_delay_relief", True))
        if near_end and toward_center and use_delay_relief:

            idx = -2 if len(self._sp_buf) >= 2 else -1
            theta_sp_delayed = float(self._sp_buf[idx])

        else:
            theta_sp_delayed = float(self._sp_buf[0])
        
        self._theta_sp_delayed_last = float(theta_sp_delayed)

        # 1st-order lag
        alpha = self.ctrl_dt / (self.servo_tau_s + self.ctrl_dt)
        self._theta_cmd = self._theta_cmd + alpha * (theta_sp_delayed - self._theta_cmd)

        # --- compute servo "movement" penalty signal (rad/s) ---
        theta_cmd_rate = (self._theta_cmd - getattr(self, "_theta_cmd_prev", self._theta_cmd)) / max(1e-6, self.ctrl_dt)
        self._theta_cmd_prev = float(self._theta_cmd)

        # normalize by a safe rate (use your physical limit; fallback to setpoint_rate_limit)
        rate_norm = float(getattr(self, "servo_rate_norm", self.setpoint_rate_limit))  # rad/s
        u = float(theta_cmd_rate / max(1e-6, rate_norm))

        # Precompute once
        kp_tau = float(getattr(self, "kp_tau", 1.0))
        kd_tau = float(getattr(self, "kd_tau", 0.30))
        tau_stall = float(getattr(self, "tau_stall", 0.11))
        omega_nl  = float(getattr(self, "omega_nl", 45.0))
        tau_peak_scale = float(getattr(self, "tau_peak_scale", 1.2))
        tau_peak = tau_peak_scale * tau_stall

        for _ in range(self.frame_skip):
            theta, thdot = self.p.getJointState(self.robot_id, self.hinge_joint)[:2]
            theta = float(theta); thdot = float(thdot)

            err = float(self._theta_cmd) - theta
            tau_cmd = kp_tau * err - kd_tau * thdot

            tau_avail = tau_peak * max(0.0, 1.0 - abs(thdot) / max(1e-6, omega_nl))
            tau = float(np.clip(tau_cmd, -tau_avail, +tau_avail))

            self.p.setJointMotorControl2(
                self.robot_id, self.hinge_joint,
                controlMode=self.p.TORQUE_CONTROL,
                force=tau
            )
            self.p.stepSimulation()

        # ---------------- observe ----------------
        obs = self._get_obs()
        x, xdot, theta_obs, thdot_obs, _theta_sp_delayed_last, _theta_cmd  = map(float, obs)
        self._last_obs = obs.copy()


        # ---------------- episode metrics update (unchanged) ----------------
        self._ep_step += 1
        ax = abs(x)
        # --- always-on position penalty (prevents "park off-center") ---
        w_x = float(getattr(self, "w_x", 0.35))  # try 0.2–0.8
        r_x = -w_x * (abs(x) / max(1e-6, self.x_limit))**2


        if ax > self._max_absx:
            self._max_absx = ax

        if ax < self.band_m:
            self._in_band_ctr += 1
            if self._in_band_ctr >= self._hold_steps_req and self._first_hold_step is None:
                self._first_hold_step = self._ep_step - self._hold_steps_req
        else:
            self._in_band_ctr = 0

        # ---------------- reward: center bump + servo jerk penalty ----------------

        # 1) center reward
        sigma = float(getattr(self, "gauss_sigma_m", 0.010))
        sigma = max(1e-4, sigma)
        r_center = math.exp(-0.5 * (x / sigma) ** 2)

        # 2) servo movement penalty (gated)
        w_u = float(getattr(self, "w_u", 0.10))
        r_offset = float(getattr(self, "r_offset", 0.0))

        # cap u to prevent rare spikes dominating
        u_cap = float(getattr(self, "u_cap", 2.0))
        u2 = float(np.clip(u, -u_cap, +u_cap))

        # gate: strong near center, weak far away
        g = r_center ** 2   # try 2 or 3
        r_servo = -(w_u * g) * (u2 * u2)

        # small velocity penalty ONLY near center (encourages true settle)
        v0 = float(getattr(self, "v0", 0.25))        # m/s scale
        w_v = float(getattr(self, "w_v", 0.02))      # small: 0.01–0.05
        g_v = r_center**2                             # only when near center
        r_v = -(w_v * g_v) * (xdot / max(1e-6, v0))**2

        # --- progress shaping: reward moving toward center ---
        w_p = float(getattr(self, "w_p", 0.25))  # start 0.15–0.35
        progress = float(self._ax_prev - abs(x))
        self._ax_prev = abs(x)

        progress = float(np.clip(progress, -0.01, +0.01))  # clamp meters/step
        r_progress = w_p * (progress / max(1e-6, self.ctrl_dt))


        # --- continuous end-zone penalty: discourage surfing near ends ---
        r_endzone = 0.0
        if self.use_endzone_penalty:
            if abs(x) > self.end_zone_start:
                denom = max(1e-6, (self.x_limit - self.end_zone_start))
                z = (abs(x) - self.end_zone_start) / denom  # 0..1
                w_end = float(getattr(self, "w_end", 0.35))  # start 0.2–0.6
                r_endzone = -w_end * (z * z)


        reward = r_center + r_servo + r_v + r_progress + r_endzone + r_x + r_offset

        if self._in_band_ctr == self._hold_steps_req:   # just achieved first hold
            reward += float(getattr(self, "r_success_bonus", 10.0))


        # ---------------- safety penalties + termination ----------------
        terminated = False
        truncated = False
        done_reason = None

        # Angle violation (terminate)
        angle_violation = abs(theta_obs) > self.theta_limit
        if angle_violation:
            reward -= 10.0
            terminated = True
            done_reason = "angle_violation"

        # Endstop contact logic
        contacts = self.p.getContactPoints(bodyA=self.ball_id, bodyB=self.robot_id)
        hit_endstop = any(c[4] in self.endstop_links for c in contacts)
        if hit_endstop:
            reward -= 5.0
            self._end_contact_steps += 1
        else:
            self._end_contact_steps = 0

        # End stuck logic
        in_end_zone = ax > self.end_zone_start
        if in_end_zone and (abs(xdot) < 0.012):
            self._end_stuck_steps += 1
            reward -= 0.05
        else:
            self._end_stuck_steps = 0

        if (not terminated) and (self._end_contact_steps >= self.end_contact_steps_limit):
            truncated = True
            done_reason = "end_contact"

        if (not terminated) and (self._end_stuck_steps >= self.end_stuck_steps_limit):
            truncated = True
            done_reason = "end_stuck"

        info = {
            "x": float(x),
            "xdot": float(xdot),
            "theta": float(theta_obs),
            "thdot": float(thdot_obs),
            "theta_sp": float(theta_sp),
            "theta_sp_delayed": float(theta_sp_delayed),
            "theta_cmd": float(self._theta_cmd),
            "hit_endstop": bool(hit_endstop),
            "angle_violation": bool(angle_violation),
            "done_reason": done_reason,

        }

        if terminated or truncated:
            success = (self._first_hold_step is not None)
            settling_time_s = (self._first_hold_step / self.ctrl_hz) if success else float("inf")
            info["metrics/success"] = float(success)
            info["metrics/settling_time_s"] = float(settling_time_s)
            info["metrics/max_overshoot_m"] = float(self._max_absx)
            info["metrics/band_m"] = float(self.band_m)
            info["metrics/hold_s"] = float(self.hold_s)

        return obs, float(reward), bool(terminated), bool(truncated), info

    # ---------------- render / close ----------------
    def render(self):
        if self.render_mode == "rgb_array":
            # Basic camera capture (optional)
            width, height = 640, 360
            view = self.p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=[0, 0, 0.05],
                distance=0.55,
                yaw=35,
                pitch=-25,
                roll=0,
                upAxisIndex=2,
            )
            proj = self.p.computeProjectionMatrixFOV(fov=60, aspect=width/height, nearVal=0.01, farVal=5.0)
            _, _, px, _, _ = self.p.getCameraImage(width, height, viewMatrix=view, projectionMatrix=proj)
            rgb = np.array(px, dtype=np.uint8)[:, :, :3]
            return rgb
        return None

    def close(self):
        if getattr(self, "p", None) is not None:
            self.p.disconnect()
            self.p = None
