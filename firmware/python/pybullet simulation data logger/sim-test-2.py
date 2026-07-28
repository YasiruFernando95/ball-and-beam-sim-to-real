"""
Automated Simulation Experiments for Ball-on-Beam
Runs 3 experiments matching hardware tests and saves CSV files
"""

import time
import math
import csv
import numpy as np
from pathlib import Path
from typing import List, Dict

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

from ball_beam_env_v2 import BallBeamCleanEnv


# =============================================================================
# CONFIGURATION
# =============================================================================

class Config:
    """Configuration for simulation experiments"""
    
    # Paths
    BASE_DIR = Path(__file__).resolve().parent
    MODEL_DIR = BASE_DIR / "models"
    MODEL_NAME = "sac_ballbeam_simple_32"
    MODEL_PATH = MODEL_DIR / MODEL_NAME
    
    # Output
    OUTPUT_DIR = BASE_DIR / "simulation_results_3"
    
    # Control parameters (match hardware)
    CTRL_HZ = 100.0
    DT = 1.0 / CTRL_HZ
    
    # Experiment durations
    TRIAL_DURATION_S = 30.0
    
    # Experiment A: Regulation at center
    EXP_A_TRIALS = 10
    EXP_A_INITIAL_X = 0.0  # Start at center
    
    # Experiment B: Step response
    EXP_B_TRIALS = 10
    EXP_B_INITIAL_X = 0.10  # Start at ±10cm (will alternate)
    
    # Experiment C: Disturbance rejection
    EXP_C_TRIALS = 5
    EXP_C_DISTURBANCE_TIMES = [10.0, 20.0]  # Apply disturbances at these times
    EXP_C_DISTURBANCE_MAGNITUDE = 0.05  # ±5cm disturbance (increased from 3cm)
    
    # Environment parameters (match training)
    THETA_LIMIT_DEG = 25.0
    SERVO_DELAY_STEPS = 13
    SERVO_TAU_S = 0.070
    SETPOINT_RATE_LIMIT_DEGPS = 160.0


# =============================================================================
# CSV WRITER
# =============================================================================

class SimulationCSVWriter:
    """Write simulation data to CSV in hardware-compatible format"""
    
    def __init__(self, filepath: Path, experiment: str, trial: int, notes: str = ""):
        self.filepath = filepath
        self.file = open(filepath, 'w', newline='')
        self.writer = csv.writer(self.file)
        
        # Write header matching hardware format
        self.writer.writerow([
            'time', 'time_rel', 'dt',
            'x', 'xdot', 'theta', 'theta_deg', 'thdot',
            'theta_cmd', 'theta_cmd_dot',
            'action_raw', 'action_used',
            'setpoint_desired', 'setpoint_sent',
            'armed', 'rl_enabled',
            'error', 'error_sq'
        ])
        
        # Write metadata
        self.file.write(f"# Experiment: {experiment}\n")
        self.file.write(f"# Trial: {trial}\n")
        self.file.write(f"# Controller: RL (Simulation)\n")
        self.file.write(f"# Start time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        if notes:
            self.file.write(f"# Notes: {notes}\n")
        self.file.flush()
        
        self.t_start = time.time()
        self.prev_theta_cmd = 0.0
    
    def write_row(self, t_rel: float, x: float, xdot: float, theta: float, thdot: float,
                  theta_cmd: float, action: float, setpoint_deg: float):
        """Write one data row"""
        
        # Calculate derivative of theta_cmd
        theta_cmd_dot = (theta_cmd - self.prev_theta_cmd) / Config.DT
        self.prev_theta_cmd = theta_cmd
        
        # Calculate error metrics
        error = abs(x)
        error_sq = x ** 2
        
        row = [
            f"{self.t_start + t_rel:.6f}",  # time (absolute)
            f"{t_rel:.6f}",                  # time_rel
            f"{Config.DT:.6f}",              # dt
            f"{x:.6f}",                      # x
            f"{xdot:.6f}",                   # xdot
            f"{theta:.6f}",                  # theta (rad)
            f"{math.degrees(theta):.4f}",    # theta_deg
            f"{thdot:.6f}",                  # thdot (rad/s)
            f"{theta_cmd:.6f}",              # theta_cmd
            f"{theta_cmd_dot:.6f}",          # theta_cmd_dot
            f"{action:.6f}",                 # action_raw
            f"{action:.6f}",                 # action_used (same in sim)
            f"{setpoint_deg:.4f}",           # setpoint_desired
            f"{setpoint_deg:.4f}",           # setpoint_sent (same in sim)
            "1",                             # armed
            "1",                             # rl_enabled
            f"{error:.6f}",                  # error
            f"{error_sq:.8f}"                # error_sq
        ]
        
        self.writer.writerow(row)
    
    def close(self):
        """Close the CSV file"""
        self.file.close()


# =============================================================================
# ENVIRONMENT SETUP
# =============================================================================

def make_env(render: bool = False):
    """Create environment matching hardware configuration"""
    env = BallBeamCleanEnv(
        render_mode="human" if render else None,
        ctrl_hz=Config.CTRL_HZ,
        time_step=1.0 / 240.0,
        theta_limit_deg=Config.THETA_LIMIT_DEG,
        servo_delay_steps=Config.SERVO_DELAY_STEPS,
        servo_tau_s=Config.SERVO_TAU_S,
        setpoint_rate_limit_degps=Config.SETPOINT_RATE_LIMIT_DEGPS,
    )
    return env


def reset_env_at_position(vec_env, x_pos: float):
    """
    Reset environment with ball at specific position.
    Uses options parameter to override randomness.
    """
    # First do a normal reset to initialize everything
    obs = vec_env.reset()
    
    # Get base environment
    base_env: BallBeamCleanEnv = vec_env.envs[0].unwrapped
    
    # Now directly set the ball position using PyBullet
    # Get current beam frame
    beam_pos, beam_orn = base_env._beam_frame()
    
    # Set desired position in beam frame (x along beam, at valley height)
    local_ball = [x_pos, 0.0, base_env._valley_z_beamframe + base_env.ball_r + base_env._spawn_eps]
    
    # Transform to world coordinates
    world_ball, _ = base_env.p.multiplyTransforms(beam_pos, beam_orn, local_ball, [0, 0, 0, 1])
    
    # Set ball position and zero velocity
    base_env.p.resetBasePositionAndOrientation(base_env.ball_id, world_ball, [0, 0, 0, 1])
    base_env.p.resetBaseVelocity(base_env.ball_id, [0, 0, 0], [0, 0, 0])
    
    # Let physics settle for a few steps
    for _ in range(10):
        base_env.p.stepSimulation()
    
    # Get new observation
    obs = base_env._get_obs()
    return np.array([obs])


def apply_disturbance(vec_env, magnitude: float):
    """
    Apply a disturbance to ball position and velocity.
    This creates a more visible effect like a manual push.
    """
    base_env: BallBeamCleanEnv = vec_env.envs[0].unwrapped
    
    # Get current ball state in beam frame
    obs = base_env._get_obs()
    current_x = float(obs[0])  # Current x position in beam frame
    
    # Calculate new position after disturbance
    new_x = current_x + magnitude
    
    # Get beam frame
    beam_pos, beam_orn = base_env._beam_frame()
    
    # Set new position in beam frame
    local_ball = [new_x, 0.0, base_env._valley_z_beamframe + base_env.ball_r + base_env._spawn_eps]
    
    # Transform to world coordinates
    world_ball, _ = base_env.p.multiplyTransforms(beam_pos, beam_orn, local_ball, [0, 0, 0, 1])
    
    # Apply position change
    base_env.p.resetBasePositionAndOrientation(base_env.ball_id, world_ball, [0, 0, 0, 1])
    
    # Add velocity in beam frame (like a push)
    velocity_magnitude = magnitude * 2.0  # Velocity proportional to displacement
    
    # Get beam x-axis direction in world frame
    beam_mat = base_env.p.getMatrixFromQuaternion(beam_orn)
    beam_x_w = np.array([beam_mat[0], beam_mat[3], beam_mat[6]], dtype=np.float32)
    
    # Set velocity along beam axis
    velocity_world = beam_x_w * velocity_magnitude
    base_env.p.resetBaseVelocity(base_env.ball_id, velocity_world.tolist(), [0, 0, 0])


# =============================================================================
# EXPERIMENT RUNNERS
# =============================================================================

def run_single_trial(vec_env, model, csv_writer: SimulationCSVWriter,
                    initial_x: float = 0.0,
                    disturbance_times: List[float] = None,
                    disturbance_magnitudes: List[float] = None):
    """
    Run a single trial and log data.
    
    Args:
        vec_env: Vectorized environment
        model: Trained SAC model
        csv_writer: CSV writer for logging
        initial_x: Starting position for ball (in meters)
        disturbance_times: List of times to apply disturbances (for Exp C)
        disturbance_magnitudes: Magnitudes of disturbances
    """
    base_env: BallBeamCleanEnv = vec_env.envs[0].unwrapped
    theta_limit_rad = float(getattr(base_env, "theta_limit", math.radians(Config.THETA_LIMIT_DEG)))
    
    n_steps = int(Config.TRIAL_DURATION_S / Config.DT)
    
    # Reset environment with specific initial position
    obs = reset_env_at_position(vec_env, initial_x)
    
    # Track disturbances
    disturbance_applied = [False] * (len(disturbance_times) if disturbance_times else 0)
    
    for k in range(n_steps):
        # Get action from policy
        action, _ = model.predict(obs, deterministic=True)
        
        # Step environment
        obs_new, reward, done, info = vec_env.step(action)
        
        # Extract observation
        obs0 = obs_new[0]
        x, xdot, th, thdot, _theta_sp_delayed_last, _theta_cmd = map(float, obs0)
        
        # Extract action and command
        action_val = float(np.asarray(action).ravel()[0])
        action_val = float(np.clip(action_val, -theta_limit_rad, +theta_limit_rad))
        theta_cmd_rad = float(getattr(base_env, "_theta_cmd", action_val))
        
        # Current time
        sim_t = k * Config.DT
        
        # Apply disturbances (for Experiment C)
        if disturbance_times is not None:
            for i, dist_time in enumerate(disturbance_times):
                if not disturbance_applied[i] and sim_t >= dist_time:
                    magnitude = disturbance_magnitudes[i] if disturbance_magnitudes else 0.05
                    apply_disturbance(vec_env, magnitude)
                    disturbance_applied[i] = True
                    print(f"  Applied disturbance: {magnitude*100:+.1f}cm at t={sim_t:.2f}s")
        
        # Write to CSV
        csv_writer.write_row(
            t_rel=sim_t,
            x=x,
            xdot=xdot,
            theta=th,
            thdot=thdot,
            theta_cmd=theta_cmd_rad,
            action=action_val,
            setpoint_deg=math.degrees(action_val)
        )
        
        obs = obs_new
        
        # Handle episode end - but don't reset during trials!
        if bool(done[0]) and k < n_steps - 1:
            # If episode ends prematurely, reset to same position
            print(f"  Episode ended at t={sim_t:.2f}s, resetting to continue trial...")
            obs = reset_env_at_position(vec_env, initial_x)
    
    csv_writer.close()


def run_experiment_a():
    """Experiment A: Regulation at center"""
    print("\n" + "="*60)
    print("EXPERIMENT A: Regulation at Center")
    print("="*60)
    print(f"Running {Config.EXP_A_TRIALS} trials, {Config.TRIAL_DURATION_S}s each")
    print(f"Initial position: {Config.EXP_A_INITIAL_X*100:.1f}cm (center)\n")
    
    # Create environment
    vec_env = DummyVecEnv([lambda: make_env(render=False)])
    vec_env = VecMonitor(vec_env)
    
    # Load model
    model = SAC.load(str(Config.MODEL_PATH), env=vec_env)
    
    # Run trials
    for trial in range(1, Config.EXP_A_TRIALS + 1):
        print(f"Trial {trial}/{Config.EXP_A_TRIALS}...", end=" ")
        
        # Create CSV writer
        output_file = Config.OUTPUT_DIR / f"sim_exp_a_trial_{trial}.csv"
        csv_writer = SimulationCSVWriter(
            output_file,
            experiment="A (Regulation)",
            trial=trial,
            notes=f"Ball starts at center (0cm)"
        )
        
        # Run trial with center start
        run_single_trial(vec_env, model, csv_writer, initial_x=Config.EXP_A_INITIAL_X)
        
        print(f"Done. Saved to {output_file.name}")
    
    vec_env.close()
    print("\nExperiment A complete!")


def run_experiment_b():
    """Experiment B: Step response from ±10cm"""
    print("\n" + "="*60)
    print("EXPERIMENT B: Step Response")
    print("="*60)
    print(f"Running {Config.EXP_B_TRIALS} trials, {Config.TRIAL_DURATION_S}s each")
    print(f"Initial positions: alternating +10cm and -10cm\n")
    
    # Create environment
    vec_env = DummyVecEnv([lambda: make_env(render=False)])
    vec_env = VecMonitor(vec_env)
    
    # Load model
    model = SAC.load(str(Config.MODEL_PATH), env=vec_env)
    
    # Run trials (alternate +10cm and -10cm)
    for trial in range(1, Config.EXP_B_TRIALS + 1):
        # Alternate between +10cm and -10cm
        initial_x = Config.EXP_B_INITIAL_X if (trial % 2 == 1) else -Config.EXP_B_INITIAL_X
        
        print(f"Trial {trial}/{Config.EXP_B_TRIALS} (start: {initial_x*100:+.1f}cm)...", end=" ")
        
        # Create CSV writer
        output_file = Config.OUTPUT_DIR / f"sim_exp_b_trial_{trial}.csv"
        csv_writer = SimulationCSVWriter(
            output_file,
            experiment="B (Step Response)",
            trial=trial,
            notes=f"Ball starts at {initial_x*100:+.1f}cm"
        )
        
        # Run trial with specified start position
        run_single_trial(vec_env, model, csv_writer, initial_x=initial_x)
        
        print(f"Done. Saved to {output_file.name}")
    
    vec_env.close()
    print("\nExperiment B complete!")


def run_experiment_c():
    """Experiment C: Disturbance rejection"""
    print("\n" + "="*60)
    print("EXPERIMENT C: Disturbance Rejection")
    print("="*60)
    print(f"Running {Config.EXP_C_TRIALS} trials, {Config.TRIAL_DURATION_S}s each")
    print(f"Disturbances at: {Config.EXP_C_DISTURBANCE_TIMES} seconds")
    print(f"Disturbance magnitude: ±{Config.EXP_C_DISTURBANCE_MAGNITUDE*100:.1f}cm\n")
    
    # Create environment
    vec_env = DummyVecEnv([lambda: make_env(render=False)])
    vec_env = VecMonitor(vec_env)
    
    # Load model
    model = SAC.load(str(Config.MODEL_PATH), env=vec_env)
    
    # Run trials
    for trial in range(1, Config.EXP_C_TRIALS + 1):
        print(f"Trial {trial}/{Config.EXP_C_TRIALS}...")
        
        # Alternate disturbance directions
        magnitudes = [
            Config.EXP_C_DISTURBANCE_MAGNITUDE if trial % 2 == 1 else -Config.EXP_C_DISTURBANCE_MAGNITUDE,
            -Config.EXP_C_DISTURBANCE_MAGNITUDE if trial % 2 == 1 else Config.EXP_C_DISTURBANCE_MAGNITUDE
        ]
        
        # Create CSV writer
        output_file = Config.OUTPUT_DIR / f"sim_exp_c_trial_{trial}.csv"
        csv_writer = SimulationCSVWriter(
            output_file,
            experiment="C (Disturbance Rejection)",
            trial=trial,
            notes=f"Disturbances: {magnitudes[0]*100:+.1f}cm @ {Config.EXP_C_DISTURBANCE_TIMES[0]}s, "
                  f"{magnitudes[1]*100:+.1f}cm @ {Config.EXP_C_DISTURBANCE_TIMES[1]}s"
        )
        
        # Run trial with disturbances (start at center)
        run_single_trial(
            vec_env, model, csv_writer,
            initial_x=0.0,  # Start at center
            disturbance_times=Config.EXP_C_DISTURBANCE_TIMES,
            disturbance_magnitudes=magnitudes
        )
        
        print(f"  Done. Saved to {output_file.name}")
    
    vec_env.close()
    print("\nExperiment C complete!")


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Run all experiments"""
    
    print("\n" + "="*60)
    print("AUTOMATED SIMULATION EXPERIMENTS")
    print("="*60)
    print(f"Model: {Config.MODEL_PATH}")
    print(f"Output directory: {Config.OUTPUT_DIR}")
    print(f"Control frequency: {Config.CTRL_HZ} Hz")
    print(f"Trial duration: {Config.TRIAL_DURATION_S}s")
    
    # Create output directory
    Config.OUTPUT_DIR.mkdir(exist_ok=True, parents=True)
    
    # Run experiments
    start_time = time.time()
    
    try:
        run_experiment_a()
        run_experiment_b()
        run_experiment_c()
        
        # Summary
        total_time = time.time() - start_time
        total_trials = Config.EXP_A_TRIALS + Config.EXP_B_TRIALS + Config.EXP_C_TRIALS
        
        print("\n" + "="*60)
        print("ALL EXPERIMENTS COMPLETE!")
        print("="*60)
        print(f"Total trials: {total_trials}")
        print(f"Total time: {total_time:.1f}s")
        print(f"Output: {Config.OUTPUT_DIR}/")
        print("\nCSV files are ready for comparison with hardware data!")
        print("Use plot_results_with_sim.py to visualize all results together.")
        
    except KeyboardInterrupt:
        print("\n\nExperiments interrupted by user.")
    except Exception as e:
        print(f"\n\nError during experiments: {e}")
        raise


if __name__ == "__main__":
    main()