"""
Hardware Inference for Ball-on-Beam Platform
CSV Logger for Paper Data Collection and PID Comparison
"""

import time
import math
import threading
import queue
import re
import csv
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple
from pathlib import Path
from collections import deque

import numpy as np
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from stable_baselines3 import SAC


# =============================================================================
# CONFIGURATION
# =============================================================================

class Config:
    """Centralized configuration for hardware control"""
    
    # Paths
    BASE_DIR = Path(__file__).resolve().parent
    MODEL_DIR = BASE_DIR / "models"
    MODEL_NAME = "sac_ballbeam_simple_32"
    DEFAULT_MODEL_PATH = MODEL_DIR / MODEL_NAME
    
    # Serial communication
    DEFAULT_BAUD = 115200
    DEFAULT_SEND_HZ = 100
    DEFAULT_TIMEOUT_S = 0.25
    
    # Telemetry units from MCU
    ANGLE_IS_DEG = True
    RATE_IS_DEG_PER_S = True
    
    # Command protocol
    TX_SETPOINT_CMD = "THSP"
    TX_STOP_CMD = "STOP"
    TX_RESET_CMD = "RST"
    
    # Action mapping
    DEFAULT_ANGLE_LIMIT_DEG = 25.0
    DEFAULT_A_DEADZONE = 0.0
    DEFAULT_A_CLAMP = 1.0
    DEFAULT_SP_RATE_LIMIT_DEGPS = 120.0
    DEFAULT_SP_NOISE_DEG = 0.0
    DEFAULT_FLIP_X = False
    
    # Observation bias (for centering)
    DEFAULT_X_BIAS = 0.0  # Set to -0.02 if ball settles at +2cm
    
    # CSV logging
    DEFAULT_CSV_BUFFER_SIZE = 100  # Write every N samples
    
    # Trial timing
    TRIAL_DURATION_S = 30.0  # Auto-stop after this duration
    
    # Observation space (6D: x, xdot, theta, thdot, theta_cmd, theta_cmd_dot)
    OBS_DIM = 6
    
    # State limits (for reference, not normalization)
    X_MAX = 0.15  # meters
    XD_MAX = 0.6  # m/s
    TH_MAX = math.radians(25.0)  # rad
    THD_MAX = math.radians(300.0)  # rad/s
    THCMD_MAX = TH_MAX
    
    # Actuator model (must match training)
    DEFAULT_SERVO_DELAY_STEPS = 2
    DEFAULT_SERVO_TAU_S = 0.04


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class Telemetry:
    """Telemetry data from MCU"""
    t_wall: float
    x: Optional[float] = None  # meters
    xdot: Optional[float] = None  # m/s
    theta: Optional[float] = None  # rad
    thdot: Optional[float] = None  # rad/s
    theta_cmd: Optional[float] = None  # rad (MCU's actual command)
    motor_speed: Optional[float] = None  # rad/s
    raw: str = ""
    
    def is_valid(self) -> bool:
        """Check if all required fields are present"""
        return all([
            self.x is not None,
            self.xdot is not None,
            self.theta is not None,
            self.thdot is not None
        ])


@dataclass
class ObservationState:
    """Full 6D observation state"""
    x: float
    xdot: float
    theta: float
    thdot: float
    theta_cmd: float
    theta_cmd_dot: float
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array for policy input"""
        return np.array([
            self.x,
            self.xdot,
            self.theta,
            self.thdot,
            self.theta_cmd,
            self.theta_cmd_dot
        ], dtype=np.float32)


# =============================================================================
# TELEMETRY PARSING
# =============================================================================

def parse_kv_telemetry(line: str) -> Dict[str, float]:
    """
    Parse key-value telemetry lines.
    Example: "X:0.0123,XD:-0.01,TH:2.50,THD:-12.0,W:123.4"
    Returns dict of {key: float}.
    """
    out: Dict[str, float] = {}
    parts = re.split(r"[,\s]+", line.strip())
    for p in parts:
        if ":" in p:
            k, v = p.split(":", 1)
            k = k.strip().upper()
            v = v.strip()
            try:
                out[k] = float(v)
            except ValueError:
                pass
    return out


def build_telemetry_from_dict(kv: Dict[str, float], t_wall: float, raw: str) -> Telemetry:
    """
    Build Telemetry object from parsed key-value dict.
    Handles unit conversions from MCU to SI units.
    """
    tel = Telemetry(t_wall=t_wall, raw=raw)
    
    # Position and velocity (assume meters)
    tel.x = kv.get("X")
    tel.xdot = kv.get("XD")
    
    # Angle measurements
    if "TH" in kv:
        th_val = kv["TH"]
        tel.theta = math.radians(th_val) if Config.ANGLE_IS_DEG else th_val
    
    if "THD" in kv:
        thd_val = kv["THD"]
        tel.thdot = math.radians(thd_val) if Config.RATE_IS_DEG_PER_S else thd_val
    
    # Command angle (from MCU)
    if "THCMD" in kv:
        thcmd_val = kv["THCMD"]
        tel.theta_cmd = math.radians(thcmd_val) if Config.ANGLE_IS_DEG else thcmd_val
    
    # Motor speed
    if "W" in kv:
        w_val = kv["W"]
        tel.motor_speed = math.radians(w_val) if Config.RATE_IS_DEG_PER_S else w_val
    
    return tel


# =============================================================================
# SERVO MODEL
# =============================================================================

class ServoModel:
    """
    First-order servo model with delay to reconstruct theta_cmd states.
    Must match training environment's actuator model.
    """
    
    def __init__(self, delay_steps: int = Config.DEFAULT_SERVO_DELAY_STEPS,
                 tau_s: float = Config.DEFAULT_SERVO_TAU_S,
                 dt_s: float = 0.01):
        self.delay_steps = delay_steps
        self.tau_s = tau_s
        self.dt_s = dt_s
        self.alpha = dt_s / (tau_s + dt_s)  # low-pass filter coefficient
        
        # State tracking
        self.theta_cmd_actual = 0.0  # current filtered command
        self.theta_cmd_prev = 0.0
        self.delay_buffer = deque([0.0] * max(delay_steps, 1), maxlen=max(delay_steps, 1))
    
    def reset(self):
        """Reset servo model state"""
        self.theta_cmd_actual = 0.0
        self.theta_cmd_prev = 0.0
        self.delay_buffer.clear()
        self.delay_buffer.extend([0.0] * self.delay_buffer.maxlen)
    
    def update(self, sp_desired_rad: float) -> Tuple[float, float]:
        """
        Update servo model with new desired setpoint.
        
        Args:
            sp_desired_rad: Desired setpoint in radians
            
        Returns:
            (theta_cmd, theta_cmd_dot): Current command and its derivative
        """
        # Add to delay buffer
        self.delay_buffer.append(sp_desired_rad)
        
        # Get delayed setpoint
        sp_delayed = self.delay_buffer[0]
        
        # Apply first-order filter
        self.theta_cmd_prev = self.theta_cmd_actual
        self.theta_cmd_actual += self.alpha * (sp_delayed - self.theta_cmd_actual)
        
        # Compute derivative
        theta_cmd_dot = (self.theta_cmd_actual - self.theta_cmd_prev) / self.dt_s
        
        return self.theta_cmd_actual, theta_cmd_dot
    
    def get_state(self) -> Tuple[float, float]:
        """Get current servo state"""
        return self.theta_cmd_actual, (self.theta_cmd_actual - self.theta_cmd_prev) / self.dt_s


# =============================================================================
# OBSERVATION BUILDER
# =============================================================================

class ObservationBuilder:
    """Builds 6D observations from telemetry and servo model"""
    
    def __init__(self, servo_model: ServoModel, flip_x: bool = False, x_bias: float = 0.0):
        self.servo_model = servo_model
        self.flip_x = flip_x
        self.x_bias = x_bias
    
    def build_observation(self, tel: Telemetry) -> Optional[ObservationState]:
        """
        Build 6D observation from telemetry and servo model state.
        
        Returns None if telemetry is invalid.
        """
        if not tel.is_valid():
            return None
        
        # Get servo model state
        theta_cmd, theta_cmd_dot = self.servo_model.get_state()
        
        # Build observation with bias
        x = tel.x + self.x_bias
        if self.flip_x:
            x = -x
            
        return ObservationState(
            x=x,
            xdot=tel.xdot,
            theta=tel.theta,
            thdot=tel.thdot,
            theta_cmd=theta_cmd,
            theta_cmd_dot=theta_cmd_dot
        )


# =============================================================================
# ACTION PROCESSOR
# =============================================================================

class ActionProcessor:
    """Processes policy actions into hardware commands"""
    
    def __init__(self,
                 angle_limit_deg: float = Config.DEFAULT_ANGLE_LIMIT_DEG,
                 deadzone: float = Config.DEFAULT_A_DEADZONE,
                 clamp: float = Config.DEFAULT_A_CLAMP,
                 rate_limit_degps: float = Config.DEFAULT_SP_RATE_LIMIT_DEGPS,
                 noise_deg: float = Config.DEFAULT_SP_NOISE_DEG,
                 dt_s: float = 0.01):
        
        self.angle_limit_deg = angle_limit_deg
        self.deadzone = deadzone
        self.clamp = clamp
        self.rate_limit_degps = rate_limit_degps
        self.noise_deg = noise_deg
        self.dt_s = dt_s
        
        self.last_sp_deg = 0.0
    
    def reset(self):
        """Reset processor state"""
        self.last_sp_deg = 0.0
    
    def process_action(self, action_raw: float) -> Tuple[float, float, float]:
        """
        Process raw policy action into setpoint to send to MCU.
        
        Args:
            action_raw: Raw action from policy (typically in [-1, 1])
            
        Returns:
            (action_used, sp_desired_deg, sp_sent_deg): 
                - action_used: action after clamp/deadzone
                - sp_desired_deg: desired setpoint before rate limiting
                - sp_sent_deg: final setpoint to send (after rate limiting and noise)
        """
        # Apply deadzone and clamp
        a = float(action_raw)
        if abs(a) < self.deadzone:
            a = 0.0
        a = np.clip(a, -self.clamp, self.clamp)
        
        # Map to angle setpoint
        sp_desired_deg = a * self.angle_limit_deg
        
        # Apply rate limiting
        max_change = self.rate_limit_degps * self.dt_s
        delta = sp_desired_deg - self.last_sp_deg
        delta_clamped = np.clip(delta, -max_change, max_change)
        sp_sent_deg = self.last_sp_deg + delta_clamped
        
        # Add noise if configured
        if self.noise_deg > 0:
            sp_sent_deg += np.random.uniform(-self.noise_deg, self.noise_deg)
        
        # Update state
        self.last_sp_deg = sp_sent_deg
        
        return a, sp_desired_deg, sp_sent_deg


# =============================================================================
# CSV DATA LOGGER
# =============================================================================

class CSVLogger:
    """
    CSV logger for paper data collection and analysis.
    Creates a single CSV file per run with all data.
    
    CSV Columns:
    - time: Wall clock time (seconds since epoch)
    - time_rel: Relative time since start (seconds)
    - dt: Time step (seconds)
    - x: Ball position (meters)
    - xdot: Ball velocity (m/s)
    - theta: Beam angle (radians)
    - theta_deg: Beam angle (degrees, for convenience)
    - thdot: Beam angular velocity (rad/s)
    - theta_cmd: Command angle from servo model (radians)
    - theta_cmd_dot: Command angle derivative (rad/s)
    - action_raw: Raw policy action
    - action_used: Action after deadzone/clamp
    - setpoint_desired: Desired setpoint before rate limit (degrees)
    - setpoint_sent: Actual setpoint sent to MCU (degrees)
    - armed: System armed (0/1)
    - rl_enabled: RL control enabled (0/1)
    - error: Absolute position error |x| (meters)
    - error_sq: Squared position error (meters²)
    """
    
    def __init__(self, base_dir: str = "runs", buffer_size: int = Config.DEFAULT_CSV_BUFFER_SIZE):
        self.base_dir = Path(base_dir)
        self.buffer_size = buffer_size
        self._enabled = False
        self._file_path: Optional[Path] = None
        self._csv_file = None
        self._csv_writer = None
        self._lock = threading.Lock()
        self._buffer = []
        self._t_start = None
        self._sample_count = 0
    
    @property
    def enabled(self) -> bool:
        return self._enabled
    
    @property
    def file_path(self) -> str:
        return str(self._file_path) if self._file_path else ""
    
    @property
    def sample_count(self) -> int:
        return self._sample_count
    
    def start(self, run_name: str, controller: str = "RL", notes: str = ""):
        """
        Start logging a new run.
        
        Args:
            run_name: Name for this run
            controller: Controller type (e.g., "RL", "PID")
            notes: Additional notes about the run
        """
        with self._lock:
            if self._enabled:
                self.stop()
            
            # Create directory and filename
            ts = time.strftime("%Y%m%d_%H%M%S")
            safe_name = "".join([c if c.isalnum() or c in "-_" else "_" for c in str(run_name)])
            self._file_path = self.base_dir / f"{ts}_{controller}_{safe_name}.csv"
            self._file_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Open CSV file
            self._csv_file = open(self._file_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            
            # Write header
            self._csv_writer.writerow([
                'time', 'time_rel', 'dt',
                'x', 'xdot', 'theta', 'theta_deg', 'thdot',
                'theta_cmd', 'theta_cmd_dot',
                'action_raw', 'action_used',
                'setpoint_desired', 'setpoint_sent',
                'armed', 'rl_enabled',
                'error', 'error_sq'
            ])
            
            # Write metadata as comments (if CSV reader supports it)
            self._csv_file.write(f"# Run: {run_name}\n")
            self._csv_file.write(f"# Controller: {controller}\n")
            self._csv_file.write(f"# Start time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            if notes:
                self._csv_file.write(f"# Notes: {notes}\n")
            self._csv_file.write(f"# Observation dim: {Config.OBS_DIM}\n")
            
            self._csv_file.flush()
            
            self._buffer = []
            self._t_start = time.time()
            self._sample_count = 0
            self._enabled = True
    
    def stop(self):
        """Stop logging and close file"""
        with self._lock:
            if not self._enabled:
                return
            
            # Flush remaining buffer
            self._flush_locked()
            
            # Close file
            if self._csv_file:
                self._csv_file.close()
                self._csv_file = None
                self._csv_writer = None
            
            self._enabled = False
    
    def add(self, *, t_wall: float, dt: float, tel: Telemetry, obs: ObservationState,
            action_raw: float, action_used: float, sp_desired_deg: float, sp_sent_deg: float,
            armed: bool, rl_enabled: bool):
        """Add a data sample"""
        with self._lock:
            if not self._enabled:
                return
            
            # Calculate relative time
            t_rel = t_wall - self._t_start if self._t_start else 0.0
            
            # Calculate error metrics
            error = abs(tel.x) if tel.x is not None else float('nan')
            error_sq = (tel.x ** 2) if tel.x is not None else float('nan')
            
            # Create row
            row = [
                f"{t_wall:.6f}",
                f"{t_rel:.6f}",
                f"{dt:.6f}",
                f"{tel.x:.6f}" if tel.x is not None else "",
                f"{tel.xdot:.6f}" if tel.xdot is not None else "",
                f"{tel.theta:.6f}" if tel.theta is not None else "",
                f"{math.degrees(tel.theta):.4f}" if tel.theta is not None else "",
                f"{tel.thdot:.6f}" if tel.thdot is not None else "",
                f"{obs.theta_cmd:.6f}",
                f"{obs.theta_cmd_dot:.6f}",
                f"{action_raw:.6f}",
                f"{action_used:.6f}",
                f"{sp_desired_deg:.4f}",
                f"{sp_sent_deg:.4f}",
                "1" if armed else "0",
                "1" if rl_enabled else "0",
                f"{error:.6f}",
                f"{error_sq:.8f}"
            ]
            
            self._buffer.append(row)
            self._sample_count += 1
            
            # Flush if buffer full
            if len(self._buffer) >= self.buffer_size:
                self._flush_locked()
    
    def _flush_locked(self):
        """Flush buffer to disk"""
        if not self._buffer or not self._csv_writer:
            return
        
        self._csv_writer.writerows(self._buffer)
        self._csv_file.flush()
        self._buffer = []


# =============================================================================
# SERIAL COMMUNICATION
# =============================================================================

class SerialComm:
    """Handles serial communication with MCU"""
    
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self._read_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._tel_queue = queue.Queue(maxsize=100)
    
    def connect(self, port: str, baud: int = Config.DEFAULT_BAUD, timeout: float = 0.1) -> bool:
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            time.sleep(0.5)  # Allow connection to stabilize
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # Start read thread
            self._stop_event.clear()
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            
            return True
        except Exception as e:
            print(f"Serial connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        self._stop_event.set()
        if self._read_thread:
            self._read_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
    
    def _read_loop(self):
        """Background thread to read serial data"""
        while not self._stop_event.is_set() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        t_wall = time.time()
                        try:
                            self._tel_queue.put_nowait((t_wall, line))
                        except queue.Full:
                            pass  # Drop old data if queue full
                else:
                    time.sleep(0.001)
            except Exception:
                break
    
    def get_telemetry(self) -> Optional[Tuple[float, str]]:
        """Get next telemetry line (non-blocking)"""
        try:
            return self._tel_queue.get_nowait()
        except queue.Empty:
            return None
    
    def send_command(self, cmd: str):
        """Send command to MCU"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(f"{cmd}\n".encode("utf-8"))
            except Exception as e:
                print(f"Send error: {e}")
    
    def send_setpoint(self, angle_deg: float):
        """Send angle setpoint command"""
        self.send_command(f"{Config.TX_SETPOINT_CMD} {angle_deg:.2f}")
    
    def send_stop(self):
        """Send stop command"""
        self.send_command(Config.TX_STOP_CMD)
    
    def send_reset(self):
        """Send reset command"""
        self.send_command(Config.TX_RESET_CMD)


# =============================================================================
# HARDWARE CONTROLLER
# =============================================================================

class HardwareController:
    """Main hardware control loop"""
    
    def __init__(self):
        # Components
        self.serial_comm = SerialComm()
        self.servo_model = ServoModel()
        self.obs_builder = ObservationBuilder(self.servo_model)
        self.action_processor = ActionProcessor()
        self.logger = CSVLogger()
        
        # Policy
        self.policy: Optional[SAC] = None
        
        # State
        self.armed = False
        self.enable_rl = False
        self.emergency_stop = False
        self.latest_tel: Optional[Telemetry] = None
        self.sp_sent_deg = 0.0
        
        # Timing
        self.send_hz = Config.DEFAULT_SEND_HZ
        self.dt_s = 1.0 / self.send_hz
        self.timeout_s = Config.DEFAULT_TIMEOUT_S
        self.last_tel_time = 0.0
        
        # Trial timer
        self.trial_start_time = None
        self.trial_duration = Config.TRIAL_DURATION_S
        
        # Control loop
        self._loop_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
    
    @property
    def ser(self):
        """Expose serial connection for UI status checks"""
        return self.serial_comm.ser
    
    @property
    def time_remaining(self) -> float:
        """Get remaining trial time in seconds"""
        if not self.armed or self.trial_start_time is None:
            return self.trial_duration
        elapsed = time.time() - self.trial_start_time
        return max(0.0, self.trial_duration - elapsed)
    
    def load_policy(self, model_path: str):
        """Load SAC policy from file"""
        try:
            self.policy = SAC.load(model_path)
            print(f"Policy loaded from: {model_path}")
        except Exception as e:
            raise RuntimeError(f"Failed to load policy: {e}")
    
    def connect(self, port: str, baud: int = Config.DEFAULT_BAUD) -> bool:
        """Connect to hardware"""
        return self.serial_comm.connect(port, baud)
    
    def disconnect(self):
        """Disconnect from hardware"""
        self.stop_now()
        self.serial_comm.disconnect()
    
    def arm(self):
        """Arm the system and start trial timer"""
        self.armed = True
        self.emergency_stop = False
        self.servo_model.reset()
        self.action_processor.reset()
        self.last_tel_time = time.time()
        self.trial_start_time = time.time()  # Start timer
    
    def stop_now(self):
        """Emergency stop"""
        self.armed = False
        self.emergency_stop = True
        self.serial_comm.send_stop()
        self.serial_comm.send_setpoint(0.0)
        self.sp_sent_deg = 0.0
        self.trial_start_time = None  # Clear timer
    
    def reset_mcu(self):
        """Reset MCU"""
        self.stop_now()
        self.serial_comm.send_reset()
    
    def start_logging(self, run_name: str, controller: str = "RL", notes: str = ""):
        """Start data logging"""
        self.logger.start(run_name, controller, notes)
    
    def stop_logging(self):
        """Stop data logging"""
        print("stop_logging() called")
        self.logger.stop()
        if self.logger.file_path:
            print(f"Logging stopped. Saved to: {self.logger.file_path}")
    
    def update_params(self, **kwargs):
        """Update controller parameters"""
        if "send_hz" in kwargs:
            self.send_hz = kwargs["send_hz"]
            self.dt_s = 1.0 / self.send_hz
            self.servo_model.dt_s = self.dt_s
            self.action_processor.dt_s = self.dt_s
        
        if "timeout_s" in kwargs:
            self.timeout_s = kwargs["timeout_s"]
        
        if "flip_x" in kwargs:
            self.obs_builder.flip_x = kwargs["flip_x"]
        
        if "x_bias" in kwargs:
            self.obs_builder.x_bias = kwargs["x_bias"]
        
        # Action processor params
        for key in ["angle_limit_deg", "deadzone", "clamp", "rate_limit_degps", "noise_deg"]:
            if key in kwargs:
                setattr(self.action_processor, key, kwargs[key])
        
        # Servo model params
        if "servo_delay_steps" in kwargs or "servo_tau_s" in kwargs:
            delay = kwargs.get("servo_delay_steps", self.servo_model.delay_steps)
            tau = kwargs.get("servo_tau_s", self.servo_model.tau_s)
            self.servo_model = ServoModel(delay, tau, self.dt_s)
            self.obs_builder.servo_model = self.servo_model
    
    def start_control_loop(self):
        """Start background control loop"""
        if self._loop_thread and self._loop_thread.is_alive():
            return
        
        self._stop_event.clear()
        self._loop_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._loop_thread.start()
    
    def stop_control_loop(self):
        """Stop background control loop"""
        self._stop_event.set()
        if self._loop_thread:
            self._loop_thread.join(timeout=2.0)
    
    def _control_loop(self):
        """Main control loop (runs in background thread)"""
        next_send = time.time()
        
        while not self._stop_event.is_set():
            now = time.time()
            
            # Check trial timer - auto-stop after duration
            if self.armed and self.trial_start_time is not None:
                elapsed = now - self.trial_start_time
                if elapsed >= self.trial_duration:
                    print(f"\n=== AUTO-STOP TRIGGERED ===")
                    print(f"Trial duration reached ({self.trial_duration}s), auto-stopping...")
                    print(f"Logger enabled: {self.logger.enabled}")
                    
                    # Stop logging FIRST
                    self.stop_logging()
                    
                    # Then disarm (this clears trial_start_time)
                    self.stop_now()
                    
                    print(f"After stop - Logger enabled: {self.logger.enabled}")
                    print("=========================\n")
            
            # Process incoming telemetry
            tel_data = self.serial_comm.get_telemetry()
            if tel_data:
                t_wall, line = tel_data
                self._process_telemetry(t_wall, line)
            
            # Control tick
            if now >= next_send:
                self._control_tick(now)
                next_send = now + self.dt_s
            
            # Sleep to avoid busy-waiting
            time.sleep(0.001)
    
    def _process_telemetry(self, t_wall: float, line: str):
        """Process incoming telemetry line"""
        kv = parse_kv_telemetry(line)
        if not kv:
            return
        
        tel = build_telemetry_from_dict(kv, t_wall, line)
        self.latest_tel = tel
        self.last_tel_time = t_wall
    
    def _control_tick(self, now: float):
        """Execute one control tick"""
        # Check timeout
        if now - self.last_tel_time > self.timeout_s:
            if self.armed and not self.emergency_stop:
                print("Telemetry timeout - stopping")
                self.stop_now()
            return
        
        # Get latest telemetry
        tel = self.latest_tel
        if not tel or not tel.is_valid():
            return
        
        # Build observation
        obs_state = self.obs_builder.build_observation(tel)
        if obs_state is None:
            return
        
        obs_array = obs_state.to_array()
        
        # Compute action
        action_raw = 0.0
        if self.armed and self.enable_rl and self.policy:
            try:
                action_raw, _ = self.policy.predict(obs_array, deterministic=True)
                action_raw = float(action_raw)
            except Exception as e:
                print(f"Policy error: {e}")
                action_raw = 0.0
        
        # Process action
        action_used, sp_desired_deg, sp_sent_deg = self.action_processor.process_action(action_raw)
        
        # Send command if armed
        if self.armed and not self.emergency_stop:
            self.serial_comm.send_setpoint(sp_sent_deg)
            self.sp_sent_deg = sp_sent_deg
            
            # Update servo model
            sp_sent_rad = math.radians(sp_sent_deg)
            self.servo_model.update(sp_sent_rad)
        else:
            sp_sent_deg = 0.0
            self.sp_sent_deg = 0.0
        
        # Log data
        self.logger.add(
            t_wall=tel.t_wall,
            dt=self.dt_s,
            tel=tel,
            obs=obs_state,
            action_raw=action_raw,
            action_used=action_used,
            sp_desired_deg=sp_desired_deg,
            sp_sent_deg=sp_sent_deg,
            armed=self.armed,
            rl_enabled=self.enable_rl
        )


# =============================================================================
# GUI APPLICATION
# =============================================================================

class App(tk.Tk):
    """GUI application for hardware control"""
    
    def __init__(self):
        super().__init__()
        
        self.title("Ball-on-Beam Hardware Control - CSV Logger")
        self.geometry("1000x650")
        self.resizable(False, False)
        
        # Controller
        self.ctrl = HardwareController()
        
        # Build UI
        self._build_ui()
        
        # Start control loop
        self.ctrl.start_control_loop()
        
        # UI update loop
        self.after(33, self._update_ui_loop)
        
        # Cleanup on close
        self.protocol("WM_DELETE_WINDOW", self._on_close)
    
    def _list_ports(self) -> List[str]:
        """List available serial ports"""
        return [p.device for p in serial.tools.list_ports.comports()]
    
    def _default_port(self) -> str:
        """Get default serial port"""
        ports = self._list_ports()
        return ports[0] if ports else ""
    
    def _refresh_ports(self):
        """Refresh port list"""
        self.port_box["values"] = self._list_ports()
    
    def _browse_model(self):
        """Browse for model file"""
        path = filedialog.askopenfilename(
            title="Select Model",
            initialdir=Config.MODEL_DIR,
            filetypes=[("ZIP files", "*.zip"), ("All files", "*.*")]
        )
        if path:
            self.model_path_var.set(path)
    
    def _load_model(self):
        """Load policy model"""
        try:
            path = self.model_path_var.get()
            self.ctrl.load_policy(path)
            messagebox.showinfo("Success", f"Model loaded:\n{path}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load model:\n{e}")
    
    def _apply_params(self):
        """Apply UI parameters to controller"""
        try:
            self.ctrl.update_params(
                send_hz=self.hz_var.get(),
                angle_limit_deg=self.angle_limit_var.get(),
                deadzone=self.deadzone_var.get(),
                clamp=self.clamp_var.get(),
                rate_limit_degps=self.rate_var.get(),
                noise_deg=self.noise_var.get(),
                flip_x=self.flipx_var.get(),
                x_bias=self.x_bias_var.get()
            )
        except Exception as e:
            print(f"Param update error: {e}")
    
    def _on_connect(self):
        """Connect to hardware"""
        try:
            port = self.port_var.get()
            baud = self.baud_var.get()
            if self.ctrl.connect(port, baud):
                self.btn_connect.config(state="disabled")
                self.btn_disconnect.config(state="normal")
                messagebox.showinfo("Connected", f"Connected to {port}")
            else:
                messagebox.showerror("Error", "Connection failed")
        except Exception as e:
            messagebox.showerror("Error", f"Connection error:\n{e}")
    
    def _on_disconnect(self):
        """Disconnect from hardware"""
        try:
            self.ctrl.disconnect()
            self.btn_connect.config(state="normal")
            self.btn_disconnect.config(state="disabled")
        except Exception as e:
            messagebox.showerror("Error", f"Disconnect error:\n{e}")
    
    def _toggle_rl(self):
        """Toggle RL control"""
        self.ctrl.enable_rl = self.enable_var.get()
    
    def _on_arm(self):
        """Arm the system and auto-start logging"""
        try:
            if not self.ctrl.ser:
                messagebox.showwarning("Not Connected", "Connect to hardware first")
                return
            
            # Get run parameters from dialog
            dialog = LogStartDialog(self)
            self.wait_window(dialog)
            
            if dialog.result:
                run_name = dialog.result['run_name']
                controller = dialog.result['controller']
                notes = dialog.result['notes']
                
                # Start logging first
                self.ctrl.start_logging(run_name=run_name, controller=controller, notes=notes)
                
                # Then ARM (which starts timer)
                self.ctrl.arm()
                
                print(f"ARMED - Trial will run for {Config.TRIAL_DURATION_S} seconds")
        except Exception as e:
            messagebox.showerror("Error", f"Arm error:\n{e}")
    
    def _on_stop(self):
        """Emergency stop"""
        try:
            self.ctrl.stop_now()
        except Exception as e:
            messagebox.showerror("Error", f"Stop error:\n{e}")
    
    def _on_reset(self):
        """Reset MCU"""
        try:
            self.ctrl.reset_mcu()
        except Exception as e:
            messagebox.showerror("Error", f"Reset error:\n{e}")
    
    def _on_start_log(self):
        """Start logging"""
        # Get run parameters
        dialog = LogStartDialog(self)
        self.wait_window(dialog)
        
        if dialog.result:
            try:
                run_name = dialog.result['run_name']
                controller = dialog.result['controller']
                notes = dialog.result['notes']
                self.ctrl.start_logging(run_name=run_name, controller=controller, notes=notes)
            except Exception as e:
                messagebox.showerror("Logging", f"Failed to start logging:\n{e}")
    
    def _on_stop_log(self):
        """Stop logging"""
        try:
            self.ctrl.stop_logging()
            if self.ctrl.logger.file_path:
                messagebox.showinfo("Logging Stopped", 
                    f"Logged {self.ctrl.logger.sample_count} samples to:\n{self.ctrl.logger.file_path}")
        except Exception as e:
            messagebox.showerror("Logging", f"Failed to stop logging:\n{e}")
    
    def _build_ui(self):
        """Build UI components"""
        # Connection controls
        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=8)
        
        ttk.Label(top, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value=self._default_port())
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=18, values=self._list_ports())
        self.port_box.pack(side="left", padx=6)
        
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left", padx=4)
        
        ttk.Label(top, text="Baud:").pack(side="left", padx=(12, 0))
        self.baud_var = tk.IntVar(value=Config.DEFAULT_BAUD)
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side="left", padx=6)
        
        self.btn_connect = ttk.Button(top, text="Connect", command=self._on_connect)
        self.btn_connect.pack(side="left", padx=6)
        
        self.btn_disconnect = ttk.Button(top, text="Disconnect", command=self._on_disconnect, state="disabled")
        self.btn_disconnect.pack(side="left", padx=6)
        
        ttk.Separator(self).pack(fill="x", padx=10, pady=6)
        
        # Model controls
        mid = ttk.Frame(self)
        mid.pack(fill="x", padx=10)
        
        ttk.Label(mid, text="Model:").pack(side="left")
        self.model_path_var = tk.StringVar(value=str(Config.DEFAULT_MODEL_PATH))
        ttk.Entry(mid, textvariable=self.model_path_var, width=62).pack(side="left", padx=6)
        
        ttk.Button(mid, text="Browse", command=self._browse_model).pack(side="left", padx=4)
        ttk.Button(mid, text="Load", command=self._load_model).pack(side="left", padx=4)
        
        self.enable_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(mid, text="Enable RL", variable=self.enable_var, command=self._toggle_rl).pack(side="left", padx=(18, 0))
        
        ttk.Separator(self).pack(fill="x", padx=10, pady=6)
        
        # Parameters
        mid2 = ttk.Frame(self)
        mid2.pack(fill="x", padx=10, pady=6)
        
        ttk.Label(mid2, text="Deadzone:").pack(side="left")
        self.deadzone_var = tk.DoubleVar(value=Config.DEFAULT_A_DEADZONE)
        ttk.Entry(mid2, textvariable=self.deadzone_var, width=6).pack(side="left", padx=6)
        
        ttk.Label(mid2, text="Clamp:").pack(side="left")
        self.clamp_var = tk.DoubleVar(value=Config.DEFAULT_A_CLAMP)
        ttk.Entry(mid2, textvariable=self.clamp_var, width=6).pack(side="left", padx=6)
        
        ttk.Label(mid2, text="Send Hz:").pack(side="left")
        self.hz_var = tk.IntVar(value=Config.DEFAULT_SEND_HZ)
        ttk.Entry(mid2, textvariable=self.hz_var, width=6).pack(side="left", padx=6)
        
        ttk.Label(mid2, text="Angle lim (deg):").pack(side="left", padx=(12, 0))
        self.angle_limit_var = tk.DoubleVar(value=Config.DEFAULT_ANGLE_LIMIT_DEG)
        ttk.Entry(mid2, textvariable=self.angle_limit_var, width=6).pack(side="left", padx=6)
        
        ttk.Label(mid2, text="Rate lim (deg/s):").pack(side="left", padx=(12, 0))
        self.rate_var = tk.DoubleVar(value=Config.DEFAULT_SP_RATE_LIMIT_DEGPS)
        ttk.Entry(mid2, textvariable=self.rate_var, width=7).pack(side="left", padx=6)
        
        ttk.Label(mid2, text="SP noise (deg):").pack(side="left", padx=(12, 0))
        self.noise_var = tk.DoubleVar(value=Config.DEFAULT_SP_NOISE_DEG)
        ttk.Entry(mid2, textvariable=self.noise_var, width=6).pack(side="left", padx=6)
        
        self.flipx_var = tk.BooleanVar(value=Config.DEFAULT_FLIP_X)
        ttk.Checkbutton(mid2, text="Flip x", variable=self.flipx_var).pack(side="left", padx=(12, 0))
        
        ttk.Label(mid2, text="X bias (m):").pack(side="left", padx=(12, 0))
        self.x_bias_var = tk.DoubleVar(value=Config.DEFAULT_X_BIAS)
        ttk.Entry(mid2, textvariable=self.x_bias_var, width=7).pack(side="left", padx=6)
        
        ttk.Separator(self).pack(fill="x", padx=10, pady=6)
        
        # Status display
        grid = ttk.Frame(self)
        grid.pack(fill="both", expand=True, padx=10, pady=6)
        
        self.lbl_status = ttk.Label(grid, text="Status: Disconnected")
        self.lbl_status.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))
        
        self.var_x = tk.StringVar(value="—")
        self.var_th = tk.StringVar(value="—")
        self.var_w = tk.StringVar(value="—")
        self.var_sp = tk.StringVar(value="0.00")
        self.var_raw = tk.StringVar(value="")
        self.var_log = tk.StringVar(value="Logging: OFF")
        
        ttk.Label(grid, text="Ball position x:").grid(row=1, column=0, sticky="e", padx=6, pady=6)
        ttk.Label(grid, textvariable=self.var_x, font=("Consolas", 14)).grid(row=1, column=1, sticky="w")
        
        ttk.Label(grid, text="Beam angle θ:").grid(row=2, column=0, sticky="e", padx=6, pady=6)
        ttk.Label(grid, textvariable=self.var_th, font=("Consolas", 14)).grid(row=2, column=1, sticky="w")
        
        ttk.Label(grid, text="Motor speed:").grid(row=3, column=0, sticky="e", padx=6, pady=6)
        ttk.Label(grid, textvariable=self.var_w, font=("Consolas", 14)).grid(row=3, column=1, sticky="w")
        
        ttk.Label(grid, text="Sent θ_sp:").grid(row=1, column=2, sticky="e", padx=6, pady=6)
        ttk.Label(grid, textvariable=self.var_sp, font=("Consolas", 14)).grid(row=1, column=3, sticky="w")
        
        ttk.Label(grid, text="Last raw line:").grid(row=4, column=0, sticky="ne", padx=6, pady=(10, 6))
        ttk.Label(grid, textvariable=self.var_raw, wraplength=760, justify="left").grid(row=4, column=1, columnspan=3, sticky="w", pady=(10, 6))
        
        ttk.Label(grid, textvariable=self.var_log, font=("", 9)).grid(row=5, column=0, columnspan=4, sticky="w", pady=(10, 0))
        
        # Control buttons
        btns = ttk.Frame(self)
        btns.pack(fill="x", padx=10, pady=8)
        
        self.btn_stop = tk.Button(
            btns,
            text="EMERGENCY STOP",
            command=self._on_stop,
            bg="red",
            fg="white",
            activebackground="#cc0000",
            activeforeground="white",
            font=("Helvetica", 14, "bold"),
            width=20,
            height=2,
            relief="raised",
            bd=4
        )
        self.btn_stop.pack(side="left", padx=(0, 10), pady=4)
        
        ttk.Button(btns, text="ARM / RUN", command=self._on_arm).pack(side="left", padx=8)
        ttk.Button(btns, text="RESET MCU", command=self._on_reset).pack(side="left", padx=8)
        
        # Timer display
        timer_frame = ttk.Frame(btns)
        timer_frame.pack(side="left", padx=20)
        ttk.Label(timer_frame, text="Time:", font=("", 9)).pack(side="left")
        self.var_timer = tk.StringVar(value="30.0s")
        self.lbl_timer = ttk.Label(timer_frame, textvariable=self.var_timer, font=("Consolas", 14, "bold"), foreground="#0066cc")
        self.lbl_timer.pack(side="left", padx=5)
        
        ttk.Button(btns, text="Start Log", command=self._on_start_log).pack(side="left", padx=(24, 6))
        ttk.Button(btns, text="Stop Log", command=self._on_stop_log).pack(side="left", padx=6)
        
        ttk.Label(btns, text="CSV format for paper analysis", font=("", 9)).pack(side="left", padx=10)
    
    def _update_ui_loop(self):
        """Update UI with latest state"""
        # Apply parameters
        try:
            self._apply_params()
        except Exception:
            pass
        
        # Update telemetry display
        tel = self.ctrl.latest_tel
        if tel is not None:
            self.var_x.set(f"{tel.x:+.4f}" if tel.x is not None else "—")
            
            if tel.theta is not None:
                th_deg = math.degrees(tel.theta)
                self.var_th.set(f"{th_deg:+.2f} deg")
            else:
                self.var_th.set("—")
            
            if tel.motor_speed is not None:
                w_deg = math.degrees(tel.motor_speed)
                self.var_w.set(f"{w_deg:+.1f} deg/s")
            else:
                self.var_w.set("—")
            
            self.var_sp.set(f"{self.ctrl.sp_sent_deg:+.2f} deg")
            self.var_raw.set(tel.raw)
        
        # Update status
        if not self.ctrl.ser:
            self.lbl_status.config(text="Status: Disconnected")
        else:
            st = "STOPPED" if self.ctrl.emergency_stop else ("ARMED" if self.ctrl.armed else "DISARMED")
            rl = "ON" if self.ctrl.enable_rl else "OFF"
            self.lbl_status.config(text=f"Status: Connected | RL={rl} | {st}")
        
        # Update logging status
        if self.ctrl.logger.enabled:
            self.var_log.set(f"Logging: ON | Samples: {self.ctrl.logger.sample_count} | File: {Path(self.ctrl.logger.file_path).name}")
        else:
            last = self.ctrl.logger.file_path
            self.var_log.set(f"Logging: OFF | Last: {Path(last).name}" if last else "Logging: OFF")
        
        # Update stop button
        if self.ctrl.emergency_stop:
            self.btn_stop.config(text="STOPPED", bg="#990000")
        else:
            self.btn_stop.config(text="EMERGENCY STOP", bg="red")
        
        # Update timer display - always update regardless of armed status
        try:
            time_left = self.ctrl.time_remaining
            self.var_timer.set(f"{time_left:.1f}s")
            
            # Change timer color based on status
            if time_left <= 5.0 and self.ctrl.armed:
                self.lbl_timer.config(foreground="red")
            elif self.ctrl.armed:
                self.lbl_timer.config(foreground="#28a745")
            else:
                self.lbl_timer.config(foreground="#0066cc")
        except Exception:
            # Fallback if timer fails
            self.var_timer.set("--")
        
        self.after(33, self._update_ui_loop)
    
    def _on_close(self):
        """Cleanup on window close"""
        try:
            self.ctrl.stop_now()
            self.ctrl.stop_logging()
            self.ctrl.stop_control_loop()
            self.ctrl.disconnect()
        except Exception:
            pass
        self.destroy()


class LogStartDialog(tk.Toplevel):
    """Dialog for starting a logging session"""
    
    def __init__(self, parent):
        super().__init__(parent)
        self.title("Start Logging")
        self.geometry("450x220")
        self.resizable(False, False)
        
        self.result = None
        
        # Run name
        ttk.Label(self, text="Run Name:").grid(row=0, column=0, sticky="e", padx=10, pady=10)
        self.run_name_var = tk.StringVar(value="trial_1")
        ttk.Entry(self, textvariable=self.run_name_var, width=30).grid(row=0, column=1, padx=10, pady=10)
        
        # Controller type
        ttk.Label(self, text="Controller:").grid(row=1, column=0, sticky="e", padx=10, pady=10)
        self.controller_var = tk.StringVar(value="RL")
        controller_frame = ttk.Frame(self)
        controller_frame.grid(row=1, column=1, sticky="w", padx=10, pady=10)
        ttk.Radiobutton(controller_frame, text="RL", variable=self.controller_var, value="RL").pack(side="left", padx=5)
        ttk.Radiobutton(controller_frame, text="PID", variable=self.controller_var, value="PID").pack(side="left", padx=5)
        ttk.Radiobutton(controller_frame, text="Other", variable=self.controller_var, value="Other").pack(side="left", padx=5)
        
        # Notes
        ttk.Label(self, text="Notes:").grid(row=2, column=0, sticky="ne", padx=10, pady=10)
        self.notes_text = tk.Text(self, width=30, height=4)
        self.notes_text.grid(row=2, column=1, padx=10, pady=10)
        
        # Buttons
        btn_frame = ttk.Frame(self)
        btn_frame.grid(row=3, column=0, columnspan=2, pady=10)
        ttk.Button(btn_frame, text="Start", command=self._on_start).pack(side="left", padx=10)
        ttk.Button(btn_frame, text="Cancel", command=self._on_cancel).pack(side="left", padx=10)
        
        # Center on parent
        self.transient(parent)
        self.grab_set()
    
    def _on_start(self):
        run_name = self.run_name_var.get().strip()
        if not run_name:
            messagebox.showwarning("Invalid Input", "Please enter a run name")
            return
        
        self.result = {
            'run_name': run_name,
            'controller': self.controller_var.get(),
            'notes': self.notes_text.get("1.0", "end-1c").strip()
        }
        self.destroy()
    
    def _on_cancel(self):
        self.result = None
        self.destroy()


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    app = App()
    app.mainloop()