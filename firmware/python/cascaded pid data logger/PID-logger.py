"""
PID Controller Data Logger for Ball-on-Beam Platform
CSV Logger matching RL format for easy comparison
PID runs entirely on MCU, this just logs the telemetry
"""

import time
import math
import threading
import queue
import re
import csv
from typing import Optional, Dict, List, Tuple
from pathlib import Path

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox, filedialog


# =============================================================================
# CONFIGURATION
# =============================================================================

class Config:
    """Configuration for PID logger"""
    
    # Serial communication
    DEFAULT_BAUD = 115200
    
    # Telemetry parsing
    # MCU sends: "D:%.3f,A:%.2f,W:%.3f\n"
    # D: distance (ball position, -1 to 1)
    # A: angle (degrees)
    # W: angular velocity (units unclear, likely rad/s)
    
    # Conversion factors
    DISTANCE_TO_METERS = 0.15  # -1 to 1 maps to -0.15m to +0.15m (beam half-length)
    ANGLE_IS_DEG = True
    RATE_IS_DEG_PER_S = True  # Update if W is in different units
    
    # CSV logging
    CSV_BUFFER_SIZE = 100  # Write every N samples
    
    # Trial timing - NEW!
    TRIAL_DURATION_S = 30.0  # Auto-stop after this duration
    START_POSITION_TARGET = 0.0  # Expected start position (0 for regulation, 0.10 for step)
    START_POSITION_TOLERANCE = 0.02  # Warn if ball not within ±2cm


# =============================================================================
# TELEMETRY PARSING
# =============================================================================

def parse_pid_telemetry(line: str) -> Optional[Dict[str, float]]:
    """
    Parse PID telemetry line.
    Format: "D:0.123,A:-2.40,W:0.55,ARM:1"
    Returns dict with keys 'd', 'a', 'w', 'arm' or None if parse fails.
    """
    try:
        kv = {}
        parts = re.split(r"[,\s]+", line.strip())
        for p in parts:
            if ":" in p:
                k, v = p.split(":", 1)
                k = k.strip().upper()
                v = v.strip()
                try:
                    kv[k] = float(v)
                except ValueError:
                    pass
        
        # Check for required fields (ARM is optional for backward compatibility)
        if 'D' in kv and 'A' in kv and 'W' in kv:
            return kv
        return None
    except Exception:
        return None


# =============================================================================
# CSV DATA LOGGER
# =============================================================================

class PIDCSVLogger:
    """
    CSV logger for PID controller data.
    Creates CSV files matching the RL logger format for easy comparison.
    
    CSV Columns:
    - time: Wall clock time (seconds since epoch)
    - time_rel: Relative time since start (seconds)
    - dt: Time step (seconds)
    - x: Ball position (meters)
    - xdot: Ball velocity (m/s) - computed from position
    - theta: Beam angle (radians)
    - theta_deg: Beam angle (degrees)
    - thdot: Beam angular velocity (rad/s)
    - error: Absolute position error |x| (meters)
    - error_sq: Squared position error (meters²)
    - d_raw: Raw distance value from MCU (-1 to 1)
    - a_raw: Raw angle value from MCU (degrees)
    - w_raw: Raw angular velocity from MCU
    """
    
    def __init__(self, base_dir: str = "runs", buffer_size: int = Config.CSV_BUFFER_SIZE):
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
        
        # For velocity estimation
        self._prev_x = None
        self._prev_t = None
    
    @property
    def enabled(self) -> bool:
        return self._enabled
    
    @property
    def file_path(self) -> str:
        return str(self._file_path) if self._file_path else ""
    
    @property
    def sample_count(self) -> int:
        return self._sample_count
    
    def start(self, run_name: str, notes: str = ""):
        """
        Start logging a new run.
        
        Args:
            run_name: Name for this run
            notes: Additional notes about the run
        """
        with self._lock:
            if self._enabled:
                self.stop()
            
            # Create directory and filename
            ts = time.strftime("%Y%m%d_%H%M%S")
            safe_name = "".join([c if c.isalnum() or c in "-_" else "_" for c in str(run_name)])
            self._file_path = self.base_dir / f"{ts}_PID_{safe_name}.csv"
            self._file_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Open CSV file
            self._csv_file = open(self._file_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            
            # Write header (matching RL logger format)
            self._csv_writer.writerow([
                'time', 'time_rel', 'dt',
                'x', 'xdot', 'theta', 'theta_deg', 'thdot',
                'error', 'error_sq',
                'd_raw', 'a_raw', 'w_raw'
            ])
            
            # Write metadata as comments
            self._csv_file.write(f"# Run: {run_name}\n")
            self._csv_file.write(f"# Controller: PID\n")
            self._csv_file.write(f"# Start time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            if notes:
                self._csv_file.write(f"# Notes: {notes}\n")
            self._csv_file.write(f"# Distance scale: {Config.DISTANCE_TO_METERS} m per unit\n")
            
            self._csv_file.flush()
            
            self._buffer = []
            self._t_start = time.time()
            self._sample_count = 0
            self._prev_x = None
            self._prev_t = None
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
            self._prev_x = None
            self._prev_t = None
    
    def add(self, t_wall: float, d_raw: float, a_raw: float, w_raw: float):
        """
        Add a data sample from PID telemetry.
        
        Args:
            t_wall: Wall clock timestamp
            d_raw: Distance from MCU (-1 to 1)
            a_raw: Angle from MCU (degrees)
            w_raw: Angular velocity from MCU
        """
        with self._lock:
            if not self._enabled:
                return
            
            # Convert to standard units
            x = d_raw * Config.DISTANCE_TO_METERS  # meters
            theta_deg = a_raw  # already in degrees
            theta = math.radians(theta_deg)  # radians
            
            # Convert angular velocity (assuming deg/s from MCU)
            if Config.RATE_IS_DEG_PER_S:
                thdot = math.radians(w_raw)
            else:
                thdot = w_raw
            
            # Estimate velocity from position derivative
            if self._prev_x is not None and self._prev_t is not None:
                dt = t_wall - self._prev_t
                if dt > 0:
                    xdot = (x - self._prev_x) / dt
                else:
                    xdot = 0.0
            else:
                xdot = 0.0
                dt = 0.01  # First sample, assume 100Hz
            
            # Update previous values
            self._prev_x = x
            self._prev_t = t_wall
            
            # Calculate relative time
            t_rel = t_wall - self._t_start if self._t_start else 0.0
            
            # Calculate error metrics
            error = abs(x)
            error_sq = x ** 2
            
            # Create row (matching RL logger format)
            row = [
                f"{t_wall:.6f}",
                f"{t_rel:.6f}",
                f"{dt:.6f}",
                f"{x:.6f}",
                f"{xdot:.6f}",
                f"{theta:.6f}",
                f"{theta_deg:.4f}",
                f"{thdot:.6f}",
                f"{error:.6f}",
                f"{error_sq:.8f}",
                f"{d_raw:.6f}",
                f"{a_raw:.4f}",
                f"{w_raw:.6f}"
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
            time.sleep(0.5)
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
                            pass
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


# =============================================================================
# PID LOGGER CONTROLLER
# =============================================================================

class PIDLoggerController:
    """Main controller for PID data logging"""
    
    def __init__(self):
        self.serial_comm = SerialComm()
        self.logger = PIDCSVLogger()
        
        # Latest telemetry for display
        self.latest_d = None
        self.latest_a = None
        self.latest_w = None
        self.latest_x = None
        self.latest_arm_status = 0
        self.latest_raw = ""
        
        # ARM/DISARM and timer - NEW!
        self.armed = False
        self.trial_start_time = None
        self.trial_duration = Config.TRIAL_DURATION_S
        
        # Processing loop
        self._loop_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
    
    @property
    def ser(self):
        """Expose serial connection for UI"""
        return self.serial_comm.ser
    
    @property
    def time_remaining(self) -> float:
        """Get remaining trial time in seconds"""
        if not self.armed or self.trial_start_time is None:
            return self.trial_duration
        elapsed = time.time() - self.trial_start_time
        return max(0.0, self.trial_duration - elapsed)
    
    def connect(self, port: str, baud: int = Config.DEFAULT_BAUD) -> bool:
        """Connect to hardware"""
        return self.serial_comm.connect(port, baud)
    
    def disconnect(self):
        """Disconnect from hardware"""
        self.disarm()  # Make sure to disarm on disconnect
        self.serial_comm.disconnect()
    
    def arm(self):
        """ARM the PID outer loop and start timer"""
        if not self.serial_comm.ser or not self.serial_comm.ser.is_open:
            return False
        
        # Send ARM command to MCU
        self.serial_comm.ser.write(b"ARM\n")
        self.armed = True
        self.trial_start_time = time.time()
        return True
    
    def disarm(self):
        """DISARM the PID outer loop"""
        if self.serial_comm.ser and self.serial_comm.ser.is_open:
            self.serial_comm.ser.write(b"DISARM\n")
        self.armed = False
        self.trial_start_time = None
    
    def start_logging(self, run_name: str, notes: str = ""):
        """Start data logging"""
        self.logger.start(run_name, notes)
    
    def stop_logging(self):
        """Stop data logging"""
        self.logger.stop()
    
    def start_processing_loop(self):
        """Start background processing loop"""
        if self._loop_thread and self._loop_thread.is_alive():
            return
        
        self._stop_event.clear()
        self._loop_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self._loop_thread.start()
    
    def stop_processing_loop(self):
        """Stop background processing loop"""
        self._stop_event.set()
        if self._loop_thread:
            self._loop_thread.join(timeout=2.0)
    
    def _processing_loop(self):
        """Main processing loop"""
        while not self._stop_event.is_set():
            # Get telemetry
            tel_data = self.serial_comm.get_telemetry()
            if tel_data:
                t_wall, line = tel_data
                self._process_telemetry(t_wall, line)
            
            # Check timer - auto-disarm after duration
            if self.armed and self.trial_start_time is not None:
                elapsed = time.time() - self.trial_start_time
                if elapsed >= self.trial_duration:
                    print(f"Trial duration reached ({self.trial_duration}s), auto-disarming...")
                    self.disarm()
                    self.stop_logging()
            
            time.sleep(0.001)
    
    def _process_telemetry(self, t_wall: float, line: str):
        """Process incoming telemetry line"""
        self.latest_raw = line
        
        kv = parse_pid_telemetry(line)
        if kv is None:
            return
        
        # Extract values
        d_raw = kv['D']
        a_raw = kv['A']
        w_raw = kv['W']
        arm_status = int(kv.get('ARM', 0))  # Default to 0 if not present
        
        # Store for display
        self.latest_d = d_raw
        self.latest_a = a_raw
        self.latest_w = w_raw
        self.latest_x = d_raw * Config.DISTANCE_TO_METERS
        self.latest_arm_status = arm_status
        
        # Log if enabled
        self.logger.add(t_wall, d_raw, a_raw, w_raw)


# =============================================================================
# GUI APPLICATION
# =============================================================================

class App(tk.Tk):
    """GUI application for PID data logging"""
    
    def __init__(self):
        super().__init__()
        
        self.title("Ball-on-Beam PID Logger - CSV Format")
        self.geometry("900x500")
        self.resizable(False, False)
        
        # Controller
        self.ctrl = PIDLoggerController()
        
        # Build UI
        self._build_ui()
        
        # Start processing loop
        self.ctrl.start_processing_loop()
        
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
    
    def _on_arm(self):
        """ARM the PID controller and start trial"""
        try:
            if not self.ctrl.ser:
                messagebox.showwarning("Not Connected", "Connect to hardware first")
                return
            
            # Check ball position if desired
            if self.ctrl.latest_x is not None:
                target = Config.START_POSITION_TARGET
                tolerance = Config.START_POSITION_TOLERANCE
                if abs(self.ctrl.latest_x - target) > tolerance:
                    result = messagebox.askyesno(
                        "Position Warning",
                        f"Ball is at {self.ctrl.latest_x*100:.1f}cm, expected {target*100:.1f}cm.\n"
                        f"ARM anyway?"
                    )
                    if not result:
                        return
            
            # Start logging and ARM
            dialog = LogStartDialog(self)
            self.wait_window(dialog)
            
            if dialog.result:
                run_name = dialog.result['run_name']
                notes = dialog.result['notes']
                self.ctrl.start_logging(run_name=run_name, notes=notes)
                self.ctrl.arm()
                print(f"ARMED - Trial will run for {Config.TRIAL_DURATION_S} seconds")
        except Exception as e:
            messagebox.showerror("Error", f"ARM error:\n{e}")
    
    def _on_disarm(self):
        """DISARM the PID controller"""
        try:
            self.ctrl.disarm()
            self.ctrl.stop_logging()
            print("DISARMED")
        except Exception as e:
            messagebox.showerror("Error", f"DISARM error:\n{e}")
    
    def _on_start_log(self):
        """Start logging"""
        dialog = LogStartDialog(self)
        self.wait_window(dialog)
        
        if dialog.result:
            try:
                run_name = dialog.result['run_name']
                notes = dialog.result['notes']
                self.ctrl.start_logging(run_name=run_name, notes=notes)
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
        # Header
        header = ttk.Frame(self)
        header.pack(fill="x", padx=10, pady=10)
        ttk.Label(header, text="PID Controller Data Logger", font=("", 14, "bold")).pack()
        ttk.Label(header, text="Logs PID telemetry in same CSV format as RL for comparison", 
                 font=("", 9)).pack()
        
        ttk.Separator(self).pack(fill="x", padx=10, pady=6)
        
        # Connection controls
        conn = ttk.Frame(self)
        conn.pack(fill="x", padx=10, pady=8)
        
        ttk.Label(conn, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value=self._default_port())
        self.port_box = ttk.Combobox(conn, textvariable=self.port_var, width=18, values=self._list_ports())
        self.port_box.pack(side="left", padx=6)
        
        ttk.Button(conn, text="Refresh", command=self._refresh_ports).pack(side="left", padx=4)
        
        ttk.Label(conn, text="Baud:").pack(side="left", padx=(12, 0))
        self.baud_var = tk.IntVar(value=Config.DEFAULT_BAUD)
        ttk.Entry(conn, textvariable=self.baud_var, width=8).pack(side="left", padx=6)
        
        self.btn_connect = ttk.Button(conn, text="Connect", command=self._on_connect)
        self.btn_connect.pack(side="left", padx=6)
        
        self.btn_disconnect = ttk.Button(conn, text="Disconnect", command=self._on_disconnect, state="disabled")
        self.btn_disconnect.pack(side="left", padx=6)
        
        ttk.Separator(self).pack(fill="x", padx=10, pady=6)
        
        # Telemetry display
        display = ttk.LabelFrame(self, text="Current Telemetry", padding=10)
        display.pack(fill="both", expand=True, padx=10, pady=6)
        
        self.var_status = tk.StringVar(value="Status: Disconnected")
        ttk.Label(display, textvariable=self.var_status, font=("", 10, "bold")).pack(pady=(0, 10))
        
        grid = ttk.Frame(display)
        grid.pack(fill="both", expand=True)
        
        self.var_x = tk.StringVar(value="—")
        self.var_d = tk.StringVar(value="—")
        self.var_a = tk.StringVar(value="—")
        self.var_w = tk.StringVar(value="—")
        self.var_raw = tk.StringVar(value="")
        
        ttk.Label(grid, text="Ball position x:").grid(row=0, column=0, sticky="e", padx=6, pady=8)
        ttk.Label(grid, textvariable=self.var_x, font=("Consolas", 14)).grid(row=0, column=1, sticky="w")
        ttk.Label(grid, text="(from D × 0.15m)", font=("", 8)).grid(row=0, column=2, sticky="w", padx=6)
        
        ttk.Label(grid, text="Distance D (raw):").grid(row=1, column=0, sticky="e", padx=6, pady=8)
        ttk.Label(grid, textvariable=self.var_d, font=("Consolas", 14)).grid(row=1, column=1, sticky="w")
        ttk.Label(grid, text="(-1 to +1)", font=("", 8)).grid(row=1, column=2, sticky="w", padx=6)
        
        ttk.Label(grid, text="Angle A:").grid(row=2, column=0, sticky="e", padx=6, pady=8)
        ttk.Label(grid, textvariable=self.var_a, font=("Consolas", 14)).grid(row=2, column=1, sticky="w")
        ttk.Label(grid, text="(degrees)", font=("", 8)).grid(row=2, column=2, sticky="w", padx=6)
        
        ttk.Label(grid, text="Angular velocity W:").grid(row=3, column=0, sticky="e", padx=6, pady=8)
        ttk.Label(grid, textvariable=self.var_w, font=("Consolas", 14)).grid(row=3, column=1, sticky="w")
        ttk.Label(grid, text="(deg/s assumed)", font=("", 8)).grid(row=3, column=2, sticky="w", padx=6)
        
        ttk.Label(grid, text="Last raw line:").grid(row=4, column=0, sticky="ne", padx=6, pady=(10, 6))
        ttk.Label(grid, textvariable=self.var_raw, wraplength=600, justify="left").grid(row=4, column=1, columnspan=2, sticky="w", pady=(10, 6))
        
        ttk.Separator(self).pack(fill="x", padx=10, pady=6)
        
        # ARM/DISARM and Timer controls - NEW!
        arm_frame = ttk.Frame(self)
        arm_frame.pack(fill="x", padx=10, pady=8)
        
        # Big ARM button (green)
        self.btn_arm = tk.Button(
            arm_frame,
            text="ARM / START TRIAL",
            command=self._on_arm,
            bg="#28a745",
            fg="white",
            activebackground="#218838",
            activeforeground="white",
            font=("Helvetica", 12, "bold"),
            width=18,
            height=2,
            relief="raised",
            bd=4
        )
        self.btn_arm.pack(side="left", padx=(0, 10), pady=4)
        
        # DISARM button (red)
        self.btn_disarm = tk.Button(
            arm_frame,
            text="DISARM / STOP",
            command=self._on_disarm,
            bg="#dc3545",
            fg="white",
            activebackground="#c82333",
            activeforeground="white",
            font=("Helvetica", 12, "bold"),
            width=15,
            height=2,
            relief="raised",
            bd=4
        )
        self.btn_disarm.pack(side="left", padx=10, pady=4)
        
        # Timer display
        timer_display = ttk.Frame(arm_frame)
        timer_display.pack(side="left", padx=20)
        ttk.Label(timer_display, text="Time Remaining:", font=("", 10)).pack()
        self.var_timer = tk.StringVar(value="30.0s")
        self.lbl_timer = ttk.Label(
            timer_display,
            textvariable=self.var_timer,
            font=("Consolas", 18, "bold"),
            foreground="#0066cc"
        )
        self.lbl_timer.pack()

        
        ttk.Separator(self).pack(fill="x", padx=10, pady=6)
        
        # Logging status
        log_frame = ttk.Frame(self)
        log_frame.pack(fill="x", padx=10, pady=8)
        
        self.var_log = tk.StringVar(value="Logging: OFF")
        ttk.Label(log_frame, textvariable=self.var_log, font=("", 9)).pack(side="left")
    
    def _update_ui_loop(self):
        """Update UI with latest state"""
        # Update telemetry display
        if self.ctrl.latest_x is not None:
            self.var_x.set(f"{self.ctrl.latest_x:+.4f} m")
        else:
            self.var_x.set("—")
        
        if self.ctrl.latest_d is not None:
            self.var_d.set(f"{self.ctrl.latest_d:+.3f}")
        else:
            self.var_d.set("—")
        
        if self.ctrl.latest_a is not None:
            self.var_a.set(f"{self.ctrl.latest_a:+.2f}°")
        else:
            self.var_a.set("—")
        
        if self.ctrl.latest_w is not None:
            self.var_w.set(f"{self.ctrl.latest_w:+.3f}")
        else:
            self.var_w.set("—")
        
        self.var_raw.set(self.ctrl.latest_raw)
        
        # Update status - NEW! Show ARM status
        if not self.ctrl.ser:
            self.var_status.set("Status: Disconnected")
        else:
            arm_text = "ARMED" if self.ctrl.armed else "DISARMED"
            mcu_arm = "MCU-ARMED" if self.ctrl.latest_arm_status else "MCU-DISARMED"
            self.var_status.set(f"Status: Connected | {arm_text} | {mcu_arm}")
        
        # Update timer - NEW!
        time_left = self.ctrl.time_remaining
        self.var_timer.set(f"{time_left:.1f}s")
        
        # Change timer color based on time
        if time_left <= 5.0 and self.ctrl.armed:
            self.lbl_timer.config(foreground="red")
        elif self.ctrl.armed:
            self.lbl_timer.config(foreground="#28a745")
        else:
            self.lbl_timer.config(foreground="#0066cc")
        
        # Update ARM button state
        if self.ctrl.armed:
            self.btn_arm.config(text="TRIAL RUNNING", bg="#ffc107", state="disabled")
        else:
            self.btn_arm.config(text="ARM / START TRIAL", bg="#28a745", state="normal")
        
        # Update logging status
        if self.ctrl.logger.enabled:
            self.var_log.set(f"Logging: ON | Samples: {self.ctrl.logger.sample_count} | File: {Path(self.ctrl.logger.file_path).name}")
        else:
            last = self.ctrl.logger.file_path
            self.var_log.set(f"Logging: OFF | Last: {Path(last).name}" if last else "Logging: OFF")
        
        self.after(33, self._update_ui_loop)
    
    def _on_close(self):
        """Cleanup on window close"""
        try:
            self.ctrl.stop_logging()
            self.ctrl.stop_processing_loop()
            self.ctrl.disconnect()
        except Exception:
            pass
        self.destroy()


class LogStartDialog(tk.Toplevel):
    """Dialog for starting a logging session"""
    
    def __init__(self, parent):
        super().__init__(parent)
        self.title("Start PID Logging")
        self.geometry("400x180")
        self.resizable(False, False)
        
        self.result = None
        
        # Run name
        ttk.Label(self, text="Run Name:").grid(row=0, column=0, sticky="e", padx=10, pady=10)
        self.run_name_var = tk.StringVar(value="trial_1")
        ttk.Entry(self, textvariable=self.run_name_var, width=30).grid(row=0, column=1, padx=10, pady=10)
        
        # Notes
        ttk.Label(self, text="Notes:").grid(row=1, column=0, sticky="ne", padx=10, pady=10)
        self.notes_text = tk.Text(self, width=30, height=4)
        self.notes_text.grid(row=1, column=1, padx=10, pady=10)
        
        # Buttons
        btn_frame = ttk.Frame(self)
        btn_frame.grid(row=2, column=0, columnspan=2, pady=10)
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