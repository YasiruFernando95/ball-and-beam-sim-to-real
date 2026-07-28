import time
import math
import threading
import re
import csv
from dataclasses import dataclass
from typing import Optional, Dict, List

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# -----------------------------
# Telemetry parsing (KV format)
# -----------------------------
def parse_kv_telemetry(line: str) -> Dict[str, float]:
    """
    Parses lines like:
      "X:0.0123,XD:-0.01,TH:2.50,THD:-12.0,W:123.4"
      "D:0.123,A:-2.40,W:0.55"
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


@dataclass
class Telemetry:
    t_wall: float
    x: Optional[float] = None
    xdot: Optional[float] = None
    th_deg: Optional[float] = None
    thdot_deg_s: Optional[float] = None
    w_deg_s: Optional[float] = None
    raw: str = ""


# -----------------------------
# Step-test state machine
# -----------------------------
@dataclass
class StepEvent:
    label: str          # e.g., "REST", "STEP+0.10"
    u_target: float
    duration_s: float


def build_step_sequence(
    amplitudes: List[float],
    step_hold_s: float,
    rest_s: float,
    include_zero_rest_before_each: bool = True
) -> List[StepEvent]:
    """
    Build symmetric sequence:
      [REST] -> +A hold -> 0 rest -> -A hold -> 0 rest -> next amplitude ...
    """
    seq: List[StepEvent] = []
    if rest_s > 0:
        seq.append(StepEvent("REST_START", 0.0, rest_s))

    for A in amplitudes:
        A = abs(float(A))
        if include_zero_rest_before_each and rest_s > 0:
            seq.append(StepEvent(f"REST_BEFORE_{A:.3f}", 0.0, rest_s))

        seq.append(StepEvent(f"STEP_POS_{A:.3f}", +A, step_hold_s))
        if rest_s > 0:
            seq.append(StepEvent(f"REST_AFTER_POS_{A:.3f}", 0.0, rest_s))

        seq.append(StepEvent(f"STEP_NEG_{A:.3f}", -A, step_hold_s))
        if rest_s > 0:
            seq.append(StepEvent(f"REST_AFTER_NEG_{A:.3f}", 0.0, rest_s))

    # Final rest to settle
    if rest_s > 0:
        seq.append(StepEvent("REST_END", 0.0, rest_s))

    return seq


# -----------------------------
# Controller (Serial + Safety + Logging + Test runner)
# -----------------------------
class MotorIDController:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.stop_event = threading.Event()

        self.rx_thread: Optional[threading.Thread] = None
        self.tx_thread: Optional[threading.Thread] = None

        self.latest_tel: Optional[Telemetry] = None
        self.last_rx_time = 0.0

        # Safety/config
        self.send_hz = 100
        self.timeout_s = 0.25
        self.u_clamp = 0.20
        self.slew_per_s = 0.50  # U per second

        # Command
        self.emergency_stop = True
        self.u_target = 0.0
        self.u_sent = 0.0

        # Test state
        self.test_active = False
        self.test_seq: List[StepEvent] = []
        self.test_idx = 0
        self.test_event_start = 0.0
        self.test_repeat = 1
        self.test_repeat_left = 0
        self.test_run_id = 0
        self.test_name = "UNNAMED"

        # Logging
        self.log_enabled = False
        self.log_path: Optional[str] = None
        self._log_fh = None
        self._log_writer = None
        self._log_t0 = None

    def list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port: str, baud: int):
        self.disconnect()

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1, write_timeout=0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.stop_event.clear()
        self.last_rx_time = time.time()

        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_thread.start()

        # Start safe
        self.stop_now()

    def disconnect(self):
        self.stop_now()
        self.stop_event.set()

        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.5)
        if self.tx_thread and self.tx_thread.is_alive():
            self.tx_thread.join(timeout=0.5)

        self.rx_thread = None
        self.tx_thread = None

        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

        self.latest_tel = None
        self.last_rx_time = 0.0

        self._stop_logging()

    def send_line(self, s: str):
        if not self.ser:
            return
        try:
            self.ser.write(s.encode("ascii"))
        except Exception:
            pass

    def stop_now(self):
        self.emergency_stop = True
        self.test_active = False
        self.u_target = 0.0
        self.u_sent = 0.0
        # send hard stop
        self.send_line("U 0\n")
        self.send_line("STOP\n")

    def arm(self):
        self.emergency_stop = False

    def set_target(self, u: float):
        u = float(u)
        u = max(-self.u_clamp, min(self.u_clamp, u))
        self.u_target = u

    # ---------- Logging ----------
    def start_logging(self, path: str, test_name: str):
        self.log_path = path
        self.test_name = test_name

        self._log_fh = open(path, "w", newline="", encoding="utf-8")
        self._log_writer = csv.writer(self._log_fh)

        # CSV header: include test metadata columns
        self._log_writer.writerow([
            "t_s",
            "test_name",
            "test_run_id",
            "test_active",
            "phase_label",
            "u_target",
            "u_sent",
            "x",
            "xdot",
            "th_deg",
            "thdot_deg_s",
            "w_deg_s",
            "raw"
        ])
        self._log_t0 = time.time()
        self.log_enabled = True

    def _stop_logging(self):
        self.log_enabled = False
        self._log_t0 = None
        if self._log_fh:
            try:
                self._log_fh.flush()
                self._log_fh.close()
            except Exception:
                pass
        self._log_fh = None
        self._log_writer = None

    def _log_row(self, tel: Optional[Telemetry], phase_label: str):
        if not self.log_enabled or not self._log_writer or self._log_t0 is None:
            return
        t_s = time.time() - self._log_t0

        def f(v):
            return "" if v is None else f"{v:.6f}"

        raw = "" if tel is None else tel.raw
        self._log_writer.writerow([
            f"{t_s:.6f}",
            self.test_name,
            str(self.test_run_id),
            "1" if self.test_active else "0",
            phase_label,
            f"{self.u_target:.6f}",
            f"{self.u_sent:.6f}",
            "" if tel is None else f(tel.x),
            "" if tel is None else f(tel.xdot),
            "" if tel is None else f(tel.th_deg),
            "" if tel is None else f(tel.thdot_deg_s),
            "" if tel is None else f(tel.w_deg_s),
            raw
        ])

        # flush ~1 Hz
        if int(t_s) != int(t_s - 0.02):
            try:
                self._log_fh.flush()
            except Exception:
                pass

    # ---------- Test Runner ----------
    def start_step_test(self, seq: List[StepEvent], repeats: int, test_run_id: int):
        if self.emergency_stop:
            raise RuntimeError("Cannot start test while STOPPED. Press ARM / RUN first.")
        if not self.ser:
            raise RuntimeError("Not connected.")
        if not seq:
            raise RuntimeError("Empty test sequence.")

        self.test_seq = seq
        self.test_idx = 0
        self.test_event_start = time.time()
        self.test_repeat = max(1, int(repeats))
        self.test_repeat_left = self.test_repeat
        self.test_active = True
        self.test_run_id = int(test_run_id)

        # Begin first event immediately
        self.set_target(self.test_seq[0].u_target)

    def abort_test(self):
        self.test_active = False
        self.set_target(0.0)

    def _advance_test_if_needed(self):
        if not self.test_active:
            return "IDLE"

        if self.test_idx >= len(self.test_seq):
            # finished one sequence
            self.test_repeat_left -= 1
            if self.test_repeat_left <= 0:
                # done all repeats
                self.test_active = False
                self.set_target(0.0)
                return "DONE"
            # repeat: restart sequence
            self.test_idx = 0
            self.test_event_start = time.time()
            self.set_target(self.test_seq[0].u_target)
            return self.test_seq[0].label

        ev = self.test_seq[self.test_idx]
        elapsed = time.time() - self.test_event_start
        if elapsed >= ev.duration_s:
            # move to next event
            self.test_idx += 1
            if self.test_idx >= len(self.test_seq):
                return "SEQ_END"
            self.test_event_start = time.time()
            ev = self.test_seq[self.test_idx]
            self.set_target(ev.u_target)
            return ev.label

        return ev.label

    # ---------- Threads ----------
    def _rx_loop(self):
        assert self.ser is not None
        buf = b""
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode("ascii", errors="ignore").strip()
                    if not s:
                        continue
                    tel = self._line_to_tel(s)
                    if tel:
                        self.last_rx_time = time.time()
                        self.latest_tel = tel
            except Exception:
                break

    def _line_to_tel(self, s: str) -> Optional[Telemetry]:
        kv = parse_kv_telemetry(s)
        if not kv:
            return None

        x = kv.get("X", kv.get("D", None))
        xdot = kv.get("XD", None)

        th = kv.get("TH", kv.get("A", None))
        thdot = kv.get("THD", None)

        w = kv.get("W", None)

        return Telemetry(
            t_wall=time.time(),
            x=x,
            xdot=xdot,
            th_deg=None if th is None else float(th),
            thdot_deg_s=None if thdot is None else float(thdot),
            w_deg_s=None if w is None else float(w),
            raw=s
        )

    def _tx_loop(self):
        period = 1.0 / max(1.0, float(self.send_hz))
        next_t = time.time()
        last_t = time.time()

        # For logging phase labels even if telemetry is missing:
        last_phase_label = "IDLE"

        while not self.stop_event.is_set():
            now = time.time()
            if now < next_t:
                time.sleep(min(0.002, next_t - now))
                continue
            next_t += period

            if not self.ser:
                continue

            # Telemetry timeout safety
            if (time.time() - self.last_rx_time) > self.timeout_s:
                self.u_sent = 0.0
                self.send_line("U 0\n")
                # still log with last known phase
                self._log_row(self.latest_tel, "TEL_TIMEOUT")
                continue

            # Emergency stop safety
            if self.emergency_stop:
                self.u_sent = 0.0
                self.send_line("U 0\n")
                self._log_row(self.latest_tel, "STOPPED")
                continue

            # Update test state machine
            phase_label = self._advance_test_if_needed()
            if phase_label not in ("IDLE", "DONE", "SEQ_END"):
                last_phase_label = phase_label
            elif self.test_active:
                # if transient label, keep last meaningful
                phase_label = last_phase_label

            # Slew-rate limit u_sent -> u_target
            dt = max(1e-6, now - last_t)
            last_t = now
            max_step = self.slew_per_s * dt

            err = self.u_target - self.u_sent
            if err > max_step:
                self.u_sent += max_step
            elif err < -max_step:
                self.u_sent -= max_step
            else:
                self.u_sent = self.u_target

            # Final clamp
            self.u_sent = max(-self.u_clamp, min(self.u_clamp, self.u_sent))

            # Send command
            self.send_line(f"U {self.u_sent:.4f}\n")

            # Log at send rate (clean for system ID)
            self._log_row(self.latest_tel, phase_label if self.test_active else "MANUAL")


# -----------------------------
# GUI
# -----------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Motor ID Step-Test Logger (No-Load / Beam-Load)")
        self.geometry("980x640")

        self.ctrl = MotorIDController()

        self._build_ui()
        self._ui_loop()

    def _build_ui(self):
        # --- Connection row ---
        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=8)

        ttk.Label(top, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value="")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=18, values=self.ctrl.list_ports())
        self.port_box.pack(side="left", padx=6)

        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left", padx=4)

        ttk.Label(top, text="Baud:").pack(side="left", padx=(12, 0))
        self.baud_var = tk.IntVar(value=115200)
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side="left", padx=6)

        self.btn_connect = ttk.Button(top, text="Connect", command=self._on_connect)
        self.btn_connect.pack(side="left", padx=6)

        self.btn_disconnect = ttk.Button(top, text="Disconnect", command=self._on_disconnect, state="disabled")
        self.btn_disconnect.pack(side="left", padx=6)

        ttk.Separator(self).pack(fill="x", padx=10, pady=6)

        # --- Safety/config ---
        cfg = ttk.LabelFrame(self, text="Safety / Send Config")
        cfg.pack(fill="x", padx=10, pady=6)

        row1 = ttk.Frame(cfg)
        row1.pack(fill="x", padx=10, pady=6)

        self.sendhz_var = tk.IntVar(value=100)
        self.timeout_var = tk.DoubleVar(value=0.25)
        self.clamp_var = tk.DoubleVar(value=0.20)
        self.slew_var = tk.DoubleVar(value=0.50)

        ttk.Label(row1, text="Send Hz:").pack(side="left")
        ttk.Entry(row1, textvariable=self.sendhz_var, width=7).pack(side="left", padx=6)

        ttk.Label(row1, text="Timeout (s):").pack(side="left", padx=(12, 0))
        ttk.Entry(row1, textvariable=self.timeout_var, width=7).pack(side="left", padx=6)

        ttk.Label(row1, text="U clamp (±):").pack(side="left", padx=(12, 0))
        ttk.Entry(row1, textvariable=self.clamp_var, width=7).pack(side="left", padx=6)

        ttk.Label(row1, text="Slew (U/s):").pack(side="left", padx=(12, 0))
        ttk.Entry(row1, textvariable=self.slew_var, width=7).pack(side="left", padx=6)

        row2 = ttk.Frame(cfg)
        row2.pack(fill="x", padx=10, pady=(0, 10))

        self.btn_arm = ttk.Button(row2, text="ARM / RUN", command=self._on_arm)
        self.btn_arm.pack(side="left")

        self.btn_stop = ttk.Button(row2, text="EMERGENCY STOP", command=self._on_stop)
        self.btn_stop.pack(side="left", padx=10)

        ttk.Button(row2, text="Manual Center (U=0)", command=self._center_manual).pack(side="left", padx=10)

        self.status_var = tk.StringVar(value="Disconnected | STOPPED")
        ttk.Label(row2, textvariable=self.status_var).pack(side="right")

        # --- Manual slider ---
        man = ttk.LabelFrame(self, text="Manual Command (for quick checks)")
        man.pack(fill="x", padx=10, pady=6)

        self.u_slider_var = tk.DoubleVar(value=0.0)
        self.u_slider = ttk.Scale(man, from_=-0.20, to=+0.20, orient="horizontal",
                                  variable=self.u_slider_var, command=self._on_slider)
        self.u_slider.pack(fill="x", padx=10, pady=10)

        self.lbl_u = ttk.Label(man, text="U target: +0.0000   |   U sent: +0.0000", font=("Consolas", 14))
        self.lbl_u.pack(padx=10, pady=(0, 10))

        # --- Test config ---
        test = ttk.LabelFrame(self, text="Step-Test (clean data, both directions)")
        test.pack(fill="x", padx=10, pady=6)

        rowt1 = ttk.Frame(test)
        rowt1.pack(fill="x", padx=10, pady=6)

        self.amps_var = tk.StringVar(value="0.10,0.15,0.20")
        self.hold_var = tk.DoubleVar(value=2.0)
        self.rest_var = tk.DoubleVar(value=2.0)
        self.reps_var = tk.IntVar(value=2)
        self.runid_var = tk.IntVar(value=1)

        ttk.Label(rowt1, text="Amplitudes (comma):").pack(side="left")
        ttk.Entry(rowt1, textvariable=self.amps_var, width=22).pack(side="left", padx=6)

        ttk.Label(rowt1, text="Hold (s):").pack(side="left", padx=(12, 0))
        ttk.Entry(rowt1, textvariable=self.hold_var, width=7).pack(side="left", padx=6)

        ttk.Label(rowt1, text="Rest (s):").pack(side="left", padx=(12, 0))
        ttk.Entry(rowt1, textvariable=self.rest_var, width=7).pack(side="left", padx=6)

        ttk.Label(rowt1, text="Repeats:").pack(side="left", padx=(12, 0))
        ttk.Entry(rowt1, textvariable=self.reps_var, width=6).pack(side="left", padx=6)

        ttk.Label(rowt1, text="Run ID:").pack(side="left", padx=(12, 0))
        ttk.Entry(rowt1, textvariable=self.runid_var, width=6).pack(side="left", padx=6)

        rowt2 = ttk.Frame(test)
        rowt2.pack(fill="x", padx=10, pady=(0, 10))

        self.testname_var = tk.StringVar(value="NO_LOAD" )
        ttk.Label(rowt2, text="Test name (e.g., NO_LOAD / BEAM_LOAD):").pack(side="left")
        ttk.Entry(rowt2, textvariable=self.testname_var, width=18).pack(side="left", padx=6)

        self.btn_start_test = ttk.Button(rowt2, text="Start Step-Test", command=self._start_test)
        self.btn_start_test.pack(side="left", padx=10)

        self.btn_abort_test = ttk.Button(rowt2, text="Abort Test", command=self._abort_test)
        self.btn_abort_test.pack(side="left")

        # --- Logger ---
        log = ttk.LabelFrame(self, text="CSV Logger")
        log.pack(fill="x", padx=10, pady=6)

        self.csv_path_var = tk.StringVar(value="")
        ttk.Entry(log, textvariable=self.csv_path_var, width=80).pack(side="left", padx=10, pady=10)
        ttk.Button(log, text="Choose CSV...", command=self._choose_csv).pack(side="left", padx=6)

        self.log_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(log, text="Enable logging", variable=self.log_var, command=self._toggle_logging)\
            .pack(side="left", padx=10)

        # --- Telemetry panel ---
        tel = ttk.LabelFrame(self, text="Telemetry")
        tel.pack(fill="both", expand=True, padx=10, pady=6)

        self.var_w = tk.StringVar(value="—")
        self.var_th = tk.StringVar(value="—")
        self.var_x = tk.StringVar(value="—")
        self.var_raw = tk.StringVar(value="")

        g = ttk.Frame(tel)
        g.pack(fill="x", padx=10, pady=8)

        ttk.Label(g, text="W (deg/s):").grid(row=0, column=0, sticky="e", padx=6, pady=4)
        ttk.Label(g, textvariable=self.var_w, font=("Consolas", 14)).grid(row=0, column=1, sticky="w")

        ttk.Label(g, text="TH (deg):").grid(row=1, column=0, sticky="e", padx=6, pady=4)
        ttk.Label(g, textvariable=self.var_th, font=("Consolas", 14)).grid(row=1, column=1, sticky="w")

        ttk.Label(g, text="X:").grid(row=2, column=0, sticky="e", padx=6, pady=4)
        ttk.Label(g, textvariable=self.var_x, font=("Consolas", 14)).grid(row=2, column=1, sticky="w")

        ttk.Label(tel, text="Raw line:").pack(anchor="w", padx=10)
        ttk.Label(tel, textvariable=self.var_raw, wraplength=940, justify="left")\
            .pack(fill="x", padx=10, pady=(0, 10))

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- UI handlers ----------
    def _refresh_ports(self):
        ports = self.ctrl.list_ports()
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _apply_cfg(self) -> bool:
        try:
            self.ctrl.send_hz = int(self.sendhz_var.get())
            self.ctrl.timeout_s = float(self.timeout_var.get())
            self.ctrl.u_clamp = float(self.clamp_var.get())
            self.ctrl.slew_per_s = float(self.slew_var.get())
        except Exception:
            messagebox.showwarning("Config", "Invalid config values.")
            return False

        # Update manual slider range
        self.u_slider.configure(from_=-self.ctrl.u_clamp, to=+self.ctrl.u_clamp)
        # Re-clamp target
        self.ctrl.set_target(self.u_slider_var.get())
        return True

    def _on_connect(self):
        if not self._apply_cfg():
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Port", "Select a COM port.")
            return
        try:
            self.ctrl.connect(port, int(self.baud_var.get()))
            self.btn_connect.config(state="disabled")
            self.btn_disconnect.config(state="normal")
            self.status_var.set(f"Connected to {port} | STOPPED")
            self._center_manual()
        except Exception as e:
            messagebox.showerror("Connect failed", str(e))

    def _on_disconnect(self):
        self.ctrl.disconnect()
        self.btn_connect.config(state="normal")
        self.btn_disconnect.config(state="disabled")
        self.status_var.set("Disconnected | STOPPED")
        self.log_var.set(False)

    def _on_arm(self):
        if not self._apply_cfg():
            return
        self.ctrl.arm()

    def _on_stop(self):
        self.ctrl.stop_now()
        self._center_manual()

    def _center_manual(self):
        self.u_slider_var.set(0.0)
        self.ctrl.set_target(0.0)

    def _on_slider(self, _evt=None):
        # manual changes should abort test to avoid mixing datasets
        if self.ctrl.test_active:
            self.ctrl.abort_test()
        self.ctrl.set_target(self.u_slider_var.get())

    def _choose_csv(self):
        path = filedialog.asksaveasfilename(
            title="Save CSV",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")]
        )
        if path:
            self.csv_path_var.set(path)

    def _toggle_logging(self):
        want = bool(self.log_var.get())
        if want:
            if not self.ctrl.ser:
                messagebox.showwarning("Logger", "Connect first.")
                self.log_var.set(False)
                return
            path = self.csv_path_var.get().strip()
            if not path:
                messagebox.showwarning("Logger", "Choose a CSV path first.")
                self.log_var.set(False)
                return
            try:
                self.ctrl.start_logging(path, self.testname_var.get().strip() or "UNNAMED")
            except Exception as e:
                messagebox.showerror("Logger", str(e))
                self.log_var.set(False)
        else:
            self.ctrl._stop_logging()

    def _parse_amps(self) -> List[float]:
        s = self.amps_var.get().strip()
        amps = []
        for part in s.split(","):
            part = part.strip()
            if not part:
                continue
            amps.append(abs(float(part)))
        # clamp amplitudes to current clamp
        amps = [min(a, self.ctrl.u_clamp) for a in amps]
        # remove duplicates while keeping order
        out = []
        for a in amps:
            if a not in out:
                out.append(a)
        return out

    def _start_test(self):
        if not self._apply_cfg():
            return
        if not self.ctrl.ser:
            messagebox.showwarning("Test", "Connect first.")
            return
        if self.ctrl.emergency_stop:
            messagebox.showwarning("Test", "Press ARM / RUN first (then start test).")
            return
        if not self.ctrl.log_enabled:
            messagebox.showwarning("Test", "Enable logging first (CSV).")
            return

        try:
            amps = self._parse_amps()
            hold_s = float(self.hold_var.get())
            rest_s = float(self.rest_var.get())
            reps = int(self.reps_var.get())
            run_id = int(self.runid_var.get())
        except Exception:
            messagebox.showwarning("Test", "Invalid test parameters.")
            return

        if hold_s <= 0.0:
            messagebox.showwarning("Test", "Hold time must be > 0.")
            return

        seq = build_step_sequence(amps, step_hold_s=hold_s, rest_s=rest_s)
        try:
            # update test name in logger metadata
            self.ctrl.test_name = self.testname_var.get().strip() or "UNNAMED"
            self.ctrl.start_step_test(seq, repeats=reps, test_run_id=run_id)
        except Exception as e:
            messagebox.showerror("Test start failed", str(e))

    def _abort_test(self):
        self.ctrl.abort_test()
        self._center_manual()

    def _ui_loop(self):
        # status line
        if self.ctrl.ser:
            state = "STOPPED" if self.ctrl.emergency_stop else ("TESTING" if self.ctrl.test_active else "RUNNING")
            self.status_var.set(f"Connected | {state}")
        else:
            self.status_var.set("Disconnected | STOPPED")

        self.lbl_u.config(text=f"U target: {self.ctrl.u_target:+.4f}   |   U sent: {self.ctrl.u_sent:+.4f}")

        tel = self.ctrl.latest_tel
        if tel:
            self.var_w.set("—" if tel.w_deg_s is None else f"{tel.w_deg_s:+.1f}")
            self.var_th.set("—" if tel.th_deg is None else f"{tel.th_deg:+.2f}")
            self.var_x.set("—" if tel.x is None else f"{tel.x:+.4f}")
            self.var_raw.set(tel.raw)

        self.after(33, self._ui_loop)

    def _on_close(self):
        try:
            self.ctrl.stop_now()
            self.ctrl.disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()
