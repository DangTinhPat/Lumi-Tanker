"""Small Tk dashboard for the native MuJoCo process."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk

from .native_backend import NativeBackend

BG = "#16181d"
CARD = "#22252e"
FG = "#e8eaf0"
MUTED = "#98a0b0"
GOOD = "#3ecf8e"
WARN = "#ffb454"
BAD = "#ff5c68"
NATURAL_BODY_HEIGHT_M = 0.1484350374792
# These bounds come from the native FK envelope (knee -1.2..0.458407 rad)
# with the axle fore/aft coordinate held at natural ENABLE.  They are not a
# claim that the STEP contains calibrated physical stop phasing.
# Keep a small clearance margin inside the measured FK envelope. The native
# runner still rejects any target that cannot be reached from the live encoder
# seed, and reports the rejection through telemetry.
MIN_BODY_HEIGHT_M = 0.140
# The 0.200 m target is the best short native trial so far (not a long-duration
# station-hold certification). The lower slider bound remains the FK/CAD
# envelope for inspection; the native core still performs the live feasibility
# and capture checks before starting the path.
BEST_TESTED_BODY_HEIGHT_M = 0.200
# The high endpoint is the native FK/CAD envelope, not a knee-stop command.
MAX_BODY_HEIGHT_M = 0.245


def _posture_to_height(position: float) -> float:
    """Map one operator posture axis around natural ENABLE to FK height."""
    position = max(-1.0, min(1.0, float(position)))
    if position >= 0.0:
        return NATURAL_BODY_HEIGHT_M - position * (NATURAL_BODY_HEIGHT_M - MIN_BODY_HEIGHT_M)
    return NATURAL_BODY_HEIGHT_M + (-position) * (MAX_BODY_HEIGHT_M - NATURAL_BODY_HEIGHT_M)


def _height_to_posture(height_m: float) -> float:
    height_m = max(MIN_BODY_HEIGHT_M, min(MAX_BODY_HEIGHT_M, float(height_m)))
    if height_m <= NATURAL_BODY_HEIGHT_M:
        return ((NATURAL_BODY_HEIGHT_M - height_m) /
                (NATURAL_BODY_HEIGHT_M - MIN_BODY_HEIGHT_M))
    return -((height_m - NATURAL_BODY_HEIGHT_M) /
              (MAX_BODY_HEIGHT_M - NATURAL_BODY_HEIGHT_M))


def _sample_curve(curves, mode, amount):
    table = curves["tables"][mode]
    x = max(0.0, min(1.0, float(amount))) * (len(table) - 1)
    index = min(int(x), len(table) - 2)
    fraction = x - index
    return tuple((1.0 - fraction) * table[index][i] +
                 fraction * table[index + 1][i] for i in range(4))


def _smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


class NativeDashboard:
    def __init__(self, root: tk.Tk, backend: NativeBackend):
        self.root = root
        self.backend = backend
        self.enabled = False
        self.estop = False
        self.drive = (0.0, 0.0)
        self._drive_button_active = False
        self.dance_active = False
        self.show_started_s = None
        self._posture_send_job = None
        self._posture_drag_active = False
        self._posture_last_sent_height = None
        self._posture_send_period_ms = 120
        self._posture_quiet_since = None
        self._show_waiting = False
        self._status_window = None
        self._log_window = None
        self.telemetry = tk.StringVar(value="No telemetry")
        self._last_log_text = ""
        self.logbox = None
        self.curves = self._load_pose_curves()
        self.root.title("TAP · native MuJoCo")
        self.root.configure(bg=BG)
        self.root.geometry("900x520")
        self.root.minsize(760, 460)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._style()
        self._build()
        self._bind_keys()
        self.tick()

    def _style(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=FG)
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background=CARD)
        style.configure("TLabel", background=BG, foreground=FG)
        style.configure("Card.TLabel", background=CARD, foreground=FG)
        style.configure("TButton", padding=(5, 3))
        style.configure("Card.TLabelframe", background=CARD)
        style.configure("Card.TLabelframe.Label", background=CARD, foreground=FG)

    def _build(self):
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer)
        header.pack(fill="x")
        ttk.Label(header, text="TAP robot · native MuJoCo", font=("", 16, "bold")).pack(side="left")
        self.sim_button = ttk.Button(header, text="Start SIM", command=self.toggle)
        self.sim_button.pack(side="right", padx=(8, 0))
        self.mode = tk.Label(header, text=" offline ", bg="#333", fg=FG, padx=12, pady=4)
        self.mode.pack(side="right")
        toolbar = ttk.Frame(outer, style="Card.TFrame", padding=10)
        toolbar.pack(fill="x", pady=(10, 8))
        toolbar.columnconfigure(0, weight=1)
        toolbar.columnconfigure(1, weight=2)

        drive = ttk.LabelFrame(toolbar, text="Điều hướng", style="Card.TLabelframe", padding=10)
        drive.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        # The native core still owns acceleration and torque limits, so this
        # only picks how much speed to ask for. It is adjustable because the
        # interesting behaviour - how level the body stays while accelerating,
        # and how hard a reversal is - only shows up well above the old fixed
        # 0.10 m/s. The top of the range is the runner's own DRIVE clamp.
        self.speed = tk.DoubleVar(value=0.30)
        self.yaw = tk.DoubleVar(value=0.35)
        self.speed_readout = tk.StringVar(value="0.30 m/s")
        pad = ttk.Frame(drive, style="Card.TFrame")
        pad.pack(pady=4)
        buttons = (("▲", 0, 1, 1, 0), ("◄", 1, 0, 0, 1),
                   ("►", 1, 2, 0, -1), ("▼", 2, 1, -1, 0))
        for text, row, col, forward, yaw in buttons:
            button = ttk.Button(pad, text=text, width=4)
            button.grid(row=row, column=col, padx=2, pady=2)
            button.bind(
                "<ButtonPress-1>",
                lambda _event, f=forward, y=yaw: self._drive_press(f, y),
            )
            button.bind("<ButtonRelease-1>", self._drive_release)
        # Catch a release outside the button so a lost pointer-up cannot leave
        # the simulated robot driving indefinitely.
        self.root.bind("<ButtonRelease-1>", self._drive_release, add="+")
        speed_row = ttk.Frame(drive, style="Card.TFrame")
        speed_row.pack(fill="x", pady=(6, 0))
        ttk.Label(speed_row, textvariable=self.speed_readout,
                  style="Card.TLabel").pack(anchor="w")
        tk.Scale(speed_row, from_=0.05, to=1.00, resolution=0.05,
                 orient="horizontal", showvalue=False, variable=self.speed,
                 command=self._speed_preview, bg=CARD, fg=FG,
                 troughcolor="#3a4050", highlightthickness=0, bd=0,
                 length=150).pack(fill="x")
        ttk.Button(drive, text="Reset mô phỏng", command=self.reset).pack(
            fill="x", pady=(4, 0))

        posture = ttk.LabelFrame(toolbar, text="Chiều cao thân", style="Card.TLabelframe", padding=10)
        posture.grid(row=0, column=1, sticky="nsew")
        ttk.Label(posture, text="−1 extension · 0 natural · +1 squat",
                  style="Card.TLabel").pack(anchor="w")
        self.posture_var = tk.DoubleVar(value=0.0)
        self.height_var = tk.DoubleVar(value=NATURAL_BODY_HEIGHT_M)
        self.height_readout = tk.StringVar(
            value=f"Natural ENABLE · {NATURAL_BODY_HEIGHT_M:.3f} m")
        self.posture_scale = tk.Scale(
            posture, from_=1.0, to=-1.0, resolution=0.01,
            orient="horizontal", showvalue=False,
            variable=self.posture_var, command=self._posture_preview,
            bg=CARD, fg=FG, troughcolor="#3a4050", highlightthickness=0,
            bd=0, length=260)
        self.posture_scale.pack(fill="x", pady=1)
        # The preview is cheap, but HEIGHT is throttled to a bounded UI rate.
        # The native core owns the kHz control loop and retargets the active
        # task-space path from its last emitted pose, so the slider can follow
        # the operator without restarting a move on every Tk event.
        self.posture_scale.bind("<ButtonPress-1>", self._posture_press)
        self.posture_scale.bind("<ButtonRelease-1>", self._posture_release)
        ttk.Label(posture, textvariable=self.height_readout,
                  style="Card.TLabel").pack(anchor="w")
        height_presets = ttk.Frame(posture, style="Card.TFrame")
        height_presets.pack(fill="x")
        for index, (label, height) in enumerate((
                ("Natural", NATURAL_BODY_HEIGHT_M),
                ("Max standup", MAX_BODY_HEIGHT_M),
                ("Deep squat", MIN_BODY_HEIGHT_M))):
            ttk.Button(height_presets, text=label,
                       command=lambda h=height: self.send_height(h)).grid(
                           row=0, column=index,
                           sticky="ew", padx=1, pady=1)
        for column in range(3):
            height_presets.columnconfigure(column, weight=1)

        skill = ttk.LabelFrame(toolbar, text="Skill", style="Card.TLabelframe", padding=6)
        skill.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Label(skill, text="Gimbal VMC · xoay tại chỗ + đổi chiều cao theo nhịp",
                  style="Card.TLabel").pack(side="left", padx=(2, 10))
        ttk.Button(skill, text="Múa ↻",
                   command=lambda: self.start_dance(1.0)).pack(side="left", padx=2)
        ttk.Button(skill, text="Múa ↺",
                   command=lambda: self.start_dance(-1.0)).pack(side="left", padx=2)
        ttk.Button(skill, text="Dừng skill",
                   command=self.stop_skill).pack(side="left", padx=2)

        body = ttk.Frame(outer, style="Card.TFrame", padding=18)
        body.pack(fill="both", expand=True)
        self.viewer_status = tk.StringVar(value="Đang khởi động native MuJoCo viewer…")
        ttk.Label(
            body,
            text="MuJoCo 3-D viewer mở trong cửa sổ riêng.\n"
                 "Dùng phím mũi tên hoặc các nút điều hướng; slider và preset điều khiển chiều cao.",
            style="Card.TLabel",
            justify="center",
        ).pack(expand=True)

    def _open_status_window(self):
        if self._status_window is not None and self._status_window.winfo_exists():
            self._status_window.deiconify()
            self._status_window.lift()
            return

        window = tk.Toplevel(self.root)
        self._status_window = window
        window.title("TAP · Trạng thái")
        window.geometry("700x360")
        window.minsize(520, 240)
        window.configure(bg=BG)
        window.transient(self.root)
        card = ttk.Frame(window, style="Card.TFrame", padding=14)
        card.pack(fill="both", expand=True)
        ttk.Label(
            card,
            textvariable=self.viewer_status,
            style="Card.TLabel",
            font=("", 14, "bold"),
        ).pack(anchor="w", fill="x")
        ttk.Label(
            card,
            textvariable=self.telemetry,
            style="Card.TLabel",
            justify="left",
        ).pack(anchor="nw", fill="both", expand=True, pady=(14, 0))

        def close():
            self._status_window = None
            window.destroy()

        window.protocol("WM_DELETE_WINDOW", close)

    def _open_log_window(self):
        if self._log_window is not None and self._log_window.winfo_exists():
            self._log_window.deiconify()
            self._log_window.lift()
            return

        window = tk.Toplevel(self.root)
        self._log_window = window
        window.title("TAP · Log")
        window.geometry("800x460")
        window.minsize(560, 280)
        window.configure(bg=BG)
        window.transient(self.root)

        frame = ttk.Frame(window, style="Card.TFrame", padding=10)
        frame.pack(fill="both", expand=True)
        scrollbar = ttk.Scrollbar(frame, orient="vertical")
        scrollbar.pack(side="right", fill="y")
        self.logbox = tk.Text(
            frame,
            bg=CARD,
            fg=MUTED,
            relief="flat",
            wrap="none",
            state="disabled",
            yscrollcommand=scrollbar.set,
        )
        self.logbox.pack(side="left", fill="both", expand=True)
        scrollbar.configure(command=self.logbox.yview)
        self._last_log_text = ""
        self._refresh_logbox()

        def close():
            self._log_window = None
            self.logbox = None
            window.destroy()

        window.protocol("WM_DELETE_WINDOW", close)

    def _refresh_logbox(self):
        if self.logbox is None:
            return
        log_text = "\n".join(self.backend.log)
        if log_text == self._last_log_text:
            return
        self.logbox.configure(state="normal")
        self.logbox.delete("1.0", "end")
        if log_text:
            self.logbox.insert("end", log_text + "\n")
        self.logbox.configure(state="disabled")
        self.logbox.see("end")
        self._last_log_text = log_text

    def _load_pose_curves(self):
        path = Path(__file__).resolve().parents[1] / "config" / "pose_curves.json"
        try:
            with path.open(encoding="utf-8") as stream:
                curves = json.load(stream)
            if not all(mode in curves["tables"] for mode in ("squat", "roll_left", "roll_right")):
                raise ValueError("pose curve modes missing")
            return curves
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"cannot load CAD pose curves: {path}") from exc

    def _natural_enable_pose(self):
        """Return the mechanical zero used when ENABLE is first acquired."""
        return self.curves.get("natural_enable_pose", self.curves["rest_pose"])

    def _bind_keys(self):
        keys = {"Up": (1.0, 0.0), "Down": (-1.0, 0.0), "Left": (0.0, 1.0), "Right": (0.0, -1.0)}
        for key, (forward, yaw) in keys.items():
            self.root.bind(f"<KeyPress-{key}>", lambda _, f=forward, y=yaw: self.set_drive(f, y))
            self.root.bind(f"<KeyRelease-{key}>", lambda _: self.stop_drive())
        self.root.bind("<Escape>", lambda _: self.emergency_stop())

    def toggle(self):
        try:
            if self.backend.running:
                self.stop_skill()
                self.backend.stop()
                self.enabled = False
            else:
                # A new runner is a new simulation session; an old GUI-side
                # estop flag must not shadow its fresh startup state.
                self.estop = False
                self._posture_last_sent_height = None
                self.backend.start()
                # Launching the operator simulation means "spawn and stand".
                # The runner still waits for its delayed/noisy IMU seed before
                # the core can enter Enabling/Balance; this is not a bypass of
                # the embedded startup qualification.
                self.enable()
        except Exception as exc:
            # Operator actions are non-modal.  Keep failures in the optional
            # log/status views so a dialog cannot interrupt a control gesture.
            self.backend.log.append(f"ERROR|dashboard={exc}")
            self.viewer_status.set(f"Lỗi native runner: {exc}")

    def _cancel_pending_posture(self):
        self._posture_quiet_since = None
        self._show_waiting = False

    def enable(self):
        if self.backend.running and not self.estop:
            self.backend.send("ENABLE")
            self.enabled = True

    def disable(self):
        self._cancel_pending_posture()
        self.stop_skill()
        self.backend.send("DISABLE")
        self.enabled = False
        self._posture_last_sent_height = None
        self.stop_drive()

    def reset(self):
        self._cancel_pending_posture()
        self.stop_skill()
        was_enabled = self.enabled and not self.estop
        self.backend.send("RESET")
        self.enabled = False
        self.estop = False
        self._posture_last_sent_height = None
        if was_enabled:
            self.backend.send("ENABLE")
            self.enabled = True

    def emergency_stop(self):
        self._cancel_pending_posture()
        self.stop_skill()
        self.backend.send("ESTOP")
        self.estop = True
        self.enabled = False

    def stand_up(self):
        if self.backend.running and not self.estop:
            self._cancel_pending_posture()
            self.stop_skill()
            self.backend.command_drive(0.0, 0.0)
            self.backend.send("STAND_UP")

    def clear_estop(self):
        self._cancel_pending_posture()
        self.backend.send("CLEAR_ESTOP")
        self.estop = False

    def _speed_preview(self, _value=None):
        self.speed_readout.set(f"{float(self.speed.get()):.2f} m/s")

    def set_drive(self, forward: float, yaw: float):
        if self.enabled and not self.estop:
            fields = self.backend.telemetry
            drive_ready = (fields.get("mode") == "Balance" and
                           fields.get("station_eligible") == "1" and
                           fields.get("station_capture_ready") == "1")
            if (forward != 0.0 or yaw != 0.0) and not drive_ready:
                self.backend.log.append("DRIVE_BLOCKED|reason=station_not_ready")
                return
            self._cancel_pending_posture()
            self.stop_skill()
        self.drive = (forward * float(self.speed.get()), yaw * float(self.yaw.get()))

    def _drive_press(self, forward: float, yaw: float):
        self._drive_button_active = True
        self.set_drive(forward, yaw)

    def _drive_release(self, _event=None):
        if self._drive_button_active:
            self._drive_button_active = False
            self.stop_drive()

    def stop_drive(self):
        self._drive_button_active = False
        self.stop_skill()
        self.drive = (0.0, 0.0)
        self.backend.command_stop()

    def _posture_capture_ready(self):
        fields = self.backend.telemetry
        if fields.get("mode") != "Balance":
            return False
        try:
            # During an active height path the core deliberately owns a smooth
            # retarget stream. Waiting for the station quiet latch here would
            # queue every slider update until the old path ended, which made
            # the slider appear dead and encouraged repeated preset presses.
            if (fields.get("height_motion") == "1" and
                    fields.get("station_hold") == "1"):
                return True
            # Prefer the controller's adaptive capture calculation.  It uses
            # the live pendulum length, wheel velocity and IMU state, so the UI
            # does not duplicate a fixed pose-specific threshold.  The numeric
            # fallback keeps the client compatible with an older binary.
            if all(key in fields for key in
                   ("station_eligible", "station_capture_ready", "station_hold")):
                return (fields.get("station_eligible") == "1" and
                        fields.get("station_capture_ready") == "1" and
                        fields.get("station_hold") == "1")
            return (abs(float(fields.get("pitch_rate", "nan"))) < 0.25 and
                    abs(float(fields.get("wheel_velocity", "nan"))) < 0.08 and
                    abs(float(fields.get("lqr_velocity", "nan"))) < 0.08)
        except (TypeError, ValueError):
            return False

    def _posture_quiet_for_action(self):
        fields = self.backend.telemetry
        if (fields.get("mode") == "Balance" and
                fields.get("height_motion") == "1" and
                fields.get("station_hold") == "1"):
            return True
        if not self._posture_capture_ready():
            self._posture_quiet_since = None
            return False
        now = time.monotonic()
        if self._posture_quiet_since is None:
            self._posture_quiet_since = now
            return False
        return now - self._posture_quiet_since >= 0.25

    def _queue_pose(self, pose):
        if not self.backend.running or not self.enabled or self.estop:
            return
        self.stop_skill()
        # POSE is an operator override. Queueing it behind the station latch
        # made the buttons appear dead during a height transition; the native
        # planner now cancels/retargets the active posture smoothly.
        self.backend.command_pose(*tuple(pose))
        self._posture_quiet_since = None

    def _posture_preview(self, value):
        try:
            height = _posture_to_height(float(value))
            label = ("deep squat" if float(value) > 0.01 else
                     "max extension" if float(value) < -0.01 else
                     "natural ENABLE")
            self.height_var.set(height)
            self.height_readout.set(f"{label} · {height:.3f} m")
            if self._posture_drag_active:
                self._schedule_posture_send()
        except (TypeError, ValueError):
            self.height_readout.set("--")

    def _posture_press(self, _event):
        self._posture_drag_active = True
        self._schedule_posture_send()

    def _posture_release(self, _event):
        if self._posture_send_job is not None:
            self.root.after_cancel(self._posture_send_job)
            self._posture_send_job = None
        self._posture_drag_active = False
        # Always queue the exact release position; the core starts it only
        # after its sensor-derived capture gate is ready.
        self._send_posture_target()

    def _schedule_posture_send(self):
        if self._posture_send_job is not None:
            self.root.after_cancel(self._posture_send_job)
        self._posture_send_job = self.root.after(
            self._posture_send_period_ms, self._flush_posture_drag)

    def _flush_posture_drag(self):
        self._posture_send_job = None
        if self._posture_drag_active:
            self._send_posture_target()

    def _send_posture_target(self):
        try:
            value = _posture_to_height(float(self.posture_var.get()))
        except (TypeError, ValueError):
            return
        self.send_height(value)

    def apply_posture(self):
        try:
            self.send_height(_posture_to_height(float(self.posture_var.get())))
        except (TypeError, ValueError):
            pass

    def apply_height(self):
        try:
            self.posture_var.set(_height_to_posture(float(self.height_var.get())))
            self.apply_posture()
        except (TypeError, ValueError):
            pass

    def send_height(self, height_m):
        if not self.backend.running or not self.enabled or self.estop:
            return
        self.stop_skill()
        self.drive = (0.0, 0.0)
        value = max(MIN_BODY_HEIGHT_M, min(MAX_BODY_HEIGHT_M, float(height_m)))
        if (self._posture_last_sent_height is not None and
                abs(value - self._posture_last_sent_height) <= 1.0e-4):
            return
        self.height_var.set(value)
        self.posture_var.set(_height_to_posture(value))
        # Send the latest target immediately.  The native core already owns
        # the adaptive capture/pending gate; duplicating that gate in Tk made
        # a button wait for station_hold after a previous pose had ended.
        self.backend.command_height(value)
        self._posture_last_sent_height = value
        self._posture_quiet_since = None

    def send_preset(self, mode, amount):
        if not self.backend.running or not self.enabled or self.estop:
            return
        if mode is None:
            self.send_height(NATURAL_BODY_HEIGHT_M)
        elif mode == "squat":
            self.posture_var.set(float(amount))
            self.apply_posture()
        else:
            # Roll previews remain angle-curve diagnostics; the operator no
            # longer enters four independent joint angles.
            self._queue_pose(_sample_curve(self.curves, mode, amount))

    def start_show(self):
        if self.backend.running and self.enabled and not self.estop:
            self.stop_skill()
            self.drive = (0.0, 0.0)
            self._show_waiting = True
            self._posture_quiet_since = None
            self.show_started_s = None

    def stop_skill(self):
        if self.dance_active:
            self.backend.command_dance_stop()
            self.dance_active = False
        self._cancel_pending_posture()
        self.show_started_s = None

    def start_dance(self, direction=1.0):
        if not self.backend.running or not self.enabled or self.estop:
            return
        self._cancel_pending_posture()
        self.show_started_s = None
        self.drive = (0.0, 0.0)
        self.backend.command_dance(direction)
        self.dance_active = True

    def _show_pose(self, elapsed):
        # Reuse the checked-in CAD neutral/CoM-aware curves, scheduled at UI
        # rate while the native C++ planner remains the actuator owner.
        stages = [(None, 1.0, 0.0, 0.0)]
        for mode in ("squat", "roll_left", "roll_right"):
            stages.extend(((mode, 3.0, 0.0, .5), (mode, 1.5, .5, .5),
                           (mode, 3.0, .5, 0.0), (None, 1.0, 0.0, 0.0)))
        remaining = elapsed
        for mode, duration, start, end in stages:
            if remaining <= duration:
                if mode is None:
                    return tuple(self._natural_enable_pose())
                amount = start + (end - start) * _smoothstep(remaining / duration)
                return _sample_curve(self.curves, mode, amount)
            remaining -= duration
        self.stop_skill()
        return tuple(self._natural_enable_pose())

    def tick(self):
        self.backend.drain()
        fields = self.backend.telemetry
        if (self._posture_last_sent_height is not None and
                (fields.get("height_rejected") == "1" or
                 (fields.get("height_request") == "0" and
                  fields.get("height_motion") == "0" and
                  fields.get("height_settled") == "0"))):
            # A rejected/interrupted transaction must be retryable with the
            # same preset; otherwise the UI silently deduplicates the click.
            self._posture_last_sent_height = None
        mode = fields.get("mode", "offline")
        self.sim_button.configure(text="Stop SIM" if self.backend.running else "Start SIM")
        self.mode.configure(text=f" {mode} ", bg=GOOD if mode in ("Balance", "Drive", "PoseChange") else BAD if mode in ("Fallen", "EmergencyStop") else WARN)
        self.viewer_status.set(
            "MuJoCo 3-D viewer connected" if self.backend.viewer_running
            else "MuJoCo 3-D viewer closed (control process may still be running)"
            if self.backend.running else
            "Nhấn Start SIM để mở MuJoCo 3-D viewer")
        self.telemetry.set(
            f"t={fields.get('t', '--')} s   pitch={fields.get('pitch', '--')} rad   pitch_rate={fields.get('pitch_rate', '--')} rad/s\n"
            f"phase={fields.get('flight', '--')}   airborne={fields.get('airborne', '--')}   touchdown={fields.get('touchdown', '--')}   torque_valid={fields.get('torque_valid', '--')}\n"
            f"skill={fields.get('skill', '--')}   dance_height={fields.get('dance_height_target', '--')} m   dance_yaw={fields.get('dance_yaw_rate', '--')} rad/s\n"
            f"lqr_pitch={fields.get('lqr_pitch_state', '--')} rad   station_capture={fields.get('station_capture_distance', '--')} m   COM-line={fields.get('wb_com_line_angle', '--')} rad\n"
            f"body height={self.backend.view.get('body_height', '--')} m   Fz hypothesis L/R={fields.get('fz_l', '--')} / {fields.get('fz_r', '--')} N\n"
            f"loop={fields.get('loop_hz', '--')} Hz   ground_truth_to_controller={fields.get('ground_truth_to_controller', '--')}"
        )
        if self._show_waiting and self._posture_quiet_for_action():
            self._show_waiting = False
            self.show_started_s = time.monotonic()
            self._posture_quiet_since = None
        if self.show_started_s is not None and self.backend.running and self.enabled and not self.estop:
            pose = self._show_pose(time.monotonic() - self.show_started_s)
            self.backend.command_pose(*pose)
        elif (self.enabled and self.backend.running and not self.estop and
              not self.dance_active):
            self.backend.command_drive(*self.drive)
        self.root.after(20, self.tick)

    def close(self):
        self.disable()
        self.backend.stop()
        self.root.destroy()


def parse_options(argv=None):
    parser = argparse.ArgumentParser(description="TAP native MuJoCo dashboard")
    parser.add_argument("--rate", type=int, choices=(2000, 3000, 4000), default=3000)
    return parser.parse_args(argv)


def main(argv=None):
    options = parse_options(argv)
    root = tk.Tk()
    NativeDashboard(root, NativeBackend(rate_hz=options.rate))
    root.mainloop()


if __name__ == "__main__":
    main()
