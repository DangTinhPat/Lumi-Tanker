#!/usr/bin/env python3
"""Simple USB-serial controller GUI for the TAP ESP32 firmware.

Run with: python3 tools/tap_serial_control_gui.py
Requires pyserial and Tkinter. The GUI sends START / DRIVE / STOP / DISABLE /
ESTOP commands to the firmware console and displays its serial diagnostics.
The person follower is a GUI toggle that sends DRIVE/STOP over the same
serial link and leaves the balance session active when tracking stops.
"""

from __future__ import annotations

import glob
import json
import math
import os
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "gui"))

from gui.motion_api import MotionAPI, MotionLimits
from gui.command_router import CommandResult, RobotCommandRouter
from gui.vision_behavior import VisionBehaviorConfig, VisionTarget, Vbc
from gui.vision_process import PersonVisionProcess

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Keep the window available to explain the dependency.
    serial = None
    list_ports = None


BAUD = 921600  # firmware v3 AppV3 (was 115200: the link saturated)
# Every received and sent line is also appended here (one file per session),
# so a second tool can read the robot log without opening the serial port.
LOG_DIR = os.path.expanduser("~/tap_logs")
HEARTBEAT_S = 0.20
HMOVE_REPEAT_S = 0.10   # firmware stops a height move not refreshed in 0.3 s
JOY_SIZE = 180          # joystick canvas [px]
JOY_KNOB = 18           # knob radius [px]
JOY_DEADZONE = 0.08
VISION_CAMERA_SOURCE = "/dev/video4"
# Approximate FOV is sufficient for a small image-centering test; it is not a
# distance or camera pose measurement.
VISION_HORIZONTAL_FOV_RAD = math.radians(60.0)
MODE_RE = re.compile(r"CONTROL mode=(\S+) enable=(\d+)")
ALGO_RE = re.compile(r"V3 TUNE algo=(LQR|PID|WB)\b")
SENS_RE = re.compile(
    r"SENS .*?can_legs=(\d+) uart_wheels=(\d+) homed=(\d+) "
    r"joint_q=\[([^]]+)\] pitch=([+-]?[\d.]+) pitch_rate=([+-]?[\d.]+)"
)
CONTROL_MODE_RE = re.compile(r"CONTROL mode=(\S+)")
AXLE_X_RE = re.compile(r"\baxle_x=([+-]?[\d.]+)")
HEIGHT_RE = re.compile(r"\bh=([0-9.]+).*?\bhtgt=([0-9.]+)")
WHEEL_POS_RE = re.compile(
    r"\bwheel_lx=([+-]?[\d.]+)\s+wheel_rx=([+-]?[\d.]+)")


class SerialLink:
    """Small line-oriented serial reader/writer with a background reader."""

    def __init__(self, incoming: queue.Queue[str]) -> None:
        self.incoming = incoming
        self.port = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()
        self.reader_thread: threading.Thread | None = None
        self.log_file = None
        self.log_start = time.monotonic()
        self.telemetry: dict[str, str] = {}
        self.telemetry_received_monotonic_s = 0.0

    def _log(self, text: str) -> None:
        if self.log_file is not None:
            try:
                self.log_file.write(f"{time.monotonic() - self.log_start:10.3f} {text}\n")
            except OSError:
                pass

    def log_event(self, text: str) -> None:
        """Write an application diagnostic into the active serial session log."""
        self._log(f"[event] {text}")

    @property
    def connected(self) -> bool:
        return self.port is not None and self.port.is_open

    @property
    def running(self) -> bool:
        return self.connected

    def open(self, device: str) -> None:
        if serial is None:
            raise RuntimeError("Thiếu pyserial. Cài bằng: python3 -m pip install pyserial")
        port = serial.Serial(port=None, baudrate=BAUD, timeout=0.15,
                             write_timeout=0.3)
        # Keep the CP2102 reset lines inactive when opening the GUI.
        port.dtr = False
        port.rts = False
        port.port = device
        port.open()
        self.port = port
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, time.strftime("tap_%Y%m%d_%H%M%S.log"))
        self.log_file = open(path, "a", buffering=1, encoding="utf-8")
        self.log_start = time.monotonic()
        self.telemetry.clear()
        self.telemetry_received_monotonic_s = 0.0
        self.incoming.put(f"[log] {path}")
        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

    def send(self, command: str) -> bool:
        with self.write_lock:
            if not self.connected:
                return False
            try:
                self.port.write((command + "\n").encode("ascii"))
                if command != "HEARTBEAT":
                    self._log(f">> {command}")
                return True
            except (OSError, serial.SerialException) as exc:
                self.incoming.put(f"[serial write error] {exc}")
                return False

    def command_drive(self, velocity: float, yaw_rate: float) -> bool:
        return self.send(f"DRIVE {velocity:.7g} {yaw_rate:.7g}")

    def command_stop(self) -> bool:
        return self.send("STOP")

    def command_estop(self) -> bool:
        return self.send("ESTOP")

    def close(self) -> None:
        self.stop_event.set()
        port, self.port = self.port, None
        if port is not None:
            try:
                if port.is_open:
                    port.close()
            except OSError:
                pass
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=0.5)
        self.reader_thread = None
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None

    def _read_loop(self) -> None:
        port = self.port
        if port is None:
            return
        while not self.stop_event.is_set():
            try:
                raw = port.readline()
            except (OSError, serial.SerialException, TypeError) as exc:
                # TypeError: pyserial reading a port that close() just shut.
                if not self.stop_event.is_set():
                    self.incoming.put(f"[serial read error] {exc}")
                return
            if raw:
                line = raw.decode("utf-8", errors="replace").rstrip()
                self._log(line)
                self._update_telemetry(line)
                self.incoming.put(line)

    def _update_telemetry(self, line: str) -> None:
        now = time.monotonic()
        if match := CONTROL_MODE_RE.search(line):
            mode = match.group(1)
            self.telemetry["mode"] = "Balance" if mode == "Balancing" else mode
        if match := AXLE_X_RE.search(line):
            self.telemetry["wheel_position"] = match.group(1)
        if match := WHEEL_POS_RE.search(line):
            self.telemetry["wheel_left_position"] = match.group(1)
            self.telemetry["wheel_right_position"] = match.group(2)
            self.telemetry_received_monotonic_s = now
        if match := HEIGHT_RE.search(line):
            self.telemetry["height_m"] = str(float(match.group(1)) * 0.001)
            self.telemetry["height_target_m"] = str(float(match.group(2)) * 0.001)


class TapSerialControlApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("TAP — điều khiển qua USB")
        root.geometry("850x800")
        root.minsize(760, 600)

        self.incoming: queue.Queue[str] = queue.Queue()
        # One pending voice call at most: the command tree explicitly forbids
        # replaying a backlog of stale robot actions after the GUI stalls.
        self.voice_commands: queue.Queue[tuple[dict | str, queue.Queue[dict]]] = queue.Queue(maxsize=1)
        self.pending_voice: dict[str, dict] = {}
        self.pending_voice_lock = threading.Lock()
        self._tk_thread_id = threading.get_ident()
        self.link = SerialLink(self.incoming)
        self.last_vision_diag_log_s = 0.0
        self.port_var = tk.StringVar()
        self.lying_zero_confirmed = tk.BooleanVar(value=False)
        self.connection_var = tk.StringVar(value="Chưa kết nối")
        self.mode_var = tk.StringVar(value="—")
        self.algorithm_var = tk.StringVar(value="Chưa đọc được")
        self.enable_var = tk.StringVar(value="—")
        self.imu_var = tk.StringVar(value="Chưa có dữ liệu")
        self.feedback_var = tk.StringVar(value="CAN chân: —   UART bánh: —   Zero chân: —")
        self.speed_var = tk.DoubleVar(value=0.08)
        self.yaw_var = tk.DoubleVar(value=0.25)
        self.session_active = False
        self.firmware_ready = False
        self.last_heartbeat = 0.0
        self.held: set[str] = set()
        # Virtual joystick (-1..1): x right, y forward. Height hold: +1/-1/0.
        self.joy_x = 0.0
        self.joy_y = 0.0
        self.joy_active = False
        self.last_drive_sent = 0.0
        self.last_drive_cmd = ""
        self.height_dir = 0
        self.last_hmove_sent = 0.0
        self.height_var = tk.StringVar(value="Chiều cao: —")
        self.tier_var = tk.StringVar(value="Tầng 2: —")
        self.vision_process = PersonVisionProcess(ROOT / "blue_object_vision")
        self.motion = MotionAPI(
            self.link,
            limits=MotionLimits(max_linear_m_s=0.20, max_yaw_rad_s=0.40),
            require_station_capture=False,
        )
        self._follow_stop_status = "Vision: đã dừng"
        self.command_router = RobotCommandRouter(
            self.motion,
            can_control=self._can_control,
            set_follow=self._set_follow_state,
            send_console=self._send,
            emergency_stop=self._send_estop,
            on_result=self._on_command_result,
        )
        self.vision_motion = self.command_router.motion_source("tracking")
        self.vision_behavior: Vbc | None = None
        self.vision_record: dict | None = None
        self.vision_status_var = tk.StringVar(value="Vision: tắt")

        self._build_ui()
        self.root.bind("<KeyPress>", self._key_press)
        self.root.bind("<KeyRelease>", self._key_release)
        self.root.bind("<Escape>", lambda _event: self.estop())
        self.root.bind("<FocusOut>", lambda _event: self._release_all())
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh_ports()
        self.root.after(50, self._tick)
        # Connect straight away so the session log starts with the GUI.
        self.root.after(300, self._auto_connect)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")
        ttk.Label(top, text="Cổng ESP32:").pack(side="left")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var,
                                     width=34, state="readonly")
        self.port_box.pack(side="left", padx=6)
        ttk.Button(top, text="Tìm cổng", command=self.refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(top, text="Kết nối", command=self.connect)
        self.connect_button.pack(side="left", padx=5)
        ttk.Label(top, textvariable=self.connection_var).pack(side="left", padx=8)

        status = ttk.LabelFrame(self.root, text="Trạng thái firmware", padding=10)
        status.pack(fill="x", padx=10, pady=(0, 8))
        ttk.Label(status, text="Mode:").grid(row=0, column=0, sticky="w")
        ttk.Label(status, textvariable=self.mode_var, width=18).grid(row=0, column=1, sticky="w")
        ttk.Label(status, text="Motor enable:").grid(row=0, column=2, sticky="w", padx=(20, 0))
        ttk.Label(status, textvariable=self.enable_var, width=10).grid(row=0, column=3, sticky="w")
        ttk.Label(status, text="Thuật toán:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(status, textvariable=self.algorithm_var, width=18).grid(
            row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Label(status, text="Pitch robot (trục roll IMU):").grid(
            row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Label(status, textvariable=self.imu_var).grid(
            row=2, column=1, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Label(status, textvariable=self.feedback_var).grid(
            row=3, column=0, columnspan=4, sticky="w", pady=(5, 0))

        actions = ttk.LabelFrame(self.root, text="Khởi động, dừng và di chuyển", padding=10)
        actions.pack(fill="x", padx=10, pady=8)
        self.start_button = ttk.Button(actions, text="▶ START / RELEASE",
                                       command=self.start, state="disabled")
        self.start_button.grid(row=0, column=0, padx=4, pady=3, sticky="ew")
        ttk.Button(actions, text="■ DISABLE", command=self.disable).grid(
            row=0, column=1, padx=4, pady=3, sticky="ew")
        ttk.Button(actions, text="DỪNG DI CHUYỂN", command=self.stop_drive).grid(
            row=0, column=2, padx=4, pady=3, sticky="ew")
        ttk.Button(actions, text="ESTOP", command=self.estop).grid(
            row=0, column=3, padx=4, pady=3, sticky="ew")
        ttk.Button(actions, text="CLEAR E-STOP", command=self.clear_estop).grid(
            row=0, column=4, padx=4, pady=3, sticky="ew")
        # One flow: START on the stand -> READY (wheels hold) -> carry to the
        # floor, hold it at its standing point -> let go (auto) or press GO.
        ttk.Button(actions, text="GO — sẵn sàng (bấm trên giá, rồi đặt xuống sàn và buông tay)",
                   command=self.go).grid(
            row=1, column=0, columnspan=5, padx=4, pady=6, sticky="ew")
        # v3.1 height (hip-to-axle): hold to raise/lower, release to stop.
        up = ttk.Button(actions, text="▲ Nâng chiều cao (giữ)  R")
        up.grid(row=2, column=0, columnspan=2, padx=4, pady=3, sticky="ew")
        down = ttk.Button(actions, text="▼ Hạ chiều cao (giữ)  F")
        down.grid(row=2, column=2, columnspan=2, padx=4, pady=3, sticky="ew")
        for button, direction in ((up, 1), (down, -1)):
            button.bind("<ButtonPress-1>", lambda _e, d=direction: self._height_hold(d))
            button.bind("<ButtonRelease-1>", lambda _e: self._height_hold(0))
        ttk.Label(actions, textvariable=self.height_var).grid(
            row=2, column=4, padx=4, pady=3, sticky="w")
        self._direction_button(actions, "▲ TIẾN (giữ)", "up", 3, 0, columnspan=2)
        self._direction_button(actions, "▼ LÙI (giữ)", "down", 3, 3, columnspan=2)
        self.vision_button = ttk.Button(
            actions, text="Theo dõi người", command=self.toggle_vision)
        self.vision_button.grid(row=4, column=0, columnspan=5, padx=4, pady=(8, 2),
                                sticky="ew")
        ttk.Label(actions, textvariable=self.vision_status_var).grid(
            row=5, column=0, columnspan=5, padx=4, pady=(0, 3), sticky="w")
        ttk.Checkbutton(actions,
                        text="Đã bật nguồn motor khi robot ở đúng tư thế nằm HOME (mốc encoder)",
                        variable=self.lying_zero_confirmed).grid(
            row=6, column=0, columnspan=5, padx=4, pady=3, sticky="w")
        ttk.Button(actions, text="ĐỨNG DẬY từ tư thế nằm xấp → 130 mm",
                   command=self.standup).grid(
            row=7, column=0, columnspan=5, padx=4, pady=3, sticky="ew")
        for column in range(5):
            actions.columnconfigure(column, weight=1)

        # v3.3 two-tier control + maneuver preview: always on in the firmware
        # (no toggles); SET vpitch/trunk/tprev 0 in the console for A/B tests.
        tiers = ttk.LabelFrame(self.root, text="v3.3 điều khiển phân tầng (luôn bật)", padding=10)
        tiers.pack(fill="x", padx=10, pady=(0, 8))
        ttk.Label(tiers, textvariable=self.tier_var).grid(row=0, column=0, sticky="w")

        # Measurements for the hardware sessions (docs/V3.3_HANDOFF.md §7):
        # recorded at 250 Hz in the firmware, dumped as RC lines into the log.
        measure = ttk.LabelFrame(self.root, text="Đo đạc (ghi 250 Hz, tự đổ vào log)", padding=10)
        measure.pack(fill="x", padx=10, pady=(0, 8))
        for column, (label, command) in enumerate((
                ("Ghi 6 s (REC)", "REC 6"),
                ("Gối trên giá (KNEETEST)", "KNEETEST 0.05"),
                ("Gối trên sàn (KNEETEST FLOOR)", "KNEETEST FLOOR 0.03"),
                ("Hông trên giá (HIPTEST)", "HIPTEST 0.05"))):
            ttk.Button(measure, text=label, command=lambda c=command: self._send(c)).grid(
                row=0, column=column, padx=4, sticky="ew")
            measure.columnconfigure(column, weight=1)

        console = ttk.LabelFrame(self.root, text="Lệnh console (vd: SET leg_kp 15 · SET leg_kd 0.5 · GET)",
                                 padding=10)
        console.pack(fill="x", padx=10, pady=(0, 8))
        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(console, textvariable=self.command_var)
        self.command_entry.pack(side="left", fill="x", expand=True)
        self.command_entry.bind("<Return>", lambda _event: self.send_console())
        ttk.Button(console, text="Gửi", command=self.send_console).pack(side="left", padx=(6, 0))

        drive = ttk.LabelFrame(self.root, text="Điều hướng · giữ nút để chạy", padding=10)
        drive.pack(fill="x", padx=10, pady=8)
        ttk.Label(drive, text="Tốc độ tiến/lùi (m/s)").grid(row=0, column=0, sticky="w")
        ttk.Scale(drive, from_=0.02, to=0.55, variable=self.speed_var,
                  orient="horizontal").grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Label(drive, textvariable=self._number_var(self.speed_var, "%.2f")).grid(
            row=0, column=2, sticky="e")
        ttk.Label(drive, text="Tốc độ quay (rad/s)").grid(row=1, column=0, sticky="w")
        ttk.Scale(drive, from_=0.05, to=1.5, variable=self.yaw_var,
                  orient="horizontal").grid(row=1, column=1, sticky="ew", padx=8)
        ttk.Label(drive, textvariable=self._number_var(self.yaw_var, "%.2f")).grid(
            row=1, column=2, sticky="e")
        drive.columnconfigure(1, weight=1)

        # Virtual joystick: drag the knob; speed scales with the distance.
        # Up = forward, right = turn right. Release -> knob recentres, stop.
        size = JOY_SIZE
        self.joy = tk.Canvas(drive, width=size, height=size, highlightthickness=0)
        self.joy.grid(row=2, column=0, columnspan=3, pady=(10, 2))
        c = size / 2
        r = size / 2 - 6
        self.joy.create_oval(c - r, c - r, c + r, c + r, outline="#888", width=2)
        self.joy.create_line(c, c - r, c, c + r, fill="#ccc")
        self.joy.create_line(c - r, c, c + r, c, fill="#ccc")
        k = JOY_KNOB
        self.joy_knob = self.joy.create_oval(c - k, c - k, c + k, c + k,
                                             fill="#3b7dd8", outline="")
        self.joy.bind("<ButtonPress-1>", self._joy_move)
        self.joy.bind("<B1-Motion>", self._joy_move)
        self.joy.bind("<ButtonRelease-1>", self._joy_release)
        ttk.Label(drive, text="Kéo joystick bằng chuột (thả ra là dừng). Phím W/A/S/D vẫn dùng "
                              "được; R/F giữ để nâng/hạ chiều cao.").grid(
            row=4, column=0, columnspan=3, sticky="w", pady=(8, 0))
        warning = ttk.Label(
            self.root,
            text="START yêu cầu firmware enable motor; nếu build đang khóa Enable, lý do sẽ hiện "
                 "trong serial log.",
            wraplength=810, foreground="#9a4a00")
        warning.pack(fill="x", padx=12, pady=(4, 8))

        logs = ttk.LabelFrame(self.root, text="Serial log", padding=6)
        logs.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.log = tk.Text(logs, height=12, wrap="none", state="disabled")
        self.log.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(logs, command=self.log.yview)
        scroll.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=scroll.set)

    @staticmethod
    def _number_var(variable: tk.DoubleVar, fmt: str) -> tk.StringVar:
        result = tk.StringVar()
        def update(*_args: object) -> None:
            result.set(fmt % variable.get())
        variable.trace_add("write", update)
        update()
        return result

    def _direction_button(self, parent: ttk.Frame, text: str, direction: str,
                          row: int, column: int, columnspan: int = 1) -> None:
        button = ttk.Button(parent, text=text, width=16)
        button.grid(row=row, column=column, columnspan=columnspan,
                    padx=4, pady=3, ipadx=6, ipady=6, sticky="ew")
        button.bind("<ButtonPress-1>",
                    lambda _event, key=direction: self._hold(key))
        button.bind("<ButtonRelease-1>",
                    lambda _event, key=direction: self._release(key))

    def refresh_ports(self) -> None:
        ports = ([item.device for item in list_ports.comports()]
                 if list_ports is not None else [])
        if not ports:
            ports = sorted(glob.glob("/dev/serial/by-id/*CP210*") +
                           glob.glob("/dev/ttyUSB*"))
        self.port_box["values"] = ports
        if self.port_var.get() not in ports:
            self.port_var.set(ports[0] if ports else "")
        if serial is None:
            self._append_log("Cần pyserial: python3 -m pip install pyserial")

    def _auto_connect(self) -> None:
        if self.link.connected or serial is None:
            return
        by_id = sorted(glob.glob("/dev/serial/by-id/*CP210*"))
        if by_id:
            self.port_var.set(os.path.realpath(by_id[0]))
        if self.port_var.get():
            self.connect()

    def connect(self) -> None:
        self.lying_zero_confirmed.set(False)
        if self.link.connected:
            self._release_all()
            self.disable()
            self.link.close()
            self.connection_var.set("Đã ngắt kết nối")
            self.connect_button.configure(text="Kết nối")
            return
        device = self.port_var.get().strip()
        if not device:
            messagebox.showinfo("Chưa có cổng", "Chọn cổng CP2102 của ESP32 trước.")
            return
        try:
            self.link.open(device)
        except Exception as exc:  # Port availability differs across Linux setups.
            messagebox.showerror("Không kết nối được", str(exc))
            return
        self.session_active = False
        self.firmware_ready = False
        self.start_button.configure(state="disabled")
        self.connection_var.set(f"Đã mở {device}; chờ firmware boot...")
        self.connect_button.configure(text="Ngắt kết nối")
        self._append_log(
            f"Đã mở {device} @ {BAUD}; chờ SENS sau boot rồi mới cho START.")
        self._send("GET")  # read-only: show the active balancer in the status panel

    def _send(self, command: str) -> bool:
        if not self.link.send(command):
            self._append_log("Không gửi được: chưa kết nối serial.")
            return False
        self._append_log(f">> {command}")
        return True

    def start(self) -> None:
        if not self.link.connected:
            self._append_log("Kết nối ESP32 qua USB trước khi START.")
            return
        if not self.firmware_ready:
            self._append_log("Chưa nhận SENS sau boot; chưa gửi START.")
            return
        self.session_active = True
        self._send("START")
        self.last_heartbeat = time.monotonic()

    def standup(self) -> None:
        if not self.link.connected or not self.firmware_ready:
            self._append_log("Chưa kết nối/chưa có SENS; chưa gửi STANDUP.")
            return
        if self.mode_var.get() not in ("Idle", "Fault"):
            self._append_log("DISABLE trước khi đứng dậy.")
            return
        if not self.lying_zero_confirmed.get():
            self._append_log("Xác nhận nguồn motor đã bật ở tư thế HOME; mốc encoder phải đúng trước khi đứng dậy.")
            return
        self._clear_drive_inputs()
        for command in ("SET lying 1", "SET standup 1", "STANDUP"):
            if not self._send(command):
                return
        self.session_active = True
        self.last_heartbeat = time.monotonic()

    def tilt_test(self) -> None:
        # Same session/heartbeat handling as START; the firmware then drives the
        # wheels from pitch only (no balance loop) for checking tilt direction.
        if not self.link.connected or not self.firmware_ready:
            self._append_log("Chưa kết nối/chưa có SENS; chưa gửi TILTTEST.")
            return
        self.session_active = True
        self._send("TILTTEST")
        self.last_heartbeat = time.monotonic()

    def go(self) -> None:
        # GO also starts from Idle, so it needs the same session heartbeat.
        if not self.link.connected or not self.firmware_ready:
            self._append_log("Chưa kết nối/chưa có SENS; chưa gửi GO.")
            return
        self.session_active = True
        self._send("GO")
        self.last_heartbeat = time.monotonic()

    def stop_drive(self) -> None:
        self.stop_vision("Vision: đã dừng")
        self._clear_drive_inputs()
        self.command_router.stop_all()

    def disable(self) -> None:
        self.stop_vision("Vision: dừng do DISABLE")
        self._clear_drive_inputs()
        self.command_router.stop_all()
        self.session_active = False
        self._send("DISABLE")

    def estop(self) -> None:
        self._follow_stop_status = "Vision: dừng do ESTOP"
        self.command_router.execute({
            "command_id": f"gui-estop-{time.time_ns()}",
            "source": "manual_ui",
            "action": "emergency_stop",
            "args": {"reason": "operator pressed ESTOP"},
        })

    def clear_estop(self) -> None:
        self.session_active = False
        self._send("CLEAR_ESTOP")

    def _hold(self, direction: str) -> None:
        if (not self.link.connected or not self.session_active or
                self.mode_var.get() != "Balancing"):
            self._append_log("Chỉ điều khiển tiến/lùi khi ESP đang Balance.")
            return
        if self.vision_process.running:
            self.stop_vision("Vision: dừng để nhận điều khiển tay")
        if direction not in self.held:
            self.held.add(direction)
            self._publish_drive()

    def _release(self, direction: str) -> None:
        self.held.discard(direction)
        self._publish_drive()

    def _release_all(self) -> None:
        if self.held or self.joy_active:
            self._clear_drive_inputs()
            self._publish_drive()
        self._height_hold(0)

    def _clear_drive_inputs(self) -> None:
        self.held.clear()
        self.joy_active = False
        self.joy_x = self.joy_y = 0.0
        if hasattr(self, "joy"):
            c = JOY_SIZE / 2
            self.joy.coords(self.joy_knob, c - JOY_KNOB, c - JOY_KNOB,
                            c + JOY_KNOB, c + JOY_KNOB)

    def send_console(self) -> None:
        text = self.command_var.get().strip()
        if not text:
            return
        if not self.link.connected:
            self._append_log("Kết nối ESP32 qua USB trước.")
            return
        self._send(text)
        self.command_var.set("")

    def _key_press(self, event: tk.Event) -> None:
        if event.widget is getattr(self, "command_entry", None):
            return  # typing a console command must not drive the robot
        height = {"r": 1, "f": -1}.get(event.keysym.lower())
        if height:
            self._height_hold(height)
            return
        direction = {"w": "up", "s": "down", "a": "left", "d": "right"}.get(
            event.keysym.lower())
        if direction:
            self._hold(direction)

    def _key_release(self, event: tk.Event) -> None:
        if event.widget is getattr(self, "command_entry", None):
            return
        if event.keysym.lower() in ("r", "f"):
            self._height_hold(0)
            return
        direction = {"w": "up", "s": "down", "a": "left", "d": "right"}.get(
            event.keysym.lower())
        if direction:
            self._release(direction)

    def _drive_values(self) -> tuple[float, float]:
        if self.joy_active:
            fwd_in, turn_in = self.joy_y, self.joy_x
        else:
            fwd_in = int("up" in self.held) - int("down" in self.held)
            turn_in = int("right" in self.held) - int("left" in self.held)
        forward = self.speed_var.get() * fwd_in
        # On the robot (2026-09-28) a positive DRIVE yaw turns it right.
        yaw = self.yaw_var.get() * turn_in
        return forward, yaw

    def _drive_command(self) -> str:
        forward, yaw = self._drive_values()
        return f"DRIVE {forward:.3f} {yaw:.3f}"

    def _publish_drive(self) -> None:
        command = self._drive_command()
        forward, yaw = self._drive_values()
        if not self.command_router.command_velocity("manual_ui", forward, yaw):
            if forward != 0.0 or yaw != 0.0:
                self._append_log("Lệnh tay bị chặn: ESP chưa Balance hoặc follower đang ưu tiên.")
        self.last_drive_cmd = command
        self.last_drive_sent = time.monotonic()

    def _joy_move(self, event: tk.Event) -> None:
        if self.vision_process.running:
            self.stop_vision("Vision: dừng để nhận điều khiển tay")
        c = JOY_SIZE / 2
        r = JOY_SIZE / 2 - 6 - JOY_KNOB
        dx, dy = event.x - c, event.y - c
        dist = (dx * dx + dy * dy) ** 0.5
        if dist > r:
            dx, dy = dx * r / dist, dy * r / dist
        self.joy.coords(self.joy_knob, c + dx - JOY_KNOB, c + dy - JOY_KNOB,
                        c + dx + JOY_KNOB, c + dy + JOY_KNOB)
        x, y = dx / r, -dy / r
        # Dead zone around the centre.
        self.joy_x = 0.0 if abs(x) < JOY_DEADZONE else x
        self.joy_y = 0.0 if abs(y) < JOY_DEADZONE else y
        self.joy_active = True

    @staticmethod
    def _vision_target(record: dict | None) -> VisionTarget | None:
        if record is None or record.get("target_visible") is not True:
            return None
        try:
            fov_value = record.get("horizontal_fov_rad")
            return VisionTarget(
                horizontal_error_px=float(record["horizontal_offset_px"]),
                image_width_px=float(record["image_width_px"]),
                observed_at_s=float(record["timestamp_monotonic_s"]),
                horizontal_fov_rad=(None if fov_value is None else float(fov_value)),
                depth_trend=str(record.get("depth_trend", "UNKNOWN")),
                area_change_fraction=(
                    None if record.get("area_change_fraction") is None
                    else float(record["area_change_fraction"])),
                area_fraction=(
                    None if record.get("area_fraction") is None
                    else float(record["area_fraction"])),
            )
        except (KeyError, TypeError, ValueError):
            return None

    def _can_control(self) -> bool:
        return (self.link.connected and self.session_active and
                self.mode_var.get() == "Balancing")

    def _on_command_result(self, result: CommandResult) -> None:
        self._append_log(
            f"[command {result.status}] {result.command_id} "
            f"{result.source}:{result.action} — {result.message}")

    def execute_robot_command(self, command: dict | str) -> dict:
        """Thread-safe voice/tool entry point for one JSON command envelope."""
        if threading.get_ident() != self._tk_thread_id:
            response: queue.Queue[dict] = queue.Queue(maxsize=1)
            command_id, action = self._voice_command_identity(command)
            item = (command, response)
            queued = {
                "command_id": command_id,
                "status": "accepted",
                "message": "command queued for GUI arbitration",
            }
            if command_id:
                with self.pending_voice_lock:
                    if command_id in self.pending_voice:
                        return {
                            "command_id": command_id,
                            "status": "rejected",
                            "message": "command_id is already pending in the GUI queue",
                        }
                    self.pending_voice[command_id] = queued
            try:
                self.voice_commands.put_nowait(item)
            except queue.Full:
                if action != "emergency_stop":
                    if command_id:
                        with self.pending_voice_lock:
                            self.pending_voice.pop(command_id, None)
                    return {
                        "command_id": command_id,
                        "status": "rejected",
                        "message": "voice command queue is full; no command was queued",
                    }
                # ESTOP may replace a basic call that has not reached the GUI
                # arbiter yet. Complete that caller as cancelled, then queue
                # ESTOP as the only pending request.
                try:
                    pending_command, pending_response = self.voice_commands.get_nowait()
                    pending_id, _ = self._voice_command_identity(pending_command)
                    if pending_id:
                        with self.pending_voice_lock:
                            self.pending_voice.pop(pending_id, None)
                    pending_response.put_nowait({
                        "command_id": pending_id,
                        "status": "cancelled",
                        "message": "replaced by queued emergency_stop",
                    })
                except queue.Empty:
                    pass
                try:
                    self.voice_commands.put_nowait(item)
                except queue.Full:
                    if command_id:
                        with self.pending_voice_lock:
                            self.pending_voice.pop(command_id, None)
                    return {
                        "command_id": command_id,
                        "status": "rejected",
                        "message": "could not prioritize emergency_stop in GUI queue",
                    }
            try:
                return response.get(timeout=2.0)
            except queue.Empty:
                return {
                    "command_id": command_id,
                    "status": "accepted",
                    "message": "command remains queued for GUI arbitration",
                }
        return self._execute_robot_command_on_gui(command)

    @staticmethod
    def _voice_command_identity(command: dict | str) -> tuple[str, str]:
        try:
            envelope = json.loads(command) if isinstance(command, str) else command
        except (TypeError, json.JSONDecodeError):
            return "", ""
        if not isinstance(envelope, dict):
            return "", ""
        return str(envelope.get("command_id", "")), str(envelope.get("action", ""))

    def get_robot_command_status(self, command_id: str) -> dict | None:
        """Query the latest status after a voice command returned running/active."""
        result = self.command_router.status(command_id)
        if result is not None:
            return result
        with self.pending_voice_lock:
            return self.pending_voice.get(command_id)

    def _execute_robot_command_on_gui(self, command: dict | str) -> dict:
        if isinstance(command, str):
            result = self.command_router.submit_json_line(command, source_override="luvox")
            self._append_log(f"[voice result] {result['status']}: {result['message']}")
        else:
            envelope = {**command, "source": "luvox"}
            result = self.command_router.execute(envelope).as_dict()
        if result.get("status") in ("running", "active", "completed"):
            if result.get("action") in ("move", "turn", "set_height", "follow", "emergency_stop"):
                self._clear_drive_inputs()
                self.height_dir = 0
        return result

    def _send_estop(self) -> bool:
        self._clear_drive_inputs()
        self.height_dir = 0
        self.session_active = False
        return self.link.command_estop()

    def _set_follow_state(self, enabled: bool) -> bool:
        if enabled:
            return self._start_vision_local()
        self._stop_vision_local(self._follow_stop_status)
        return True

    def toggle_vision(self) -> None:
        enabled = not self.command_router.follow_active
        if enabled and (self.held or self.joy_active):
            self.vision_status_var.set("Thả điều khiển tay trước khi bật Vision")
            return
        self._follow_stop_status = "Vision: đã dừng"
        result = self.command_router.execute({
            "command_id": f"gui-follow-{time.time_ns()}",
            "source": "manual_ui",
            "action": "follow",
            "args": {"enabled": enabled},
        })
        if result.status == "rejected" or result.status == "blocked":
            self.vision_status_var.set(result.message)

    def _start_vision_local(self) -> bool:
        try:
            self.vision_behavior = Vbc(
                self.vision_motion,
                VisionBehaviorConfig(
                    horizontal_fov_rad=VISION_HORIZONTAL_FOV_RAD,
                    # The person tracker already applies a median + One Euro
                    # filter to the box center. Keep this second filter short
                    # so it does not add enough lag to miss a moving person.
                    filter_time_constant_s=0.06,
                    center_enter_rad=0.045,
                    center_exit_rad=0.09,
                    center_reacquire_confirm_s=0.10,
                    yaw_gain_s_inv=2.2,
                    yaw_derivative_time_s=0.18,
                    bearing_rate_filter_time_constant_s=0.12,
                    max_bearing_rate_rad_s=1.5,
                    max_yaw_rad_s=0.40,
                    max_yaw_accel_rad_s2=1.5,
                    approach_accel_m_s2=0.7,
                    yaw_sign=1.0,
                    approach_speed_m_s=0.18,
                    max_observation_age_s=0.18,
                    range_filter_time_constant_s=0.08,
                    range_rate_filter_time_constant_s=0.10,
                    range_predict_time_s=0.18,
                    max_range_rate_log_s=0.8,
                    range_enter_error=0.035,
                    range_exit_error=0.012,
                    range_confirm_s=0.08,
                    range_stop_dwell_s=0.10,
                    range_min_speed_m_s=0.10,
                ),
            )
            self.vision_record = None
            self.vision_process.start(source=VISION_CAMERA_SOURCE, show_window=True)
            self.vision_status_var.set("Vision: mở camera, chờ nhận diện người…")
            return True
        except Exception as exc:
            self.vision_behavior = None
            self.vision_status_var.set(f"Vision lỗi: {exc}")
            self._append_log(f"[vision error] {exc}")
            return False

    def stop_vision(self, status: str = "Vision: tắt") -> None:
        self._follow_stop_status = status
        if self.command_router.follow_active:
            self.command_router.execute({
                "command_id": f"gui-follow-off-{time.time_ns()}",
                "source": "system",
                "action": "follow",
                "args": {"enabled": False},
            })
        else:
            self._stop_vision_local(status)

    def _stop_vision_local(self, status: str) -> None:
        self.vision_process.stop()
        if self.vision_behavior is not None:
            self.vision_behavior.update(None)
        self.vision_behavior = None
        self.vision_record = None
        self.vision_status_var.set(status)

    def _tick_vision(self) -> None:
        if not self.vision_process.running:
            if self.vision_behavior is not None:
                detail = (self.vision_process.errors[-1]
                          if self.vision_process.errors else "camera tracker đã thoát")
                self.stop_vision(f"Vision lỗi: {detail}")
            self.vision_button.configure(text="Theo dõi người")
            return
        self.vision_button.configure(text="Dừng theo dõi")
        if self.mode_var.get() != "Balancing" or not self.session_active:
            self.stop_vision("Vision: dừng vì robot không còn Balance")
            return
        record = self.vision_process.read_latest()
        if record is not None:
            self.vision_record = record
        target = self._vision_target(self.vision_record)
        usable = self.vision_behavior.update(target)
        now = time.monotonic()
        if self.vision_record is not None:
            record_time = float(self.vision_record.get(
                "timestamp_monotonic_s", now))
            frame_age_ms = max(0.0, (now - record_time) * 1000.0)
            processing_fps = self.vision_record.get("processing_fps")
            inference_ms = self.vision_record.get("inference_ms")
            offset_px = self.vision_record.get("horizontal_offset_px")
            diagnostics = (
                f"VISION track={self.vision_record.get('track_id')} "
                f"state={self.vision_record.get('state')} "
                f"dx_px={offset_px} fps={processing_fps} "
                f"infer_ms={inference_ms} age_ms={frame_age_ms:.0f} "
                f"bearing={self.vision_behavior.bearing_estimate_rad} "
                f"bearing_rate={self.vision_behavior.bearing_rate_rad_s:+.3f} "
                f"range_err={self.vision_behavior.range_error_log:+.3f} "
                f"range_rate={self.vision_behavior.range_rate_log_s:+.3f} "
                f"range_pred={self.vision_behavior.predicted_range_error_log:+.3f} "
                f"linear={self.vision_behavior.linear_command_m_s:+.3f} "
                f"yaw={self.vision_behavior.yaw_command_rad_s:+.3f}")
            if now - self.last_vision_diag_log_s >= 0.20:
                self.link.log_event(diagnostics)
                self.last_vision_diag_log_s = now
        if usable:
            linear = self.vision_behavior.linear_command_m_s
            if linear > 0.0:
                detail = f"kích thước nhỏ hơn mốc — tiến {linear:.2f} m/s"
            elif linear < 0.0:
                detail = f"kích thước lớn hơn mốc — lùi {abs(linear):.2f} m/s"
            elif self.vision_behavior.reference_area_fraction is None:
                detail = "đang căn giữa để lấy mốc kích thước"
            elif self.vision_behavior.range_motion_state > 0:
                detail = "kích thước gần mốc — giảm tốc tiến"
            elif self.vision_behavior.range_motion_state < 0:
                detail = "kích thước gần mốc — giảm tốc lùi"
            else:
                detail = "đã ở vùng kích thước mục tiêu — giữ tiến/lùi"
            if self.vision_record is not None:
                self.vision_status_var.set(
                    f"Vision: {detail} · dx {offset_px:+.0f}px · "
                    f"{float(processing_fps or 0.0):.1f} FPS · "
                    f"age {frame_age_ms:.0f}ms · "
                    f"yaw {self.vision_behavior.yaw_command_rad_s:+.2f}")
            else:
                self.vision_status_var.set(f"Vision: {detail}")
        else:
            self.vision_status_var.set(
                "Vision: mất dấu / frame cũ — đã gửi STOP" if self.vision_record is None
                else f"Vision: frame cũ {frame_age_ms:.0f}ms — đã gửi STOP")

    def _joy_release(self, _event: tk.Event) -> None:
        c = JOY_SIZE / 2
        self.joy.coords(self.joy_knob, c - JOY_KNOB, c - JOY_KNOB, c + JOY_KNOB, c + JOY_KNOB)
        self.joy_x = self.joy_y = 0.0
        self.joy_active = False
        self._publish_drive()

    def _height_hold(self, direction: int) -> None:
        if direction == self.height_dir:
            return
        if direction and self.vision_process.running:
            self.stop_vision("Vision: dừng để nhận điều khiển tay")
        self.height_dir = direction
        if not self.command_router.height_hold(direction):
            self._append_log("Không thể nâng/hạ: cần ESP đang Balance và follower đã dừng.")
        self.last_hmove_sent = time.monotonic()

    def _tick(self) -> None:
        while True:
            try:
                command, response = self.voice_commands.get_nowait()
            except queue.Empty:
                break
            try:
                result = self._execute_robot_command_on_gui(command)
                command_id, _ = self._voice_command_identity(command)
                if command_id:
                    with self.pending_voice_lock:
                        self.pending_voice.pop(command_id, None)
                response.put_nowait(result)
            except Exception as exc:
                command_id, _ = self._voice_command_identity(command)
                if command_id:
                    with self.pending_voice_lock:
                        self.pending_voice.pop(command_id, None)
                response.put_nowait({"status": "rejected", "message": str(exc)})
        while True:
            try:
                line = self.incoming.get_nowait()
            except queue.Empty:
                break
            self._parse_status(line)
            # Recording dumps (thousands of RC/HT samples) go to the log file
            # only; the window shows their BEGIN/END lines.
            if (line.startswith("RC ") or line.startswith("HT ")) and \
                    "BEGIN" not in line and "END" not in line:
                continue
            self._append_log(line)

        now = time.monotonic()
        if self.link.connected:
            # Held buttons and joystick both refresh at 5 Hz while active. This
            # keeps the same command alive over serial and avoids depending on
            # a single press event for a long manual move.
            if self.joy_active or self.held:
                command = self._drive_command()
                if (command != self.last_drive_cmd or now - self.last_drive_sent >= 0.2):
                    forward, yaw = self._drive_values()
                    self.command_router.command_velocity("manual_ui", forward, yaw)
                    self.last_drive_cmd = command
                    self.last_drive_sent = now
            # Height hold-to-run: the firmware stops if not refreshed in 0.3 s.
            if self.height_dir and now - self.last_hmove_sent >= HMOVE_REPEAT_S:
                self.command_router.height_hold(self.height_dir)
                self.last_hmove_sent = now
        if self.link.connected and self.session_active and now - self.last_heartbeat >= HEARTBEAT_S:
            self.link.send("HEARTBEAT")
            self.last_heartbeat = now
        self._tick_vision()
        self.command_router.update(now)
        self.root.after(33, self._tick)

    def _parse_status(self, line: str) -> None:
        if line.startswith("TR "):
            height = re.search(r"\bh=([0-9.]+)", line)
            target = re.search(r"\bhtgt=([0-9.]+)", line)
            if height:
                text = f"Chiều cao: {float(height.group(1)):.0f} mm"
                if target and abs(float(target.group(1)) - float(height.group(1))) > 0.5:
                    text += f" → {float(target.group(1)):.0f}"
                self.height_var.set(text)
            tv = re.search(r"\btv=([-+0-9.]+)", line)
            dhip = re.search(r"\bdhip=([-+0-9.]+)", line)
            ta = re.search(r"\bta=([0-9.]+)", line)
            pitch = re.search(r"\bpitch=([-+0-9.]+)", line)
            lvl = re.search(r"\blvl=([-+0-9.]+)", line)
            held = re.search(r"\bho=([01])", line)
            if tv and dhip and ta:
                text = (f"Con lắc ảo θv: {float(tv.group(1)):+.3f} rad   "
                        f"Hông tầng 2: {float(dhip.group(1)):+.3f} rad   "
                        f"quyền: {float(ta.group(1)):.2f}")
                if pitch and lvl:
                    text += (f"\nThân pitch: {float(pitch.group(1)):+.3f} rad   "
                             f"Hông cân thân (tĩnh): {float(lvl.group(1)):+.3f} rad")
                roll = re.search(r"\broll=([-+0-9.]+)", line)
                rdh = re.search(r"\brdh=([-+0-9.]+)", line)
                if roll and rdh:
                    text += (f"\nThân roll: {float(roll.group(1)):+.3f} rad (+ = trái cao)   "
                             f"Chân trái/phải: ±{float(rdh.group(1)):+.1f} mm")
                if held and held.group(1) == "1":
                    text += "   [đang bị giữ/nhấc: bánh giới hạn]"
                self.tier_var.set(text)
        if match := MODE_RE.search(line):
            mode = match.group(1)
            self.mode_var.set(mode)
            self.enable_var.set("Bật" if match.group(2) == "1" else "Tắt")
            if mode != "Balancing" and self.command_router.active_level > 0:
                self.command_router.stop_all()
                self._clear_drive_inputs()
            if mode != "Balancing" and (self.held or self.joy_active):
                self._clear_drive_inputs()
                self._send("STOP")
            if mode != "Balancing" and self.vision_process.running:
                self.stop_vision("Vision: dừng vì robot không còn Balance")
        if match := ALGO_RE.search(line):
            self.algorithm_var.set(match.group(1))
        if match := SENS_RE.search(line):
            if not self.firmware_ready:
                self.firmware_ready = True
                self.connection_var.set("Firmware sẵn sàng")
                self.start_button.configure(state="normal")
            self.feedback_var.set(
                f"CAN chân: {'OK' if match.group(1) == '1' else '—'}   "
                f"UART bánh: {'OK' if match.group(2) == '1' else '—'}   "
                f"Zero chân: {'OK' if match.group(3) == '1' else '—'}")
            self.imu_var.set(
                f"IMU pitch: {float(match.group(5)):+.3f} rad   "
                f"pitch rate: {float(match.group(6)):+.3f} rad/s")

    def _append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 500:
            self.log.delete("1.0", f"{lines - 500}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def close(self) -> None:
        self.stop_vision("Vision: giao diện đóng")
        self.command_router.stop_all()
        if self.link.connected and self.session_active:
            self.link.send("DISABLE")
        self.link.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    TapSerialControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
