#!/usr/bin/env python3
"""Leg-jog GUI (DRAFT): điều khiển 4 khớp chân GDS68 và 2 bánh BL4820 từ máy tính.

Nạp firmware tools/leg_jog_fw vào ESP32, rồi chạy GUI này. GUI nói chuyện với
ESP32 qua cổng CP2102 (115200 baud); ESP32 thay USB-CAN-A của
TEST1/gds68_controller.py, tự giữ vòng MIT 100 Hz cho cả 4 khớp và lái 2 bánh
qua UART1/UART2.

    pio run -d tools/leg_jog_fw -t upload --upload-port /dev/ttyUSB1
    python3 tools/leg_jog_gui.py

An toàn:
- Robot đặt trên giá đỡ: DISABLE, ESTOP hoặc bất kỳ lỗi nào đều đưa cả 4 khớp
  về IDLE, chân sẽ rũ xuống.
- Nút jog là "giữ để chạy": nhả tay là dừng và giữ vị trí.
- GUI gửi JOG mỗi 50 ms làm nhịp sống; ESP32 không nhận lệnh > 0.5 s thì tự
  thả IDLE. GUI bị treo > 0.3 s thì nhịp sống chỉ gửi jog = 0 (giữ vị trí).
- Bánh xe: giữ nút để chạy, nhả là phanh 0.3 s rồi thả lỏng. Chế độ "Quay liên
  tục": bấm 1 lần là chạy, bấm lại hoặc "⏹ Dừng bánh" là dừng. Tốc độ theo đơn
  vị raw của giao thức (±400, giả định ≈ RPM, CHƯA đo). Bánh chạy được cả khi
  chân chưa ENABLE; ESTOP / DISABLE / lỗi / mất lệnh đều tắt torque bánh.
- Phím Esc = ESTOP.
- Mở cổng serial sẽ reset ESP32 (mạch auto-reset của CP2102); firmware khởi
  động lại ở IDLE.
- Vị trí là tương đối so với lúc driver bật nguồn (GDS68 không có góc tuyệt đối
  ở trục ra); giới hạn hành trình tính quanh vị trí lúc ENABLE.

Toạ độ theo quy ước robot (MotorMapping.hpp): hông "+" = nâng chân về phía đầu
robot ở cả hai bên; gối "-" = gập.
"""

import math
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

BAUD = 115200
SEND_PERIOD_S = 0.05
GUI_LEASE_S = 0.3
STATUS_STALE_S = 0.5

JOINTS = ("LH", "LK", "RH", "RK")  # Firmware order (kVerifiedLegCanBindings).
JOINT_TEXT = {
    "LH": "Hông trái (3)",
    "LK": "Gối trái (4)",
    "RH": "Hông phải (1)",
    "RK": "Gối phải (2)",
}
HIP_HINT = ("− hạ", "+ nâng")
KNEE_HINT = ("− gập", "+ duỗi")
FIELDS_PER_JOINT = 8
WHEELS = ("LW", "RW")  # Firmware order: UART1, UART2.
WHEEL_TEXT = {"LW": "Trái (UART1)", "RW": "Phải (UART2)"}
WHEEL_STATE_TEXT = {0: "thả lỏng", 1: "đang chạy", 2: "LỖI · nhả nút"}
FIELDS_PER_WHEEL = 6
WHEEL_RAW_MAX = 400
WHEEL_BTN_BG = "#3a2a10"
WHEEL_COUNTS = 1024      # Present Position counts per revolution.
RPM_WINDOW_S = 1.0       # Sliding window for the position-derived RPM.

BG = "#0a0e1a"
CARD = "#171f30"
BORDER = "#232c3d"
GREEN = "#22c55e"
BLUE = "#3b82f6"
ORANGE = "#f59e0b"
RED = "#ef4444"
CYAN = "#06b6d4"
TEXT = "#f1f5f9"
TEXT_DIM = "#94a3b8"
MONO = ("JetBrains Mono", 10)
MODE_COLORS = {
    "DISABLED": TEXT_DIM,
    "ENGAGING": ORANGE,
    "ACTIVE": GREEN,
    "FAULT": RED,
    "ESTOP": RED,
}


def parse_status(line):
    """Parse one `ST,...` line from LegJog::formatStatus, or return None."""
    parts = line.strip().split(",")
    expected = 6 + len(JOINTS) * FIELDS_PER_JOINT + len(WHEELS) * FIELDS_PER_WHEEL
    if len(parts) != expected or parts[0] != "ST":
        return None
    try:
        status = {
            "mode": parts[1],
            "kp": float(parts[2]),
            "kd": float(parts[3]),
            "speed": float(parts[4]),
            "travel": float(parts[5]),
            "joints": {},
            "wheels": {},
        }
        for i, name in enumerate(JOINTS):
            f = parts[6 + i * FIELDS_PER_JOINT: 6 + (i + 1) * FIELDS_PER_JOINT]
            status["joints"][name] = {
                "q": float(f[0]),
                "target": float(f[1]),
                "start": float(f[2]),
                "tau": float(f[3]),
                "age_ms": int(f[4]),
                "axis_state": int(f[5]),
                "axis_error": int(f[6], 16),
                "bad_echo": int(f[7]),
            }
        base = 6 + len(JOINTS) * FIELDS_PER_JOINT
        for i, name in enumerate(WHEELS):
            f = parts[base + i * FIELDS_PER_WHEEL: base + (i + 1) * FIELDS_PER_WHEEL]
            status["wheels"][name] = {
                "state": int(f[0]),
                "target": int(f[1]),
                "speed": int(f[2]),
                "pos_raw": int(f[3]),
                "age_ms": int(f[4]),
                "err": int(f[5]),
            }
    except (ValueError, IndexError):
        return None
    return status


class Link:
    """Serial link to the ESP32: reader thread + 20 Hz JOG/WHEEL heartbeat sender."""

    def __init__(self, rx_queue):
        self.rx_queue = rx_queue
        self.ser = None
        self._write_lock = threading.Lock()
        self._jog_lock = threading.Lock()
        self._jog = [0.0] * len(JOINTS)
        self._wheel = [0] * len(WHEELS)
        self._gui_stamp = 0.0
        self._stop = threading.Event()
        self._threads = []

    def open(self, port):
        if serial is None:
            raise RuntimeError("pyserial chưa được cài: pip install pyserial")
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = BAUD
        ser.timeout = 0.05
        ser.write_timeout = 0.2
        ser.dtr = False
        ser.rts = False
        ser.open()
        self.ser = ser
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._reader, daemon=True),
            threading.Thread(target=self._sender, daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def close(self):
        if self.ser is not None:
            self.write_line("DISABLE")
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads = []
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def write_line(self, text):
        with self._write_lock:
            if not self.is_open():
                return False
            try:
                self.ser.write((text + "\n").encode("ascii"))
                return True
            except Exception as exc:  # noqa: BLE001 - report any serial failure
                self.rx_queue.put(("err", f"Lỗi ghi serial: {exc}"))
                self._stop.set()
                return False

    def publish_jog(self, vector, wheel_vector=None):
        """Called by the Tk loop; also renews the GUI lease."""
        with self._jog_lock:
            self._jog = list(vector)
            self._wheel = list(wheel_vector) if wheel_vector else [0] * len(WHEELS)
            self._gui_stamp = time.monotonic()

    def _reader(self):
        buffer = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(512)
            except Exception as exc:  # noqa: BLE001
                self.rx_queue.put(("err", f"Lỗi đọc serial: {exc}"))
                self._stop.set()
                return
            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                self.rx_queue.put(("line", raw.decode("ascii", "replace").strip()))
            if len(buffer) > 4096:
                buffer = b""

    def _sender(self):
        next_send = time.monotonic()
        while not self._stop.is_set():
            with self._jog_lock:
                fresh = time.monotonic() - self._gui_stamp <= GUI_LEASE_S
                vector = self._jog if fresh else [0.0] * len(JOINTS)
                wheels = self._wheel if fresh else [0] * len(WHEELS)
            self.write_line("JOG " + " ".join(f"{v:.4f}" for v in vector))
            self.write_line("WHEEL " + " ".join(str(int(v)) for v in wheels))
            next_send += SEND_PERIOD_S
            delay = next_send - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_send = time.monotonic()


class LegJogApp:
    def __init__(self, root):
        self.root = root
        root.title("Leg Jog (DRAFT) — 4 khớp GDS68 + 2 bánh qua ESP32")
        root.configure(bg=BG)
        root.geometry("1400x860")
        root.minsize(1300, 760)

        self.rx_queue = queue.Queue()
        self.link = Link(self.rx_queue)
        self.holds = {}  # button -> (joints, direction)
        self.wheel_holds = {}  # button -> (left dir, right dir)
        self.wheel_latch_button = None  # Continuous mode: the latched button.
        self.wheel_latch = (0, 0)
        # Position history per wheel for the measured RPM: (t, unwrapped counts).
        self.wheel_history = {name: [] for name in WHEELS}
        self.wheel_unwrapped = {name: None for name in WHEELS}
        self.last_status = None
        self.last_status_time = 0.0

        self.kp_var = tk.DoubleVar(value=8.0)
        self.kd_var = tk.DoubleVar(value=0.6)
        self.speed_var = tk.DoubleVar(value=0.3)
        self.travel_var = tk.DoubleVar(value=1.2)
        self.wheel_speed_var = tk.IntVar(value=60)
        self.wheel_continuous_var = tk.BooleanVar(value=False)
        self.port_var = tk.StringVar()

        self._build()
        root.bind("<Escape>", lambda _e: self.command("ESTOP"))
        # Window lost focus (alt-tab): a button release may never arrive.
        # Continuous wheel rotation is deliberate and keeps running.
        root.bind("<FocusOut>",
                  lambda e: self._release_holds() if e.widget is root else None)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._tick()
        self._log("info", "Sẵn sàng. Chọn cổng CP2102 của ESP32 và kết nối. Esc = ESTOP.")

    # ------------------------------------------------------------ layout
    def _card(self, parent, title):
        outer = tk.Frame(parent, bg=CARD, highlightbackground=BORDER,
                         highlightthickness=1)
        tk.Label(outer, text=title.upper(), bg=CARD, fg=TEXT_DIM,
                 font=("Segoe UI", 9, "bold")).pack(anchor="w", padx=14, pady=(10, 6))
        return outer

    def _button(self, parent, text, command, bg, fg, **kw):
        return tk.Button(parent, text=text, command=command, bg=bg, fg=fg,
                         activebackground=bg, activeforeground=fg, relief="flat",
                         font=("Segoe UI", 10, "bold"), **kw)

    def _build(self):
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill="both", expand=True, padx=12, pady=12)
        main.columnconfigure(0, weight=0, minsize=300)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(2, weight=1)

        self._build_left(main).grid(row=0, column=0, rowspan=3, sticky="nsew",
                                    padx=(0, 12))
        self._build_joints(main).grid(row=0, column=1, sticky="nsew", pady=(0, 12))
        self._build_wheels(main).grid(row=1, column=1, sticky="nsew", pady=(0, 12))
        self._build_log(main).grid(row=2, column=1, sticky="nsew")

    def _build_left(self, parent):
        card = self._card(parent, "Kết nối ESP32 / điều khiển")
        pad = dict(padx=14)

        row = tk.Frame(card, bg=CARD)
        row.pack(fill="x", **pad, pady=(2, 8))
        self.port_combo = ttk.Combobox(row, textvariable=self.port_var, state="readonly")
        self.port_combo.pack(side="left", fill="x", expand=True)
        tk.Button(row, text="⟳", command=self._refresh_ports, bg="#111827", fg=TEXT,
                  relief="flat", width=3).pack(side="left", padx=(4, 0))
        self._refresh_ports()
        self.connect_btn = self._button(card, "🔌 Kết nối", self._toggle_connect,
                                        BLUE, "white", pady=6)
        self.connect_btn.pack(fill="x", **pad, pady=(0, 10))

        self.mode_lbl = tk.Label(card, text="CHƯA KẾT NỐI", bg=CARD, fg=TEXT_DIM,
                                 font=("Segoe UI", 16, "bold"))
        self.mode_lbl.pack(anchor="w", **pad)
        self.event_lbl = tk.Label(card, text="", bg=CARD, fg=ORANGE, wraplength=270,
                                  justify="left", font=("Segoe UI", 9))
        self.event_lbl.pack(anchor="w", **pad, pady=(0, 8))

        grid = tk.Frame(card, bg=CARD)
        grid.pack(fill="x", **pad)
        grid.columnconfigure((0, 1), weight=1)
        buttons = [
            ("⚡ ENABLE", lambda: self.command("ENABLE"), "#123a24", GREEN),
            ("⏹ DISABLE", lambda: self.command("DISABLE"), "#3d1414", RED),
            ("✋ HOLD", lambda: self.command("HOLD"), "#111827", TEXT),
            ("↺ CLEAR", lambda: self.command("CLEAR"), "#111827", TEXT),
            ("⤺ Về vị trí lúc Enable", lambda: self.command("GOTO_START"),
             "#102a43", CYAN),
        ]
        for i, (text, cmd, bg, fg) in enumerate(buttons):
            b = self._button(grid, text, cmd, bg, fg, pady=6)
            b.grid(row=i // 2, column=i % 2, sticky="ew", padx=2, pady=2,
                   columnspan=2 if i == len(buttons) - 1 else 1)
        self._button(card, "⛔ ESTOP (Esc)", lambda: self.command("ESTOP"),
                     RED, "white", pady=10).pack(fill="x", **pad, pady=(8, 10))

        tk.Frame(card, bg=BORDER, height=1).pack(fill="x", padx=14, pady=6)
        self._slider(card, "Kp (N·m/rad)", self.kp_var, 0.0, 40.0, 0.5, self._send_gains)
        self._slider(card, "Kd (N·m·s/rad)", self.kd_var, 0.0, 3.0, 0.05, self._send_gains)
        self._slider(card, "Tốc độ jog / đi tới (rad/s)", self.speed_var, 0.02, 1.5,
                     0.01, self._send_speed)
        self._slider(card, "Giới hạn hành trình ± quanh lúc Enable (rad)",
                     self.travel_var, 0.1, 3.0, 0.05, self._send_limit)
        tk.Label(card, text="Chạy trên giá đỡ: DISABLE / lỗi làm chân rũ xuống.",
                 bg=CARD, fg=ORANGE, wraplength=270, justify="left",
                 font=("Segoe UI", 9)).pack(anchor="w", **pad, pady=(8, 12))
        return card

    def _slider(self, parent, label, var, lo, hi, res, on_release):
        row = tk.Frame(parent, bg=CARD)
        row.pack(fill="x", padx=14, pady=3)
        top = tk.Frame(row, bg=CARD)
        top.pack(fill="x")
        tk.Label(top, text=label, bg=CARD, fg=TEXT_DIM,
                 font=("Segoe UI", 9)).pack(side="left")
        value = tk.Label(top, text=f"{var.get():.2f}", bg=CARD, fg=TEXT, font=MONO)
        value.pack(side="right")
        scale = tk.Scale(row, from_=lo, to=hi, resolution=res, orient="horizontal",
                         variable=var, showvalue=False, bg=CARD, fg=TEXT,
                         troughcolor="#111827", highlightthickness=0,
                         command=lambda v: value.config(text=f"{float(v):.2f}"))
        scale.pack(fill="x")
        scale.bind("<ButtonRelease-1>", lambda _e: on_release())

    def _build_joints(self, parent):
        card = self._card(parent, "4 khớp chân (toạ độ robot, tương đối lúc bật driver)")
        table = tk.Frame(card, bg=CARD)
        table.pack(fill="x", padx=14, pady=(0, 10))
        headers = ("Khớp (node)", "Vị trí rad (độ)", "Δ Enable", "Đích", "Mô-men",
                   "fb ms", "Axis", "Jog (giữ)", "")
        for c, h in enumerate(headers):
            tk.Label(table, text=h, bg=CARD, fg=TEXT_DIM,
                     font=("Segoe UI", 9, "bold")).grid(row=0, column=c, padx=4, sticky="w")

        self.cells = {}
        self.goto_vars = {}
        for r, name in enumerate(JOINTS, start=1):
            tk.Label(table, text=JOINT_TEXT[name], bg=CARD, fg=TEXT,
                     font=("Segoe UI", 10, "bold")).grid(row=r, column=0, sticky="w", padx=4, pady=3)
            cells = {}
            for c, key in enumerate(("q", "delta", "target", "tau", "age", "axis"),
                                    start=1):
                lbl = tk.Label(table, text="—", bg=CARD, fg=CYAN, font=MONO,
                               width=16 if key == "q" else 7, anchor="e")
                lbl.grid(row=r, column=c, padx=4)
                cells[key] = lbl
            self.cells[name] = cells
            hint = HIP_HINT if name.endswith("H") else KNEE_HINT
            minus = self._button(table, hint[0], None, "#102a43", CYAN, width=7)
            plus = self._button(table, hint[1], None, "#102a43", CYAN, width=7)
            minus.grid(row=r, column=7, padx=2)
            plus.grid(row=r, column=8, padx=2)
            self._bind_hold(minus, (name,), -1)
            self._bind_hold(plus, (name,), +1)

        groups = tk.Frame(card, bg=CARD)
        groups.pack(fill="x", padx=14, pady=(4, 12))
        for label, joints, hint in (("Cả 2 hông", ("LH", "RH"), HIP_HINT),
                                    ("Cả 2 gối", ("LK", "RK"), KNEE_HINT)):
            tk.Label(groups, text=label, bg=CARD, fg=TEXT,
                     font=("Segoe UI", 10, "bold")).pack(side="left", padx=(0, 6))
            minus = self._button(groups, hint[0], None, "#2a1a3d", "#c4b5fd", width=7)
            plus = self._button(groups, hint[1], None, "#2a1a3d", "#c4b5fd", width=7)
            minus.pack(side="left", padx=2)
            plus.pack(side="left", padx=(2, 18))
            self._bind_hold(minus, joints, -1)
            self._bind_hold(plus, joints, +1)

        # Go-to row: one target per joint, sent together.
        goto = tk.Frame(card, bg=CARD)
        goto.pack(fill="x", padx=14, pady=(0, 12))
        tk.Label(goto, text="Đi tới (rad):", bg=CARD, fg=TEXT,
                 font=("Segoe UI", 10, "bold")).pack(side="left", padx=(0, 8))
        for name in JOINTS:
            tk.Label(goto, text=name, bg=CARD, fg=TEXT_DIM,
                     font=("Segoe UI", 9, "bold")).pack(side="left", padx=(6, 2))
            var = tk.StringVar(value="")
            self.goto_vars[name] = var
            tk.Entry(goto, textvariable=var, width=8, bg="#111827", fg=TEXT,
                     insertbackground=TEXT, relief="flat").pack(side="left")
        self._button(goto, "Đi tới", self._goto_all, "#0c3540", CYAN,
                     padx=10).pack(side="left", padx=(12, 4))
        self._button(goto, "Điền vị trí hiện tại", self._fill_goto, "#111827",
                     TEXT, padx=10).pack(side="left", padx=4)
        return card

    def _build_wheels(self, parent):
        card = self._card(parent, "2 bánh xe BL4820 (UART) · + = robot tiến · raw ≈ RPM, chưa đo")
        table = tk.Frame(card, bg=CARD)
        table.pack(fill="x", padx=14, pady=(0, 6))
        headers = ("Bánh", "Trạng thái", "Lệnh raw", "Đo raw", "RPM đo",
                   "Vị trí °", "fb ms", "err", "Giữ để chạy", "")
        for c, h in enumerate(headers):
            tk.Label(table, text=h, bg=CARD, fg=TEXT_DIM,
                     font=("Segoe UI", 9, "bold")).grid(row=0, column=c, padx=4, sticky="w")
        self.wheel_cells = {}
        for r, name in enumerate(WHEELS, start=1):
            tk.Label(table, text=WHEEL_TEXT[name], bg=CARD, fg=TEXT,
                     font=("Segoe UI", 10, "bold")).grid(row=r, column=0, sticky="w",
                                                         padx=4, pady=3)
            cells = {}
            for c, key in enumerate(("state", "target", "speed", "rpm", "pos", "age",
                                     "err"), start=1):
                lbl = tk.Label(table, text="—", bg=CARD, fg=CYAN, font=MONO,
                               width=13 if key == "state" else 6, anchor="e")
                lbl.grid(row=r, column=c, padx=4)
                cells[key] = lbl
            self.wheel_cells[name] = cells
            left_only = (-1, 0) if name == "LW" else (0, -1)
            back = self._button(table, "◀ Lùi", None, WHEEL_BTN_BG, ORANGE, width=7)
            fwd = self._button(table, "Tiến ▶", None, WHEEL_BTN_BG, ORANGE, width=7)
            back.grid(row=r, column=8, padx=2)
            fwd.grid(row=r, column=9, padx=2)
            self._bind_wheel_hold(back, left_only)
            self._bind_wheel_hold(fwd, tuple(-d for d in left_only))

        group = tk.Frame(card, bg=CARD)
        group.pack(fill="x", padx=14, pady=(2, 4))
        tk.Label(group, text="Cả 2 bánh", bg=CARD, fg=TEXT,
                 font=("Segoe UI", 10, "bold")).pack(side="left", padx=(0, 6))
        for text, dirs in (("◀ Lùi", (-1, -1)), ("Tiến ▶", (1, 1)),
                           ("⟲ Xoay trái", (-1, 1)), ("Xoay phải ⟳", (1, -1))):
            b = self._button(group, text, None, WHEEL_BTN_BG, ORANGE, width=10)
            b.pack(side="left", padx=2)
            self._bind_wheel_hold(b, dirs)
        self._button(group, "⏹ Dừng bánh", self._stop_wheels, "#3d1414", RED,
                     width=11).pack(side="left", padx=(12, 2))

        speed_row = tk.Frame(card, bg=CARD)
        speed_row.pack(fill="x", padx=14, pady=(4, 12))
        tk.Label(speed_row, text="Tốc độ bánh (raw, 5–400):", bg=CARD, fg=TEXT_DIM,
                 font=("Segoe UI", 9)).pack(side="left", padx=(0, 6))
        value = tk.Label(speed_row, text=str(self.wheel_speed_var.get()), bg=CARD,
                         fg=TEXT, font=MONO, width=4)
        tk.Scale(speed_row, from_=5, to=WHEEL_RAW_MAX, resolution=5, orient="horizontal",
                 variable=self.wheel_speed_var, showvalue=False, length=320, bg=CARD,
                 fg=TEXT, troughcolor="#111827", highlightthickness=0,
                 command=lambda v: value.config(text=str(int(float(v))))
                 ).pack(side="left")
        value.pack(side="left", padx=6)
        tk.Label(speed_row, text="(lần đầu ≈60)", bg=CARD, fg=ORANGE,
                 font=("Segoe UI", 9)).pack(side="left", padx=(4, 0))
        tk.Checkbutton(speed_row, text="Quay liên tục (bấm 1 lần)",
                       variable=self.wheel_continuous_var,
                       command=self._on_continuous_toggle, bg=CARD, fg=TEXT,
                       selectcolor="#111827", activebackground=CARD,
                       activeforeground=TEXT, font=("Segoe UI", 10, "bold")
                       ).pack(side="left", padx=(18, 0))
        return card

    def _build_log(self, parent):
        card = self._card(parent, "Log")
        self.log_text = tk.Text(card, bg="#050810", fg=TEXT, font=("Consolas", 9),
                                relief="flat", wrap="word", state="disabled", height=8)
        self.log_text.pack(fill="both", expand=True, padx=14, pady=(0, 14))
        for tag, color in (("ok", GREEN), ("err", RED), ("info", ORANGE),
                           ("dim", TEXT_DIM), ("time", TEXT_DIM)):
            self.log_text.tag_config(tag, foreground=color)
        return card

    # ------------------------------------------------------------ jog holds
    def _bind_hold(self, button, joints, direction):
        button.bind("<ButtonPress-1>",
                    lambda _e: self._press(button, joints, direction))
        button.bind("<ButtonRelease-1>", lambda _e: self._release(button))

    # Tk's own Button bindings show the pressed (sunken) look; the handlers
    # below only track state so the two never fight over `relief`.
    def _press(self, button, joints, direction):
        if not self.last_status or self.last_status["mode"] != "ACTIVE":
            self._log("info", "Jog chỉ chạy khi ACTIVE (bấm ENABLE trước).")
            return
        self.holds[button] = (joints, direction)

    def _release(self, button):
        self.holds.pop(button, None)

    def _release_holds(self):
        """Drop every press-and-hold (legs and wheels); keep a wheel latch."""
        self.holds.clear()
        self.wheel_holds.clear()

    def _release_all(self):
        self._release_holds()
        self._stop_wheel_latch()

    def _bind_wheel_hold(self, button, dirs):
        button.bind("<ButtonPress-1>", lambda _e: self._press_wheel(button, dirs))
        button.bind("<ButtonRelease-1>", lambda _e: self._release_wheel(button))

    def _press_wheel(self, button, dirs):
        if not self.last_status or self.last_status["mode"] in ("FAULT", "ESTOP"):
            self._log("info", "Bánh không chạy khi chưa kết nối hoặc đang FAULT/ESTOP "
                              "(bấm CLEAR).")
            return
        if not self.wheel_continuous_var.get():
            self.wheel_holds[button] = dirs
            return
        if self.wheel_latch_button is button:
            self._stop_wheel_latch()
            self._log("info", "Dừng quay liên tục.")
            return
        self._stop_wheel_latch()
        self.wheel_latch_button = button
        self.wheel_latch = dirs
        button.config(bg=ORANGE, fg="#1a1205", activebackground=ORANGE,
                      activeforeground="#1a1205")
        self._log("info", f"Quay liên tục: trái {dirs[0]:+d}, phải {dirs[1]:+d} × "
                          f"{int(self.wheel_speed_var.get())} raw. Bấm lại hoặc "
                          "⏹ Dừng bánh để dừng.")

    def _release_wheel(self, button):
        self.wheel_holds.pop(button, None)

    def _release_wheels(self):
        self.wheel_holds.clear()
        self._stop_wheel_latch()

    def _stop_wheel_latch(self):
        button = self.wheel_latch_button
        self.wheel_latch_button = None
        self.wheel_latch = (0, 0)
        if button is not None:
            button.config(bg=WHEEL_BTN_BG, fg=ORANGE, activebackground=WHEEL_BTN_BG,
                          activeforeground=ORANGE)

    def _stop_wheels(self):
        self._release_wheels()
        self._log("info", "Dừng bánh.")

    def _on_continuous_toggle(self):
        self._release_wheels()
        self._log("info", "Chế độ bánh: " + ("QUAY LIÊN TỤC (bấm 1 lần)"
                                             if self.wheel_continuous_var.get()
                                             else "giữ để chạy"))

    def _wheel_vector(self):
        dirs = list(self.wheel_latch)
        for held in self.wheel_holds.values():
            for i, d in enumerate(held):
                dirs[i] = max(-1, min(1, dirs[i] + d))
        speed = max(0, min(WHEEL_RAW_MAX, int(self.wheel_speed_var.get())))
        return [d * speed for d in dirs]

    def _jog_vector(self):
        directions = {name: 0 for name in JOINTS}
        for joints, direction in self.holds.values():
            for name in joints:
                directions[name] = max(-1, min(1, directions[name] + direction))
        speed = self.speed_var.get()
        return [directions[name] * speed for name in JOINTS]

    # ------------------------------------------------------------ commands
    def command(self, text):
        if not self.link.is_open():
            self._log("err", "Chưa kết nối ESP32.")
            return
        if text in ("DISABLE", "ESTOP"):
            self._release_all()
        self.link.write_line(text)
        self._log("dim", f"→ {text}")

    def _send_gains(self):
        self.command(f"GAINS {self.kp_var.get():.3f} {self.kd_var.get():.3f}")

    def _send_speed(self):
        self.command(f"SPEED {self.speed_var.get():.3f}")

    def _send_limit(self):
        self.command(f"LIMIT {self.travel_var.get():.3f}")

    def _send_settings(self):
        self._send_gains()
        self._send_speed()
        self._send_limit()

    def _goto_all(self):
        """Send GOTO for every joint whose box holds a number (blank = skip)."""
        targets = []
        for name in JOINTS:
            text = self.goto_vars[name].get().strip().replace(",", ".")
            if not text:
                continue
            try:
                targets.append((name, float(text)))
            except ValueError:
                self._log("err", f"{name}: '{text}' không phải số (rad).")
                return
        if not targets:
            self._log("info", "Nhập góc (rad) cho ít nhất một khớp.")
            return
        for name, value in targets:
            self.command(f"GOTO {JOINTS.index(name)} {value:.4f}")

    def _fill_goto(self):
        if not self.last_status:
            self._log("info", "Chưa có vị trí từ ESP32.")
            return
        for name in JOINTS:
            self.goto_vars[name].set(f"{self.last_status['joints'][name]['q']:.3f}")

    # ------------------------------------------------------------ connection
    def _refresh_ports(self):
        ports = []
        if serial is not None:
            found = list(serial.tools.list_ports.comports())
            preferred = [p.device for p in found
                         if "CP210" in (p.description or "") or "10C4:EA60" in (p.hwid or "").upper()]
            others = [p.device for p in found
                      if p.device not in preferred and p.device.startswith(("/dev/ttyUSB", "/dev/ttyACM", "COM"))]
            ports = preferred + others
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.link.is_open():
            self._release_all()
            self.link.close()
            self.connect_btn.config(text="🔌 Kết nối", bg=BLUE)
            self.last_status = None
            self._log("info", "Đã ngắt kết nối (đã gửi DISABLE).")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("Chưa chọn cổng", "Chọn cổng CP2102 của ESP32.")
            return
        try:
            self.link.open(port)
        except Exception as exc:  # noqa: BLE001
            self._log("err", f"Không mở được {port}: {exc}")
            return
        self.connect_btn.config(text="⛔ Ngắt kết nối", bg=RED)
        self._log("info", f"Đã mở {port}. ESP32 sẽ reset và khởi động ở IDLE.")

    # ------------------------------------------------------------ periodic
    def _tick(self):
        if self.link.is_open():
            self.link.publish_jog(self._jog_vector(), self._wheel_vector())
        self._drain_rx()
        self._refresh_mode()
        self.root.after(50, self._tick)

    def _drain_rx(self):
        try:
            while True:
                kind, payload = self.rx_queue.get_nowait()
                if kind == "err":
                    self._log("err", payload)
                    continue
                status = parse_status(payload)
                if status is not None:
                    self.last_status = status
                    self.last_status_time = time.monotonic()
                    self._show_status(status)
                elif payload.startswith("EV "):
                    tag = "err" if ("FAULT" in payload or "ERR" in payload
                                    or "ESTOP" in payload) else "ok"
                    self._log(tag, payload[3:])
                    if tag == "err" or "ACTIVE" in payload or "BOOT" in payload:
                        self.event_lbl.config(text=payload[3:])
                    if "BOOT" in payload:
                        self._send_settings()
                elif payload:
                    self._log("dim", payload)
        except queue.Empty:
            pass

    def _show_status(self, status):
        for name in JOINTS:
            j = status["joints"][name]
            c = self.cells[name]
            c["q"].config(text=f"{j['q']:+.3f} ({math.degrees(j['q']):+6.1f}°)")
            engaged = status["mode"] in ("ENGAGING", "ACTIVE")
            c["delta"].config(text=f"{j['q'] - j['start']:+.3f}" if engaged else "—")
            c["target"].config(text=f"{j['target']:+.3f}" if engaged else "—")
            c["tau"].config(text=f"{j['tau']:+.2f}")
            age_ok = 0 <= j["age_ms"] <= 50
            c["age"].config(text=str(j["age_ms"]), fg=CYAN if age_ok else RED)
            axis = f"{j['axis_state']}"
            if j["axis_error"]:
                axis += f" E{j['axis_error']:X}"
            if j["bad_echo"]:
                axis += f" echo!{j['bad_echo']}"
            c["axis"].config(text=axis, fg=RED if (j["axis_error"] or j["bad_echo"]) else CYAN)
        for name in WHEELS:
            w = status["wheels"][name]
            c = self.wheel_cells[name]
            c["state"].config(text=WHEEL_STATE_TEXT.get(w["state"], str(w["state"])),
                              fg={0: TEXT_DIM, 1: GREEN, 2: RED}.get(w["state"], TEXT))
            c["target"].config(text=f"{w['target']:+d}")
            c["speed"].config(text=f"{w['speed']:+d}")
            rpm = self._measured_rpm(name, w["pos_raw"])
            c["rpm"].config(text="—" if rpm is None else f"{rpm:+.0f}")
            c["pos"].config(text=f"{w['pos_raw'] * 360.0 / 1024.0:.1f}")
            age_ok = 0 <= w["age_ms"] <= 50
            c["age"].config(text=str(w["age_ms"]), fg=CYAN if age_ok else RED)
            c["err"].config(text=f"0x{w['err']:02X}", fg=RED if w["err"] else CYAN)
            if w["state"] == 2 and self.wheel_latch_button is not None:
                self._stop_wheel_latch()
                self._log("err", f"{name} mất feedback: đã dừng quay liên tục.")

    def _measured_rpm(self, name, pos_raw):
        """RPM from Present Position deltas over RPM_WINDOW_S, independent of
        the raw speed unit; compare it with the commanded raw speed (C2)."""
        now = time.monotonic()
        previous = self.wheel_unwrapped[name]
        if previous is None:
            unwrapped = pos_raw
        else:
            delta = pos_raw - (previous % WHEEL_COUNTS)
            if delta > WHEEL_COUNTS // 2:
                delta -= WHEEL_COUNTS
            elif delta < -WHEEL_COUNTS // 2:
                delta += WHEEL_COUNTS
            unwrapped = previous + delta
        self.wheel_unwrapped[name] = unwrapped
        history = self.wheel_history[name]
        history.append((now, unwrapped))
        while history and now - history[0][0] > RPM_WINDOW_S:
            history.pop(0)
        span = history[-1][0] - history[0][0]
        if len(history) < 3 or span < 0.3:
            return None
        return (history[-1][1] - history[0][1]) / WHEEL_COUNTS / span * 60.0

    def _refresh_mode(self):
        if not self.link.is_open():
            self.mode_lbl.config(text="CHƯA KẾT NỐI", fg=TEXT_DIM)
            return
        if not self.last_status or time.monotonic() - self.last_status_time > STATUS_STALE_S:
            self.mode_lbl.config(text="KHÔNG CÓ STATUS", fg=RED)
            self._release_all()
            return
        mode = self.last_status["mode"]
        self.mode_lbl.config(text=mode, fg=MODE_COLORS.get(mode, TEXT))
        if mode != "ACTIVE" and self.holds:
            self.holds.clear()
        if mode in ("FAULT", "ESTOP") and (self.wheel_holds or self.wheel_latch_button):
            self._release_wheels()

    # ------------------------------------------------------------ log
    def _log(self, tag, text):
        self.log_text.config(state="normal")
        self.log_text.insert("end", time.strftime("[%H:%M:%S] "), "time")
        self.log_text.insert("end", text + "\n", tag)
        self.log_text.see("end")
        lines = int(self.log_text.index("end-1c").split(".")[0])
        if lines > 400:
            self.log_text.delete("1.0", f"{lines - 400}.0")
        self.log_text.config(state="disabled")

    def on_close(self):
        self._release_all()
        if self.link.is_open():
            self.link.close()
        self.root.destroy()


def main():
    if serial is None:
        print("pyserial chưa được cài: pip install pyserial")
    root = tk.Tk()
    LegJogApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
