#!/usr/bin/env python3
"""Simple USB-serial controller GUI for the TAP ESP32 firmware.

Run with: python3 tools/tap_serial_control_gui.py
Requires pyserial and Tkinter. The GUI sends START / DRIVE / STOP / DISABLE /
ESTOP commands to the firmware console and displays its serial diagnostics.
"""

from __future__ import annotations

import glob
import os
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Keep the window available to explain the dependency.
    serial = None
    list_ports = None


BAUD = 115200
# Every received and sent line is also appended here (one file per session),
# so a second tool can read the robot log without opening the serial port.
LOG_DIR = os.path.expanduser("~/tap_logs")
HEARTBEAT_S = 0.20
MODE_RE = re.compile(r"CONTROL mode=(\S+) enable=(\d+)")
SENS_RE = re.compile(
    r"SENS .*?can_legs=(\d+) uart_wheels=(\d+) homed=(\d+) "
    r"joint_q=\[[^]]+\] pitch=([+-]?[\d.]+) pitch_rate=([+-]?[\d.]+)"
)


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

    def _log(self, text: str) -> None:
        if self.log_file is not None:
            try:
                self.log_file.write(f"{time.monotonic() - self.log_start:10.3f} {text}\n")
            except OSError:
                pass

    @property
    def connected(self) -> bool:
        return self.port is not None and self.port.is_open

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
            except (OSError, serial.SerialException) as exc:
                self.incoming.put(f"[serial read error] {exc}")
                return
            if raw:
                line = raw.decode("utf-8", errors="replace").rstrip()
                self._log(line)
                self.incoming.put(line)


class TapSerialControlApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("TAP — điều khiển qua USB")
        root.geometry("850x690")
        root.minsize(760, 600)

        self.incoming: queue.Queue[str] = queue.Queue()
        self.link = SerialLink(self.incoming)
        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="Chưa kết nối")
        self.mode_var = tk.StringVar(value="—")
        self.enable_var = tk.StringVar(value="—")
        self.imu_var = tk.StringVar(value="Chưa có dữ liệu")
        self.feedback_var = tk.StringVar(value="CAN chân: —   UART bánh: —   Zero chân: —")
        self.speed_var = tk.DoubleVar(value=0.08)
        self.yaw_var = tk.DoubleVar(value=0.25)
        self.session_active = False
        self.firmware_ready = False
        self.last_heartbeat = 0.0
        self.held: set[str] = set()

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
        ttk.Label(status, text="Pitch robot (trục roll IMU):").grid(
            row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(status, textvariable=self.imu_var).grid(
            row=1, column=1, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Label(status, textvariable=self.feedback_var).grid(
            row=2, column=0, columnspan=4, sticky="w", pady=(5, 0))

        actions = ttk.LabelFrame(self.root, text="Khởi động và dừng", padding=10)
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
        # v3.1 height (hip-to-axle, mm): the legs move only while balancing.
        for column, (label, mm) in enumerate(
                [("Cao nhất 189", 189), ("170 mm", 170), ("150 mm", 150),
                 ("130 mm", 130), ("Thấp nhất 110", 110)]):
            ttk.Button(actions, text=label,
                       command=lambda mm=mm: self._send(f"SET height {mm}")).grid(
                row=2, column=column, padx=4, pady=3, sticky="ew")
        for column in range(5):
            actions.columnconfigure(column, weight=1)

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
        ttk.Scale(drive, from_=0.02, to=0.25, variable=self.speed_var,
                  orient="horizontal").grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Label(drive, textvariable=self._number_var(self.speed_var, "%.2f")).grid(
            row=0, column=2, sticky="e")
        ttk.Label(drive, text="Tốc độ quay (rad/s)").grid(row=1, column=0, sticky="w")
        ttk.Scale(drive, from_=0.05, to=0.8, variable=self.yaw_var,
                  orient="horizontal").grid(row=1, column=1, sticky="ew", padx=8)
        ttk.Label(drive, textvariable=self._number_var(self.yaw_var, "%.2f")).grid(
            row=1, column=2, sticky="e")
        drive.columnconfigure(1, weight=1)

        pad = ttk.Frame(drive, padding=(0, 10, 0, 2))
        pad.grid(row=2, column=0, columnspan=3)
        self._direction_button(pad, "↑\nTiến  W", "up", 0, 1)
        self._direction_button(pad, "←\nQuay trái  A", "left", 1, 0)
        self._direction_button(pad, "↓\nLùi  S", "down", 1, 1)
        self._direction_button(pad, "→\nQuay phải  D", "right", 1, 2)
        ttk.Label(drive, text="Phím tắt W/A/S/D; nhả phím hoặc mất focus sẽ gửi lệnh dừng.").grid(
            row=3, column=0, columnspan=3, sticky="w", pady=(8, 0))

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
                          row: int, column: int) -> None:
        button = ttk.Button(parent, text=text, width=16)
        button.grid(row=row, column=column, padx=4, pady=3, ipadx=6, ipady=6)
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

    def _send(self, command: str) -> None:
        if not self.link.send(command):
            self._append_log("Không gửi được: chưa kết nối serial.")
            return
        self._append_log(f">> {command}")

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
        self.held.clear()
        self._send("STOP")

    def disable(self) -> None:
        self.held.clear()
        self.session_active = False
        self._send("DISABLE")

    def estop(self) -> None:
        self.held.clear()
        self.session_active = False
        self._send("ESTOP")

    def clear_estop(self) -> None:
        self.session_active = False
        self._send("CLEAR_ESTOP")

    def _hold(self, direction: str) -> None:
        if direction not in self.held:
            self.held.add(direction)
            self._publish_drive()

    def _release(self, direction: str) -> None:
        self.held.discard(direction)
        self._publish_drive()

    def _release_all(self) -> None:
        if self.held:
            self.held.clear()
            self._publish_drive()

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
        direction = {"w": "up", "s": "down", "a": "left", "d": "right"}.get(
            event.keysym.lower())
        if direction:
            self._hold(direction)

    def _key_release(self, event: tk.Event) -> None:
        if event.widget is getattr(self, "command_entry", None):
            return
        direction = {"w": "up", "s": "down", "a": "left", "d": "right"}.get(
            event.keysym.lower())
        if direction:
            self._release(direction)

    def _publish_drive(self) -> None:
        forward = self.speed_var.get() * (
            int("up" in self.held) - int("down" in self.held))
        yaw = self.yaw_var.get() * (
            int("right" in self.held) - int("left" in self.held))
        self._send(f"DRIVE {forward:.3f} {yaw:.3f}")

    def _tick(self) -> None:
        while True:
            try:
                line = self.incoming.get_nowait()
            except queue.Empty:
                break
            self._parse_status(line)
            self._append_log(line)

        now = time.monotonic()
        if self.link.connected and self.session_active and now - self.last_heartbeat >= HEARTBEAT_S:
            self.link.send("HEARTBEAT")
            self.last_heartbeat = now
        self.root.after(50, self._tick)

    def _parse_status(self, line: str) -> None:
        if match := MODE_RE.search(line):
            self.mode_var.set(match.group(1))
            self.enable_var.set("Bật" if match.group(2) == "1" else "Tắt")
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
                f"IMU pitch: {float(match.group(4)):+.3f} rad   "
                f"pitch rate: {float(match.group(5)):+.3f} rad/s")

    def _append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 500:
            self.log.delete("1.0", f"{lines - 500}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def close(self) -> None:
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
