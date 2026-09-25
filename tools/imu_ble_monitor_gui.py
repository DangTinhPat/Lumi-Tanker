#!/usr/bin/env python3
"""Live Serial monitor for the TAP MPU6500/filter/Dabble bring-up image.

This GUI is read-only: it never writes commands to the serial port and never
touches CAN/UART motor transports.
"""

from __future__ import annotations

import argparse
from collections import deque
import math
import queue
import re
import threading
import tkinter as tk
import time
from tkinter import ttk

import serial
import serial.tools.list_ports


IMU_RE = re.compile(
    r"IMU\s+ax=(?P<ax>[+-]?\d+(?:\.\d+)?)\s+"
    r"ay=(?P<ay>[+-]?\d+(?:\.\d+)?)\s+"
    r"az=(?P<az>[+-]?\d+(?:\.\d+)?)\s+"
    r"gx=(?P<gx>[+-]?\d+(?:\.\d+)?)\s+"
    r"gy=(?P<gy>[+-]?\d+(?:\.\d+)?)\s+"
    r"gz=(?P<gz>[+-]?\d+(?:\.\d+)?)\s+"
    r"seed=(?P<seed>[01])\s+pitch=(?P<pitch>[+-]?\d+(?:\.\d+)?)\s+"
    r"rate=(?P<rate>[+-]?\d+(?:\.\d+)?)\s+"
    r"accel_valid=(?P<valid>[01])\s+trust=(?P<trust>[+-]?\d+(?:\.\d+)?)"
)
WHO_RE = re.compile(r"WHO_AM_I=0x([0-9a-fA-F]+)\s+\((PASS|FAIL)\)")
CTRL_RE = re.compile(r"CTRL\s+mode=(?P<mode>[A-Za-z]+)\s+enable=(?P<enable>[01])")
GAMEPAD_RE = re.compile(
    r"GAMEPAD\s+" +
    r"\s+".join(f"{name}=(?P<{name}>[01])" for name in (
        "up", "down", "left", "right", "start", "select",
        "triangle", "circle", "cross", "square"))
)


BUTTON_LABELS = {
    "up": ("UP", "FORWARD"),
    "down": ("DOWN", "REVERSE"),
    "left": ("LEFT", "YAW LEFT"),
    "right": ("RIGHT", "YAW RIGHT"),
    "start": ("START", "ENABLE"),
    "select": ("SELECT", "DISABLE"),
    "triangle": ("TRIANGLE", "MAX STAND"),
    "circle": ("CIRCLE", "DEEP SQUAT"),
    "cross": ("CROSS", "NATURAL"),
    "square": ("SQUARE", "ESTOP"),
}


class MonitorGui:
    def __init__(self, root: tk.Tk, port: str, baud: int) -> None:
        self.root = root
        self.port_name = port
        self.baud = baud
        self.serial_port: serial.Serial | None = None
        self.stop_event = threading.Event()
        self.lines: queue.Queue[str] = queue.Queue()
        self.stats = {
            "samples": 0,
            "valid": 0,
            "trust_sum": 0.0,
            "ble_events": 0,
            "gamepad_events": 0,
            "first_sample_s": 0.0,
            "last_sample_s": 0.0,
        }
        self.history = deque(maxlen=300)
        self.charts: dict[str, tuple[tk.Canvas, float, float, str, str]] = {}
        self.root.title("TAP IMU + Filter + Dabble Monitor")
        self.root.geometry("1050x820")

        self.status = {
            key: tk.StringVar(value=value)
            for key, value in {
                "imu": "unknown", "ble": "disconnected", "seed": "no",
                "pitch": "--", "rate": "--", "accel": "--", "trust": "--",
                "controller": "unknown",
            }.items()
        }
        self.buttons: dict[str, tk.Label] = {}
        self._build_ui()
        self.root.after(50, self._drain_lines)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.connect()

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Serial port:").pack(side="left")
        self.port_var = tk.StringVar(value=self.port_name)
        ttk.Entry(top, textvariable=self.port_var, width=20).pack(side="left", padx=5)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        ttk.Button(top, text="Connect", command=self.connect).pack(side="left", padx=5)
        self.connection_label = ttk.Label(top, text="offline")
        self.connection_label.pack(side="left", padx=8)

        metrics = ttk.LabelFrame(self.root, text="Telemetry", padding=8)
        metrics.pack(fill="x", padx=8, pady=(0, 8))
        for column, (key, label) in enumerate((
            ("imu", "IMU"), ("ble", "BLE"), ("seed", "Filter seed"),
            ("pitch", "Pitch"), ("rate", "Pitch rate"),
            ("accel", "Accel |a|"), ("trust", "Accel trust"),
            ("controller", "Controller"))):
            cell = ttk.Frame(metrics)
            cell.grid(row=0, column=column, padx=7)
            ttk.Label(cell, text=label).pack()
            ttk.Label(cell, textvariable=self.status[key], width=12,
                      anchor="center").pack()
        self.performance = tk.StringVar(value="samples=0 | log rate=-- Hz | valid=-- | trust=-- | BLE events=0 | GamePad events=0")
        ttk.Label(metrics, textvariable=self.performance, anchor="w").grid(
            row=1, column=0, columnspan=8, sticky="ew", padx=7, pady=(7, 0))

        gamepad = ttk.LabelFrame(self.root, text="Dabble GamePad (read-only)", padding=8)
        gamepad.pack(fill="x", padx=8, pady=(0, 8))
        for column, key in enumerate(("up", "down", "left", "right")):
            self._button_cell(gamepad, key, 0, column)
        for column, key in enumerate(("start", "select", "triangle", "circle", "cross", "square")):
            self._button_cell(gamepad, key, 1, column)

        charts = ttk.LabelFrame(self.root, text="Live filter plots (last 300 reports)", padding=5)
        charts.pack(fill="x", padx=8, pady=(0, 8))
        for index, (key, lower, upper, color, unit) in enumerate((
            ("pitch", -0.30, 0.30, "#55c2ff", "rad"),
            ("rate", -2.00, 2.00, "#ffb454", "rad/s"),
            ("accel", 8.00, 12.00, "#7ee787", "m/s²"),
            ("trust", 0.00, 1.00, "#d2a8ff", ""),
        )):
            canvas = tk.Canvas(charts, width=460, height=145, bg="#111820",
                               highlightthickness=1, highlightbackground="#48515c")
            canvas.grid(row=index // 2, column=index % 2, padx=4, pady=4, sticky="ew")
            self.charts[key] = (canvas, lower, upper, color, unit)
        charts.columnconfigure(0, weight=1)
        charts.columnconfigure(1, weight=1)
        self.root.after(100, self._draw_charts)

        log_frame = ttk.LabelFrame(self.root, text="Serial log", padding=5)
        log_frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.log = tk.Text(log_frame, height=7, state="disabled", wrap="none")
        self.log.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(log_frame, command=self.log.yview)
        scrollbar.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=scrollbar.set)

    def _button_cell(self, parent: ttk.Frame, key: str, row: int, column: int) -> None:
        label, _ = BUTTON_LABELS[key]
        widget = tk.Label(parent, text=label, width=12, height=2,
                          bg="#343434", fg="white", relief="groove")
        widget.grid(row=row, column=column, padx=4, pady=3, sticky="ew")
        self.buttons[key] = widget

    def refresh_ports(self) -> None:
        ports = [port.device for port in serial.tools.list_ports.comports()]
        if "/dev/ttyUSB0" in ports:
            self.port_var.set("/dev/ttyUSB0")
        elif ports:
            self.port_var.set(ports[0])
        self._append_log("ports: " + ", ".join(ports or ["none"]))

    def connect(self) -> None:
        self.disconnect()
        try:
            self.serial_port = serial.Serial(self.port_var.get(), self.baud, timeout=0.2)
            self.connection_label.configure(text="connected")
            self._append_log(f"connected to {self.port_var.get()} @ {self.baud}")
            threading.Thread(target=self._reader, daemon=True).start()
        except serial.SerialException as exc:
            self.connection_label.configure(text="offline")
            self._append_log(f"serial error: {exc}")

    def disconnect(self) -> None:
        port, self.serial_port = self.serial_port, None
        if port is not None:
            port.close()
        self.connection_label.configure(text="offline")

    def _reader(self) -> None:
        port = self.serial_port
        if port is None:
            return
        while not self.stop_event.is_set() and port is self.serial_port:
            try:
                line = port.readline()
            except (serial.SerialException, OSError, TypeError) as exc:
                self.lines.put(f"serial error: {exc}")
                return
            if line:
                self.lines.put(line.decode("utf-8", errors="replace").rstrip())

    def _drain_lines(self) -> None:
        while True:
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                break
            self._append_log(line)
            self._parse_line(line)
        self.root.after(50, self._drain_lines)

    def _parse_line(self, line: str) -> None:
        if match := WHO_RE.search(line):
            self.status["imu"].set("PASS 0x" + match.group(1).upper())
        if "BLE connected" in line:
            self.status["ble"].set("connected")
            self.stats["ble_events"] += 1
        elif "BLE disconnected" in line:
            self.status["ble"].set("disconnected")
            self.stats["ble_events"] += 1
        if match := CTRL_RE.search(line):
            state = "enable" if match.group("enable") == "1" else "safe"
            self.status["controller"].set(f"{match.group('mode')} / {state}")
        if match := IMU_RE.search(line):
            now = time.monotonic()
            self.stats["samples"] += 1
            if self.stats["first_sample_s"] == 0.0:
                self.stats["first_sample_s"] = now
            self.stats["last_sample_s"] = now
            if match.group("valid") == "1":
                self.stats["valid"] += 1
            trust = float(match.group("trust"))
            self.stats["trust_sum"] += trust
            accel_magnitude = math.sqrt(sum(
                float(match.group(axis)) ** 2 for axis in ("ax", "ay", "az")))
            self.status["seed"].set("yes" if match.group("seed") == "1" else "no")
            self.status["pitch"].set(match.group("pitch") + " rad")
            self.status["rate"].set(match.group("rate") + " rad/s")
            self.status["accel"].set(f"{accel_magnitude:.2f} m/s²")
            self.status["trust"].set(f"{trust:.2f} ({'ok' if match.group('valid') == '1' else 'bad'})")
            self.history.append((
                float(match.group("pitch")),
                float(match.group("rate")),
                accel_magnitude,
                trust,
            ))
            count = self.stats["samples"]
            elapsed = self.stats["last_sample_s"] - self.stats["first_sample_s"]
            rate = (count - 1) / elapsed if elapsed > 0.0 and count > 1 else 0.0
            valid_ratio = 100.0 * self.stats["valid"] / count
            trust_mean = self.stats["trust_sum"] / count
            self.performance.set(
                f"samples={count} | log rate={rate:.1f} Hz | valid={valid_ratio:.1f}% "
                f"| trust={trust_mean:.2f} | BLE events={self.stats['ble_events']} "
                f"| GamePad events={self.stats['gamepad_events']}"
            )
        if match := GAMEPAD_RE.search(line):
            self.stats["gamepad_events"] += 1
            for key, widget in self.buttons.items():
                active = match.group(key) == "1"
                widget.configure(bg="#17823b" if active else "#343434")

    def _draw_charts(self) -> None:
        values = {
            "pitch": 0,
            "rate": 1,
            "accel": 2,
            "trust": 3,
        }
        samples = list(self.history)
        for key, (canvas, lower, upper, color, unit) in self.charts.items():
            canvas.delete("all")
            width = max(canvas.winfo_width(), 460)
            height = max(canvas.winfo_height(), 145)
            left, top, right, bottom = 42, 22, 8, 20
            plot_width = width - left - right
            plot_height = height - top - bottom
            canvas.create_text(left, 9, anchor="w", fill="#d7dee8",
                               text=f"{key.title()} {unit}".rstrip())
            for fraction in (0.0, 0.5, 1.0):
                y = top + fraction * plot_height
                value = upper - fraction * (upper - lower)
                canvas.create_line(left, y, width - right, y, fill="#28333e")
                canvas.create_text(left - 5, y, anchor="e", fill="#8391a2",
                                   text=f"{value:.2g}")
            if lower < 0.0 < upper:
                zero_y = top + (upper / (upper - lower)) * plot_height
                canvas.create_line(left, zero_y, width - right, zero_y,
                                   fill="#596675", dash=(3, 3))
            if samples:
                points = []
                denominator = max(len(samples) - 1, 1)
                for index, sample in enumerate(samples):
                    value = max(lower, min(upper, sample[values[key]]))
                    x = left + index * plot_width / denominator
                    y = top + (upper - value) / (upper - lower) * plot_height
                    points.extend((x, y))
                if len(points) >= 4:
                    canvas.create_line(*points, fill=color, width=2)
        self.root.after(100, self._draw_charts)

    def _append_log(self, line: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.configure(state="disabled")
        self.log.see("end")

    def close(self) -> None:
        self.stop_event.set()
        self.disconnect()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    root = tk.Tk()
    MonitorGui(root, args.port, args.baud)
    root.mainloop()


if __name__ == "__main__":
    main()
