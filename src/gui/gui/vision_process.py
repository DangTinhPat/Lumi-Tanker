"""Subprocess bridge from the standalone blue-target tracker to the dashboard."""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import queue
import subprocess
import sys
import threading
from typing import Optional


def calibrated_horizontal_fov(calibration_path: Path) -> float:
    """Load rectified horizontal FOV from the camera calibration archive."""
    import cv2

    with np.load(calibration_path) as calibration:
        if "horizontal_fov_rad" in calibration:
            fov = float(calibration["horizontal_fov_rad"])
        else:
            matrix = calibration["camera_matrix"]
            distortion = calibration["dist_coeffs"]
            width, height = (int(value) for value in calibration["image_size"])
            rectified, _ = cv2.getOptimalNewCameraMatrix(
                matrix, distortion, (width, height), 0.0, (width, height)
            )
            fov = 2.0 * math.atan(width / (2.0 * float(rectified[0, 0])))
    if not math.isfinite(fov) or not 0.0 < fov < math.pi:
        raise ValueError("camera calibration contains an invalid horizontal FOV")
    return fov


class BlueVisionProcess:
    """Own one tracker child and expose its latest JSONL measurement."""

    def __init__(self, vision_dir: Path):
        self.vision_dir = Path(vision_dir)
        self.process: Optional[subprocess.Popen[str]] = None
        self.records: queue.Queue[dict] = queue.Queue(maxsize=4)
        self.errors = deque(maxlen=20)
        self._threads: list[threading.Thread] = []

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self, source: str = "/dev/video4", show_window: bool = False) -> None:
        if self.running:
            return
        script = self.vision_dir / "track_black_square.py"
        if not script.is_file():
            raise RuntimeError(f"vision tracker is missing: {script}")
        self.records = queue.Queue(maxsize=4)
        self.errors = deque(maxlen=20)
        command = [sys.executable, str(script), "--source", source,
                   "--output-format", "jsonl"]
        if not show_window:
            command.append("--no-window")
        self.process = subprocess.Popen(
            command,
            cwd=self.vision_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        process = self.process
        self._threads = [
            threading.Thread(target=self._read_records,
                             args=(process, self.records, self.errors), daemon=True),
            threading.Thread(target=self._read_errors,
                             args=(process, self.errors), daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def read_latest(self) -> Optional[dict]:
        latest = None
        while True:
            try:
                latest = self.records.get_nowait()
            except queue.Empty:
                return latest

    def stop(self) -> None:
        process = self.process
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.process = None

    def _read_records(self, process, records, errors) -> None:
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            try:
                record = json.loads(line)
                if not isinstance(record, dict):
                    raise ValueError("JSONL record is not an object")
            except (json.JSONDecodeError, ValueError) as exc:
                errors.append(f"invalid vision JSONL: {exc}")
                continue
            try:
                records.put_nowait(record)
            except queue.Full:
                try:
                    records.get_nowait()
                except queue.Empty:
                    pass
                records.put_nowait(record)

    def _read_errors(self, process, errors) -> None:
        if process is None or process.stderr is None:
            return
        for line in process.stderr:
            message = line.strip()
            if message:
                errors.append(message)
