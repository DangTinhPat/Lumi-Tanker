"""Process and IPC boundary for the native MuJoCo runner."""

from __future__ import annotations

from collections import deque
import math
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
from typing import Dict, Optional


def workspace_root() -> Path:
    here = Path(__file__).resolve()
    for candidate in (here, *here.parents):
        if (candidate / "AGENTS.md").is_file() and (candidate / "controller").is_dir():
            return candidate
    raise RuntimeError("cannot locate TAP workspace")


class NativeBackend:
    """Slow UI client; the native child owns the kHz control loop."""

    def __init__(self, root: Optional[Path] = None, rate_hz: int = 3000):
        self.root = Path(root or workspace_root())
        self.rate_hz = int(rate_hz)
        self.process: Optional[subprocess.Popen[str]] = None
        self.lines: queue.Queue[str] = queue.Queue(maxsize=256)
        self.telemetry: Dict[str, str] = {}
        # TAP_SIM_LOG=<file>: append the telemetry (STATE) and the commands sent, for offline analysis.
        log_path = os.environ.get("TAP_SIM_LOG")
        self._sim_log = open(log_path, "a", buffering=1) if log_path else None
        self.telemetry_received_monotonic_s = 0.0
        self.view: Dict[str, str] = {}
        self.log = deque(maxlen=200)
        self._reader: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._viewer_process: Optional[subprocess.Popen[str]] = None
        self._command_socket: Optional[socket.socket] = None
        self._command_port = 37681
        self._ready = threading.Event()

    def runner_path(self) -> Path:
        candidates = [
            self.root / "build/simulation/bin/tap_mujoco_gim_v3",
            self.root / "build/native/bin/tap_mujoco_gim_v3",
            self.root / "build/bin/tap_mujoco_gim_v3",
            self.root / "build/tap_mujoco_gim_v3/tap_mujoco_gim_v3",
        ]
        for path in candidates:
            if path.is_file() and path.stat().st_mode & 0o111:
                return path
        raise RuntimeError("GIM V3 runner is not built; build tap_mujoco_gim_v3 first")

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    @property
    def viewer_running(self) -> bool:
        return (self._viewer_process is not None and
                self._viewer_process.poll() is None)

    def start(self) -> None:
        with self._lock:
            if self.running:
                return
            runner = self.runner_path()
            # TAP_SIM_MODEL=<xml>: another model (e.g. the rough terrain, tools/build_rough_terrain.py)
            model = Path(os.environ.get("TAP_SIM_MODEL") or self.root / "simulation/model/gim_dynamic.xml")
            if not model.is_absolute():
                model = self.root / model
            if not model.is_file():
                raise RuntimeError(f"GIM dynamic model is missing: {model}")
            self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._ready.clear()
            command = [str(runner), "--model", str(model), "--rate", str(self.rate_hz),
                       "--command-port", str(self._command_port), "--realtime"]
            # v3.5 jump tuning override for trials: TAP_JUMP_SPEC="key=val,key=val,..."
            # (keys: applyJumpSpec in gim_v3_runner.cpp).
            if os.environ.get("TAP_JUMP_SPEC"):
                command += ["--jump", os.environ["TAP_JUMP_SPEC"]]
            # Roll balance is on in the dashboard sim (TAP_SIM_ROLL=0 turns it off); the headless tests keep it off.
            command += ["--roll", "0" if os.environ.get("TAP_SIM_ROLL") == "0" else "1"]
            # TAP_SIM_START=<keyframe>: start in another MJCF keyframe, e.g. "home" (the lying pose, shown frozen:
            # the V3 start-up sequence only knows the extended spawn pose)
            if os.environ.get("TAP_SIM_START"):
                command += ["--start-key", os.environ["TAP_SIM_START"]]
            # TAP_SIM_ARGS="--standup-set lift=1": extra runner options for A/B trials (shell-split).
            if os.environ.get("TAP_SIM_ARGS"):
                import shlex
                command += shlex.split(os.environ["TAP_SIM_ARGS"])
            self.process = subprocess.Popen(
                command,
                cwd=self.root, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1)
            self._reader = threading.Thread(target=self._read_output, daemon=True)
            self._reader.start()
            try:
                if not self._ready.wait(timeout=2.0):
                    raise RuntimeError("native runner did not open command transport")
                self._open_mujoco_viewer(model)
            except Exception:
                self.stop()
                raise

    def _open_mujoco_viewer(self, model_path: Path) -> None:
        """Open the real MuJoCo 3-D renderer used by the native model.

        The C++ child remains the sole physics/control owner.  This Python
        viewer loads the identical MJCF and mirrors only the debug VIEW
        snapshot, so no viewer pose/contact data can enter RobotState or CMF.
        """
        viewer_script = self.root / "gui/tap_gui/native_viewer.py"
        if not viewer_script.is_file():
            raise RuntimeError(f"native MuJoCo viewer is missing: {viewer_script}")
        self._viewer_process = subprocess.Popen(
            [sys.executable, str(viewer_script), "--model", str(model_path)],
            cwd=self.root, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT, text=True, bufsize=1)
        os.set_blocking(self._viewer_process.stdin.fileno(), False)

    def _sync_mujoco_viewer(self) -> None:
        if not self.viewer_running or self._viewer_process.stdin is None:
            return
        try:
            line = "VIEW|" + "|".join(
                f"{key}={value}" for key, value in self.view.items()) + "\n"
            packet = line.encode("ascii")
            # Atomic nonblocking snapshots: a busy renderer must not stall GUI input.
            if len(packet) <= 4096:
                os.write(self._viewer_process.stdin.fileno(), packet)
        except (BrokenPipeError, OSError):
            # A busy or closed viewer only drops a debug frame.
            return

    def _close_mujoco_viewer(self) -> None:
        process = self._viewer_process
        if process is None:
            return
        try:
            if process.stdin is not None:
                process.stdin.close()
            process.wait(timeout=2.0)
        except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        finally:
            self._viewer_process = None

    def _read_output(self) -> None:
        process = self.process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            if line.startswith("READY|"):
                self._ready.set()
            line = line.rstrip("\n")
            try:
                self.lines.put_nowait(line)
            except queue.Full:
                try:
                    self.lines.get_nowait()
                except queue.Empty:
                    pass
                self.lines.put_nowait(line)

    @staticmethod
    def _parse(line: str) -> Dict[str, str]:
        return {part.split("=", 1)[0]: part.split("=", 1)[1]
                for part in line.split("|")[1:] if "=" in part}

    def drain(self) -> int:
        count = 0
        view_changed = False
        for _ in range(self.lines.qsize()):
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                break
            count += 1
            if line.startswith("STATE|"):
                if self._sim_log is not None:
                    self._sim_log.write(line + "\n")
                self.telemetry = self._parse(line)
                self.telemetry_received_monotonic_s = time.monotonic()
            elif line.startswith("VIEW|"):
                self.view = self._parse(line)
                view_changed = True
            else:
                self.log.append(line)
        if view_changed:
            self._sync_mujoco_viewer()
        return count

    def send(self, command: str) -> None:
        if self._sim_log is not None:
            self._sim_log.write(f"CMD|{command}|t={self.telemetry.get('t', '')}\n")
        process = self.process
        if process is None or process.poll() is not None:
            return
        try:
            packet = (command.rstrip("\n") + "\n").encode("ascii")
            if self._command_socket is not None:
                # One datagram is one BLE-like command write; no controller
                # response is required on this path.
                self._command_socket.sendto(
                    packet, ("127.0.0.1", self._command_port))
            elif process.stdin is not None:
                process.stdin.write(packet.decode("ascii"))
                process.stdin.flush()
        except (BrokenPipeError, OSError):
            pass

    def stop(self) -> None:
        self._close_mujoco_viewer()
        process = self.process
        if process is None:
            return
        self.send("QUIT")
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.process = None
        if self._command_socket is not None:
            self._command_socket.close()
            self._command_socket = None

    def command_drive(self, velocity: float, yaw_rate: float) -> None:
        if math.isfinite(velocity) and math.isfinite(yaw_rate):
            self.send(f"DRIVE {velocity:.7g} {yaw_rate:.7g}")

    def command_stop(self) -> None:
        self.send("STOP")

    def command_pose(self, hip: float, knee: float,
                     right_hip: Optional[float] = None,
                     right_knee: Optional[float] = None) -> None:
        if right_hip is None:
            right_hip = hip
        if right_knee is None:
            right_knee = knee
        values = (hip, knee, right_hip, right_knee)
        if all(math.isfinite(value) for value in values):
            self.send("POSE " + " ".join(f"{value:.7g}" for value in values))

    def command_height(self, body_height_m: float) -> None:
        """Request a symmetric FK height target; C++ owns the inverse FK."""
        if math.isfinite(body_height_m):
            self.send(f"HEIGHT {body_height_m:.7g}")

    @property
    def starts_lying(self) -> bool:
        """The sim starts in another keyframe (TAP_SIM_START, e.g. "home"): ENABLE is not the way to start it."""
        return bool(os.environ.get("TAP_SIM_START"))

    def command_standup(self) -> None:
        """Start autonomous fold/pause/flip/capture from TAP_SIM_START=home."""
        self.send("STANDUP")

    def command_standup_flip(self) -> None:
        """Start the pitch-to-level phase from the held kneeling pose."""
        self.send("FLIP")

    def command_jump(self, forward_m: float = 0.0) -> None:
        """v3.5 jump: vertical, or forward over forward_m metres (run-up, jump, land, stop)."""
        # "JUMP 0" is the vertical jump ("JUMP" alone is the default forward hop)
        if math.isfinite(forward_m) and forward_m > 0.0:
            self.send(f"JUMP {forward_m:.3f}")
        else:
            self.send("JUMP 0")

    def command_dance(self, direction: float = 1.0) -> None:
        if math.isfinite(direction):
            self.send(f"DANCE START {'-1' if direction < 0 else '1'}")

    def command_dance_stop(self) -> None:
        self.send("DANCE STOP")
