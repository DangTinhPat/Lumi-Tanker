"""Motion backend that forwards commands to controller_console's FIFO."""

from __future__ import annotations

import math
import os
from pathlib import Path
import stat


class FifoMotionBackend:
    """Write DRIVE/STOP lines to an already-running controller console FIFO.

    This backend never opens the ESP32 serial port. The serial console owns that
    port and forwards FIFO lines to firmware, avoiding a second open that could
    reset the ESP32 while it is operating.
    """

    def __init__(self, fifo_path: Path):
        self.fifo_path = Path(fifo_path)
        try:
            mode = self.fifo_path.stat().st_mode
        except OSError as exc:
            raise RuntimeError(f"cannot access controller FIFO: {self.fifo_path}") from exc
        if not stat.S_ISFIFO(mode):
            raise ValueError(f"controller path is not a FIFO: {self.fifo_path}")

    def command_drive(self, velocity: float, yaw_rate: float) -> None:
        if not (math.isfinite(velocity) and math.isfinite(yaw_rate)):
            self.command_stop()
            return
        self._write(f"DRIVE {velocity:.7g} {yaw_rate:.7g}")

    def command_stop(self) -> None:
        self._write("STOP")

    def _write(self, command: str) -> None:
        descriptor = os.open(self.fifo_path, os.O_WRONLY | os.O_NONBLOCK)
        try:
            os.write(descriptor, (command + "\n").encode("ascii"))
        finally:
            os.close(descriptor)
