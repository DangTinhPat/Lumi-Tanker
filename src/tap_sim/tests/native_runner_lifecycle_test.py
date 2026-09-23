#!/usr/bin/env python3
"""Black-box regression for native runner command transactions.

This deliberately exercises only the public line IPC.  It proves that the
runner stops, re-arms, retires a pose request and resets the physical model;
it does not use VIEW as controller input.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import selectors
import subprocess
import time


def fields(line: str):
    return {part.split("=", 1)[0]: part.split("=", 1)[1]
            for part in line.split("|")[1:] if "=" in part}


class Session:
    def __init__(self, runner: Path, model: Path):
        self.process = subprocess.Popen(
            # Wall-clock pacing is a GUI concern; this IPC transaction test
            # runs the deterministic native loop as fast as the host allows
            # so a loaded detailed CAD visual mesh cannot starve the timeout.
            [str(runner), "--model", str(model), "--rate", "3000",
             "--sensor-rate", "500", "--seed", "6814"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
            bufsize=1)
        assert self.process.stdout is not None
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)

    def send(self, command: str):
        assert self.process.stdin is not None
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()

    def read(self, timeout: float = 0.25):
        events = self.selector.select(timeout)
        if not events:
            return None
        line = self.process.stdout.readline()
        return line.rstrip("\n") if line else None

    def wait_state(self, predicate, timeout: float = 3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.read(min(0.25, max(0.0, deadline - time.monotonic())))
            if line is None:
                continue
            if line.startswith("STATE|"):
                state = fields(line)
                if predicate(state):
                    return state
        raise AssertionError("timed out waiting for native STATE transaction")

    def wait_view(self, timeout: float = 1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.read(min(0.25, max(0.0, deadline - time.monotonic())))
            if line and line.startswith("VIEW|"):
                return fields(line)
        raise AssertionError("timed out waiting for native VIEW")

    def close(self):
        if self.process.poll() is None:
            self.send("QUIT")
            self.process.wait(timeout=2.0)
        self.selector.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()

    session = Session(args.runner, args.model)
    try:
        session.wait_state(lambda _: True)  # READY is followed by STATE later.
        session.send("ENABLE")
        session.wait_state(lambda state: state.get("mode") == "Balance", 15.0)

        session.send("DISABLE")
        session.wait_state(lambda state: state.get("mode") == "EmergencyStop")
        session.send("ENABLE")
        session.wait_state(lambda state: state.get("mode") == "Balance" and
                           state.get("station_eligible") == "1" and
                           state.get("station_capture_ready") == "1" and
                           state.get("station_hold") == "1", 15.0)

        # The operator path is one task-space height transaction. Independent
        # four-angle POSE input is retained only as a low-level compatibility
        # boundary and is not the GUI posture control.
        session.send("HEIGHT 0.200")
        session.wait_state(lambda state: state.get("height_motion") == "1", 15.0)
        session.wait_state(lambda state: state.get("height_motion") == "0" and
                           state.get("mode") == "Balance", 15.0)

        session.send("RESET")
        reset_state = session.wait_state(lambda state: state.get("mode") == "Disabled")
        view = session.wait_view()
        assert abs(float(view["x"])) < 0.02
        assert abs(float(view["z"]) - 0.134000000) < 1.0e-6
        assert abs(float(view["lh"]) - 1.123132841) < 0.03
        assert abs(float(view["rh"]) - 1.121352471) < 0.03
        assert abs(float(view["lk"]) + 1.214584702) < 0.03
        assert abs(float(view["rk"]) + 1.213451540) < 0.03
        assert reset_state["ground_truth_to_controller"] == "0"
        print("PASS native lifecycle: DISABLE/ENABLE, automatic POSE completion, physical RESET")
        return 0
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
