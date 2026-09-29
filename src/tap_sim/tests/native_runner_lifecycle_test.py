#!/usr/bin/env python3
"""Black-box regression for V3.1 lifecycle and public line IPC."""

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
    def __init__(self, runner: Path, model: Path,
                 keyframe: str = "v3_hang"):
        self.process = subprocess.Popen(
            # Wall-clock pacing is a GUI concern; this IPC transaction test
            # runs the deterministic native loop as fast as the host allows
            # so a loaded detailed CAD visual mesh cannot starve the timeout.
            [str(runner), "--model", str(model), "--keyframe", keyframe,
             "--rate", "3000",
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

    def wait_event(self, prefix: str, timeout: float = 5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.read(min(0.25, max(0.0, deadline - time.monotonic())))
            if line and line.startswith(prefix):
                return line
            if line and line.startswith("STATE|"):
                assert fields(line).get("mode") != "Balance", line
        raise AssertionError(f"timed out waiting for {prefix}")

    def close(self):
        if self.process.poll() is None:
            self.send("QUIT")
            deadline = time.monotonic() + 5.0
            while self.process.poll() is None and time.monotonic() < deadline:
                self.read(0.1)  # Drain telemetry so the child cannot block on stdout.
            if self.process.poll() is None:
                self.process.terminate()
                self.process.wait(timeout=1.0)
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
        session.wait_state(lambda state: state.get("mode") == "Disabled")
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
        assert abs(float(view["z"]) - 0.2285603142) < 1.0e-6
        assert abs(float(view["lh"]) - 0.3882076041) < 0.03
        assert abs(float(view["rh"]) - 0.3882076041) < 0.03
        assert abs(float(view["lk"])) < 0.03
        assert abs(float(view["rk"])) < 0.03
        assert reset_state["ground_truth_to_controller"] == "0"

        home = subprocess.run(
            [str(args.runner), "--model", str(args.model), "--rate", "3000",
             "--sensor-rate", "500",
             "--run-for", "0.5"],
            input="ENABLE\n", check=True, capture_output=True, text=True)
        assert "ERROR|v3_start_rejected=legs_not_at_hang_zero" in home.stdout
        assert not any(fields(line).get("mode") == "Balance"
                       for line in home.stdout.splitlines()
                       if line.startswith("STATE|"))
        view = next(fields(line) for line in home.stdout.splitlines()
                    if line.startswith("VIEW|"))
        assert abs(float(view["z"]) - 0.0698965) < 1.0e-4
        assert abs(float(view["qy"]) - 0.0000332149) < 1.0e-4
        settled = next(fields(line) for line in home.stdout.splitlines()
                       if line.startswith("STATE|") and
                       float(fields(line)["t"]) >= 0.02)
        for channel in ("enc_lh", "enc_lk", "enc_lw",
                        "enc_rh", "enc_rk", "enc_rw"):
            assert abs(float(settled[channel])) < 0.03, (channel, settled[channel])
        # VIEW telemetry is formatted to six significant digits.
        assert abs(float(view["lh"]) - 2.27392) < 1.0e-5
        assert abs(float(view["lk"]) + 1.2) < 1.0e-5
        assert abs(float(view["rh"]) - 2.27392) < 1.0e-5
        assert abs(float(view["rk"]) + 1.2) < 1.0e-5

        # Rocking stage 1 from the prone homing pose: hips fold slowly to 1.5,
        # fling down and lock at 1.2 (pivot) - knees held, no V3.1 handoff.
        prone = Session(args.runner, args.model, "power_off_collapse_current")
        try:
            prone.wait_state(lambda state: state.get("imu_seeded") == "1" and
                             state.get("imu_fresh") == "1" and
                             state.get("enc_fresh") == "1", 5.0)
            prone.send("STAND_UP")
            fling = fields(prone.wait_event("STAND_UP_STAGE|name=fling", 8.0))
            assert float(fling["n_to_p_mm"]) < 65.0, fling
            assert 1.45 <= float(fling["hip"]) < 1.65, fling
            pivot = fields(prone.wait_event("STAND_UP_STAGE|name=pivot", 2.0))
            assert abs(float(pivot["hip"]) - 1.2) < 1e-4, pivot
            held = prone.wait_state(lambda state: True, 2.0)
            assert abs(float(held["joint_lk"]) + 1.2) < 0.08, held
            assert held["sim_contact_thigh"] == "0" and held["sim_contact_shin"] == "0", held
        finally:
            prone.close()
        print("PASS V3.1 lifecycle and rocking stage 1: lower, fling, pivot; "
              "knees held, no leg-floor contact")
        return 0
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
