#!/usr/bin/env python3
"""Regression for an early 0.3 m/s drive followed by a rapid stop."""

from __future__ import annotations

import argparse
from pathlib import Path
import selectors
import subprocess
import time


def fields(line: str):
    return {part.split("=", 1)[0]: part.split("=", 1)[1]
            for part in line.split("|")[1:] if "=" in part}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()

    process = subprocess.Popen(
        [str(args.runner), "--model", str(args.model), "--rate", "3000",
         "--sensor-rate", "500", "--seed", "6814", "--realtime"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)
    assert process.stdin is not None and process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)

    def send(command: str) -> None:
        process.stdin.write(command + "\n")
        process.stdin.flush()

    early_rejected = False
    early_attempted = False
    start = None
    stop = None
    peak_pitch = 0.0
    cruise_wheel_speeds = []
    final_state = None
    try:
        deadline = time.monotonic() + 18.0
        while time.monotonic() < deadline:
            if not selector.select(0.5):
                continue
            line = process.stdout.readline().rstrip("\n")
            if line.startswith("READY|"):
                send("ENABLE")
            elif line.startswith("DRIVE_REJECTED|") and "station_not_ready" in line:
                early_rejected = True
            elif line.startswith("STATE|"):
                state = fields(line)
                final_state = state
                sim_time = float(state["t"])
                peak_pitch = max(peak_pitch, abs(float(state["pitch"])))
                if (start is None and not early_attempted and
                        state.get("mode") == "Balance"):
                    # This is deliberately too early; the runner must refuse it.
                    send("DRIVE 0.3 0")
                    early_attempted = True
                if (start is None and sim_time >= 2.0 and
                        state.get("mode") == "Balance" and
                        state.get("station_hold") == "1" and
                        state.get("station_capture_ready") == "1"):
                    start = sim_time
                    send("DRIVE 0.3 0")
                elif start is not None and stop is None:
                    if state.get("mode") == "Fallen":
                        raise AssertionError(
                            f"0.3 m/s cruise fell at t={sim_time:.3f}s")
                    if sim_time >= start + 1.5:
                        cruise_wheel_speeds.append(float(state["wheel_velocity"]))
                    if sim_time < start + 3.0:
                        continue
                    stop = sim_time
                    send("DRIVE 0 0")
                if final_state.get("mode") == "Fallen":
                    raise AssertionError(
                        f"rapid 0.3 m/s stop fell at t={sim_time:.3f}s")
                if stop is not None and sim_time >= stop + 7.0:
                    break
    finally:
        if process.poll() is None:
            try:
                send("QUIT")
            except BrokenPipeError:
                pass
            process.wait(timeout=4.0)
        selector.close()

    assert early_rejected, "early DRIVE was not rejected before station capture"
    assert start is not None and stop is not None, "rapid drive/stop was not exercised"
    assert final_state is not None and final_state.get("mode") == "Balance"
    assert peak_pitch < 0.15, f"rapid stop peak pitch was {peak_pitch:.4f} rad"
    assert cruise_wheel_speeds, "0.3 m/s cruise window was not exercised"
    assert max(cruise_wheel_speeds) - min(cruise_wheel_speeds) < 0.12, (
        "0.3 m/s wheel speed surged instead of settling: "
        f"{min(cruise_wheel_speeds):.3f}..{max(cruise_wheel_speeds):.3f} m/s")
    cruise_mean = sum(cruise_wheel_speeds) / len(cruise_wheel_speeds)
    assert 0.22 < cruise_mean < 0.36, (
        f"0.3 m/s cruise mean was {cruise_mean:.3f} m/s")
    assert final_state.get("station_hold") == "1"
    print(f"PASS drive guard + 0.3 cruise/rapid stop: "
          f"mean={cruise_mean:.3f} m/s peak_pitch={peak_pitch:.4f} rad")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
