#!/usr/bin/env python3
"""Closed-loop native regression for drive -> adaptive stop -> station hold.

The test uses only the controller-facing STATE stream.  VIEW/debug pose data
is intentionally ignored, so this checks the same sensor-only boundary used
by the native controller.
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


def run_trial(runner: Path, model: Path, drive_m_s: float):
    process = subprocess.Popen(
        [str(runner), "--model", str(model), "--rate", "3000",
         "--sensor-rate", "500", "--realtime", "--seed", "6814"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    assert process.stdin is not None
    assert process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)

    def send(command: str):
        process.stdin.write(command + "\n")
        process.stdin.flush()

    states = []
    drove = False
    stopped = False
    stop_position = None
    deadline = time.monotonic() + 20.0
    try:
        while time.monotonic() < deadline:
            events = selector.select(0.5)
            if not events:
                continue
            line = process.stdout.readline().rstrip("\n")
            if line.startswith("READY|"):
                send("ENABLE")
            elif line.startswith("STATE|"):
                state = fields(line)
                states.append(state)
                sim_time = float(state["t"])
                if state.get("mode") == "Balance" and sim_time > 2.0 and not drove:
                    send(f"DRIVE {drive_m_s:.6g} 0")
                    drove = True
                if drove and sim_time > 6.0 and not stopped:
                    stop_position = float(state["wheel_position"])
                    send("DRIVE 0 0")
                    stopped = True
                if stopped and sim_time >= 14.0:
                    break
        assert stopped, "stop command was not issued"
        assert states and float(states[-1]["t"]) >= 14.0
        after_stop = [state for state in states if float(state["t"]) >= 6.0]
        assert all(state["mode"] != "Fallen" for state in after_stop)
        assert max(abs(float(state["pitch"])) for state in after_stop) < 0.20
        assert any(state["station_hold"] == "1" for state in after_stop)
        assert stop_position is not None
        direction = 1.0 if drive_m_s > 0.0 else -1.0
        signed_final_displacement = direction * (
            float(after_stop[-1]["wheel_position"]) - stop_position)
        signed_peak_displacement = max(
            direction * (float(state["wheel_position"]) - stop_position)
            for state in after_stop)
        # Releasing DRIVE may coast past the release point, but it must not
        # settle behind it: that is the visible "return to the old place"
        # regression this test is meant to catch.
        assert signed_final_displacement > -0.05, (
            f"stop returned behind release point: {signed_final_displacement:.3f} m")
        assert signed_peak_displacement < 0.35, (
            f"stop coasted too far: {signed_peak_displacement:.3f} m")
        return states, signed_final_displacement, signed_peak_displacement
    finally:
        if process.poll() is None:
            try:
                send("QUIT")
            except BrokenPipeError:
                pass
            process.wait(timeout=3.0)
        selector.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()

    summaries = []
    # Include reverse motion explicitly: the stop edge must brake a negative
    # wheel/body momentum without turning it into a backward fall.
    for drive_m_s in (0.15, -0.30, 0.30):
        states, final_displacement, peak_displacement = run_trial(
            args.runner, args.model, drive_m_s)
        after_stop = [state for state in states if float(state["t"]) >= 6.0]
        summaries.append((drive_m_s, final_displacement, peak_displacement,
                          max(abs(float(state["pitch"])) for state in after_stop),
                          next(state["t"] for state in after_stop
                               if state["station_hold"] == "1")))
    print("PASS native adaptive stop: " + ", ".join(
        f"drive={speed:.2f}m/s final_delta={final_delta:.3f}m "
        f"peak_delta={peak_delta:.3f}m max_pitch={pitch:.3f}rad hold_at={hold}s"
        for speed, final_delta, peak_delta, pitch, hold in summaries))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
