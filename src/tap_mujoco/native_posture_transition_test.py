#!/usr/bin/env python3
"""Closed-loop regression for chained native height transactions.

The test deliberately sends the next HEIGHT command on the first telemetry
sample after the previous transaction retires.  That is the operator race that
used to bypass the smooth path and make deep -> squat fall.  VIEW is collected
only for offline drift scoring; it is never sent to the controller.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import selectors
import subprocess


NATURAL = 0.1484350375
SQUAT_25 = 0.1463262781
DEEP = 0.140


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
         "--sensor-rate", "500", "--seed", "6814"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)
    assert process.stdin is not None and process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)

    def send(command: str) -> None:
        process.stdin.write(command + "\n")
        process.stdin.flush()

    targets = [SQUAT_25, DEEP, SQUAT_25, NATURAL]
    next_target = 0
    active_start = None
    active_target = None
    transitions = []
    states = []
    views = []
    try:
        while next_target < len(targets):
            events = selector.select(1.0)
            if not events:
                continue
            line = process.stdout.readline().rstrip("\n")
            if not line:
                continue
            if line.startswith("READY|"):
                send("ENABLE")
            elif line.startswith("STATE|"):
                state = fields(line)
                states.append(state)
                sim_time = float(state["t"])
                if (active_target is None and next_target == 0 and
                        state.get("mode") == "Balance" and
                        state.get("station_hold") == "1"):
                    active_target = targets[next_target]
                    active_start = sim_time
                    send(f"HEIGHT {active_target:.9f}")
                elif active_target is not None:
                    if (state.get("height_motion") == "1" and
                            active_start == sim_time):
                        pass
                    retired = (state.get("height_motion") == "0" and
                               state.get("height_request") == "0" and
                               sim_time > active_start)
                    if retired:
                        transitions.append((active_start, sim_time,
                                            active_target))
                        next_target += 1
                        active_target = None
                        active_start = None
                        if next_target < len(targets):
                            # The command is intentionally issued immediately
                            # on this retirement edge, without a manual pause.
                            active_target = targets[next_target]
                            active_start = sim_time
                            send(f"HEIGHT {active_target:.9f}")
                if state.get("mode") == "Fallen":
                    raise AssertionError(
                        f"Fallen during posture chain at t={sim_time:.3f}s")
            elif line.startswith("VIEW|"):
                views.append(fields(line))
    finally:
        if process.poll() is None:
            try:
                send("QUIT")
            except BrokenPipeError:
                pass
            process.wait(timeout=3.0)
        selector.close()

    if len(transitions) != len(targets):
        raise AssertionError(f"only retired {len(transitions)}/{len(targets)} HEIGHT transactions")
    if not states:
        raise AssertionError("native runner produced no STATE telemetry")

    # Score each transition using the simulator pose only after the run.  The
    # controller saw only delayed/noisy IMU and encoder packets.
    for start, end, target in transitions:
        window = [view for view in views if start <= float(view["t"]) <= end + 1.0]
        if len(window) < 2:
            raise AssertionError("insufficient VIEW samples for posture scoring")
        x0 = float(window[0]["x"])
        drift = max(abs(float(view["x"]) - x0) for view in window)
        if drift >= 0.25:
            raise AssertionError(
                f"HEIGHT {target:.6f} drifted {drift:.3f} m")

    scored_states = [state for state in states if float(state["t"]) >= transitions[0][0]]
    if max(abs(float(state["pitch"])) for state in scored_states) >= 0.25:
        raise AssertionError("posture chain exceeded the pitch stability envelope")

    print("PASS native posture chain: natural/25%/deep/25%/natural "
          f"({len(transitions)} immediate HEIGHT handoffs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
