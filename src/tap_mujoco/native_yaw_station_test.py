#!/usr/bin/env python3
"""Closed-loop pure-yaw regression for the native RB model.

The test is deliberately based on VIEW only for offline verification.  The
controller receives the same ENABLE/DRIVE commands and sensor stream as the
runtime; simulator pose is never fed back into the controller.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import threading
from typing import Dict, List


def fields(line: str) -> Dict[str, str]:
    return dict(item.split("=", 1) for item in line.rstrip().split("|")[1:])


def yaw_from_quaternion(view: Dict[str, str]) -> float:
    qw = float(view["qw"])
    qx = float(view["qx"])
    qy = float(view["qy"])
    qz = float(view["qz"])
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def wrapped_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", required=True)
    parser.add_argument("--model", required=True)
    args = parser.parse_args()

    process = subprocess.Popen(
        [args.runner, "--model", args.model, "--rate", "3000",
         "--sensor-rate", "500", "--seed", "6814", "--run-for", "4"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert process.stdin is not None
    assert process.stdout is not None

    views: List[Dict[str, str]] = []
    states: List[Dict[str, str]] = []
    start_t = None
    end_t = None

    process.stdin.write("ENABLE\n")
    process.stdin.flush()

    def send(command: str) -> None:
        assert process.stdin is not None
        process.stdin.write(command + "\n")
        process.stdin.flush()

    for line in process.stdout:
        if line.startswith("STATE|"):
            state = fields(line)
            states.append(state)
            sim_time = float(state["t"])
            if start_t is None and sim_time >= 2.0:
                send("DRIVE 0 0.3")
                start_t = sim_time
            elif start_t is not None and end_t is None and sim_time >= 3.0:
                send("DRIVE 0 0")
                end_t = sim_time
        elif line.startswith("VIEW|"):
            views.append(fields(line))

    process.wait(timeout=5.0)
    if process.returncode != 0 or start_t is None or end_t is None:
        raise AssertionError("yaw command was not exercised by native runner")

    window = [
        view for view in views
        if start_t + 0.06 <= float(view["t"]) <= end_t - 0.06
    ]
    if len(window) < 2:
        raise AssertionError("insufficient VIEW samples during yaw command")
    first, last = window[0], window[-1]
    dx = float(last["x"]) - float(first["x"])
    dy = float(last["y"]) - float(first["y"])
    planar_drift = math.hypot(dx, dy)
    yaw_delta = abs(wrapped_delta(yaw_from_quaternion(first),
                                 yaw_from_quaternion(last)))

    yaw_states = [
        state for state in states
        if start_t + 0.06 <= float(state["t"]) <= end_t - 0.06
    ]
    if not yaw_states or not all(state["station_hold"] == "1" and
                                 state["station_eligible"] == "1"
                                 for state in yaw_states):
        raise AssertionError("yaw released adaptive station hold")

    # A real in-place turn should produce heading change without translating
    # tens of centimetres.  The bound is intentionally far wider than the
    # few-centimetre native contact/IMU transient, while rejecting the old
    # arc-drive behaviour caused by the released anchor and 0.005 lateral mu.
    if planar_drift >= 0.08:
        raise AssertionError(f"pure yaw translated {planar_drift:.3f} m")
    if yaw_delta <= 0.15:
        raise AssertionError(f"yaw command produced only {yaw_delta:.3f} rad")

    print(f"PASS native yaw station: drift={planar_drift:.4f} m "
          f"yaw={yaw_delta:.4f} rad")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
