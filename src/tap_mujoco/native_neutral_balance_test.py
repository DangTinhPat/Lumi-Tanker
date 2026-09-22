#!/usr/bin/env python3
"""Regression for the native natural-ENABLE spawn balance hold.

Only the STATE stream is inspected.  The test does not use MuJoCo pose,
contact or constraint data as controller evidence.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess


def fields(line: str):
    return {part.split("=", 1)[0]: part.split("=", 1)[1]
            for part in line.split("|")[1:] if "=" in part}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    result = subprocess.run(
        [str(args.runner), "--model", str(args.model), "--rate", "3000",
         "--sensor-rate", "500", "--seed", "6814", "--run-for", "20"],
        input="ENABLE\n", check=True, capture_output=True, text=True)
    states = [fields(line) for line in result.stdout.splitlines()
              if line.startswith("STATE|")]
    assert states
    assert all(state["ground_truth_to_controller"] == "0" for state in states)
    assert any(state["mode"] == "Balance" for state in states)
    assert all(state["mode"] != "Fallen" for state in states)
    assert max(abs(float(state["pitch"])) for state in states) < 0.10
    settled = [state for state in states if float(state["t"]) >= 5.0]
    assert settled
    # This is deliberately much wider than sensor noise but far below the
    # pre-fix ~0.88 m/s command reached at 20 s.
    assert max(abs(float(state["lqr_velocity"])) for state in settled) < 0.14
    assert abs(float(settled[-1]["lqr_velocity"])) < 0.13
    # Station hold is judged at the sensor boundary: the fixed encoder-odometry
    # anchor must not let the robot wander a large distance while no command is
    # active. VIEW/ground-truth pose is intentionally not used here.
    assert max(abs(float(state["wheel_position"]) -
                   float(state["lqr_anchor"])) for state in settled) < 0.12
    quiet = [state for state in states if float(state["t"]) >= 10.0]
    assert quiet
    assert max(abs(float(state["lqr_velocity"])) for state in quiet) < 0.11
    quiet_positions = [float(state["wheel_position"]) for state in quiet]
    # Catch transients may use the full LQR authority; once settled, the
    # adaptive station loop must keep the sensor-derived position envelope
    # below 70 mm instead of trading a small pitch error for rolling drift.
    assert max(quiet_positions) - min(quiet_positions) < 0.07
    print("PASS native neutral balance: 3 kHz loop / 500 Hz sensors / 20 s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
