#!/usr/bin/env python3
"""Regression for the V3.1 wheel-balancer path in the native TPA plant.

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
    balancing = [state for state in states if state["mode"] == "Balance"]
    assert all(state["mode"] != "EmergencyStop" for state in states)
    assert max(abs(float(state["pitch"])) for state in balancing) < 0.15
    # Smoke-checks this controller path only. Station drift and physical
    # balance still need separate tuning and hardware evidence.
    assert max(abs(float(state["lqr_velocity"])) for state in balancing) < 0.5
    assert all(state["controller"] == "v3.1" for state in balancing)
    print("PASS V3.1 native balance path: 3 kHz loop / 500 Hz sensors / 20 s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
