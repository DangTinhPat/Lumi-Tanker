#!/usr/bin/env python3
"""Sensor-only CMF phase test for the 30 cm / 1.5 m/s setup."""

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
         "--landing-30cm", "--run-for", "0.75"],
        check=True, capture_output=True, text=True)
    states = [fields(line) for line in result.stdout.splitlines()
              if line.startswith("STATE|")]
    assert any(state["seen_ballistic"] == "1" for state in states)
    assert any(state["seen_touchdown"] == "1" for state in states)
    assert all(state["ground_truth_to_controller"] == "0" for state in states)
    # The 1.5 m/s initial approach is INSIDE the 1.6755 m/s wheel speed
    # envelope (400 rpm rated under load).  The binding hardware limit is
    # wheel torque, not wheel speed: 0.13 Nm caps ground acceleration at
    # 1.41 m/s^2.  Phase detection passes, but this test does not call the
    # post-touchdown balance result a success certificate.
    print("PASS CMF sensor-only phases: BallisticFlight -> TouchdownCandidate; recovery remains evidence-pending")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
