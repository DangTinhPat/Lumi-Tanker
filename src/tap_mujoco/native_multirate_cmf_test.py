#!/usr/bin/env python3
"""Check CMF timing is tied to sensor time, not the control-loop rate."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess


def fields(line: str):
    return {part.split("=", 1)[0]: part.split("=", 1)[1]
            for part in line.split("|")[1:] if "=" in part}


def first_time(states, key, value):
    for state in states:
        if state.get(key) == value:
            return float(state["t"])
    raise AssertionError(f"missing {key}={value}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()

    results = {}
    # 3 kHz is the canonical native loop and 4 kHz is the stress profile.
    # 3.5 kHz provides a third integration rate without treating the old
    # 1 kHz exploratory setting as a supported landing-physics profile.
    for rate in (3000, 3500, 4000):
        result = subprocess.run(
            [str(args.runner), "--model", str(args.model), "--rate", str(rate),
             "--sensor-rate", "500", "--landing-30cm", "--seed", "6814",
             "--run-for", "0.75"],
            check=True, capture_output=True, text=True)
        states = [fields(line) for line in result.stdout.splitlines()
                  if line.startswith("STATE|")]
        assert states
        assert all(state["ground_truth_to_controller"] == "0"
                   for state in states)
        assert any(state["imu_seeded"] == "1" for state in states)
        ballistic = first_time(states, "flight", "BallisticFlight")
        touchdown = first_time(states, "seen_touchdown", "1")
        # A normal enable cannot re-arm a body that remains beyond the fall
        # limit. Explicit STAND_UP is the only recovery entry point.
        assert not any(state["mode"] == "Enabling" and
                       abs(float(state["pitch"])) > 1.1
                       for state in states)
        results[rate] = (ballistic, touchdown)

    ballistic_times = [pair[0] for pair in results.values()]
    touchdown_times = [pair[1] for pair in results.values()]
    assert max(ballistic_times) - min(ballistic_times) <= 0.006
    # STATE telemetry is emitted on a 20 ms boundary; a touchdown transition
    # can therefore become visible in the adjacent telemetry bin even though
    # the 500 Hz sensor debounce is rate-independent.
    assert max(touchdown_times) - min(touchdown_times) <= 0.026
    print("PASS CMF multi-rate timing: 3/3.5/4 kHz control, 500 Hz sensors, "
          f"ballistic={results[3000][0]:.3f}s touchdown={results[3000][1]:.3f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
