#!/usr/bin/env python3
"""Black-box contract for the GIM V3 MuJoCo runner.

This test checks the boundary, not balance capability.  VIEW is allowed only
as a visualization/debug stream; the controller-facing STATE stream must not
contain simulator pose/contact/qpos data.
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
    child = subprocess.run(
        [str(args.runner), "--model", str(args.model), "--rate", "3000", "--run-for", "0.25"],
        check=True, capture_output=True, text=True)
    lines = child.stdout.splitlines()
    ready = next(line for line in lines if line.startswith("READY|"))
    ready_fields = fields(ready)
    assert ready_fields["ground_truth_to_controller"] == "0"
    states = [fields(line) for line in lines if line.startswith("STATE|")]
    assert len(states) >= 5
    forbidden = ("qpos", "qvel", "contact", "pose", "world_x", "world_z")
    for state in states:
        assert state["ground_truth_to_controller"] == "0"
        assert not any(key in state for key in forbidden)
        assert state["loop_hz"] == "3000"
        for value in state.values():
            assert "nan" not in value.lower()
    assert any(line.startswith("VIEW|") for line in lines), "debug view channel missing"
    print("PASS GIM V3 boundary: 3k fixed-step, sensor-only STATE, debug-only VIEW")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
