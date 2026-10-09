#!/usr/bin/env python3
"""GIM V3 startup, free balance and bounded maneuvers in native MuJoCo."""

import argparse
import json
import math
from pathlib import Path
import subprocess

from gim_runner_contract_test import fields


def run(runner, model, name, duration, schedule):
    p = subprocess.Popen(
        [str(runner), "--model", str(model), "--rate", "3000", "--run-for", str(duration)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)
    states, views = [], []
    next_command = 0
    try:
        p.stdin.write("DRIVE .2 0\nENABLE\n")
        p.stdin.flush()
        early_drive_rejected = False
        for line in p.stdout:
            if line.startswith("DRIVE_REJECTED|"):
                early_drive_rejected = True
            elif line.startswith("STATE|"):
                state = fields(line)
                states.append(state)
                if next_command < len(schedule) and float(state["t"]) >= schedule[next_command][0]:
                    p.stdin.write(schedule[next_command][1] + "\n")
                    p.stdin.flush()
                    next_command += 1
            elif line.startswith("VIEW|"):
                views.append(fields(line))
        assert p.wait(timeout=2) == 0
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()
        p.stdin.close()
        p.stdout.close()
    assert early_drive_rejected, "DRIVE accepted before balance startup"
    assert next_command == len(schedule) and states and views
    assert not any(s["mode"] == "Fault" for s in states), \
        f"{name}: {next((s.get('fault_reason', s) for s in states if s['mode'] == 'Fault'), '')}"
    locking = [s for s in states if s["mode"] == "Locking"]
    assert locking and all(s["startup_fixture"] == "1" for s in locking)
    free = [s for s in states if s["startup_fixture"] == "0"]
    assert free and all(s["mode"] == "Balancing" and s["drive_ready"] == "1" for s in free)
    release_t = float(free[0]["t"])
    assert 1.3 <= release_t < 1.4, "fixture failed to release after wheel settle"
    assert float(views[-1]["physics_t"]) > duration - release_t - .1
    assert all(s["ground_truth_to_controller"] == "0" for s in states)
    peak_pitch = max(abs(float(s["pitch"])) for s in free)
    assert peak_pitch < .30, f"{name}: body pitch exceeded maneuver limit"
    virtual_error = max(abs(float(s["lqr_pitch_state"]) - float(s["balance_ref"])) for s in free)
    assert virtual_error < .15, f"{name}: virtual-pendulum error exceeded limit"
    for view in views:
        pitch = 2 * math.atan2(float(view["qy"]), float(view["qw"]))
        assert abs(pitch) < .30, f"{name}: actual simulated pitch exceeded limit"
    if name == "drive_stop":
        cruise = [float(s["ground_velocity"]) for s in states if 9 <= float(s["t"]) <= 10]
        assert cruise and abs(sum(cruise) / len(cruise) - .2) < .05
        start = min(views, key=lambda v: abs(float(v["t"]) - 6))
        end = min(views, key=lambda v: abs(float(v["t"]) - 11))
        assert float(end["x"]) - float(start["x"]) > .6, "encoder speed did not move the robot"
    elif name == "height":
        for when, height in ((13, .17), (22, .13), (34, .20)):
            view = min(views, key=lambda v: abs(float(v["t"]) - when))
            assert abs(float(view["z"]) - height) < .005, "height command did not reach the CAD pose"
    elif name == "yaw":
        def yaw(view):
            w, x, y, z = [float(view[k]) for k in ("qw", "qx", "qy", "qz")]
            return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        start = min(views, key=lambda v: abs(float(v["t"]) - 6))
        end = min(views, key=lambda v: abs(float(v["t"]) - 10))
        assert yaw(end) - yaw(start) > .8, "yaw command did not turn the robot"
    tail = [s for s in states if float(s["t"]) >= duration - 2]
    assert max(abs(float(s["ground_velocity"])) for s in tail) < .05, f"{name}: failed to settle"
    assert max(abs(float(s["pitch"])) for s in tail) < .05
    print(json.dumps({"case": name, "seconds": duration, "release_s": release_t,
                      "peak_body_pitch_rad": peak_pitch, "peak_virtual_error_rad": virtual_error,
                      "final_mode": states[-1]["mode"]}), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    # Nominal-model acceptance: 0.30 rad body, 0.15 rad virtual error,
    # 0.05 m/s cruise error and final speed. These do not certify hardware.
    cases = (
        ("stand", 60, []),
        ("drive_stop", 25, [(6, "DRIVE .2 0"), (11, "STOP")]),
        ("push", 25, [(6, "PUSH 2 .2"), (10, "PUSH -2 .2")]),
        ("height", 35, [(6, "HEIGHT .17"), (14, "HEIGHT .13"), (23, "HEIGHT .20")]),
        ("yaw", 25, [(6, "DRIVE 0 .3"), (10, "STOP")]),
    )
    for name, duration, schedule in cases:
        run(args.runner, args.model, name, duration, schedule)


if __name__ == "__main__":
    main()
