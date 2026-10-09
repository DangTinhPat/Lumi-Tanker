#!/usr/bin/env python3
"""Catch self-excited wheel/hip motion after GIM crouching, with a countercase."""
import argparse
import json
import math
from pathlib import Path
import subprocess

from gim_runner_contract_test import fields


def run(args, name, duration, schedule, extra=()):
    p = subprocess.Popen([str(args.runner), "--model", str(args.model),
                          "--run-for", str(duration), *extra],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, bufsize=1)
    states, views, next_command = [], [], 0
    try:
        p.stdin.write("ENABLE\n")
        p.stdin.flush()
        with (args.evidence / (name+".log")).open("w") as log:
            for line in p.stdout:
                log.write(line)
                if line.startswith("STATE|"):
                    s = fields(line)
                    states.append(s)
                    if next_command < len(schedule) and float(s["t"]) >= schedule[next_command][0]:
                        p.stdin.write(schedule[next_command][1]+"\n")
                        p.stdin.flush()
                        next_command += 1
                elif line.startswith("VIEW|"):
                    views.append(fields(line))
        assert p.wait(timeout=2) == 0 and next_command == len(schedule)
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()
        p.stdin.close()
        p.stdout.close()
    tail = [s for s in states if float(s["t"]) >= duration-20]
    rms = lambda key: math.sqrt(sum(float(s[key])**2 for s in tail)/len(tail))
    pp = lambda key: max(float(s[key]) for s in tail)-min(float(s[key]) for s in tail)
    metrics = dict(case=name, seconds=duration, final_mode=states[-1]["mode"],
                   peak_speed=max(abs(float(s["ground_velocity"])) for s in tail),
                   rms_speed=rms("ground_velocity"), rms_gyro=rms("pitch_rate"),
                   hip_pp=max(pp("joint_lh"), pp("joint_rh")),
                   knee_pp=max(pp("joint_lk"), pp("joint_rk")),
                   quiet_gate=max(float(s["reflex_gate"]) for s in tail),
                   peak_pitch=max(abs(float(s["pitch"])) for s in states))
    (args.evidence / (name+".json")).write_text(json.dumps(metrics, indent=2)+"\n")
    print(json.dumps(metrics), flush=True)
    return states, views, metrics


def stable(states, views, metrics, height=.08):
    name = metrics["case"]
    assert not any(s["mode"] == "Fault" for s in states), name+": controller faulted"
    assert all(s["ground_truth_to_controller"] == "0" for s in states)
    assert metrics["peak_pitch"] < .30, name+": body pitch exceeded existing bound"
    assert metrics["peak_speed"] < .02, name+": wheel motion persisted at rest"
    assert metrics["rms_speed"] < .01, name+": wheel oscillation persisted"
    assert metrics["hip_pp"] < .01 and metrics["knee_pp"] < .01, name+": leg oscillation persisted"
    assert metrics["rms_gyro"] < .05, name+": body vibration persisted"
    assert metrics["quiet_gate"] < .01, name+": reflex stayed active at rest"
    assert states[-1]["startup_fixture"] == "0", name+": fixture hid instability"
    assert abs(float(views[-1]["z"])-height) < .005, name+": crouch was not reached"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    args.evidence.mkdir(parents=True, exist_ok=True)
    # Bounds chosen before accepting the fix: 20 mm/s peak, 10 mm/s RMS,
    # 0.01 rad joint travel, 0.05 rad/s gyro RMS, gate closed at quiet rest.
    _, _, counter = run(args, "prediction_off", 100, [(6, "HEIGHT .08")],
                        ("--no-lqr-prediction",))
    assert counter["peak_speed"] > .05 or counter["hip_pp"] > .03, \
        "countercase no longer reproduces the defect; investigate the fixture"
    stable(*run(args, "low_rest", 100, [(6, "HEIGHT .08")]))
    states, views, metrics = run(args, "low_push", 120,
                                [(6, "HEIGHT .08"), (65, "PUSH 2 .2"), (75, "PUSH -2 .2")])
    stable(states, views, metrics)
    for start in (65, 75):
        event = [s for s in states if start <= float(s["t"]) < start+3]
        assert max(float(s["reflex_gate"]) for s in event) > .1, "reflex failed to respond to push"
    states, views, metrics = run(args, "low_drive_stop", 120,
                [(6, "HEIGHT .08"), (65, "DRIVE .15 0"), (75, "STOP"),
                 (85, "DRIVE 0 .3"), (90, "STOP")])
    stable(states, views, metrics)
    cruise = [float(s["ground_velocity"]) for s in states if 72 <= float(s["t"]) < 74]
    assert abs(sum(cruise)/len(cruise)-.15) < .05, "low drive did not track speed"
    before = min(views, key=lambda v: abs(float(v["t"])-65))
    after = min(views, key=lambda v: abs(float(v["t"])-75))
    assert float(after["x"])-float(before["x"]) > .8, "robot did not actually drive"
    stable(*run(args, "height_cycle", 130,
                [(6, "HEIGHT .17"), (18, "HEIGHT .13"), (35, "HEIGHT .08"),
                 (65, "HEIGHT .20"), (85, "HEIGHT .08")]))
    stable(*run(args, "minimum_rest", 100, [(6, "HEIGHT .06988")]), height=.06988)


if __name__ == "__main__":
    main()
