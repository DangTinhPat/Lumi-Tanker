#!/usr/bin/env python3
"""Rocking stand-up on the GIM model: from the lying `home` pose to balanced at full height, then driving.

The runner starts in the `home` keyframe (--start-key home, physics on from the first tick), STANDUP makes
V3Controller fold the hips with the wheels rolling free, hand over to its balancer when the CoM is over the axle,
and raise the body to full height. Wheel torque is the real 0.13 N.m. Simulation evidence only (the GIM
geometry is not yet verified against the STEP).
"""
import argparse
import subprocess
from multiprocessing import Pool
from pathlib import Path

from native_runner_contract_test import fields

BALANCE_BY_S = 6.0          # the hand-over happens at ~3.5 s
FULL_KNEE_RAD = 1.6         # standing at full height (the knee is 1.69 there)
FINAL_PITCH_RAD = 0.05
DRIVE_MIN_M = 0.5           # 0.3 m/s for 4 s from rest
FINAL_SPEED_M_S = 0.05
RUN_S = 30.0


def W(delay, lag):
    return ["--wheel-delay", str(delay), "--wheel-lag", str(lag)]


# (label, STANDUP time s, runner args)
CASES = [("nominal", 0.2, []), ("late command", 0.7, []), ("command off the control tick", 0.23, []),
         ("wheel 30/50 ms", 0.2, W(0.03, 0.05)), ("wheel 35/70 ms", 0.2, W(0.035, 0.07)),
         ("wheel 70/110 ms", 0.2, W(0.07, 0.11)), ("wheel 20/30 ms", 0.2, W(0.02, 0.03)),
         ("roll balance on", 0.2, ["--roll", "1"])]


def run(job):
    runner, model, (label, at, args) = job
    cmd = [str(runner), "--model", str(model), "--rate", "3000", "--run-for", str(RUN_S), "--start-key", "home",
           "--at", f"{at}:STANDUP", "--at", "22:DRIVE 0.3 0", "--at", "26:DRIVE 0 0", *args]
    out = subprocess.run(cmd, input="", capture_output=True, text=True, timeout=600).stdout.splitlines()
    states = [fields(line) for line in out if line.startswith("STATE|")]
    views = [fields(line) for line in out if line.startswith("VIEW|")]
    assert states and views, f"{label}: no telemetry"
    return label, states, views


def check_one(result):
    label, states, views = result
    assert all(s["ground_truth_to_controller"] == "0" for s in states), label
    fault = next((s["fault_reason"] for s in states if s["mode"] == "Fault"), None)
    assert fault is None, f"{label}: fault {fault}"
    balancing = next((float(s["t"]) for s in states if s["mode"] == "Balancing"), None)
    assert balancing is not None and balancing < BALANCE_BY_S, f"{label}: balanced at {balancing}"
    end = states[-1]
    assert float(end["joint_lk"]) > FULL_KNEE_RAD, f"{label}: did not reach full height (knee {end['joint_lk']})"
    assert abs(float(end["pitch"])) < FINAL_PITCH_RAD, f"{label}: pitch {end['pitch']}"
    assert abs(float(end["ground_velocity"])) < FINAL_SPEED_M_S, f"{label}: still moving {end['ground_velocity']}"
    at_22 = next(v for v in views if float(v["t"]) >= 22.0)
    at_end = views[-1]
    drove = float(at_end["x"]) - float(at_22["x"])
    assert drove > DRIVE_MIN_M, f"{label}: drove only {drove:.2f} m"
    print({"case": label, "balancing_s": round(balancing, 2), "knee": round(float(end["joint_lk"]), 2),
           "drove_m": round(drove, 2)}, flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    with Pool(len(CASES)) as pool:
        results = pool.map(run, [(args.runner, args.model, c) for c in CASES])
    errors = []
    for r in results:
        try:
            check_one(r)
        except AssertionError as e:
            errors.append(str(e))
    assert not errors, "\n".join(errors)


if __name__ == "__main__":
    main()
