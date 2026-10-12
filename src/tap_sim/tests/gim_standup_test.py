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
STAND_CHECK_S = 19.0        # low-wheel-torque cases are scored here (standing, before any drive command)


def W(delay, lag):
    return ["--wheel-delay", str(delay), "--wheel-lag", str(lag)]


# (label, STANDUP time s, runner args, drive afterwards). The wheels carry no torque while rocking (wheels_off); after the
# hand-over the stand-up and the 0.3 m/s drive both hold down to 0.065 N.m (they fall at 0.05 N.m).
CASES = [("nominal", 0.2, [], True), ("late command", 0.7, [], True), ("command off the control tick", 0.23, [], True),
         ("wheel 30/50 ms", 0.2, W(0.03, 0.05), True), ("wheel 35/70 ms", 0.2, W(0.035, 0.07), True),
         ("wheel 70/110 ms", 0.2, W(0.07, 0.11), True), ("wheel 20/30 ms", 0.2, W(0.02, 0.03), True),
         ("roll balance on", 0.2, ["--roll", "1"], True),
         ("wheel torque 0.065 N.m", 0.2, ["--wheel-torque", "0.065"], True),
         ("wheel torque 0.065 N.m, wheel 70/110 ms, stand only", 0.2, ["--wheel-torque", "0.065", *W(0.07, 0.11)], False),
         ("wheel torque 0.065 N.m, roll on, stand only", 0.2, ["--wheel-torque", "0.065", "--roll", "1"], False)]


def run(job):
    runner, model, (label, at, args, drive) = job
    cmd = [str(runner), "--model", str(model), "--rate", "3000", "--run-for", str(RUN_S), "--start-key", "home",
           "--at", f"{at}:STANDUP", *(["--at", "22:DRIVE 0.3 0", "--at", "26:DRIVE 0 0"] if drive else []), *args]
    out = subprocess.run(cmd, input="", capture_output=True, text=True, timeout=600).stdout.splitlines()
    states = [fields(line) for line in out if line.startswith("STATE|")]
    views = [fields(line) for line in out if line.startswith("VIEW|")]
    assert states and views, f"{label}: no telemetry"
    return label, states, views, drive


def check_one(result):
    label, states, views, drive = result
    assert all(s["ground_truth_to_controller"] == "0" for s in states), label
    fault = next((s["fault_reason"] for s in states if s["mode"] == "Fault"), None)
    assert fault is None, f"{label}: fault {fault}"
    rocking = [s for s in states if s["mode"] == "StandUp" and float(s["t"]) > 0.25]
    assert rocking, f"{label}: no rocking phase"
    peak = max(max(abs(float(s["wheel_tq_l"])), abs(float(s["wheel_tq_r"]))) for s in rocking)
    assert peak < 1e-6, f"{label}: the wheels carried {peak:.3f} N.m while rocking (torque must be off)"
    if not drive:
        states = [s for s in states if float(s["t"]) <= STAND_CHECK_S]
    balancing = next((float(s["t"]) for s in states if s["mode"] == "Balancing"), None)
    assert balancing is not None and balancing < BALANCE_BY_S, f"{label}: balanced at {balancing}"
    end = states[-1]
    assert float(end["joint_lk"]) > FULL_KNEE_RAD, f"{label}: did not reach full height (knee {end['joint_lk']})"
    assert abs(float(end["pitch"])) < FINAL_PITCH_RAD, f"{label}: pitch {end['pitch']}"
    assert abs(float(end["ground_velocity"])) < FINAL_SPEED_M_S, f"{label}: still moving {end['ground_velocity']}"
    drove = 0.0
    if drive:
        at_22 = next(v for v in views if float(v["t"]) >= 22.0)
        drove = float(views[-1]["x"]) - float(at_22["x"])
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
