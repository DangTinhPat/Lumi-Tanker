#!/usr/bin/env python3
"""v3.5 vertical jump in headless native MuJoCo (RB v2 / GIM6010-8 model).

Runs tap_mujoco_gim_v3 without any viewer: balances, then JUMP. The controller plans the
jump from its own FK/IK (dip depth, lift-off speed, tuck and extend timing) and does the
whole sequence itself. wheel_clearance (lowest wheel bottom to the floor) is simulator
ground truth used only to score the result; the controller never sees it
(ground_truth_to_controller=0). Commands run at exact simulated times (--at), so a run
does not depend on how fast this process reads the pipe.
"""

import argparse
import json
import math
from pathlib import Path
import subprocess

from native_runner_contract_test import fields

PHASES = ["Idle", "Dip", "Thrust", "Flight", "Land", "Recover"]


def run(runner, model, spec, jump_at, duration, extra=(), args=(), events=None):
    cmd = [str(runner), "--model", str(model), "--rate", "3000", "--run-for", str(duration),
           *map(str, args)]
    if spec:
        cmd += ["--jump", spec]
    schedule = [(0.0, "ENABLE"), (jump_at, "JUMP")] + list(extra)
    for when, command in schedule:
        cmd += ["--at", f"{when}:{command}"]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, bufsize=1)
    states, views = [], []
    try:
        for line in p.stdout:
            if line.startswith("STATE|"):
                states.append(fields(line))
            elif line.startswith("VIEW|"):
                views.append(fields(line))
            elif line.startswith(("JUMP_REFUSED|", "JUMP_PLAN|")) and events is not None:
                events.append((line.split("|")[0], fields(line)))
        p.wait(timeout=5)
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()
        p.stdin.close()
        p.stdout.close()
    return states, views


def body_pitch(view):
    return 2 * math.atan2(float(view["qy"]), float(view["qw"]))


def summarize(states, views, jump_at):
    """Scores the jump from the first Thrust sample on."""
    thrust = next((s for s in states if int(s["jump_phase"]) == 2), None)
    phases = []
    for s in states:
        if float(s["t"]) < jump_at:
            continue
        ph = int(s["jump_phase"])
        if not phases or phases[-1][0] != ph:
            phases.append((ph, float(s["t"])))
    fault = next((s["fault_reason"] for s in states if s["mode"] == "Fault"), None)
    out = {"phases": [(PHASES[p], round(t, 3)) for p, t in phases], "fault": fault,
           "final_mode": states[-1]["mode"]}
    if thrust is None:
        return {**out, "peak_clearance_mm": 0.0, "jumped": False}
    t0 = float(thrust["t"])
    after = [s for s in states if float(s["t"]) >= t0]
    # The first flight only: from the first sample with the wheels off the floor to the first one back
    # on it (a robot that falls over throws its wheels up again, which is not the jump).
    lift = next((s for s in after if float(s["wheel_clearance"]) > 0.002), None)
    td = None
    if lift is not None:
        i_lift = after.index(lift)
        td = next((s for s in after[i_lift:] if float(s["wheel_clearance"]) < 0.002), None)
    seg = after[after.index(lift): (after.index(td) if td is not None else len(after))] if lift is not None else after
    clearance = [(float(s["t"]), float(s["wheel_clearance"])) for s in seg] or [(t0, 0.0)]
    peak_t, peak = max(clearance, key=lambda c: c[1])
    if td is not None and lift is not None:
        i = after.index(td)
        window = after[max(i - 8, 0):i + 1]          # 16 ms before the contact
        out.update({"flight_s": round(float(td["t"]) - float(lift["t"]), 3),
                    "td_virt_err_rad": round(float(td["lqr_pitch_state"]) - float(td["balance_ref"]), 3),
                    "td_pitch_rad": round(float(td["pitch"]), 3),
                    "td_pitch_rate": round(sum(float(w["pitch_rate"]) for w in window) / len(window), 2)})
    # CoM flight: from the first sample with the wheels off the floor to the CoM apex.
    if lift is not None:
        apex = max((float(x["com_z"]) for x in seg), default=0.0)
        out["com_rise_mm"] = round(1e3 * (apex - float(lift["com_z"])), 1)
        dip_start = next((x for x in states if int(x["jump_phase"]) == 1), None)
        out["dip_to_thrust_s"] = round(t0 - float(dip_start["t"]), 3) if dip_start else None
    # Body tilt over the whole jump: from just before the dip to 0.4 s after the legs are stiff again.
    dip = next((x for x in states if int(x["jump_phase"]) == 1), None)
    if dip is not None:
        td0 = float(dip["t"])
        base = float(min(states, key=lambda x: abs(float(x["t"]) - (td0 - 0.02)))["pitch"])
        end = next((float(x["t"]) for x in states if float(x["t"]) > t0 and int(x["jump_phase"]) == 0), t0 + 1.0)
        window = [abs(float(x["pitch"]) - base) for x in states if td0 <= float(x["t"]) <= end + 0.4]
        out["pitch_excursion_rad"] = round(max(window), 3) if window else None
    jv = [v for v in views if float(v["t"]) >= t0]
    out.update({"jumped": True, "thrust_t": round(t0, 3), "peak_clearance_mm": round(1e3 * peak, 1),
                "peak_t": round(peak_t, 3),
                "peak_body_pitch_rad": round(max((abs(body_pitch(v)) for v in jv), default=0.0), 3),
                "final_pitch_rad": round(max(abs(float(s["pitch"])) for s in states[-100:]), 3),
                "final_speed_m_s": round(max(abs(float(s["ground_velocity"])) for s in states[-100:]), 3)})
    return out


# Acceptance (simulation only, nominal model; not a statement about the robot): the planned jump (wheel
# bottom to floor ~11 cm; the CoM flies ~6-8 cm because the tuck and the joint torque budget cannot give
# 4 cm and 12 cm together) with no Fault, upright and (nearly) still afterwards. The body tilts in the
# dip and thrust (the hip swings to keep the CoM over the axle): the excursion is bounded, not zero.
CLEARANCE_MM = (90.0, 125.0)
COM_RISE_MAX_MM = 90.0
MAX_DIP_TO_THRUST_S = 0.35    # a quick counter-movement, not a slow crouch
MAX_PITCH_EXCURSION_RAD = 0.5
FINAL_PITCH_LIMIT_RAD = 0.06
FINAL_SPEED_LIMIT_M_S = 0.15  # the robot may still creep at a few cm/s when the run ends
R_WHEEL, O_Z = 0.04, 0.00828


def body_height(stand):
    return stand + R_WHEEL - O_Z


def W(delay, lag):
    return ["--wheel-delay", delay, "--wheel-lag", lag]


# (label, jump time s, standing hip-axle height or None, runner args). Command timing offsets of a few
# ms, jump times from 3 to 15 s after ENABLE, wheel delay/lag around the 50/75 ms default and standing
# heights 0.13-0.175 m. Known limits (not asserted, see docs/V3.5_JUMP.md): wheel delay >= 60 ms with
# lag >= 100 ms (or delay 70 ms with lag 50 ms) falls; standing at 0.12-0.125 m drifts away after landing.
CASES = ([("nominal +%d ms" % ms, 6.0 + ms * 1e-3, None, []) for ms in (0, 3, 5, 8, 10, 12)] +
         [("jump at %.0f s" % t, t, None, []) for t in (5.0, 8.0, 15.0)] +
         [("wheel %d/%d ms" % (d * 1e3, l * 1e3), 6.0, None, W(d, l))
          for d, l in ((0.03, 0.05), (0.03, 0.11), (0.05, 0.08), (0.035, 0.07), (0.07, 0.11))] +
         [("stand %.3f m" % h, 6.5, h, []) for h in (0.175, 0.17, 0.16, 0.15, 0.14, 0.13)])


def check_case(runner, model, label, at, stand, args=(), spec=""):
    extra = [(1.0, f"HEIGHT {body_height(stand):.5f}")] if stand else []
    states, views = run(runner, model, spec, at, at + 6.0, extra=extra, args=args)
    r = summarize(states, views, at)
    assert all(s["ground_truth_to_controller"] == "0" for s in states), label
    assert r["fault"] is None, f"{label}: {r}"
    assert r["jumped"], f"{label}: never jumped {r}"
    assert CLEARANCE_MM[0] <= r["peak_clearance_mm"] <= CLEARANCE_MM[1], f"{label}: clearance {r}"
    assert r["com_rise_mm"] <= COM_RISE_MAX_MM, f"{label}: CoM flight {r}"
    assert r["dip_to_thrust_s"] <= MAX_DIP_TO_THRUST_S, f"{label}: slow dip {r}"
    assert r["pitch_excursion_rad"] <= MAX_PITCH_EXCURSION_RAD, f"{label}: tilt {r}"
    assert r["final_pitch_rad"] < FINAL_PITCH_LIMIT_RAD and r["final_speed_m_s"] < FINAL_SPEED_LIMIT_M_S, \
        f"{label}: {r}"
    print(json.dumps({"case": label, "clearance_mm": r["peak_clearance_mm"], "com_rise_mm": r["com_rise_mm"],
                      "dip_s": r["dip_to_thrust_s"], "flight_s": r.get("flight_s"),
                      "tilt_rad": r["pitch_excursion_rad"], "final_pitch_rad": r["final_pitch_rad"],
                      "final_speed_m_s": r["final_speed_m_s"]}), flush=True)


def check(runner, model):
    from multiprocessing import Pool
    from functools import partial
    jobs = [(label, at, stand, args) for label, at, stand, args in CASES]
    with Pool(min(len(jobs), 8)) as pool:
        errors = [e for e in pool.map(partial(_check_job, runner, model), jobs) if e]
    assert not errors, "\n".join(errors)


def _check_job(runner, model, job):
    label, at, stand, args = job
    try:
        check_case(runner, model, label, at, stand, args)
    except AssertionError as e:
        return f"FAIL {label}: {e}"
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--check", action="store_true", help="run the acceptance cases")
    parser.add_argument("--spec", default="")
    parser.add_argument("--jump-at", type=float, default=6.0)
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()
    if args.check:
        check(args.runner, args.model)
        return
    events = []
    states, views = run(args.runner, args.model, args.spec, args.jump_at, args.duration, events=events)
    assert states and not any(s["ground_truth_to_controller"] != "0" for s in states)
    for kind, data in events:
        print(kind, json.dumps(data))
    print(json.dumps(summarize(states, views, args.jump_at)), flush=True)


if __name__ == "__main__":
    main()
