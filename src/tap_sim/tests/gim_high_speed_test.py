#!/usr/bin/env python3
"""High-speed driving on the GIM model: full stick, release, reverse, turns and slalom at speed.

The BL4820M12 wheel model (SimBl4820Wheel) keeps 0.13 N.m to 400 rpm and its motoring torque falls to 0 at
1000 rpm. Without the wheel envelope (Tier::wheel_envelope) a full-stick run reaches ~4 m/s and falls; with
it the plan stops near 3.06 m/s (motor_reserve 0.45). The long plan stop (0.3 m/s^2) must not trip the
runaway guard (it re-zeroed the wheels mid-stop at 0.8 m/s before the guard also watched the plan).
Simulation evidence only: the GIM geometry is not yet verified against the STEP.
"""
import argparse
import subprocess
from multiprocessing import Pool
from pathlib import Path

from native_runner_contract_test import fields

ENVELOPE_MAX_M_S = 3.2   # 0.04 m x (104.72 - 0.45 x 62.83) rad/s = 3.06 m/s, plus overshoot
ENVELOPE_MIN_M_S = 2.9
FINAL_SPEED_M_S = 0.05
FINAL_PITCH_RAD = 0.05
CASES = [
    ("full stick + release", [(6, "DRIVE 4.19 0"), (14, "DRIVE 0 0")], []),
    ("full -> 1.0 -> stop", [(6, "DRIVE 4.19 0"), (14, "DRIVE 1.0 0"), (17, "DRIVE 0 0")], []),
    ("full + turn 1.5", [(6, "DRIVE 4.19 0"), (14, "DRIVE 4.19 1.5"), (17, "DRIVE 0 0")], []),
    ("full -> full reverse", [(6, "DRIVE 4.19 0"), (14, "DRIVE -4.19 0"), (24, "DRIVE 0 0")], []),
    ("2.0 turn + release", [(6, "DRIVE 2.0 0"), (14.67, "DRIVE 2.0 1.0"), (18.67, "DRIVE 0 0")], []),
    ("full slalom", [(6, "DRIVE 4.19 0"), (12, "DRIVE 4.19 1.5"), (13.5, "DRIVE 4.19 -1.5"),
                     (15, "DRIVE 4.19 1.5"), (16.5, "DRIVE 0 0")], []),
    ("full, wheels 70/110 ms", [(6, "DRIVE 4.19 0"), (14, "DRIVE 0 0")],
     ["--wheel-delay", "0.07", "--wheel-lag", "0.11"]),
    ("full at 0.13 m", [(1, "HEIGHT 0.17172"), (6, "DRIVE 4.19 0"), (14, "DRIVE 0 0")], []),
]


def run(job):
    runner, model, (name, cmds, extra) = job
    end = max(t for t, _ in cmds) + 16.0
    cmd = [str(runner), "--model", str(model), "--rate", "3000", "--run-for", str(end), "--at", "0:ENABLE"]
    for t, c in cmds:
        cmd += ["--at", f"{t}:{c}"]
    p = subprocess.run(cmd + extra, input="", capture_output=True, text=True, timeout=600)
    st = [fields(line) for line in p.stdout.splitlines() if line.startswith("STATE|")]
    assert st, f"{name}: no telemetry ({p.stderr[-300:]})"
    fault = next((s["fault_reason"] for s in st if s["mode"] == "Fault"), None)
    gv = [abs(float(s["ground_velocity"])) for s in st if float(s["t"]) >= 6.0]
    return name, {"fault": fault, "v_max": round(max(gv), 2),
                  "final_v": round(max(abs(float(s["ground_velocity"])) for s in st[-50:]), 3),
                  "final_pitch": round(max(abs(float(s["pitch"])) for s in st[-50:]), 3)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    with Pool(len(CASES)) as pool:
        results = pool.map(run, [(args.runner, args.model, c) for c in CASES])
    bad = []
    for name, r in results:
        ok = (r["fault"] in (None, "none") and r["final_v"] < FINAL_SPEED_M_S and
              r["final_pitch"] < FINAL_PITCH_RAD and r["v_max"] < ENVELOPE_MAX_M_S and
              (r["v_max"] > ENVELOPE_MIN_M_S or name.startswith("2.0")))
        print("PASS" if ok else "FAIL", f"{name:24s}", r)
        if not ok:
            bad.append(name)
    assert not bad, f"high-speed cases failed: {bad}"


if __name__ == "__main__":
    main()
