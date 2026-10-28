#!/usr/bin/env python3
"""Acceptance run of the small vertical hop (docs/V3.7_HOP_20261021.md, docs/RULES.md Rule 6).

Runs tap_mujoco_gim_v3 headless with the hop enabled and the HOP command scheduled with --at (stdin timing is
not reproducible). The simulated IMU is the hardware-shaped one (--imu-hw 1: +-2 g, 44 Hz) and the leg commands
get a dead time (--leg-delay). The wheel clearance printed by the runner is ground truth used for SCORING only;
the controller never reads it.

Pass per case: no Fault, hop result Done, wheels cleared the floor by the case's minimum, and the robot is
balancing again (|pitch| < 0.1 rad, |ground speed| < 0.3 m/s) at the end. --check requires --min-pass cases.
"""
import argparse
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

LEG = "30,0.3,0.001,0.005"   # KP_S,KV,ARMATURE,CURRENT_LAG: fits the 21/10 stand-up logs (dead time ~25 ms, lag ~30 ms)

# name, extra runner args, HOP time [s], minimum wheel clearance [mm]. The nominal timing (leg dead time 20 ms, wheel
# 50 ms + 75 ms lag) is the one the stand-up logs and the wheel measurements of 18/10 point to; the other rows are
# neighbours of it. Beyond them (wheel 60/90 ms or slower, leg dead time 35 ms) the hop fails or barely lifts: see the
# map in docs/V3.7_HOP_20261021.md.
NOM = ["--leg-model", LEG, "--leg-delay", "0.02"]
CASES = [(f"nominal+{dt:.2f}", NOM, 14.0 + dt, 15.0) for dt in (0.0, 0.07, 0.13, 0.21, 0.29, 0.37, 0.45, 0.53)]
CASES += [
    ("wheel50/90", NOM + ["--wheel-lag", "0.09"], 14.0, 15.0),
    ("wheel50/90+0.2", NOM + ["--wheel-lag", "0.09"], 14.2, 15.0),
    ("noise1", NOM + ["--sensor-noise", "1.0", "--noise-seed", "1"], 14.1, 15.0),
    ("noise2", NOM + ["--sensor-noise", "1.0", "--noise-seed", "2"], 14.3, 15.0),
    ("leg dead time 30 ms", ["--leg-model", "30,0.4,0.002,0.005", "--leg-delay", "0.03"], 14.0, 8.0),
]


def parse(stdout, tmin):
    rows = []
    for line in stdout.splitlines():
        if not line.startswith("STATE|"):
            continue
        if float(line[8:line.index("|", 8)]) < tmin:
            continue
        d = {}
        for kv in line.split("|")[1:]:
            if "=" in kv:
                k, v = kv.split("=", 1)
                try:
                    d[k] = float(v)
                except ValueError:
                    d[k] = v
        rows.append(d)
    return rows


def run_case(args, case):
    name, extra, t_hop, min_mm = case
    cmd = [args.runner, "--model", args.model, "--run-for", str(t_hop + 7.0), "--state-hz", "250",
           "--at", "0.5:ENABLE", "--at", "6:HEIGHT 0.15", "--at", f"{t_hop}:HOP {args.clearance_mm:g}",
           "--imu-hw", "1", "--hop", args.hop] + extra
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=300).stdout
    rows = parse(out, t_hop - 0.1)
    if not rows:
        return name, False, "no telemetry"
    fault = next((i for i, r in enumerate(rows) if r["mode"] == "Fault"), None)
    live = rows if fault is None else rows[:fault]
    clearance = max(r["wheel_clearance"] for r in live) * 1000.0 if live else 0.0
    end = rows[-1]
    refused = "HOP_REFUSED" in out
    ok = (fault is None and not refused and int(end["hop_result"]) == 1 and clearance >= min_mm and
          abs(end["pitch"]) < 0.1 and abs(end["ground_velocity"]) < 0.3)
    why = (f"clearance {clearance:.1f} mm, result {int(end['hop_result'])}, end pitch {end['pitch']:+.3f}, "
           f"speed {end['ground_velocity']:+.2f}" + (", FAULT " + str(end["fault_reason"]) if fault is not None else "") +
           (", refused" if refused else ""))
    return name, ok, why


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runner", required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--hop", default="1", help="--hop key=value list handed to the runner")
    ap.add_argument("--clearance-mm", type=float, default=20.0)
    ap.add_argument("--min-pass", type=int, default=len(CASES))
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    with ThreadPoolExecutor(min(len(CASES), 8)) as ex:
        results = list(ex.map(lambda c: run_case(args, c), CASES))
    passed = 0
    for name, ok, why in results:
        passed += ok
        print(f"{'PASS' if ok else 'FAIL'} {name:12s} {why}")
    print(f"{passed}/{len(results)} cases passed")
    if args.check and passed < args.min_pass:
        sys.exit(1)


if __name__ == "__main__":
    main()
