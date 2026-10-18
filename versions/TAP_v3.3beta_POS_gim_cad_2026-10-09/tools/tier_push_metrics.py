#!/usr/bin/env python3
"""Push / rest metrics from a v3_tier_tests TIER_DIAG run (CSV from TR_ANIM).

  TR_ANIM=run.csv DIAG_F=8 TIER_DIAG=1 build/fw/v3_tier_tests
  python3 tools/tier_push_metrics.py run.csv [label]

Push (at DIAG's 14 s, sign from the push column):
  body_with / body_opposite  peak body pitch off level along / against the
                             push [rad] (opposite: the overshoot the
                             operator sees as "tipping the other way")
  hip_peak / hip_trough      tier-2 hip offset swing around its pre-push
                             mean [rad]
  carried_mm / return_mm     how far the push carried the axle, and how far
                             the robot then drove itself back
Rest (last 15 s): hip_pp [rad], axle_pp_mm, body_rms [rad].
"""
import csv
import math
import sys


def load(path):
    with open(path, newline="") as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def metrics(rows, push_t=14.0):
    def win(a, b):
        return [r for r in rows if a <= r["t"] < b]

    out = {}
    pre = win(push_t - 2.0, push_t) or rows[:1]
    x0 = sum(r["axle_x"] for r in pre) / len(pre)
    h0 = sum(r["trunk_hip"] for r in pre) / len(pre)
    pushed = [r for r in rows if abs(r["push"]) > 0.0]
    if pushed:
        sign = 1.0 if pushed[0]["push"] > 0.0 else -1.0
        after = win(push_t, push_t + 6.0)
        out["body_with"] = max(sign * r["body"] for r in win(push_t, push_t + 1.2))
        out["body_opposite"] = max(-sign * r["body"] for r in win(push_t + 0.3, push_t + 6.0))
        hips = [r["trunk_hip"] - h0 for r in after]
        out["hip_peak"] = max(hips)
        out["hip_trough"] = min(hips)
        xs = [sign * (r["axle_x"] - x0) for r in after]
        far = max(xs)
        out["carried_mm"] = 1000.0 * far
        out["return_mm"] = 1000.0 * (far - xs[-1])
    tail = win(rows[-1]["t"] - 15.0, rows[-1]["t"] + 1.0)
    hs = [r["trunk_hip"] for r in tail]
    xs = [r["axle_x"] for r in tail]
    out["hip_pp"] = max(hs) - min(hs)
    out["axle_pp_mm"] = 1000.0 * (max(xs) - min(xs))
    out["body_rms"] = math.sqrt(sum(r["body"] ** 2 for r in tail) / len(tail))
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    m = metrics(load(sys.argv[1]))
    label = sys.argv[2] if len(sys.argv) > 2 else sys.argv[1]
    print(label, " ".join(f"{k}={v:.4f}" for k, v in m.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
