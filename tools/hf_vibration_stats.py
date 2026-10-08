#!/usr/bin/env python3
"""Fast (>5 Hz) vibration of the tier-2 hip loop from a TAP serial log.

The TR lines are 25 Hz samples, so the band above 5 Hz is what the controller's gyro and hip
command do between 5 and 12.5 Hz plus whatever folds down from higher frequencies. Use it to
compare logs at the same disturbance level; the numbers are indicators, not true 500 Hz rms.

  python3 tools/hf_vibration_stats.py LOG [LOG ...]          # per-activity summary
  python3 tools/hf_vibration_stats.py LOG --span 323.5 341   # decompose the hip command there

Why it exists: robot logs 2026-10-07 showed the hip rattling when pushed and braked. Gyro >5 Hz
rose ~10x with the gate-open body damping (HANDOFF 5.50); the V42 sim test covers it.
"""
import argparse
import re
import sys

import numpy as np

KV = re.compile(r"(\w+)=(\[[^\]]*\]|[^\s]+)")
FS = 25.0


def load(path):
    rows = []
    with open(path, "rb") as fh:
        for raw in fh:
            line = raw.decode("latin1")
            if " TR t=" not in line:
                continue
            row = {"time": float(line.split()[0])}
            for key, val in KV.findall(line):
                if val.startswith("["):
                    try:
                        row[key] = [float(x) for x in val.strip("[]").split()]
                    except ValueError:
                        pass
                else:
                    try:
                        row[key] = float(val)
                    except ValueError:
                        row[key] = val
            rows.append(row)
    return rows


def highpass(x, fc=5.0):
    spec = np.fft.rfft(x - x.mean())
    spec[np.fft.rfftfreq(len(x), 1.0 / FS) < fc] = 0.0
    return np.fft.irfft(spec, len(x))


def reversals(x, hyst=0.005):
    n, ext, direction = 0, x[0], 0
    for v in x[1:]:
        if direction >= 0 and v < ext - hyst:
            n, direction, ext = n + 1, -1, v
        elif direction <= 0 and v > ext + hyst:
            n, direction, ext = n + 1, 1, v
        elif (direction >= 0 and v > ext) or (direction <= 0 and v < ext):
            ext = v
    return n


def segments(rows):
    t = np.array([r["time"] for r in rows])
    cuts = np.r_[0, np.where(np.diff(t) > 0.1)[0] + 1, len(t)]
    return [rows[a:b] for a, b in zip(cuts[:-1], cuts[1:]) if b - a >= 100]


def summary(path, tmin):
    rows = [r for r in load(path) if r.get("mode") == "Balancing" and r["time"] > tmin and r["ta"] > 0.5]
    out = []
    for seg in segments(rows):
        g = lambda k: np.array([r[k] for r in seg], float)
        gy_hf, dh_hf = highpass(g("rate")), highpass(g("dhip"))
        v, tvr, dh = np.abs(g("axle_v")), g("tvr"), g("dhip")
        for i in range(0, len(seg) - 50, 25):
            s = slice(i, i + 50)
            out.append((tvr[s].std(), np.sqrt(np.mean(gy_hf[s] ** 2)), np.sqrt(np.mean(dh_hf[s] ** 2)),
                        reversals(dh[s]) / 2.0, np.sqrt(np.mean(np.diff(dh[s]) ** 2)), v[s].max()))
    if not out:
        print(f"{path}: no Balancing data")
        return
    a = np.array(out)
    print(f"\n{path}: {len(a)} windows of 2 s (activity = std of theta_v_rate)")
    print("  activity [rad/s]   n   gyro>5Hz  dhip>5Hz  hip rev/s  hip step")
    for lo, hi in [(0, 0.06), (0.06, 0.12), (0.12, 0.2), (0.2, 0.35), (0.35, 5)]:
        m = (a[:, 0] >= lo) & (a[:, 0] < hi)
        if m.sum() >= 3:
            print(f"  [{lo:4.2f},{hi:4.2f})  {m.sum():6d}   {a[m, 1].mean():7.3f}   {a[m, 2].mean():7.4f}   "
                  f"{a[m, 3].mean():6.1f}    {a[m, 4].mean():7.4f}")


def decompose(path, t0, t1):
    rows = [r for r in load(path) if r.get("mode") == "Balancing" and t0 <= r["time"] <= t1]
    if len(rows) < 40:
        print("span too short")
        return
    g = lambda k: np.array([r[k] for r in rows], float)
    tv, tvr, rate, dh, lead = g("tv"), g("tvr"), g("rate"), g("dhip"), g("adpl")
    lp = np.empty_like(tv)
    lp[0] = tv[0]
    for i in range(1, len(tv)):
        lp[i] = lp[i - 1] + (0.04 / 3.0) * (tv[i] - lp[i - 1])
    terms = {"lean": tv - lp, "lead*tvr": lead * tvr, "0.08*gyro": 0.08 * rate}
    X = np.vstack([highpass(v) for v in terms.values()]).T
    y = highpass(dh)
    coef = np.linalg.lstsq(X, y, rcond=None)[0]
    var = [np.var(X[:, i] * coef[i]) for i in range(3)]
    r2 = 1.0 - np.sum((y - X @ coef) ** 2) / np.sum(y ** 2)
    print(f"{path} [{t0}, {t1}] s: gyro>5Hz {highpass(rate).std():.3f} rad/s, dhip>5Hz {y.std():.4f} rad, "
          f"hip rev/s {reversals(dh) / (t1 - t0):.1f}")
    print("  >5 Hz variance share of dhip: " + ", ".join(f"{k} {100 * v / sum(var):.0f}%" for k, v in zip(terms, var))
          + f"  (R2 {r2:.2f}, coef {np.round(coef, 2)})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--span", nargs=2, type=float, metavar=("T0", "T1"))
    ap.add_argument("--tmin", type=float, default=20.0, help="skip the start-up (s)")
    args = ap.parse_args()
    for path in args.logs:
        if args.span:
            decompose(path, *args.span)
        else:
            summary(path, args.tmin)


if __name__ == "__main__":
    sys.exit(main())
