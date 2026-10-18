#!/usr/bin/env python3
"""Standing-still statistics from robot logs and from the tier sim, side by side.

  python3 tools/tr_rest_stats.py ~/tap_logs/*.log
  python3 tools/tr_rest_stats.py --sim run.csv [--sim run2.csv ...]

Robot logs: TR lines (25 Hz) while Balancing, cut into 10 s windows that are
standing still - no DRIVE/HMOVE in the window or the 3 s before it, leg
height steady (h within 2 mm), virtual pitch within 0.1 rad (no fall or
hard push). Sim: the TR_ANIM CSV of v3_tier_tests (TIER_DIAG with no push),
after 12 s, sampled at 25 Hz like the TR line.

Per window: dhip pp/rms (tier-2 hip offset), the median of its 1 s pp and
the burst ratio (largest 1 s pp / that median; a hand on the robot reads
as a burst - --calm drops those windows), axle_x pp (encoder odometry),
rate rms (IMU pitch rate), tvr rms (virtual pitch rate), axle_v std, and
(with numpy) the IMU rate's spectral peak and its share in 1-4 Hz. Printed
per 10 mm height bin as median and 90th percentile over the windows.
"""
import argparse
import csv
import math
import re
import statistics
import sys
from collections import defaultdict

try:
    import numpy as np
except ImportError:  # spectra are optional
    np = None

KV = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")
WIN_S = 10.0
QUIET_S = 3.0


def num(v):
    try:
        return float(v)
    except ValueError:
        return float("nan")


def window_stats(rows):
    def pp(k):
        xs = [r[k] for r in rows]
        return max(xs) - min(xs)

    def rms(k, center=True):
        xs = [r[k] for r in rows if not math.isnan(r[k])]
        if not xs:
            return float("nan")
        m = statistics.fmean(xs) if center else 0.0
        return math.sqrt(statistics.fmean([(x - m) ** 2 for x in xs]))

    # Where the IMU pitch rate's power sits: the robot's standing rocking is
    # a 1.5-2.3 Hz closed-loop mode (TR, 2026-10-02), sensor noise is flat.
    pk = share = float("nan")
    xs = [r["rate"] for r in rows]
    if np is not None and len(xs) > 32 and not any(math.isnan(x) for x in xs):
        fs = (len(rows) - 1) / max(rows[-1]["t"] - rows[0]["t"], 1e-6)
        x = np.asarray(xs) - statistics.fmean(xs)
        p = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
        f = np.fft.rfftfreq(len(x), 1.0 / fs)
        if p[1:].sum() > 0:
            pk = float(f[1 + int(np.argmax(p[1:]))])
            share = float(p[(f >= 1.0) & (f < 4.0)].sum() / p[1:].sum())
    # Hands on the robot (a press, a test push) show up as bursts inside a
    # quiet window; the median of 1 s chunks is the standing level itself.
    chunks = defaultdict(list)
    for r in rows:
        chunks[int(r["t"] - rows[0]["t"])].append(r["dhip"])
    pps = sorted(max(v) - min(v) for v in chunks.values() if len(v) > 5)
    pp1 = statistics.median(pps) if pps else float("nan")
    burst = pps[-1] / max(pp1, 1e-6) if pps else float("nan")
    return {
        "h_mm": statistics.fmean([r["h"] for r in rows]),
        "dhip_pp_1s": pp1,
        "burst": burst,
        "rate_pk_hz": pk,
        "rate_1_4hz": share,
        "dhip_pp": pp("dhip"),
        "dhip_rms": rms("dhip"),
        "ax_pp_mm": 1000.0 * pp("ax"),
        "rate_rms": rms("rate", center=False),
        "tvr_rms": rms("tvr", center=False),
        "v_std": rms("v"),
    }


def cut_windows(rows, quiet_ok):
    """rows: dicts with t (s) and the signals; quiet_ok(t0, t1) -> bool."""
    out = []
    i = 0
    while i < len(rows):
        t0 = rows[i]["t"]
        j = i
        while j < len(rows) and rows[j]["t"] < t0 + WIN_S:
            j += 1
        w = rows[i:j]
        if j >= len(rows) or len(w) < 0.8 * WIN_S * 25:
            break
        hs = [r["h"] for r in w]
        if (quiet_ok(t0, t0 + WIN_S) and max(hs) - min(hs) < 2.0 and
                max(abs(r["tv"]) for r in w) < 0.1):
            out.append(window_stats(w))
        i = j
    return out


def robot_log(path):
    rows, events, hipv = [], [], []
    seg = []
    segments = []
    with open(path, "rb") as f:
        for raw in f:
            line = raw.decode("utf-8", "replace").strip()
            parts = line.split(None, 1)
            if len(parts) < 2:
                continue
            try:
                ts = float(parts[0])
            except ValueError:
                continue
            body = parts[1]
            if body.startswith(">>"):
                cmd = body[2:].split()
                if cmd and cmd[0] in ("DRIVE", "HMOVE"):
                    moving = any(abs(num(x)) > 1e-3 for x in cmd[1:3])  # DRIVE 0 0 / HMOVE 0 stop
                    events.append((ts, moving))
                continue
            if body.startswith("SENS") and "state=Balancing" in body:
                m = re.search(r"hipv_rms=(\S+)", body)
                if m:
                    hipv.append(num(m.group(1)))
                continue
            if not body.startswith("TR t="):
                continue
            kv = dict(KV.findall(body))
            if kv.get("mode") != "Balancing":
                if seg:
                    segments.append(seg)
                    seg = []
                continue
            if "h" not in kv or "dhip" not in kv:
                continue  # before v3.3 (no leg height / tier-2 fields)
            seg.append({"t": ts, "h": num(kv["h"]), "dhip": num(kv["dhip"]),
                        "ax": num(kv.get("axle_x", "nan")), "v": num(kv.get("axle_v", "nan")),
                        "rate": num(kv.get("rate", "nan")), "tvr": num(kv.get("tvr", "nan")),
                        "tv": num(kv.get("tv", "0"))})
    if seg:
        segments.append(seg)

    # Drive/height commands: a window is quiet if no moving command was
    # active in it or in the QUIET_S before it (DRIVE 0 0 ends a drive).
    def quiet_ok(t0, t1):
        active = False
        for ts, moving in events:
            if ts > t1:
                break
            if ts >= t0 - QUIET_S and moving:
                return False
            active = moving
        return not active

    wins = []
    for s in segments:
        wins += cut_windows(s, quiet_ok)
    return wins, hipv


def sim_csv(path):
    rows = []
    with open(path, newline="") as f:
        for k, r in enumerate(csv.DictReader(f)):
            if k % 4:  # 100 Hz -> 25 Hz
                continue
            t = float(r["t"])
            if t < 12.0:
                continue
            rows.append({"t": t, "h": 1000.0 * (float(r["hip_z"]) - float(r["axle_z"])),
                         "dhip": float(r["trunk_hip"]), "ax": float(r.get("gx", r["axle_x"])),
                         "v": float(r["v"]), "rate": float(r.get("rate", "nan")),
                         "tvr": float(r.get("tvr", "nan")), "tv": float(r["lean"])})
    return cut_windows(rows, lambda a, b: True)


def report(title, wins, calm=False):
    if calm:
        wins = [w for w in wins if w["burst"] <= 3.0]
    print(f"== {title}: {len(wins)} windows of {WIN_S:.0f} s{' (calm only: burst <= 3)' if calm else ''}")
    bins = defaultdict(list)
    for w in wins:
        bins[int(round(w["h_mm"] / 10.0) * 10)].append(w)
    keys = ["dhip_pp", "dhip_pp_1s", "burst", "dhip_rms", "ax_pp_mm", "rate_rms", "tvr_rms", "v_std",
            "rate_pk_hz", "rate_1_4hz"]
    print(f"{'h mm':>5} {'n':>3} " + " ".join(f"{k:>17}" for k in keys))
    for h in sorted(bins, reverse=True):
        ws = bins[h]
        cells = []
        for k in keys:
            xs = sorted(w[k] for w in ws if not math.isnan(w[k]))
            if not xs:
                cells.append(f"{'-':>17}")
                continue
            med = statistics.median(xs)
            p90 = xs[min(len(xs) - 1, int(0.9 * len(xs)))]
            cells.append(f"{med:8.4f}/{p90:<8.4f}")
        print(f"{h:>5} {len(ws):>3} " + " ".join(cells))
    print("   (median/p90 per window)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="*", help="robot logs (~/tap_logs/*.log)")
    ap.add_argument("--sim", action="append", default=[], help="v3_tier_tests TR_ANIM CSV")
    ap.add_argument("--calm", action="store_true",
                    help="drop windows with bursts (largest 1 s hip pp > 3x the median 1 s pp: hands on it)")
    a = ap.parse_args()
    if not a.logs and not a.sim:
        ap.print_help()
        return 2
    if a.logs:
        wins, hipv = [], []
        for p in a.logs:
            w, h = robot_log(p)
            wins += w
            hipv += h
        report("robot", wins, a.calm)
        if hipv:
            print(f"   SENS hipv_rms while balancing: median {statistics.median(hipv):.4f} rad/s (n {len(hipv)})")
    if a.sim:
        wins = []
        for p in a.sim:
            wins += sim_csv(p)
        report("sim", wins, a.calm)
    return 0


if __name__ == "__main__":
    sys.exit(main())
