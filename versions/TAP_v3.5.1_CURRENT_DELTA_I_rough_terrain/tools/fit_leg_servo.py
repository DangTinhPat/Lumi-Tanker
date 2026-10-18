#!/usr/bin/env python3
"""Fit the GDS68 position-servo model to a joint-test recording.

Reads a ~/tap_logs file written by tools/tap_serial_control_gui.py and takes
either the old HIPTEST dump ("HT" lines) or a recorder dump ("RC" lines from
HIPTEST / KNEETEST / KNEETEST FLOOR / REC, see firmware_esp32/src/AppV3.cpp).

Model (the one the two-body sim uses, src_fw/tests/v3_tier_tests.cpp):
    q'' = bw * (kp * (cmd(t - delay) - q) - q')
fitted by a vectorised grid search, then a finer grid around the best point.
Prints the parameters, the rms error, and the measured vs modelled gain and
phase of every sine segment.

    python3 tools/fit_leg_servo.py ~/tap_logs/tap_20260930_090326.log
    python3 tools/fit_leg_servo.py LOG --list
    python3 tools/fit_leg_servo.py LOG --index 2 --side L
    python3 tools/fit_leg_servo.py LOG --delay 3   # KNEETEST FLOOR

KNEETEST FLOOR only reaches 3 Hz, where the velocity-loop lag and the dead
time both just add phase and trade off against each other (synthetic check:
kp right, bw/delay 43/14 ms for a true 35/8 ms, same gain/phase in band).
Fix the delay from the stand test (to 8 Hz) with --delay.
"""

import argparse
import math
import re
import sys

import numpy as np

STAND_SINES = [(2.0 + k, 3.0 + k, float(1 << k)) for k in range(4)]  # 1/2/4/8 Hz


def floor_sines():
    out, t0 = [], 0.5
    for f, cycles in ((0.5, 1), (1.0, 2), (2.0, 3), (3.0, 4)):
        out.append((t0, t0 + cycles / f, f))
        t0 += cycles / f
    return out


def read_recordings(path):
    """Every HT/RC block in the log: dict(kind, header, t[s], columns)."""
    recs, cur = [], None
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            m = re.search(r"\b(HT|RC) (.*)$", raw.rstrip("\n"))
            if not m:
                continue
            tag, rest = m.group(1), m.group(2)
            if rest.startswith("BEGIN"):
                header = dict(kv.split("=", 1) for kv in rest.split()[1:] if "=" in kv)
                cols = header.get("cols", "").split(",")
                scales = [float(x) for x in header.get("scales", "").split(",") if x]
                kind = header.get("kind", "HIPTEST") if tag == "RC" else "HIPTEST"
                cur = {"kind": kind, "header": header, "cols": cols, "scales": scales,
                       "tag": tag, "rows": []}
                continue
            if rest.startswith("END"):
                if cur is not None:
                    recs.append(cur)
                cur = None
                continue
            if cur is not None:
                try:
                    cur["rows"].append([float(x) for x in rest.split()])
                except ValueError:
                    pass
    if cur is not None and cur["rows"]:
        recs.append(cur)  # dump cut short: keep what arrived
    out = []
    for r in recs:
        a = np.array(r["rows"])
        if a.ndim != 2 or len(a) < 50:
            continue
        data = {"t": a[:, 0] * 1e-3}
        names = r["cols"][1:]
        for i, name in enumerate(names):
            if i + 1 >= a.shape[1]:
                break
            scale = r["scales"][i] if r["tag"] == "RC" and i < len(r["scales"]) else 1.0
            data[name] = a[:, i + 1] * scale
        out.append({"kind": r["kind"], "header": r["header"], "data": data})
    return out


def signals(rec, side):
    """(t, cmd, q) for the joint the test excited."""
    d, kind = rec["data"], rec["kind"]
    if "hipL" in d and kind == "HIPTEST":
        cmd = d.get("hipcmd", d.get("cmd"))
        ql, qr = d["hipL"], d["hipR"]
    else:
        cmd = d["kneecmd"]
        ql, qr = d["kneeL"], d["kneeR"]
    q = ql if side == "L" else qr if side == "R" else 0.5 * (ql + qr)
    return d["t"], cmd, q


def simulate(t, cmd, q0, kp, bw, delay, sub=8):
    """Vectorised over parameter arrays kp, bw, delay (same shape)."""
    kp, bw, delay = np.broadcast_arrays(np.asarray(kp, float), np.asarray(bw, float),
                                        np.asarray(delay, float))
    q = np.full(kp.shape, q0, float)
    qd = np.zeros(kp.shape, float)
    out = np.empty((len(t),) + kp.shape)
    out[0] = q
    for i in range(1, len(t)):
        dt = (t[i] - t[i - 1]) / sub
        for k in range(sub):
            tk = t[i - 1] + (k + 0.5) * dt
            c = np.interp(tk - delay, t, cmd)
            acc = bw * (kp * (c - q) - qd)
            qd = qd + acc * dt
            q = q + qd * dt
        out[i] = q
    return out


def score(t, q, sim):
    err = sim - q.reshape((-1,) + (1,) * (sim.ndim - 1))
    err = err - err.mean(axis=0)  # a constant zero offset is not dynamics
    return np.sqrt((err ** 2).mean(axis=0))


def fit(t, cmd, q, fixed_delay=None):
    kp_g = np.linspace(4.0, 40.0, 13)
    bw_g = np.geomspace(10.0, 200.0, 14)
    d_g = np.arange(0.0, 0.031, 0.004) if fixed_delay is None else np.array([fixed_delay])
    KP, BW, D = np.meshgrid(kp_g, bw_g, d_g, indexing="ij")
    s = score(t, q, simulate(t, cmd, q[0], KP, BW, D, sub=4))
    i = np.unravel_index(np.argmin(s), s.shape)
    kp0, bw0, d0 = KP[i], BW[i], D[i]
    KP, BW, D = np.meshgrid(np.linspace(kp0 * 0.8, kp0 * 1.25, 11),
                            np.linspace(bw0 * 0.8, bw0 * 1.25, 11),
                            np.clip(np.linspace(d0 - 0.004, d0 + 0.004, 9), 0.0, None)
                            if fixed_delay is None else np.array([fixed_delay]),
                            indexing="ij")
    s = score(t, q, simulate(t, cmd, q[0], KP, BW, D))
    i = np.unravel_index(np.argmin(s), s.shape)
    return float(KP[i]), float(BW[i]), float(D[i]), float(s[i])


def sine_response(t, x, ref, t0, t1, f):
    """Gain and phase [deg] of x relative to ref at frequency f on [t0, t1)."""
    m = (t >= t0) & (t < t1)
    if m.sum() < 10:
        return float("nan"), float("nan")
    w = 2 * math.pi * f
    basis = np.exp(-1j * w * t[m])
    xr = np.sum((x[m] - x[m].mean()) * basis)
    rr = np.sum((ref[m] - ref[m].mean()) * basis)
    if abs(rr) < 1e-12:
        return float("nan"), float("nan")
    h = xr / rr
    return abs(h), math.degrees(math.atan2(h.imag, h.real))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--list", action="store_true", help="list the recordings and exit")
    ap.add_argument("--index", type=int, default=-1, help="recording index (default: last)")
    ap.add_argument("--side", choices=("L", "R", "mean"), default="mean")
    ap.add_argument("--delay", type=float, default=None, help="fix the dead time [ms]")
    args = ap.parse_args()

    recs = read_recordings(args.log)
    if not recs:
        sys.exit("no HT/RC recordings in " + args.log)
    if args.list:
        for i, r in enumerate(recs):
            print(f"[{i}] {r['kind']:15s} n={len(r['data']['t'])} "
                  f"amp={r['header'].get('amp', '?')} aborted={r['header'].get('aborted', '0')}")
        return
    rec = recs[args.index]
    if rec["kind"] == "REC":
        sys.exit("REC recordings have no excitation to fit; use --list / analyse directly")
    t, cmd, q = signals(rec, args.side)
    kp, bw, delay, rms = fit(t, cmd, q, None if args.delay is None else args.delay * 1e-3)
    swing = np.sqrt(np.mean((cmd - cmd.mean()) ** 2))
    wn = math.sqrt(kp * bw)
    print(f"{rec['kind']} side={args.side} n={len(t)} amp={rec['header'].get('amp', '?')}")
    print(f"  kp {kp:.1f} /s  bw {bw:.1f} /s  delay {delay * 1e3:.1f} ms  "
          f"(wn {wn / (2 * math.pi):.2f} Hz, zeta {bw / (2 * wn):.2f})")
    print(f"  rms error {rms * 1e3:.2f} mrad of {swing * 1e3:.1f} mrad command swing "
          f"({100 * rms / max(swing, 1e-9):.0f} %)")
    sim = simulate(t, cmd, q[0], kp, bw, delay)
    segs = floor_sines() if rec["kind"] == "KNEETEST_FLOOR" else STAND_SINES
    print("  freq   measured gain/phase   model gain/phase")
    for t0, t1, f in segs:
        g, p = sine_response(t, q, cmd, t0, t1, f)
        gm, pm = sine_response(t, sim, cmd, t0, t1, f)
        print(f"  {f:4.1f} Hz   {g:5.2f} {p:+7.1f} deg      {gm:5.2f} {pm:+7.1f} deg")
    d = rec["data"]
    if rec["kind"] == "KNEETEST_FLOOR" and "tv" in d:
        lean = d["tv"] - d["tv"][: max(1, int(0.4 * 250))].mean()
        body = d["pitch"] - d["pitch"][: max(1, int(0.4 * 250))].mean()
        print(f"  CoM line swing max {np.abs(lean).max():.4f} rad, body {np.abs(body).max():.4f} rad "
              f"(sim V21: <= 0.05 / 0.02)")


if __name__ == "__main__":
    main()
