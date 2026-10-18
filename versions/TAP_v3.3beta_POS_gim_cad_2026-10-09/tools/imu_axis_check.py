#!/usr/bin/env python3
"""Determine the MPU6500 mounting from a live controller console log.

Reads the SENS lines (`chip_acc=[x y z]`, raw chip axes) that
tools/controller_console.py writes, while the operator:

  1. holds the robot upright and still (baseline), then
  2. tilts it NOSE-DOWN by 15-30 deg and holds it 1-2 s.

Physics: tilting nose-down about the body's left axis, the accelerometer's
forward axis reads -g*sin(tilt). So the chip axis whose reading moves the
most is the forward axis, and its sign is opposite to the change. Chip Z up
reads +g at rest. Body left = up x forward (right-handed chip), which fixes
the whole map. Optionally a later LEFT-SIDE-DOWN tilt is checked: the body
left axis must then read negative.

Prints the ImuMount AxisMap for BoardConfig and whether it matches the map
the firmware currently uses (kChipYForwardXRight).
"""

import argparse
import re
import sys
import time

import numpy as np

SENS = re.compile(r"^\s*([0-9.]+) SENS .*? pitch=([-+0-9.]+) .*?"
                  r"chip_acc=\[([-+0-9.]+) ([-+0-9.]+) ([-+0-9.]+)\]")
G = 9.80665
NAMES = "XYZ"


def follow(path, from_start):
    with open(path, errors="replace") as f:
        if not from_start:
            f.seek(0, 2)
        while True:
            line = f.readline()
            if not line:
                time.sleep(0.05)
                continue
            m = SENS.match(line)
            if m:
                t, pitch, x, y, z = map(float, m.groups())
                yield t, pitch, np.array([x, y, z])


def axis_map(forward_axis, forward_sign, up_axis, up_sign):
    """Signed permutation body[i] = sign[i] * chip[axis[i]]."""
    fwd = np.zeros(3)
    fwd[forward_axis] = forward_sign
    up = np.zeros(3)
    up[up_axis] = up_sign
    left = np.cross(up, fwd)  # body Y = Z x X, in chip coordinates
    rows = [fwd, left, up]
    axes = [int(np.argmax(np.abs(r))) for r in rows]
    signs = [int(np.sign(r[a])) for r, a in zip(rows, axes)]
    return axes, signs


def describe(axes, signs):
    words = {0: ("forward", "backward"), 1: ("left", "right"), 2: ("up", "down")}
    out = []
    for chip in range(3):
        body = axes.index(chip)
        s = signs[body]
        out.append(f"chip {NAMES[chip]} -> robot {words[body][0 if s > 0 else 1]}")
    return ", ".join(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="console log being written (run*.log)")
    parser.add_argument("--from-start", action="store_true",
                        help="analyse the whole file instead of only new lines")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--tilt-deg", type=float, default=12.0,
                        help="minimum tilt from the baseline to count")
    args = parser.parse_args()

    tilt_g = G * np.sin(np.radians(args.tilt_deg))
    baseline = None
    still = []
    tilted = []
    nose = None
    deadline = time.monotonic() + args.timeout
    print("Giữ robot đứng yên, rồi chúi đầu xuống 15-30° và giữ 1-2 s.", flush=True)
    for t, pitch, acc in follow(args.log, args.from_start):
        if time.monotonic() > deadline:
            print("TIMEOUT: không thấy lần nghiêng nào đủ lớn.")
            return 2
        if baseline is None:
            if abs(np.linalg.norm(acc) - G) < 0.4:
                still.append(acc)
                if len(still) >= 5:  # ~1 s of SENS at 5 Hz
                    baseline = np.mean(still[-5:], axis=0)
                    up_axis = int(np.argmax(np.abs(baseline)))
                    up_sign = int(np.sign(baseline[up_axis]))
                    print(f"baseline chip_acc={np.round(baseline, 2)} -> chip "
                          f"{NAMES[up_axis]} {'up' if up_sign > 0 else 'down'}", flush=True)
            else:
                still.clear()
            continue
        delta = acc - baseline
        delta[up_axis] = 0.0
        if np.max(np.abs(delta)) > tilt_g and abs(np.linalg.norm(acc) - G) < 1.0:
            tilted.append((t, pitch, acc, delta))
        else:
            tilted.clear()
        if len(tilted) < 4:  # held ~0.8 s
            continue
        d = np.mean([x[3] for x in tilted], axis=0)
        a = np.mean([x[2] for x in tilted], axis=0)
        p = np.mean([x[1] for x in tilted])
        if nose is None:
            nose = (d, a, p)
            fwd_axis = int(np.argmax(np.abs(d)))
            fwd_sign = -int(np.sign(d[fwd_axis]))
            axes, signs = axis_map(fwd_axis, fwd_sign, up_axis, up_sign)
            tilt = np.degrees(np.arcsin(min(abs(d[fwd_axis]) / G, 1.0)))
            print(f"\nTILT seen at t={tilted[0][0]:.1f}s: chip_acc={np.round(a, 2)} "
                  f"(change {np.round(d, 2)}, ~{tilt:.0f} deg), firmware pitch={p:+.3f} rad")
            print("If this tilt was NOSE-DOWN:")
            print(f"  {describe(axes, signs)}")
            print(f"  BoardConfig: imu_mount::AxisMap{{{{{axes[0]}U, {axes[1]}U, {axes[2]}U}}, "
                  f"{{{signs[0]:+d}.0F, {signs[1]:+d}.0F, {signs[2]:+d}.0F}}}}")
            current = ([1, 0, 2], [1, -1, 1])
            match = (axes, signs) == current
            print(f"  current firmware map kChipYForwardXRight: "
                  f"{'MATCHES' if match else 'DOES NOT MATCH'}")
            print(f"  firmware pitch is {'positive (correct)' if p > 0.1 else 'NOT positive'}",
                  flush=True)
            if not match:
                return 1
            return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
