#!/usr/bin/env python3
"""Hip-command chatter statistics from a tap_*.log (Balancing TR lines only).

The chatter signature is large hip-command steps that reverse direction
between consecutive 40 ms TR samples (HANDOFF 5.41): on the 2026-10-01 robot
logs it was 0-2 %, on the rattling sessions of 2026-10-02/03 it was 28-50 %.

    tools/hip_chatter_stats.py ~/tap_logs/tap_20261005_101500.log [--window 10]

Prints one row per window (default 10 s of contiguous Balancing samples):
  alt%    consecutive steps > 0.01 rad that reverse sign, per sample pair
  big%    samples whose step is > 0.01 rad
  dhip sd standard deviation of the tier-2 hip offset [rad]
  rate sd body pitch-rate standard deviation (gyro) [rad/s]
  rg      mean reflex gate weight (only if the firmware logs rg=)
"""
import argparse
import os
import re
import statistics as st

FIELD = {k: re.compile(r'(?:^|\s)' + k + r'=([+-]?[\d.]+)') for k in ('dhip', 'rate', 'rg', 'h')}


def field(line, key):
    m = FIELD[key].search(line)
    return float(m.group(1)) if m else None


def windows(rows, width):
    i = 0
    while i < len(rows):
        j = i
        while j < len(rows) and rows[j][0] < rows[i][0] + width:
            j += 1
        yield rows[i:j]
        i = j


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--window', type=float, default=10.0)
    args = ap.parse_args()
    for path in args.logs:
        rows = []
        for line in open(os.path.expanduser(path), errors='ignore'):
            if ' TR ' in line and 'mode=Balancing' in line:
                d = field(line, 'dhip')
                if d is None:
                    continue
                rows.append((float(line.split()[0]), d, field(line, 'rate') or 0.0, field(line, 'rg')))
        print(f'== {path}: {len(rows)} Balancing samples')
        print('   t0[s]   win  alt%  big%  dhip sd  rate sd   rg')
        for w in windows(rows, args.window):
            if len(w) < 40:
                continue
            steps = [w[k + 1][1] - w[k][1] for k in range(len(w) - 1) if w[k + 1][0] - w[k][0] < 0.06]
            if len(steps) < 30:
                continue
            alt = sum(1 for k in range(len(steps) - 1)
                      if steps[k] * steps[k + 1] < 0 and abs(steps[k]) > 0.01 and abs(steps[k + 1]) > 0.01)
            big = sum(1 for s in steps if abs(s) > 0.01)
            rgs = [x[3] for x in w if x[3] is not None]
            print(f'  {w[0][0]:7.1f} {len(w):4d} {100 * alt / len(steps):5.0f} {100 * big / len(steps):5.0f}'
                  f'  {st.pstdev([x[1] for x in w]):7.4f}  {st.pstdev([x[2] for x in w]):7.3f}  '
                  f'{(f"{st.mean(rgs):.2f}" if rgs else "  - ")}')


if __name__ == '__main__':
    main()
