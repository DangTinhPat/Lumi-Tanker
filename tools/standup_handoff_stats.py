#!/usr/bin/env python3
"""Inspect terminal Catching samples; no robot connection or control commands."""
import argparse
import re
from pathlib import Path


def summarize(path, knee_min, lead, radius):
    groups = {}
    for line in path.read_text(errors="replace").splitlines():
        match = re.search(r" (TR|SU|SV) ", line)
        if match:
            fields = dict(re.findall(r"(\w+)=(\[[^\]]*\]|[^\s]+)", line))
            groups.setdefault(line.split()[0], {})[match[1]] = fields
    samples = []
    for group in groups.values():
        if not all(key in group for key in ("TR", "SU", "SV")):
            continue
        su, tr, sv = (group[key] for key in ("SU", "TR", "SV"))
        if su.get("phase") != "Catching" or float(su["knee_tgt"]) < knee_min:
            continue
        if sv.get("standbrake") != "1" or float(sv["weight"]) < .999:
            continue
        wheels = [float(v) for v in tr["wmeas"].strip("[]").split()]
        command = float(tr["wcmd"].strip("[]").split()[0])
        # Valid for full-blend, unsaturated measured-braking samples only.
        if su.get("wheel_limited") != "0":
            continue
        baseline = command - float(su["wheel_accel"]) * lead / radius
        samples.append((float(su["wheel_error"]), max(abs(baseline-w) for w in wheels),
                        float(su["settle"])))
    if not samples:
        print(f"{path.name}: no eligible samples")
        return
    failures = sum(raw > 4 for raw, _, _ in samples)
    baseline_failures = sum(base > 4 for _, base, _ in samples)
    print(f"{path.name}: n={len(samples)} raw_error>4={failures} "
          f"({100*failures/len(samples):.1f}%) base_error>4={baseline_failures} "
          f"base_max={max(s[1] for s in samples):.3f} "
          f"max_sampled_settle={max(s[2] for s in samples):.3f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--knee-min", type=float, default=.902)
    parser.add_argument("--lead", type=float, default=.075)
    parser.add_argument("--radius", type=float, default=.04)
    args = parser.parse_args()
    if args.radius <= 0:
        parser.error("radius must be positive")
    for path in args.logs:
        summarize(path, args.knee_min, args.lead, args.radius)


if __name__ == "__main__":
    main()
