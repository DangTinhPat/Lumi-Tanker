#!/usr/bin/env python3
"""Regenerate GUI trajectory tables after changing CAD geometry or masses."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tap_teleop'))
from tap_teleop.pose_curves import generate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xacro', type=Path, default=ROOT / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro')
    parser.add_argument('--output', type=Path, default=ROOT / 'src/tap_teleop/config/pose_curves.json')
    args = parser.parse_args()
    result = generate(args.xacro)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(f"Shortening sign: {result['shortening_sign']:+g}; neutral pose: {result['rest_pose']}")
    print(f"Squat drop: {result['squat_drop_m'] * 1000:.1f} mm; wrote {args.output}")


if __name__ == '__main__':
    main()
