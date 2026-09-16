#!/usr/bin/env python3
"""Manual jump-profile launcher. Replaces run_jump.sh (removed 2026-09-16):

    python3 tools/run_jump.py [rated|manual-curve] [extra ros2 launch args]

Requires a sourced ROS 2 environment (`source install/setup.bash` first),
same as every other tools/*.py script - this does no sourcing of its own.
"""
import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tap_launch import JUMP_PROFILES, jump_launch_command

WORKSPACE = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser(
        usage='%(prog)s [rated|manual-curve] [gui:=false ...]')
    parser.add_argument('profile', nargs='?', default='rated', choices=sorted(JUMP_PROFILES))
    parser.add_argument('launch_args', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    try:
        command = jump_launch_command(WORKSPACE, args.profile, args.launch_args)
    except ValueError as exc:
        parser.error(str(exc))
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
