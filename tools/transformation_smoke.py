#!/usr/bin/env python3
"""Sensor-only pose-transition smoke test.

All acceptance signals come from the controller's MPU6500 pitch estimate,
joint encoders, and encoder trajectory observer.  The simulator model state
is not published, subscribed to, or used for scoring.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import copy
import math

try:
    import rclpy
except ImportError:  # Offline helper tests do not require ROS to run.
    rclpy = None

ROOT = Path(__file__).resolve().parents[1]


def _ros_contract_init():
    if rclpy is not None:
        rclpy.init(args=[])


def estop_observation_zero(cache, stop_wall, now_wall, require_applied=False,
                            stop_stamp=None, now_stamp=None):
    """Accept only fresh zero command and six-channel encoder/effort evidence."""
    for key, count in (("legs", 4), ("wheels", 2)):
        sample = cache.get(key)
        if sample is None or len(sample.get("data", ())) != count:
            return False
        if sample.get("wall", -math.inf) < stop_wall or now_wall - sample["wall"] > .2:
            return False
        if any(not math.isfinite(v) or abs(v) > 1e-9 for v in sample["data"]):
            return False
    if not require_applied:
        return True
    joints = cache.get("joints")
    efforts = None if joints is None else joints.get("data", {}).get("efforts")
    if efforts is None or len(efforts) != 6:
        return False
    stamp = joints.get("stamp")
    if stop_stamp is None or now_stamp is None or stamp is None:
        return False
    if stamp <= stop_stamp + .01 or stamp > now_stamp:
        return False
    if joints.get("wall", -math.inf) < stop_wall or now_wall - joints["wall"] > .2:
        return False
    return all(math.isfinite(v) and abs(v) <= 1e-9 for v in efforts)


_geometry_last = None


def append_geometry_sample(sample_time, previous_time, current_pair, previous_pair):
    """Deduplicate encoder/IMU geometry by source pair, never by scene pose."""
    global _geometry_last
    if sample_time is None or (previous_time is not None and sample_time < previous_time):
        return False
    pair = tuple(current_pair) if current_pair is not None else None
    if pair is None or (previous_pair is not None and pair == tuple(previous_pair)):
        return False
    if _geometry_last is not None and sample_time < _geometry_last:
        return False
    _geometry_last = sample_time
    return True


def model_launch_arguments(arguments):
    allowed = {"jump_model", "jump_leg_speed", "jump_leg_velocity_damping", "spawn_z"}
    result = []
    seen = set()
    for argument in arguments:
        if ":=" not in argument:
            raise ValueError("malformed launch argument")
        key, value = argument.split(":=", 1)
        if key not in allowed or not value or key in seen:
            raise ValueError("unsupported or duplicate launch argument")
        seen.add(key)
        if key != "spawn_z":
            result.append(argument)
    return result


def validate_deep_request(depth, mode, amount):
    if depth is not None and (not math.isfinite(depth) or not 1.0 <= depth <= 1.6):
        raise ValueError("depth outside authorized range")
    if mode == "all" and depth is None and amount is None:
        return
    if mode not in ("squat",):
        raise ValueError("only squat is authorized")
    if amount is not None and amount != 1.0:
        raise ValueError("amount outside authorized range")


def check_absolute_pitch(pitch, bound):
    if bound is None:
        return
    if not math.isfinite(pitch) or abs(pitch) > bound:
        raise RuntimeError("Absolute pitch exceeds bound")


def motion_curves(mode=None):
    if mode is None:
        return ("squat", "roll_left", "roll_right")
    if mode == "squat":
        return ("squat",)
    raise KeyError(mode)


def smooth5(t):
    t = max(0.0, min(1.0, t))
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def command_due(now, last):
    return now is not None and (last is None or now - last >= .05 - 1e-12)


def merge_config(base, override):
    result = copy.deepcopy(base)
    def merge(dst, src):
        for key, value in src.items():
            if isinstance(value, dict) and isinstance(dst.get(key), dict):
                merge(dst[key], value)
            else:
                dst[key] = copy.deepcopy(value)
    merge(result, override)
    return result


def metrics(rows, anchor):
    if not rows:
        return {"peak_xy_m": 0.0, "residual_xy_m": 0.0, "path_length_m": 0.0}
    ax, ay = anchor["x"], anchor["y"]
    distances = [math.hypot(row["x"] - ax, row["y"] - ay) for row in rows]
    path = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"])
               for a, b in zip(rows, rows[1:]))
    return {"peak_xy_m": max(distances),
            "residual_xy_m": distances[-1], "path_length_m": path}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--domain', type=int, default=127)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--config', type=str, default=None)
    args = parser.parse_args()
    command = [sys.executable, str(ROOT / 'tools/sim_smoke.py'),
               '--duration', str(args.duration), '--poses',
               '--domain', str(args.domain)]
    if args.gui:
        command.append('--gui')
    if args.config:
        command += ['--controller-config', args.config]
    result = subprocess.run(command, cwd=ROOT, env=os.environ.copy())
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
