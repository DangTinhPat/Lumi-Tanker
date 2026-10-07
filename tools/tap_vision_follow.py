#!/usr/bin/env python3
"""Follow a visible blue target through an existing ESP32 serial FIFO."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "gui"))

from gui.fifo_motion_backend import FifoMotionBackend
from gui.motion_api import MotionAPI, MotionLimits
from gui.vision_behavior import VisionBehaviorConfig, VisionTarget, Vbc
from gui.vision_process import BlueVisionProcess, calibrated_horizontal_fov


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fifo", required=True,
                        help="FIFO opened by tools/controller_console.py --fifo")
    parser.add_argument("--camera-source", default="/dev/video4")
    parser.add_argument("--calibration",
                        default=str(ROOT / "blue_object_vision" /
                                    "camera_calibration.npz"))
    parser.add_argument(
        "--yaw-sign", type=float, choices=(-1.0, 1.0), required=True,
        help=("yaw polarity: +1 maps a right-side target to positive ESP DRIVE yaw; "
              "-1 maps it to negative yaw. Resolve from a supervised direction check."),
    )
    parser.add_argument(
        "--robot-balanced", action="store_true", required=True,
        help="confirm the ESP32 robot is already safely in Balance before tracking",
    )
    return parser.parse_args(argv)


def vision_target(record):
    if record is None or record.get("target_visible") is not True:
        return None
    try:
        return VisionTarget(
            horizontal_error_px=float(record["horizontal_offset_px"]),
            image_width_px=float(record["image_width_px"]),
            observed_at_s=float(record["timestamp_monotonic_s"]),
            horizontal_fov_rad=float(record["horizontal_fov_rad"]),
            depth_trend=str(record.get("depth_trend", "UNKNOWN")),
            area_change_fraction=(
                None if record.get("area_change_fraction") is None
                else float(record["area_change_fraction"])),
            area_fraction=(
                None if record.get("area_fraction") is None
                else float(record["area_fraction"])),
        )
    except (KeyError, TypeError, ValueError):
        return None


def main(argv=None):
    args = parse_args(argv)
    calibration = Path(args.calibration)
    if not calibration.is_file():
        raise SystemExit(
            f"camera calibration missing: {calibration}; run "
            "python3 blue_object_vision/calibrate_camera.py first")

    fifo_backend = FifoMotionBackend(Path(args.fifo))
    fov = calibrated_horizontal_fov(calibration)
    motion = MotionAPI(
        fifo_backend,
        limits=MotionLimits(max_linear_m_s=0.20, max_yaw_rad_s=0.40),
    )
    behavior = Vbc(
        motion,
        VisionBehaviorConfig(
            horizontal_fov_rad=fov,
            yaw_gain_s_inv=2.0,
            max_yaw_rad_s=0.40,
            max_yaw_accel_rad_s2=1.8,
            approach_accel_m_s2=0.5,
            approach_speed_m_s=0.12,
            yaw_sign=args.yaw_sign,
        ),
    )
    vision = BlueVisionProcess(ROOT / "blue_object_vision")
    vision.start(source=args.camera_source)
    print("Vision following active; it sends bounded DRIVE only and never START/GO.")
    print("Press Ctrl-C to send STOP and exit.", flush=True)

    latest_record = None
    last_status = None
    try:
        while vision.running:
            record = vision.read_latest()
            if record is not None:
                latest_record = record
            target = vision_target(latest_record)
            usable = behavior.update(target)
            status = ("tracking" if usable else "stopped/lost/stale")
            if status != last_status:
                print(f"Vision: {status}", flush=True)
                last_status = status
            time.sleep(0.02)
        detail = vision.errors[-1] if vision.errors else "camera tracker exited"
        raise RuntimeError(detail)
    except KeyboardInterrupt:
        pass
    finally:
        vision.stop()
        try:
            behavior.update(None)
        except OSError as exc:
            print(f"Could not write final STOP to FIFO: {exc}", file=sys.stderr)


if __name__ == "__main__":
    main()
