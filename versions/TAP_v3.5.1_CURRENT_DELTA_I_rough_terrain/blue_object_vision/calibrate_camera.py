#!/usr/bin/env python3
"""Calibrate webcam lens distortion from a printed checkerboard."""

import argparse
import math
from pathlib import Path
import sys

import cv2
import numpy as np


def parse_source(value):
    return int(value) if value.isdecimal() else value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="/dev/video4",
                        help="Camera device/path (default: external webcam /dev/video4).")
    default_output = Path(__file__).with_name("camera_calibration.npz")
    parser.add_argument("--output", default=str(default_output),
                        help="Where to save calibration parameters.")
    parser.add_argument("--columns", type=int, default=9,
                        help="Number of inner checkerboard corners across (default: 9).")
    parser.add_argument("--rows", type=int, default=6,
                        help="Number of inner checkerboard corners down (default: 6).")
    args = parser.parse_args()
    if args.columns < 3 or args.rows < 3:
        sys.exit("The checkerboard needs at least 3 inner corners in each direction.")

    capture = cv2.VideoCapture(parse_source(args.source))
    if not capture.isOpened():
        sys.exit(f"Cannot open camera: {args.source}")
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    pattern_size = (args.columns, args.rows)
    object_template = np.zeros((args.rows * args.columns, 3), np.float32)
    object_template[:, :2] = np.mgrid[0:args.columns, 0:args.rows].T.reshape(-1, 2)
    object_points = []
    image_points = []
    image_size = None
    print("Show the printed checkerboard to the camera. Move/tilt it between views.")
    print("Press SPACE to save a detected view; press c after 12+ views to calibrate; q quits.")

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                print("Could not read a camera frame.")
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            image_size = (gray.shape[1], gray.shape[0])

            found, corners = cv2.findChessboardCorners(
                gray, pattern_size,
                flags=(cv2.CALIB_CB_ADAPTIVE_THRESH
                       | cv2.CALIB_CB_NORMALIZE_IMAGE
                       | cv2.CALIB_CB_FAST_CHECK)
            )
            if found:
                corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )
            display = frame.copy()
            if found:
                cv2.drawChessboardCorners(display, pattern_size, corners, found)
                status = f"Board found | saved views: {len(image_points)} | SPACE capture"
            else:
                status = f"Board not found | saved views: {len(image_points)}"
            cv2.putText(display, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.58, (0, 255, 0) if found else (0, 0, 255), 2)
            cv2.imshow("Camera calibration", display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                break
            if key == ord(" ") and found:
                object_points.append(object_template.copy())
                image_points.append(corners.astype(np.float32))
                print(f"Captured view {len(image_points)}")
            elif key == ord("c"):
                if len(image_points) < 12:
                    print(f"Need at least 12 views; currently have {len(image_points)}.")
                    continue
                rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
                    object_points, image_points, image_size, None, None
                )
                rectified_matrix, _ = cv2.getOptimalNewCameraMatrix(
                    camera_matrix, dist_coeffs, image_size, 0.0, image_size
                )
                horizontal_fov_rad = 2.0 * math.atan(
                    image_size[0] / (2.0 * rectified_matrix[0, 0])
                )
                np.savez(
                    args.output,
                    camera_matrix=camera_matrix,
                    dist_coeffs=dist_coeffs,
                    image_size=np.array(image_size, dtype=np.int32),
                    horizontal_fov_rad=np.array(horizontal_fov_rad),
                    rms=np.array(rms),
                )
                print(f"Calibration saved to {args.output}; reprojection RMS={rms:.4f}; "
                      f"rectified horizontal FOV={math.degrees(horizontal_fov_rad):.2f} deg")
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
