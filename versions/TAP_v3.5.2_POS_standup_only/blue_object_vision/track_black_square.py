#!/usr/bin/env python3
"""Detect and track people in a webcam or video using Ultralytics YOLO."""

import argparse
from collections import deque
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
from vision_tracker import ApparentAreaTrendFilter, SingleTargetTracker


DEFAULT_MODEL = Path(__file__).resolve().parents[1] / "yolov8n_1(1).onnx"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect and track people with a local Ultralytics YOLO model."
    )
    parser.add_argument(
        "--source", default="/dev/video4",
        help="Camera device/path or video file (default: external webcam /dev/video4)."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL,
                        help="Ultralytics model file (default: local YOLOv8n ONNX).")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="Square YOLO inference size; lower values are faster (default: 640).")
    parser.add_argument("--confidence", type=float, default=0.20,
                        help="Lowest person confidence kept for tracking (default: 0.20).")
    parser.add_argument("--iou", type=float, default=0.45,
                        help="NMS IoU threshold (default: 0.45).")
    parser.add_argument("--min-area", type=float, default=200,
                        help="Minimum person bounding-box area in pixels (default: 200).")
    parser.add_argument("--min-area-fraction", type=float, default=0.0005,
                        help="Minimum person box fraction of frame (default: 0.0005).")
    parser.add_argument("--lost-seconds", type=float, default=0.8,
                        help="Retain a track across brief detection gaps (default: 0.8).")
    parser.add_argument("--association-gate", type=float, default=75,
                        help="Base association distance in pixels for the tracker (default: 75).")
    parser.add_argument("--width", type=int, default=640,
                        help="Maximum processing width in pixels (default: 640).")
    parser.add_argument("--height", type=int, default=480,
                        help="Maximum processing height in pixels (default: 480).")
    parser.add_argument("--fps", type=int, default=60,
                        help="Requested camera frame rate (default: 60).")
    parser.add_argument("--process-fps", type=float, default=30.0,
                        help="Maximum frames processed per second (default: 30).")
    parser.add_argument("--motion-window", type=float, default=0.8,
                        help="Seconds used to judge approach/recede from apparent area (default: 0.8).")
    parser.add_argument("--motion-threshold", type=float, default=0.10,
                        help="Relative area change needed for motion status (default: 0.10).")
    parser.add_argument("--output-format", choices=("text", "jsonl"), default="text",
                        help="Output human-readable status or one JSON record per processed frame.")
    parser.add_argument("--calibration", default="camera_calibration.npz",
                        help="Calibration file created by calibrate_camera.py, if available.")
    parser.add_argument("--no-window", action="store_true",
                        help="Do not show the video window; print tracking coordinates only.")
    return parser.parse_args()


class PersonDetector:
    """Run Ultralytics YOLO once per frame and return only person detections."""

    def __init__(self, model_path, image_size, confidence, iou):
        model_path = Path(model_path).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"YOLO model not found: {model_path}")
        if image_size <= 0 or image_size % 32:
            raise ValueError("--imgsz must be a positive multiple of 32")
        if not 0.0 < confidence < 1.0 or not 0.0 < iou < 1.0:
            raise ValueError("--confidence and --iou must be between 0 and 1")

        import torch
        from ultralytics import YOLO

        torch.set_num_threads(max(1, min(4, torch.get_num_threads())))
        self.model = YOLO(str(model_path), task="detect")
        self.confidence = confidence
        self.iou = iou
        self.image_size = image_size
        names = self.model.names
        self.person_ids = {
            int(class_id) for class_id, name in names.items()
            if str(name).strip().lower() == "person"
        }
        if not self.person_ids:
            raise ValueError(f"Model has no 'person' class: {names}")
        # Warm up so the first live frame does not include one-time setup cost.
        self.model.predict(np.zeros((image_size, image_size, 3), dtype=np.uint8),
                           imgsz=image_size, conf=confidence, iou=iou,
                           classes=sorted(self.person_ids), device="cpu",
                           verbose=False)

    def __call__(self, frame, min_area):
        result = self.model.predict(
            frame, imgsz=self.image_size, conf=self.confidence, iou=self.iou,
            classes=sorted(self.person_ids), device="cpu", verbose=False,
        )[0]
        candidates = []
        boxes = result.boxes
        if boxes is None:
            return candidates
        coordinates = boxes.xyxy.cpu().numpy()
        confidences = boxes.conf.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy()
        for (x1, y1, x2, y2), confidence, class_id in zip(
                coordinates, confidences, class_ids):
            if int(class_id) not in self.person_ids:
                continue
            x1, y1 = max(0, int(x1)), max(0, int(y1))
            x2, y2 = min(frame.shape[1], int(x2)), min(frame.shape[0], int(y2))
            width, height = x2 - x1, y2 - y1
            area = width * height
            if width <= 0 or height <= 0 or area < min_area:
                continue
            candidates.append({
                "box": (x1, y1, width, height),
                "center": (x1 + width // 2, y1 + height // 2),
                "area": area,
                "confidence": float(confidence),
            })
        return candidates


def report(args, message):
    print(message, file=sys.stderr if args.output_format == "jsonl" else sys.stdout, flush=True)


def emit_jsonl(timestamp, tracking, frame_size, motion_status, area_change,
               horizontal_fov_rad=None, processing_fps=None,
               inference_ms=None, frame_age_ms=None):
    height, width = frame_size
    visible = tracking["visible"]
    detection = tracking["detection"]
    center = tracking["center"] if visible else None
    if visible:
        dx = int(center[0] - width // 2)
        normalized_x = float(np.clip(dx / max(width / 2.0, 1.0), -1.0, 1.0))
        area = detection["area"]
        area_fraction = float(area / max(width * height, 1))
    else:
        dx = normalized_x = area = area_fraction = None
    record = {
        "timestamp_monotonic_s": timestamp,
        "image_width_px": width,
        "image_height_px": height,
        "horizontal_fov_rad": horizontal_fov_rad,
        "track_id": tracking["track_id"] if tracking["track_id"] else None,
        "state": tracking["state"],
        "target_visible": visible,
        "confidence": tracking["confidence"],
        "center_px": center,
        "center_x_normalized": normalized_x,
        "horizontal_offset_px": dx,
        "area_px": area,
        "area_fraction": area_fraction,
        "depth_trend": motion_status if visible else "UNKNOWN",
        "area_change_fraction": area_change if visible else None,
        "age_since_seen_s": tracking["age_since_seen_s"],
        "processing_fps": processing_fps,
        "inference_ms": inference_ms,
        "frame_age_ms": frame_age_ms,
    }
    print(json.dumps(record, separators=(",", ":")), flush=True)


def main():
    args = parse_args()
    if args.min_area < 0:
        sys.exit("--min-area must be non-negative")
    if not 0.0 <= args.min_area_fraction <= 1.0:
        sys.exit("--min-area-fraction must be between 0 and 1")
    if args.width <= 0 or args.height <= 0:
        sys.exit("--width and --height must be positive")
    if args.process_fps <= 0:
        sys.exit("--process-fps must be positive")
    if args.motion_window <= 0 or not 0 < args.motion_threshold < 1:
        sys.exit("--motion-window must be positive and --motion-threshold between 0 and 1")
    if args.lost_seconds <= 0:
        sys.exit("--lost-seconds must be positive")
    if args.association_gate <= 0:
        sys.exit("--association-gate must be positive")

    detector = PersonDetector(args.model, args.imgsz, args.confidence, args.iou)

    source = int(args.source) if args.source.isdecimal() else args.source
    is_v4l2_device = isinstance(source, str) and source.startswith("/dev/video")
    capture = cv2.VideoCapture(source, cv2.CAP_V4L2) if is_v4l2_device else cv2.VideoCapture(source)
    if not capture.isOpened():
        sys.exit(f"Cannot open video source: {args.source}")
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if is_v4l2_device:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    if is_v4l2_device:
        report(args, "Camera stream: "
               f"{int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
               f"{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
               f"{capture.get(cv2.CAP_PROP_FPS):.1f} fps, MJPEG requested")

    map_x = map_y = None
    horizontal_fov_rad = None
    calibration = Path(args.calibration)
    if calibration.is_file():
        data = np.load(calibration)
        camera_matrix = data["camera_matrix"]
        dist_coeffs = data["dist_coeffs"]
        calibrated_width, calibrated_height = data["image_size"]
        report(args, f"Lens calibration loaded: {calibration}")
    else:
        camera_matrix = dist_coeffs = None
        report(args, f"No calibration file at {calibration}; displaying uncorrected camera image.")

    tracker = SingleTargetTracker(max_lost_s=args.lost_seconds,
                                base_gate_px=args.association_gate)
    area_history = deque()
    trend_filter = ApparentAreaTrendFilter(
        time_constant_s=0.30,
        enter_fraction=max(args.motion_threshold + 0.03,
                           args.motion_threshold * 1.4),
        release_fraction=args.motion_threshold * 0.5,
    )
    motion_status = "SEARCHING"
    area_change = 0.0
    frame_count = 0
    last_processed_at = 0.0
    fps_window_start = time.monotonic()
    fps_window_frames = 0
    displayed_fps = 0.0
    report(args, f"YOLO person detector ready: {args.model} @ {args.imgsz}px; "
                  "press q or Esc to quit.")

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            # Timestamp at acquisition, before resize/model/tracker work. Using
            # inference completion time here hides model latency from VBC's
            # stale-observation guard and makes filters lag behind the image.
            timestamp = time.monotonic()
            if timestamp - last_processed_at < 1.0 / args.process_fps:
                continue
            last_processed_at = timestamp
            frame_count += 1
            fps_window_frames += 1
            fps_elapsed = timestamp - fps_window_start
            if fps_elapsed >= 0.5:
                displayed_fps = fps_window_frames / fps_elapsed
                fps_window_start = timestamp
                fps_window_frames = 0

            # Some cameras ignore requested capture dimensions. Resize while
            # preserving the image aspect ratio before person detection.
            height, width = frame.shape[:2]
            scale = min(1.0, args.width / width, args.height / height)
            if scale < 1.0:
                frame = cv2.resize(
                    frame, (int(round(width * scale)), int(round(height * scale))),
                    interpolation=cv2.INTER_AREA
                )

            if camera_matrix is not None:
                height, width = frame.shape[:2]
                if map_x is None:
                    # Scale intrinsics if the camera stream resolution differs
                    # from the calibration resolution.
                    scale_x = width / float(calibrated_width)
                    scale_y = height / float(calibrated_height)
                    scaled_matrix = camera_matrix.copy()
                    scaled_matrix[0, :] *= scale_x
                    scaled_matrix[1, :] *= scale_y
                    new_matrix, _ = cv2.getOptimalNewCameraMatrix(
                        scaled_matrix, dist_coeffs, (width, height), 0.0, (width, height)
                    )
                    horizontal_fov_rad = 2.0 * np.arctan(
                        width / (2.0 * new_matrix[0, 0])
                    ).item()
                    map_x, map_y = cv2.initUndistortRectifyMap(
                        scaled_matrix, dist_coeffs, None, new_matrix,
                        (width, height), cv2.CV_16SC2
                    )
                frame = cv2.remap(frame, map_x, map_y, interpolation=cv2.INTER_LINEAR)

            min_area_px = max(args.min_area, int(np.ceil(
                frame.shape[0] * frame.shape[1] * args.min_area_fraction)))
            inference_started = time.monotonic()
            candidates = detector(frame, min_area_px)
            inference_ms = (time.monotonic() - inference_started) * 1000.0
            tracking = tracker.update(candidates, timestamp)
            target = tracking["detection"]
            height, width = frame.shape[:2]
            frame_center = (width // 2, height // 2)
            cv2.line(frame, (frame_center[0], 0), (frame_center[0], height - 1),
                     (0, 255, 255), 2)

            if tracking["visible"]:
                x, y, box_width, box_height = target["box"]
                tracked = tracking["center"]
                smooth_area = tracking["area_smooth"]
                area_history.append((timestamp, smooth_area))
                while len(area_history) > 1 and timestamp - area_history[1][0] >= args.motion_window:
                    area_history.popleft()
                if area_history and timestamp - area_history[0][0] >= args.motion_window * 0.8:
                    reference_area = area_history[0][1]
                    area_change = (smooth_area - reference_area) / max(reference_area, 1.0)
                    motion_status = trend_filter.update(area_change, timestamp)
                else:
                    motion_status = "MEASURING"
                    trend_filter.reset()

                box_color = (0, 165, 255) if tracking["state"] == "COASTING" else (0, 255, 0)
                cv2.rectangle(frame, (x, y), (x + box_width, y + box_height), box_color, 2)
                cv2.line(frame, (frame_center[0], tracked[1]), tracked, (255, 100, 0), 2)
                dx, dy = tracked[0] - frame_center[0], tracked[1] - frame_center[1]
                distance_x = abs(dx)
                cv2.putText(frame, f"PERSON  dx={dx:+} px  distance={distance_x} px",
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 0), 2)
                cv2.putText(frame, f"Depth trend: {motion_status}  area change={area_change:+.0%}",
                            (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)
                state_label = "brief prediction" if tracking["state"] == "COASTING" else "tracking"
                cv2.putText(frame, f"{state_label} id={tracking['track_id']} "
                            f"confidence={tracking['confidence']:.2f}",
                            (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
                if args.output_format == "jsonl":
                    emit_jsonl(
                        timestamp, tracking, (height, width), motion_status,
                        area_change, horizontal_fov_rad, displayed_fps,
                        inference_ms, (time.monotonic() - timestamp) * 1000.0)
                elif frame_count % 10 == 0:
                    print(f"center_x={tracked[0]} center_y={tracked[1]} "
                          f"horizontal_offset_px={dx} horizontal_distance_px={distance_x} "
                          f"state={tracking['state']} "
                          f"depth_trend={motion_status} area_change={area_change:+.1%} "
                          f"confidence={tracking['confidence']:.2f} "
                          f"fps={displayed_fps:.1f} inference_ms={inference_ms:.1f}",
                          flush=True)
            else:
                area_history.clear()
                trend_filter.reset()
                if tracking["state"] == "TENTATIVE" and target is not None:
                    x, y, box_width, box_height = target["box"]
                    cv2.rectangle(frame, (x, y), (x + box_width, y + box_height),
                                  (0, 200, 255), 1)
                    cv2.putText(frame, "Confirming person...", (10, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 200, 255), 2)
                elif tracking["state"] == "COASTING":
                    cv2.putText(frame, "TARGET LOST - reacquiring", (10, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 165, 255), 2)
                else:
                    motion_status = "SEARCHING"
                    area_change = 0.0
                    cv2.putText(frame, "Person not found", (10, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                if args.output_format == "jsonl":
                    emit_jsonl(
                        timestamp, tracking, (height, width), motion_status,
                        area_change, horizontal_fov_rad, displayed_fps,
                        inference_ms, (time.monotonic() - timestamp) * 1000.0)
                elif frame_count % 10 == 0:
                    print(f"target_visible=false state={tracking['state']} "
                          f"age_since_seen_s={tracking['age_since_seen_s']}", flush=True)

            if not args.no_window:
                cv2.putText(frame, f"Processing: {displayed_fps:.0f} FPS  "
                            f"YOLO: {inference_ms:.0f} ms",
                            (max(10, width - 190), 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.58, (255, 255, 255), 2)
                cv2.putText(frame, f"YOLO input: {args.imgsz}px; min box: {min_area_px}px",
                            (10, max(20, height - 12)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.52, (255, 255, 255), 2)
                cv2.imshow("YOLOv8n person tracking", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
    finally:
        capture.release()
        if not args.no_window:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
