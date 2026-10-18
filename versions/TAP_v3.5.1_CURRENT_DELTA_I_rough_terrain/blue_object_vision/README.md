# Person vision

Standalone person detection with the local YOLOv8n ONNX model, followed by the
existing motion-aware single-target tracker. Inference uses CPU at 640 pixels
and is capped at 30 FPS at 640×480 by default. The live window shows measured
processing rate and YOLO inference time. The program does not send robot
commands.

## Run

The GUI uses Ultralytics from the configured Python environment and the local
`yolov8n_1(1).onnx` model by default:

```bash
./run.sh --source /dev/video4
```

Override the Python environment or model path if needed:

```bash
LUMI_VISION_PYTHON=/path/to/python LUMI_PERSON_MODEL=/path/to/model.onnx ./run.sh
```

The default camera is `/dev/video4` (the external webcam). Press `q` or `Esc`
in the video window to quit. To choose another source or adjust the processing
rate:

```bash
./run.sh --source /dev/video2 --process-fps 15
```

The detector keeps only the model's `person` class, then rejects detections
whose bounding boxes are smaller than the larger of 200 pixels and 0.05% of
the processed image. Inference settings can be adjusted:

```bash
./run.sh --imgsz 640 --confidence 0.25 --process-fps 15
```

The detector follows a ByteTrack-style rule: detections scoring at least 0.08
may initialize a target; weaker detections can maintain an existing association
but cannot start a new track. Initial selection favors the largest qualifying
person box to prefer the foreground person.

The active model is the standard COCO YOLOv8n ONNX model; class `person` is
class 0. The launcher expects Ultralytics and ONNX Runtime in its Python
environment. It currently uses the existing YOLO virtual environment; override
it with `LUMI_VISION_PYTHON` if that environment moves.

## Machine-readable output

To emit one JSON record per processed frame without opening a window:

```bash
./run.sh --output-format jsonl --no-window
```

Consumers should use `target_visible` and message freshness as gates. A
confirmed track may use a predicted box for at most 0.15 seconds across brief
YOLO misses (`state: COASTING`); after that it becomes invisible and control
should stop. Horizontal position is reported in pixels relative to the frame
center, with image dimensions and a monotonic timestamp taken when the frame is
captured. Records also include measured processing FPS, YOLO inference time,
and frame age at emission so control latency can be distinguished from
detection rate. When calibration is loaded, each record also carries the
horizontal FOV of the rectified image. Area change is only a relative cue, not
a distance in meters.

## Camera calibration

The attached camera currently identifies as Sonix USB `0c45:6368` through the
Linux `uvcvideo` driver. That generic UVC identity does not provide the lens's
horizontal FOV, so the tracker derives it from the checkerboard calibration.

Print `checkerboard_9x6.svg`, then run:

```bash
python3 blue_object_vision/calibrate_camera.py
```

Capture at least 12 board views from different positions and angles. Press `c`
to save `blue_object_vision/camera_calibration.npz`; the tool reports the
rectified horizontal FOV used by the tracker and Vbc. Calibration is optional
for visual tracking, but required to enable robot centering from the dashboard.

See [VISION_DESIGN.md](VISION_DESIGN.md) for the architecture and integration
boundary with the Lumi Tanker controller.
