# Blue object vision

Standalone OpenCV tracking for the blue object. It uses HSV color segmentation
and a motion-aware single-target tracker. Processing is capped at 60 FPS at
640×480 by default; the live window shows the measured processing rate. The
program does not require ROS and does not send robot commands.

## Run

Install the dependencies, then launch from any working directory:

```bash
python3 -m pip install -r requirements.txt
./run.sh
```

The default camera is `/dev/video4` (the external webcam). Press `q` or `Esc`
in the video window to quit. To choose another source or adjust the processing
rate:

```bash
./run.sh --source /dev/video2 --process-fps 60
```

The detector uses blue HSV thresholds and rejects small connected components.
By default, a candidate must be at least the larger of 300 pixels and 0.2% of
the processed image. The relative threshold scales with camera resolution:

```bash
./run.sh --hue-low 100 --hue-high 132 --min-saturation 110 \
  --min-value 20 --max-value 190 --min-area 1000 \
  --min-area-fraction 0.002
```

`--min-area-fraction 0.002` means 0.2% of the frame. A component must pass
both the absolute and relative floors; increase the fraction to ignore more
small blue objects.

## Machine-readable output

To emit one JSON record per processed frame without opening a window:

```bash
./run.sh --output-format jsonl --no-window
```

Consumers should use `target_visible` and message freshness as gates. A
coasting prediction is not a fresh measurement. Horizontal position is reported
in pixels relative to the frame center, with image dimensions and monotonic
timestamp. When calibration is loaded, each record also carries the horizontal
FOV of the rectified image. Area change is only a relative cue, not a distance
in meters.

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
