# Lumi Tanker target vision: design and implementation plan

## Goal and boundary

Provide a standalone, non-ROS vision process that tracks the user's blue target
and publishes fresh image measurements for a separate control adapter. This
process does not command motors or infer metric distance. Lumi Tanker v3.3's
control contract already has forward-velocity and yaw-rate command fields, but
the documented sensor contract excludes an onboard camera. Keep image capture
and inference on the companion computer; hand off only bounded, timestamped
measurements or setpoints through the existing external command path.

## Design choices borrowed from the surveyed repositories

- **OpenCV** ([Apache-2.0](https://github.com/opencv/opencv)): keep V4L2/MJPEG
  low-latency capture, HSV segmentation, connected components, and overlay in
  one lightweight pipeline. The target is already recognizable by color, so a
  neural detector would add model/data/runtime cost without clear benefit.
- **Norfair** ([BSD-3-Clause](https://github.com/tryolabs/norfair)): use a
  detector-independent interface: the color detector emits a center, box,
  area, and confidence; tracking associates those observations over time.
  Implement the single-target case with OpenCV's constant-velocity Kalman
  filter, so Norfair itself is not a runtime dependency.
- **ByteTrack** ([MIT](https://github.com/FoundationVision/ByteTrack)): retain
  the useful association principle of combining motion prediction and
  detection quality. Full ByteTrack is aimed at multi-object tracking with a
  learned detector; that machinery is unnecessary for one color target.
- **Supervision** ([MIT](https://github.com/roboflow/supervision)): keep
  measurements and visualization separate so the overlay can later be
  replaced without changing the control-facing data contract.
- **Ultralytics YOLO** ([AGPL-3.0](https://github.com/ultralytics/ultralytics)):
  reserve as an optional future detector if the target stops being visually
  distinct by color or multiple classes are needed. It requires suitable
  training data and a license review for product use.

No ROS dependency is introduced. No code from those projects is copied.

## Implementation stages

### 1. Camera ingress — implemented

- Prefer the USB webcam at `/dev/video4`, V4L2 backend, MJPEG, requested
  640×480, one-frame buffer, and process no more than 60 frames/s by default.
- Preserve aspect ratio if a camera ignores the requested size.
- Keep camera diagnostics off stdout when JSONL measurement output is selected.

### 2. Blue detector — implemented

- Threshold HSV using the current hue/saturation/value settings.
- Use a small median filter, closing to fill tiny holes, and opening to suppress
  isolated specks, then connected-component area filtering (300 px minimum).
- Emit detector-independent observations: bounding box, centroid, pixel area,
  mean saturation, and a heuristic confidence score.
- Do not add shape constraints.

### 3. Temporal association — implemented

- Initialize tentatively from the strongest sufficiently large blue component,
  then require a second consecutive associated detection before reporting
  `visible=true`.
- Predict centroid with a constant-velocity Kalman filter.
- Associate by distance to prediction, area consistency, and confidence to
  reduce switching to a separate blue patch; expand the gate with estimated
  image-space speed to tolerate faster lateral movement.
- Blend measured image-space velocity into the Kalman prediction and report the
  latest detected centroid directly so visible output does not lag behind the
  target.
- Coast predictions for up to 0.6 seconds after a confirmed track, but mark them `visible=false` so
  consumers never treat predicted coordinates as a current measurement.
- Reacquire after timeout with a new track ID.

### 4. Handoff format — implemented

`--output-format jsonl` emits one JSON record per processed frame. Fields include
timestamp, track ID, state, visibility, confidence, pixel center, normalized
horizontal error, area fraction, apparent-size trend, and age since last sighting.
Normalized horizontal error is -1 at the left edge and +1 at the right edge.
Use `target_visible` and message freshness as gates; ignore `center_px` when the
target is not visible. The timestamp is monotonic seconds from this process and
is intended for age calculations within the same host session.

### 5. Controller mapping — implemented in the native dashboard

- The dashboard starts the tracker as a child process and consumes its JSONL
  measurements on the Tk thread. It passes raw horizontal pixel offset, image
  width, per-frame monotonic timestamp and rectified horizontal FOV into `Vbc`.
- `Vbc` uses a filtered bearing, center hysteresis, bounded proportional yaw
  and yaw slew. Translation remains zero for this first target-centering task.
- Positive image offset means the target is right. Native positive yaw is
  counter-clockwise viewed from above, so the controller requests negative yaw
  to turn toward a right-side target. Confirm this mapping against the physical
  camera orientation before enabling physical motion.
- A lost target, invalid measurement, stale frame, stale robot telemetry,
  disabled robot, or camera process exit sends STOP. The tracker marks coasted
  Kalman predictions invisible, and the controller never follows them.
- The dashboard button is for the native runner. For the laptop-camera-to-ESP32
  path, `tools/tap_vision_follow.py` forwards the same `MotionAPI` commands to
  the existing serial console FIFO. The vision process does not open/reset the
  serial port and never sends START or GO; it requires explicit operator
  confirmation that the ESP32 robot is already balancing.

### 6. Validation stage — pending

- Record representative scenes: target alone, blue distractors, lighting
  changes, partial occlusion, rapid lateral motion, and target leaving/re-entering
  view.
- Measure false-target duration, loss/reacquisition behavior, end-to-end frame
  age, and detection latency at the 60 FPS processing cap.
- Tune thresholds and association gates using the clips; keep the selected
  target color range configurable.
- Separately validate controller signs, limits, command freshness, and stop
  behavior in a non-actuating/mock setup before any robot motion trial.
- Verify the right-target/negative-yaw and left-target/positive-yaw mapping
  with a non-actuating command sink, then with a supervised simulation trial.
- Resolve the firmware yaw-direction documentation mismatch before choosing
  `--yaw-sign` for the ESP32 path.

## Current limitations

- HSV thresholds are fixed CLI settings; confidence is a heuristic, not a
  calibrated probability.
- A second blue object close to the target can still be associated incorrectly.
- Area change gives only a relative approach/recede cue. It is affected by
  rotation, occlusion, target orientation, and lens distortion; it is not a
  metric range estimate.
- Camera calibration has not been completed. The user previously chose to
  continue with the uncorrected image.
