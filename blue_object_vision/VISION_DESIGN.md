# Lumi Tanker target vision: design and implementation plan

## Goal and boundary

Provide a standalone vision process that tracks a person
and publishes fresh image measurements for a separate control adapter. This
process does not command motors or infer metric distance. Lumi Tanker v3.3's
control contract already has forward-velocity and yaw-rate command fields, but
the documented sensor contract excludes an onboard camera. Keep image capture
and inference on the companion computer; hand off only bounded, timestamped
measurements or setpoints through the existing external command path.

## Design choices borrowed from the surveyed repositories

- **OpenCV** ([Apache-2.0](https://github.com/opencv/opencv)): keep V4L2/MJPEG
  low-latency capture, frame resizing, drawing, and overlay.
- **Norfair** ([BSD-3-Clause](https://github.com/tryolabs/norfair)): use a
  detector-independent interface: YOLO emits a center, box,
  area, and confidence; tracking associates those observations over time.
  Implement the single-target case with OpenCV's constant-velocity Kalman
  filter, so Norfair itself is not a runtime dependency.
- **ByteTrack** ([MIT](https://github.com/FoundationVision/ByteTrack)): retain
  the useful association principle of combining motion prediction and
  detection quality. Full ByteTrack is aimed at multi-object tracking with a
  learned detector; that machinery is unnecessary for one tracked person.
- **Supervision** ([MIT](https://github.com/roboflow/supervision)): keep
  measurements and visualization separate so the overlay can later be
  replaced without changing the control-facing data contract.
- **Ultralytics YOLOv8n ONNX** ([AGPL-3.0](https://github.com/ultralytics/ultralytics)):
  use the local COCO model at 640-pixel inference size. The detector exposes
  only its `person` class to the existing tracker.

No code from those projects is copied.

## Implementation stages

### 1. Camera ingress — implemented

- Prefer the USB webcam at `/dev/video4`, V4L2 backend, MJPEG, requested
  640×480, one-frame buffer, and process no more than 15 frames/s by default.
- Preserve aspect ratio if a camera ignores the requested size.
- Keep camera diagnostics off stdout when JSONL measurement output is selected.

### 2. Person detector — implemented

- Load the local YOLOv8n ONNX model with Ultralytics on CPU and warm it up at
  startup.
- Run at 640-pixel inference input with a 15 FPS processing cap by default.
- Filter to the `person` class and reject boxes smaller than 200 px or 0.05% of
  the frame.
- Emit detector-independent observations: bounding box, centroid, pixel area,
  and model confidence.

### 3. Temporal association — implemented

- Initialize tentatively from the largest sufficiently confident person box,
  then require a second consecutive associated detection before reporting
  `visible=true`.
- Predict centroid with a constant-velocity Kalman filter.
- Associate by distance to prediction, area consistency, and confidence to
  reduce switching between people; expand the gate with estimated
  image-space speed to tolerate faster lateral movement.
- Apply a median over the last three accepted boxes to drop one-frame jumps,
  then smooth center and log width/height with a One Euro filter.
- Use ByteTrack's two-stage confidence idea: high-confidence boxes may start or
  update the track; low-confidence boxes only update an existing spatially
  matched track. Recheck confidence thresholds against the V8 model's scores.
- Blend measured image-space velocity into the Kalman prediction. A predicted
  box can remain visible for 0.15 seconds (`state=COASTING`) across brief model
  misses; association is retained for up to 0.8 seconds.
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
- A lost target after the brief coast window, invalid measurement, stale frame,
  stale robot telemetry, disabled robot, or camera process exit sends STOP.
  The controller only uses the bounded brief prediction window.
- The dashboard button is for the native runner. For the laptop-camera-to-ESP32
  path, `tools/tap_vision_follow.py` forwards the same `MotionAPI` commands to
  the existing serial console FIFO. The vision process does not open/reset the
  serial port and never sends START or GO; it requires explicit operator
  confirmation that the ESP32 robot is already balancing.

### 6. Validation stage — pending

- Record representative scenes: target alone, nearby people, lighting
  changes, partial occlusion, rapid lateral motion, and target leaving/re-entering
  view.
- Measure false-target duration, loss/reacquisition behavior, end-to-end frame
  age, and detection latency at the 15 FPS processing cap.
- Tune confidence, minimum box area, and association gates using the clips.
- Separately validate controller signs, limits, command freshness, and stop
  behavior in a non-actuating/mock setup before any robot motion trial.
- Verify the right-target/negative-yaw and left-target/positive-yaw mapping
  with a non-actuating command sink, then with a supervised simulation trial.
- Resolve the firmware yaw-direction documentation mismatch before choosing
  `--yaw-sign` for the ESP32 path.

## Current limitations

- Confidence is a model score, not a calibrated probability.
- A second person close to the target can still be associated incorrectly.
- Area change gives only a relative approach/recede cue. It is affected by
  rotation, occlusion, target orientation, and lens distortion; it is not a
  metric range estimate.
- Camera calibration has not been completed. The user previously chose to
  continue with the uncorrected image.
