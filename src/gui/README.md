# Native TAP dashboard

The active dashboard is a pure Python/Tk client for `tap_mujoco_native`.
Python handles process lifecycle, operator commands and low-rate telemetry; the
native C++ child owns the MuJoCo physics and control loop.

From the repository root:

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON
cmake --build build/native -j2
python3 tools/tap_native_gui.py --rate 3000
```

`--rate 3000` is the previous default and `--rate 4000` is the stress mode.
Press `Start SIM` in the dashboard to open the real MuJoCo 3-D
viewer from `src/tap_sim/model/tap_robot.xml`, including the floor, CAD
meshes and materials. The viewer is a render-only mirror of the runner's
debug `VIEW` stream; it is never fed into the controller. The Tk window is
the command and sensor-diagnostics panel. Starting the simulation also sends one
Starting the simulation only spawns the robot in `Disabled` with physics held;
press the dashboard's explicit `ENABLE` button after confirming the natural
fixture pose. The runner then proceeds through IMU qualification to `Balance`.

The pose controls reuse the checked-in CAD/CoM-aware curves for squat,
left/right roll and the finite Show sequence. Requests still pass through the
native core planner and its joint limits.

## Basic motion API

The control path is split into two layers:

```text
vision pixel offset -> Vbc -> MotionAPI -> native runner
```

`MotionAPI` exposes forward/backward speed and yaw rate. Pass an optional
distance or angle to the same method to request a finite encoder-based action.
Without that target, the command runs continuously until another command or
`stop()`. The stateful `Vbc` converts repeated pixel offsets into smooth motion
setpoints; balance and actuation remain in the native controller.

```python
import time
from gui.motion_api import ActionStatus, MotionAPI
from gui.vision_behavior import VisionBehaviorConfig, VisionTarget, Vbc

motion = MotionAPI(backend)
motion.forward(0.10)                 # m/s
motion.backward(0.08)                # m/s
motion.rotate(0.25)                  # rad/s, runner yaw convention
motion.stop()

# Finite requests use the same methods; distances are meters and angles radians.
motion.forward(speed_m_s=0.12, distance_m=0.5, tolerance_m=0.02)
while True:
    backend.drain()
    if motion.update_action() != ActionStatus.RUNNING:
        break
    time.sleep(0.02)
print(motion.action_status)

motion.backward(speed_m_s=0.12, distance_m=0.3)
while True:
    backend.drain()
    if motion.update_action() != ActionStatus.RUNNING:
        break
    time.sleep(0.02)

motion.rotate(yaw_rate_rad_s=0.25, angle_rad=0.5, tolerance_rad=0.04)
while True:
    backend.drain()
    if motion.update_action() != ActionStatus.RUNNING:
        break
    time.sleep(0.02)

behavior = Vbc(
    motion,
    VisionBehaviorConfig(horizontal_fov_rad=calibrated_hfov_rad),
)
# Call once per behavior tick; positive pixel error means target is right.
behavior.update(VisionTarget(42, image_width_px, time.monotonic()))
# On target loss, call behavior.update(None) to stop.
```

For the integrated camera path, install OpenCV and calibrate the external webcam:

```bash
python3 -m pip install -r blue_object_vision/requirements.txt
python3 blue_object_vision/calibrate_camera.py
```

Capture at least 12 checkerboard views and press `c` to save its calibration.
Then start the native dashboard,
start SIM, ENABLE, and press **Theo dõi vật thể xanh**. The dashboard starts
the tracker on `/dev/video4`, maps fresh visible JSONL records into `VisionTarget`,
and calls the same `MotionAPI` used by operator controls. The native simulator
tracking path only rotates; forward speed remains zero. Target loss sends STOP
while leaving the tracker active for reacquisition. Manual drive, disable,
reset, ESTOP, stale robot telemetry and camera exit stop tracking and clean up
its process. The dashboard refuses to start vision control without camera
calibration.

## ESP32 hardware path

For the real robot, use the firmware GUI, which owns the ESP32 serial port and
the camera follower in one window:

```bash
python3 -m pip install pyserial opencv-python numpy
python3 tools/tap_serial_control_gui.py
```

Connect and start the firmware session as usual. Once the status panel says
`Balancing`, press **Theo dõi vật thể xanh**. The follower uses the laptop
camera (`/dev/video4`) and opens a camera window showing the center line and
detected target. It uses the detector's raw `area_fraction` to hold the apparent
target size captured when the target first enters the centered sector. A 0.35 s
low-pass filter smooths area measurements. Forward or backward motion starts
only after the log size error crosses 0.10; it releases inside 0.04, creating a
hysteresis band that prevents direction chatter. Speed grows with size error,
is capped at `0.12 m/s`, and ramps at `0.5 m/s²`. Translation is enabled only
within `0.20 rad` (about 11°) of image center; outside that sector it rotates to
recenter and pauses forward/backward motion. The `APPROACHING`/`RECEDING` label
remains a display diagnostic and does not directly command translation. This
holds a relative visual size, not a metric distance. Vision yaw is capped at
`0.40 rad/s` with `1.8 rad/s²` slew. Target loss sends `STOP`.
Pressing **Dừng theo dõi**
closes the camera window and sends `STOP`; it does not send `DISABLE`, so the ESP
balance loop continues. The GUI keeps its serial heartbeat active for the
balance session.
Do not open `controller_console.py` at the same time; the GUI is the serial-port
owner. The separate `tap_vision_follow.py` FIFO path remains available for
command-line use.

For manual forward/backward movement, hold **Tiến** or **Lùi** in the drive
panel; the speed slider sets the requested speed and releasing the button sends
zero drive. The joystick and W/S keys provide the same forward/backward input.

The hardware GUI uses an approximate 60° horizontal FOV because this mini-test
only needs image centering, not range or metric positioning. Positive pixel
error maps to positive `DRIVE` yaw, which the firmware documents as turning
right. The tracker rejects regions smaller than `max(300 px, 0.2% of the
processed frame)`. A lost/stale camera feed sends `STOP`; reacquisition resumes
centering.

`MotionAPI` defaults to `0.20 m/s` and `0.50 rad/s`; its limits are configurable
but cannot exceed the runner clamps (`1.0 m/s`, `0.8 rad/s`). The dashboard
retains its existing higher UI envelope. Vision behavior requires a calibrated
horizontal camera FOV, filters observations with a `0.12 s` time constant,
uses `0.025/0.055 rad` enter/exit center thresholds, limits yaw to `0.35 rad/s`
and yaw acceleration to `1.0 rad/s²`, and stops for target loss or observations
older than `0.25 s`. Forward approach defaults to zero because pixel offset
alone does not give ball range; a fixed approach speed can be enabled, but it
cannot maintain a safe standoff distance without range data.

This is image-based target centering, not the Stanley path-tracking algorithm:
pixel offset supplies a target bearing but no path cross-track error. The
dashboard can launch the standalone USB-camera tracker on the companion
computer. This does not add an onboard camera to the robot hardware contract.

Distance actions use the runner's encoder-based body travel estimate; turns
use differential wheel odometry with the CAD axle spacing (`0.309 m`). They
are nonblocking: call `update_action()` regularly and inspect `action_status`
(`idle`, `running`, `completed`, `cancelled`, `timed_out`, `rejected`). A
forward/backward distance is nonnegative and limited to 5 m; backward chooses
the reverse direction. A signed `angle_rad` chooses the rotation direction and
is limited to one full rotation. A zero target completes immediately and
stops. Defaults use a `0.02 m` distance tolerance and `0.04 rad` turn tolerance,
with bounded speed, braking profile and timeout. Wheel slip means these are
approximate ground motions, not exact positioning actions. Actions start only
with fresh telemetry while balancing and station capture is ready.
