# Pre-catch rocking review — 14 October 2026

**Withdrawn after hardware test, at the user's request.** The changes below
were reverted in source. The two observed attempts in `tap_20261014_134907.log`
never enabled the balancer: StandUp 22.229→Fault 24.883 s and
62.788→65.378 s, both `stand-up failed`. Both crossed the narrow catch corridor
with insufficient dwell/hip-goal qualification and increasing negative rate,
then exceeded the unchanged rear fall lean. The synthetic observer and command
tests did not cover this closed-loop shell-to-axle transition. They were not
a sufficient release gate. This document retains historical evidence only.

Branch `fw`. This revision corrects two deterministic software defects before
wheel enable. Hardware performance validation is pending. No firmware upload,
serial-port reopening or motor commands were performed in the analysis task.
The subsequent user-requested deployment is recorded below.

## Robot evidence

Source: `/home/dvt/tap_logs/tap_20261014_133001.log`. Analysis snapshot:
`/tmp/tap_20261014_133001-review.log`; parsed TR:
`/tmp/tap_20261014_133001-rows.json`. One executed StandUp in this snapshot:
StandUp at 79.361 s, Balancing at 84.363 s, wheel-envelope Fault at 105.223 s.
The operator previously confirmed manual intervention in these tests; exact
timestamps are unknown. This is not an autonomous success-rate experiment.

122 pre-catch TR samples, 79.367–84.336 s, show two large swings with wheel
torque disabled. Pitch ranges −0.663…+0.475 rad. The largest positive sampled
hip command change is −0.0888→+0.2520 rad over 41 ms at 83.966–84.007 s:
8.312 rad/s, versus configured Rise rates 0.7081/0.8845 rad/s.

At 82.035–82.075 s, estimated pitch changes −0.321→−0.024 rad in 40 ms
(17.0°), while endpoint gyro rates are −0.57/−0.41 rad/s. Another estimate
change is −0.060→−0.346 rad over 42 ms at 84.171–84.213 s, with rates
+0.25/−0.06 rad/s. These sampled discrepancies suggest accelerometer-induced
angle excursions, but 25 Hz TR cannot reconstruct the 500 Hz gyro integral or
prove every individual physical disturbance. The software estimator selection
and rate-limit defects below are independently confirmed by source and tests.

The post-catch episode also has renewed pitch motion and ends in a wheel
envelope fault. This revision does not establish that post-catch wheel control
or motor calibration has been solved.

## Changes and frames

1. Under `capturegov`, Rise uses `hip += clamp(goal-hip, -rate*dt, rate*dt)`.
   Previously `max(hip-rate*dt, goal)` limited folding only: an increasing live
   goal jumped straight to its new value. Rate selection either side of
   `hip_mid` stays unchanged, as do the initial 28° lift, knee timing/target,
   final trim, catch corridor/rate/dwell and wheel-off behavior. Flag-off keeps
   the original implementation. Level already had a symmetric rate bound.
2. Under `capturegov`, StandUp uses the existing floor-balancing gyro-dominant
   estimator gains: complementary acceleration correction 0.2/s rather than
   55/s, Kalman acceleration measurement variance 1e6 rather than 0.03. Both
   stationary and innovation bias learning are frozen during this maneuver.
   AppV3 also supplies the known pose-changing hint. Config selection is shared
   with host tests in `controllerEstimatorConfig`. `setConfig` preserves angle,
   calibration and covariance at StandUp→Balancing; no attitude reset occurs.
   Idle/Locking/Ready/Fault, legacy StandUp and TILTTEST keep their previous
   configuration. Normal Balancing retains its existing gyro-dominant config.

No new capture-point law or sensor has been introduced. This reuses the
existing bounded-reference and gyro observer paths. Support is not inferred
from accelerometer magnitude; no unsupported kinematic acceleration
compensation is added in StandUp. No wheel UART protocol, gain, fall/stale/stop
guard, geometry or physical parameter changes are part of this revision.

Frames are unchanged: MPU body +X forward, +Y left, +Z up; positive pitch is
nose-down about +Y. Core hip is homeHip + signed encoder joint angle (LH −1,
RH +1), and the six-joint order remains LH/LK/LW/RH/RK/RW. Tests use those
core-angle conventions; estimator force uses `ax=-g*sin(pitch)`.

`SET capturegov 1` selects both corrections with the existing supervisor;
`SET capturegov 0` in Idle/Fault restores the legacy profile. The core default
remains off. GUI STANDUP already selects it; no new GUI action is needed.
Boot/runtime evidence includes `V3 STANDUP estimator=gyro-dominant-bias-frozen
rise_slew=bidirectional`. Firmware cycle-time measurement remains pending;
the added scalar clamps/state selection have fixed cost and allocate nothing.

## Verification

New `v3_standup_motion_tests` exercises the actual StandUp and IMU estimator:

- Alternating live hip goal exposes the legacy upward jump (peak 370.810
  rad/s in this deliberately severe synthetic control case); the new path
  stays below its selected limit on every tick (peak 0.708 rad/s).
- Fixed-goal trajectories match the legacy hip trajectory within 1e-6 rad,
  and knee, wheel enable and wheel commands match exactly, including the
  28° first-lift breakpoint.
- With real body rate 0.3 rad/s and a 60 ms, 0.28 rad gravity-magnitude force
  direction pulse, peak attitude error is 0.2400 rad with base estimator and
  0.0033 rad with dynamic StandUp config. This is sensor synthesis, not a
  replay of raw robot IMU or a prediction of capture time.
- State/flag/TILTTEST matrix, bias freeze under misleading quiet force,
  catch continuity and recovery of the Idle absolute reference are checked.

Host configure/build uses `/tmp/tap-minbal-host`, simulator disabled.
Selected CTest suite and both firmware environments completed successfully
(results below). The existing provisional planar diagnostic is a post-catch check;
it does not model the lying shell, first rocking motion or raw IMU. There is
no verified replacement CAD model for a full pre-catch physics release gate.

Next physical acceptance: compare repeated unassisted attempts at the same
initial pose; record the operator's power/touch events. Check the new estimator
marker, hip command increments bounded by selected rate × elapsed control
time, encoder tracking, number of pre-catch swings, time to capture, subsequent
gyro RMS and any wheel faults. Initial target is catch within the unchanged
8 s timeout without repeated large swings; do not widen guards to achieve it.
Builds and synthetic tests alone do not establish this target on the robot.

## Completed results and artifacts

- `cmake -S . -B /tmp/tap-minbal-host -DTAP_BUILD_SIMULATOR=OFF
  -DTAP_BUILD_BENCHMARK=OFF` and build of the five selected targets succeeded.
- `ctest --test-dir /tmp/tap-minbal-host -R
  '^(v3_standup_motion_tests|v3_capture_governor_tests|v3_tests|motor_feedback_tests|v3_standup_tests)$'
  --output-on-failure`: **5/5 passed**, 132.86 s. The full stand-up target
  includes legacy trajectory, catch, failure and post-catch planar contracts.
- `/tmp/tap-minbal-host/bin/v3_standup_tests 0.09 capture`: all nine regression
  expectations preserved. Six provisional wheel-lag 130/200 ms cases reach
  Normal with gyro RMS 0.013–0.014 rad/s; three 35 ms cases still fail at the
  unchanged settling deadline, RMS 0.173–0.174 rad/s. Those three remain
  unresolved physical-model/calibration concerns, not successful balance.
  Output: `/tmp/tap-precatch-planar-regression.out`.
- `/home/dvt/.platformio/penv/bin/pio run -e esp32-s3-v3
  -e esp32-s3-v3-single-loop`: both succeeded. RAM 158308/327680 bytes;
  RTOS flash 1125633/6553600; single-loop flash 1124577/6553600.
  Output: `/tmp/tap-precatch-build.out`.
- RTOS binary SHA256:
  `a289bed6d5146749222ae3df0e614c35668c7aae62d7621b83b7428ad7343f9b`.
- Single-loop binary SHA256:
  `b8068530c8630d27e6ee8c1fc9e835fc5658fd200a30be04056f7ae56e2d0380`.
- `git diff --check`: clean. Existing host warning about JumpSequence's
  unhandled `Approach` enum is unrelated to this revision.

The portable new behavior is behind `capturegov`; the fixed-goal static path
matches the original and mode/flag tests preserve legacy estimator selection.
Full old/new ELF equivalence is not claimed: this is a behavior change, and
the working tree includes prior uncommitted supervisor revisions. No MuJoCo
result or new hardware performance evidence exists for this revision.

## Subsequent authorized deployment — 13:49

The user subsequently requested flashing and opening the GUI. The previous
GUI PID/session 35019 had exited; its identity/session had no surviving members.
Process inventory found no remaining TAP simulator/dashboard/follower sessions,
and the USB port had no owner before upload. Unrelated MCP processes were left
untouched.

`pio run -e esp32-s3-v3 -t upload --upload-port
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
completed in 22.356 s: **Hash of data verified**, followed by RTS reset. Uploaded
RTOS SHA is `a289bed6d5146749222ae3df0e614c35668c7aae62d7621b83b7428ad7343f9b`.
Upload output: `/tmp/tap-precatch-upload.out`.

GUI PID/session 36790, start identity 1756359, is visible and the sole serial
owner. Metadata `/tmp/tap_fw_pos_gui.json`, status
`/tmp/tap_fw_pos_gui_status.json`. New log:
`/home/dvt/tap_logs/tap_20261014_134907.log`.
Initial state connected/ready/Idle; IMU seeded, CAN legs and UART wheels have
no feedback, home unconfirmed, no bus voltage available. No START/STANDUP or
home confirmation was sent. GUI STANDUP selects `capturegov` when the operator
starts the test. Serial reopening discards previous runtime SET overrides.
Upload and GUI connectivity do not validate improved physical capture.
