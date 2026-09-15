# TAP pose curve generation

Pure geometry/curve library: no ROS node, no Tk GUI. The dashboard that
consumes this lives in the separate `gui` package (`ros2 run gui gui`); see
`src/gui/README.md` for dashboard behavior, the sim lifecycle, the
reset-standup test convenience, and a package-name collision warning against
`/home/dvt/wheel_leg`'s own `gui` package.

The native pose order is `[cad_left_hip, cad_left_knee, cad_right_hip,
cad_right_knee]`, radians, all axes +Y. Left and right use the same signs.
The provisional GUI envelope is hips `[-0.6, 0.6]`, knees `[-0.8, 0.2]`.
Negative knees shorten this CAD robot. At knee zero the computed neutral hip
is approximately `+0.1154831` rad, not zero. These limits are operational
estimates, not the supplier's yet-unmapped mechanical limits.

`tools/generate_pose_curves.py` expands the current Xacro and solves whole-body
FK numerically. Lookup tables include a nonzero rest hip; there are no old
Upkie polynomial coefficients or mirrored joint signs. Roll neutralization
uses the actual sloping support line after leveling the wheel centers.
Tables are regenerated from the installed description at dashboard startup.
Changing masses or body orientation therefore changes the trajectories.

Runtime dependencies: Xacro, NumPy. Build uses ament_cmake and
ament_cmake_python. Offline validation:

```bash
python3 tools/generate_pose_curves.py
python3 -m unittest discover -s tests -p test_pose_curves.py -v
colcon test --packages-select tap_teleop
colcon test-result --verbose
```

The mocked show-sequence/deep-squat callback tests are registered with CTest
when `BUILD_TESTING` is enabled. They do not open a window or initialize a
ROS node.

## Deep squat trajectory contract

Optional simulation-only deep squat profile, selected from `gui`:

```bash
ros2 run gui gui --deep-squat-knee 1.6
```

The argument is a positive native CAD depth, limited to 1.6 rad for simulation
testing. Startup generates a separate symmetric curve with hip bounds
`[-0.6, 1.05]`, within the URDF bounds. Controller matching uses actual table
extrema, not these wider generation bounds. Without this argument the mode is absent.
It never starts automatically. Experimental must be selected, and fresh
controller telemetry must report `pose_hip_max` and `pose_knee_min` covering
the entire generated table. Missing or incompatible limits block new deep
targets and hold the last command. Amplitude magnitude maps to the curve.
`DeepSquatTrajectory(profile, joint_speed=0.3).advance(amplitude, dt)` plans
quintic amplitude transitions with duration
`max(0.5, 11*abs(delta), 1.875*max_abs_dq_da*abs(delta)/joint_speed)` seconds.
All four joints remain on the same neutral path at at most 0.3 rad/s each.
Retargeting starts from the current amplitude, preserving position but not
claiming global acceleration continuity. GUI `dt` comes from `/clock`, not
wall time; paused, stale, backward or large forward-gap clocks never advance
the trajectory unexpectedly. A complete 0-to-1 transition takes at least 11 s.
Entry requires neutral commanded joints (0.02 rad tolerance), neutral measured
joints (0.08 rad), fresh finite joint feedback and zero operator command.
Deep-only runtime gates additionally require station enabled and fresh finite
controller telemetry with `abs(pitch) <= 0.12` rad and `pose_z >= 0.115` m.
A violation freezes the last coordinated pose without changing controller
fault semantics; restored telemetry does not automatically resume movement.
Manual drive is blocked while the deep trajectory owns the pose. A Neutral
request or mode switch returns along the same curve before handing control
back; only neutral values are written to the narrower manual joint scales.
Invalid feedback, stop, or clock discontinuity holds the deep command and
retains ownership without automatic resume. An explicit Neutral request with
valid feedback is required to return. Manual joint limits, roll curves and Show's
0.25 amplitude are unchanged. Selecting the mode cancels Show.

The optional Hold squat button holds the coordinated command without disabling
balance. Deep tracking error above 0.15 rad for 0.3 simulated seconds also
holds; fresh measurements alone never resume it. Neutral explicitly requests
the coordinated return when feedback and profile gates are valid.

The 1.6-rad profile was approved for the GUI after the main simulation QA;
this does not establish hardware safety or continuous torque capability.
The hardware encoder range 30..145 degrees is not mapped to CAD angles until
its zero offset is known. Neither the CLI cap nor URDF limits establish a
verified hardware endpoint.

The GUI cap is 1.6 rad with a minimum 11-second quintic full-amplitude transition
in each direction; larger depths are rejected. The generic trajectory helper
accepts `minimum_transition_s` to reproduce that timing in independent QA.
