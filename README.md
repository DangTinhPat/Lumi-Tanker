# TAP Robot Workspace

ROS 2 Jazzy / Gazebo workspace for the user's six-joint CAD wheeled robot.
Control algorithms are ported from DangTinhPat/wheel_leg, with native joint
signs and exact CAD-derived whole-body FK replacing the old Upkie geometry fit.
Source archive: `source_assets/3D.rar`.
Robot model: `src/tap_robot_description/description/cad_robot/robot.urdf.xacro`.

After activating a ROS 2 Jazzy environment:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch tap_robot_description cad_preview.launch.py
ros2 launch tap_sim sim.launch.py           # Gazebo starts paused, awaiting Enable
ros2 run gui gui                            # Dashboard, separate terminal, same ROS domain
```

No shell-script launchers anywhere in this repo — always **source this
workspace's own `install/setup.bash` yourself first**, every terminal, before
any of the three commands above. This matters most for `ros2 run gui gui`:
the `gui` package name matches `DangTinhPat/wheel_leg`'s own reference GUI
package, and a machine with both workspaces' overlays available (e.g. a
`ros2` wrapper that sources `wheel_leg`'s install first) could otherwise
resolve `ros2 run gui gui` to the wrong one. See `src/gui/README.md` if
that's a concern on your machine.

`ros2 run gui gui` can also launch and stop its own simulation from the
Control tab (Launch sim / Stop sim, with a Profile dropdown for Deep squat),
so a plain enable/drive/reset-standup/deep-squat test cycle needs only one
command, no second terminal. Profiles the Profile dropdown doesn't cover yet
(Stationary Show, jump profiles, precision pose below) still need a second
terminal running `ros2 launch tap_sim sim.launch.py controller_config:=<path>`
alongside `ros2 run gui gui`.

Wait for controller/sensor readiness, then use Enable in the dashboard.
Load the dashboard before unpausing: rendering/trajectory generation can briefly
delay joint feedback on this PC, correctly tripping the sensor watchdog.
After a fall, restart the simulation; reset does not reposition the robot.

For headless physics use `ros2 launch tap_sim sim.launch.py gui:=false auto_enable:=true`.
Run one TAP simulation per ROS domain / Gazebo partition. RViz preview has its
own joint-state topic and does not command the simulator.

## Stationary Show

Start a new simulation with the opt-in world-position feedback profile (do not
start it alongside an existing simulation in the same domain):

```bash
ros2 launch tap_sim sim.launch.py controller_config:="$PWD/src/tap_control/config/stationary_show.yaml"
ros2 run gui gui                     # Separate terminal, same ROS domain
```

After Enable and stable neutral feedback, check Experimental motions and press
Show. The finite 25% sequence returns to neutral between squat, left roll and
right roll, then ends. Stop show holds the last commanded pose while balance
continues. Manual drive, workflow changes, sensor/clock faults or excessive joint
tracking error cancel the sequence; it never resumes automatically.

This feature uses Gazebo world pose, not a hardware localization estimator.
It is a limited stationary performance, not zero drift, deep squat or jumping.
Larger asymmetric motions failed testing and are not enabled by the Show button.
See [transformation evidence](docs/TRANSFORMATION_TUNING.md) for exact trials,
remaining limits and [wheel kinematics](docs/WHEEL_KINEMATICS.md) for analysis.

## Deep Squat

For the separately tested deeper symmetric squat (native knee -1.6 rad), use:

```bash
ros2 run gui gui   # Deep squat and Standup are on by default; pick "Deep squat"
                   # in the Control tab's Profile dropdown, then Launch sim
```

(Or the two-terminal form: `ros2 launch tap_sim sim.launch.py
controller_config:="$PWD/src/tap_control/config/deep_squat.yaml"` +
`ros2 run gui gui` in a separate terminal.)

Select Experimental motions, then Deep squat and amplitude 1 after Enable and
neutral feedback (or just press the one-click "Deep squat" button on the Pose
tab). Full travel takes at least 11 seconds. Hold squat holds depth;
Neutral pose returns along the coordinated curve. The normal Squat/roll/Show
limits remain unchanged. See [deep squat evidence](docs/DEEP_SQUAT.md): the body
reached about 0.125 m height in two-cycle Gazebo testing. This is a tested
simulation depth, not the maximum real encoder travel; CAD/encoder zero is
still uncalibrated.

## Reset Standup (Sim-Only)

The dashboard's **Reset standup (sim-only)** button repositions the robot's
base upright via Gazebo world services (pause, teleport to the spawn pose,
unpause) and then hands off to the normal Reset controller / Clear E-stop /
Neutral pose / Enable sequence, so testing after a fall or bad pose does not
require restarting the whole simulation.

This is a simulation-only test convenience, not evidence of physical recovery
capability: it teleports the Gazebo world pose, which no hardware equivalent
exists for. Every status line it produces is prefixed `[SIM-ONLY]`. Whether
the robot actually stays upright afterward depends entirely on the balance
controller's own recovery capability from whatever leg configuration the fall
left behind — a plain world-pose teleport only moves the base, not the legs —
and that capability is separately tracked, and currently limited, in
`docs/ALGORITHM_MIGRATION.md`. See
[docs/GUI_RESET_STANDUP_QA.md](docs/GUI_RESET_STANDUP_QA.md) for the scripted
end-to-end evidence, including real runs where the choreography completed
correctly but the controller did not recover from the resulting leg pose.

## Dashboard Jump And Deep Squat

The dashboard always shows **Jump once**, initially locked. The separate
**Experimental jump (unstable landing)** checkbox opts into jump requests;
`--experimental-jump` only preselects that checkbox. Requests still require
Experimental motions, Enable, neutral measured/commanded joints, zero drive,
fresh feedback and controller jump readiness. No repeated jump is scheduled.
Jump height above 10 cm and stable landing are **not validated**.

In Deep squat, use **Deep squat depth** (0..1). Individual joint sliders are
locked while the coordinated curve owns the legs. **Neutral pose** returns on
that curve; Hold squat or a failed readiness gate holds the current target.
A fallen robot needs a new simulation, not merely another Enable click.

The bounded model/profile used for the GUI regression is:

```bash
ros2 launch tap_sim sim.launch.py controller_config:="$PWD/src/tap_control/config/precision_pose.yaml" \
  jump_model:=true jump_leg_velocity_damping:=0.6
ros2 run gui gui
```

GUI-driven Gazebo QA: `python3 tools/gui_deep_smoke.py --gui` and
`python3 tools/gui_deep_smoke.py --gui --drag` (after sourcing the workspace).
These create and clean up their own simulation session. See
[GUI evidence](docs/GUI_DEEP_SQUAT_QA.md) for the measured scope and limits.

## Packages

- `tap_robot_description`: seven CAD meshes and mass/inertia estimates.
- `tap_control`: MPC, exact whole-body FK, joint interpolation, state machine,
  candidate stand-up recovery and the ROS controller node.
- `tap_sim`: free-body model, contacts, sensors, ros2_control and paused bringup.
- `tap_teleop`: regenerated neutral squat/roll/deep-squat curve generation library (no ROS node, no GUI).
- `gui`: drive/pose dashboard, owns the one-command sim lifecycle and the sim-only reset-standup test convenience.

ROS inputs: `/cmd_vel`, `/target_pose` (LH, LK, RH, RK), `/enable`,
`/emergency_stop`, `/stand_up`, `/reset_controller`. Joint arrays use CAD signs
on both sides. Wheel outputs are rad/s, leg outputs Nm. Drive commands expire
after 0.5 seconds without updates. `/control_mode` reports control/sensor status.

## Model And Evidence

The approximate mass is 3.938 kg; wheel radius is 0.030 m and axle spacing is
0.309 m. Body mesh pitch is -90 degrees, with matching COM/inertia. Actuator
limits and mass distributions are provisional. Ideal wheel velocity servos do
not establish hardware performance. RViz remains a kinematic preview.
See [mass properties](src/tap_robot_description/description/cad_robot/PHYSICS.md)
for assumptions, [migration inventory](docs/ALGORITHM_MIGRATION.md) for verification
status, and [source provenance](docs/UPSTREAM.md). Recovery and advanced poses
require separate physical trials; build/RViz success does not prove balance.

The strict N20 simulation trial passed approximately 67 seconds of hold, drive
and moderate poses: 30-second hold drift was 0.016 m, final forward speed
0.061 m/s for a 0.06 target, and yaw change 0.4523 rad for a 0.45 target.
Tested poses cover 25% squat and 20% roll only. Roll still caused up to 0.066 m
translation and about 0.14 rad unintended yaw. Experimental recovery failed
the tested 90-degree lying pose: it timed out back to Fallen without standing.
Other fall directions are untested; recovery is not ready for hardware use.
See the migration inventory for thresholds, measured joints and the exact
configuration recorded in `log/smoke_1789149917`. That baseline binary also passed
29 ROS contracts; its lying-pose recovery failure is recorded separately in
`log/recovery_1789149922`.

## Verification

The [embedded core performance report](docs/EMBEDDED_PERFORMANCE.md) describes
the ROS-free C++17 build, reduced MPC storage, frozen-reference tests and host
benchmarks. ESP32-S3 timing and hardware integration remain future validation.

See the [pose/jump quality audit](docs/QUALITY_AUDIT.md) for current bounded-motor
evidence and reproduction commands. The separate `precision_pose.yaml` profile
passed two cold-start, two-cycle 25% pose trials: peak displacement below 10 mm,
final below 0.34 mm, measured after settling. This is not full-travel, rough-ground
or jump validation and does not change the default launch profile.

The same bounded profile also passed two cold-start repetitions of two symmetric
deep squats to the provisional -1.6 rad CAD-knee target with 11 s transitions:
peak displacement 2.49/2.11 mm and final 0.100/0.062 mm. The 3 s deep transition
failed. See the audit for exact commands and guards; hardware encoder limits
and deep asymmetric poses remain unvalidated.

Experimental jumping is developed separately from the validated legacy squat
profile. See [jump development and evidence](docs/JUMP.md) for research,
motor-envelope assumptions, single-shot controls and acceptance criteria.
`tools/run_jump.py` opts into bounded actuators and measured wheel contacts;
plain `ros2 launch tap_sim sim.launch.py` is unchanged by default. A working
jump button is not proof of greater-than-10-cm flight or successful landing.

```bash
python3 -m unittest discover -s tests -v
colcon test --packages-select tap_control tap_teleop gui
colcon test-result --verbose
python3 tools/generate_control_model.py --check
python3 tools/test_ros_contract.py
python3 tools/test_station_ros_contract.py
python3 tools/transformation_smoke.py --config src/tap_control/config/stationary_show.yaml
python3 tools/sim_smoke.py --duration 30 --exercise --poses --max-pitch-error .12
python3 tools/gui_reset_standup_smoke.py
```

Geometry generation/tests use numpy, trimesh, PyYAML and xacro. ROS dependencies
are declared in package manifests. On system ROS: `rosdep install --from-paths
src --ignore-src -r -y`. Simulation evidence is written under `log/smoke_*/`.

Persistent expert instructions live in `.agents/skills/`; [AGENTS.md](AGENTS.md)
routes model, control, ROS/Gazebo and QA work to the relevant expertise.
