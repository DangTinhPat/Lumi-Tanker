# Algorithm Migration Inventory

## Provenance And Status

Source: `https://github.com/DangTinhPat/wheel_leg.git`, local read-only reference
`../.reference/wheel_leg`, commit `701cef416b3f224925ab3b2532cf1ee5d401c703`.
Paths below are relative to that reference unless marked as TAP destinations.
Implementation paths below were inspected on 2026-09-12. Main reports the core
build, 58 core checks, 3 independent URDF checks, 2 demos and 16 Python
unittests passing. These counts are main's reported evidence, not reruns by
this documentation sidecar. Independently,
the ROS sidecar passed all 29 revised contracts against the latest safety-fix
binary fingerprinted below. The final strict physics and recovery artifacts
use that same binary, so all three evidence sources identify the same build.
Current physical evidence is the strict N20, full-dt, damping-1 trial in
`log/smoke_1789149917/summary.json`, inspected with its configuration snapshot.
It passed all guards across approximately 67.05 simulated seconds of hold,
drive and moderate poses. The 30.005-second hold drifted -0.016211 m in base X;
maximum hold pitch error was 0.0169 rad. Tested drive/yaw tracking and moderate
pose transitions passed; roll still causes translation and unintended yaw.
Recovery is implemented but FAILED the tested 90-degree lying case (details
below). Other fall directions, full pose range and broad disturbance robustness
remain untested. Final safety-fix strict trial `log/smoke_1789149917` passed.

TAP targets: `src/tap_control/{core,apps,ros2_adapter,config}` for portable algorithms,
demos, ROS adapter and parameters; `src/legacy/ros2_gazebo_sim/{launch,worlds,scripts}` for
simulation orchestration; existing `src/tap_robot_description` for CAD geometry.
GUI tooling lives in `src/tap_teleop`; curve generation is
`tools/generate_pose_curves.py`, with generated data in
`src/tap_teleop/config/pose_curves.json`. Do not create a second control core
inside the GUI. Reference source prefix below: `balance_firmware_core/`.

## Port Map

| Source | Behavior to retain | TAP target / required adaptation | Verification |
| --- | --- | --- | --- |
| `core/src/MpcBalancer.cpp` and header | Four states: position, pitch, velocity, pitch rate; finite-horizon quadratic objective, acceleration box constraints, fixed-iteration projected coordinate updates; velocity integration, yaw mixing, station hold | `src/tap_control/core/src/MpcBalancer.cpp`; parameters in `src/tap_control/config/balance_controller.yaml` | Core/ROS contracts and strict N20 hold/drive/yaw guards passed; broader convergence envelope pending |
| `core/src/WholeBodyModel.cpp` and header; `tools/gen_com_model.py` | Replace old polynomial COM model with native CAD geometry and equilibrium pitch | `src/tap_control/core/src/WholeBodyModel.cpp` now computes exact sagittal FK; `tools/generate_control_model.py` emits `GeneratedGeometry.hpp`; comparison test `src/tap_control/tests/test_urdf_model.py` | 58 core checks and 3 independent URDF checks passed per main; real mass calibration and dynamic adequacy pending |
| `core/src/JointMotionPlanner.cpp` | Cubic `3t^2-2t^3` interpolation of six joint positions; restart and duration handling | `src/tap_control/core/src/JointMotionPlanner.cpp` | Core suite and moderate physical pose joint-error guards passed; full trajectory envelope pending |
| `core/src/EmbeddedController.cpp` | Compose planner, FSM and MPC; filtered whole-body pitch reference, optional trim and settled station gate; motor command generation | `src/tap_control/core/src/EmbeddedController.cpp`; limits supplied from ROS pose configuration | Core/bilateral clamp contracts and strict moderate-pose guards passed; residual roll translation/yaw remains |
| `core/src/StateMachine.cpp` | Disabled, Enabling, StandingUp, Balance, Drive, PoseChange, Fallen, EmergencyStop | `src/tap_control/core/src/StateMachine.cpp`; wrapper preserves Fallen for explicit recovery and latches EmergencyStop | Core/ROS safety checks passed; physical reset gate passed, recovery attempt timed out to Fallen |
| EmbeddedController stand-up stages | Tuck, Rise with balance, then Settle; fall-direction-aware poses and timeout | Implemented in `src/tap_control/core/src/EmbeddedController.cpp`; native neutral standing pose and configured tuck in `src/tap_control/config/balance_controller.yaml` | FAILED tested lying-90-degree recovery; experimental only, other directions untested, not hardware-ready |
| `tools/gen_squat_curve.py`; GUI `skill_pose()` | Generate native CoM-aware stance-height poses | Implemented in `tools/generate_pose_curves.py`, `src/tap_teleop/tap_teleop/pose_curves.py` and `src/tap_teleop/config/pose_curves.json` | Python suite and strict 25% squat/stand passed; peak pose pitch error 0.0574 rad; full range unverified |
| `tools/gen_roll_curve.py`; GUI `skill_pose()` | Generate asymmetric body-tilt poses | Same implemented trajectory tooling and generated config; tests in `tests/test_pose_curves.py` | Python suite and strict 20% roll guards passed; up to 0.066 m base travel and about 0.14 rad unintended yaw change remain |
| `ros2_adapter/BalanceControllerNode.cpp` | Sensor assembly, orientation/filtering, watchdog, pose slew, command/reset triggers, command/status publishing | `src/tap_control/ros2_adapter/BalanceControllerNode.cpp`; dedicated `sim_pose_topic` default `/tap_robot/pose` | 29 revised synthetic ROS contracts passed on final safety-fix build, fingerprint below; simulated-clock resets and combined attitude pending |
| `apps/balance_demo.cpp`, `apps/mpc_demo.cpp` | Standalone core exercise/demo programs | Implemented in `src/tap_control/apps/balance_demo.cpp` and `src/tap_control/apps/mpc_demo.cpp` | Both demos passed per main; not physical balance evidence |
| `gui/scripts/gui` | Native drive/yaw, enable/stop/reset/stand-up and diagnostics | `src/gui/gui/native_dashboard.py`, entrypoint `src/gui/scripts/gui`; curve generation remains in `src/tap_teleop/tap_teleop/` | Native GUI/IPC tests cover the current dashboard boundary; this does not prove balance or landing |
| Legacy launch/config/scripts | ROS 2/Gazebo world, bridge/controller startup and smoke observation | Retained under `src/legacy/ros2_gazebo_sim/`; not part of the native runtime | Historical ROS/Gazebo evidence only |

## Robot-Specific Hazards

1. **Signs and zero:** reference WholeBodyModel negates right hip/knee and uses
   `(left-right)/2` for symmetric pose. MPC forward wheel commands have opposite
   signs, and ROS odometry mirrors that convention. TAP uses +Y on both sides.
   Adapt forward/inverse mappings together; do not negate right joints at several
   layers or rotate the CAD to disguise a control sign error. Zero CAD pose is
   not the old robot's straight-leg equilibrium.
2. **Geometry and inertia:** old WholeBodyModel defaults include mass 5.338220 kg
   and old height clamps/fits. The current BOM-backed TAP model is 4.621 kg,
   radius .040 m and half-track .1545 m, not outer half-width .167 m. Body
   mesh/inertial pitch is -pi/2 without changing joints. See model `PHYSICS.md`
   for BOM link allocation, uniform-solid COM, motor lumping and uncalibrated
   mechanical limits. Results recorded against the superseded mass allocation
   are legacy evidence and are not reusable calibration data.
3. **MPC limits:** its plant is a linear inverted pendulum driven by ground
   acceleration, not a torque/contact/inertia-constrained multibody optimizer.
   The inherited implementation integrated acceleration with `dt/2`; the core
   agent corrected this to full `dt`. Main reports 58 passing core checks and
   the latest independent ROS rerun passed 29 contracts; recorded full-dt physical
   balance evidence and its tracking/drift limitations are detailed below.
   Prediction sampling period and
   observation/update period must be reconciled. Fixed solver iterations do not
   establish convergence. Optional pitch integral trim is disabled by default.
4. **Actuation:** ROS sends wheel velocity references and proportional leg
   effort, with leg damping in Gazebo physics. Torque fields in the core do not
   prove wheel torque limits are enforced by an ideal velocity controller.
   Disable sends zero wheel velocity, which is not necessarily passive torque-off.
   Adding delayed derivative effort can duplicate physical damping.
5. **Recovery and guards:** FSM falls are checked against absolute pitch while
   MPC uses error from its pitch reference; test consistency for leaned poses.
   StandingUp bypasses the ordinary fall path. Review the settle timeout and
   subsequent mode, rather than assuming stage completion proves successful
   recovery. Old `knee=-2*hip`, mirrored tuck and zero stand poses are candidates
   for the old robot only. Supplier knee angles are not yet mapped to CAD zero.
6. **Reference trajectory generators:** old bisection helpers do not verify a sign-changing
   bracket. Reject infeasible roots and validate the fitted curve with FK.
   The roll script labels a lateral `ys` statistic as fore/aft error, and scripts
   label angular fit residuals as mm. Recompute metrics with correct units;
   comments and printed numbers are not certificates of trajectory accuracy.
   Polynomial fits cannot establish collision freedom or dynamic stability.
7. **Simulation shortcuts:** simulator orientation and pitch-based contact
   inference must stay explicitly simulation-only. Validate sensor freshness,
   world reset/time jumps, trigger clearing and fresh-enable requirements.
   Preserve separate preview and free-body simulation modes; the preview's
   display mount must not constrain the free-body simulation. Inspect
   `src/legacy/ros2_gazebo_sim/CONTRACT.md` and `tests/test_sim_description.py` for the legacy
   simulation-description contract; existence of this setup is not a physics pass.

## Acceptance Evidence

Main-reported offline evidence (latest core count includes the dt correction):

- 58 core checks: `src/tap_control/tests/core_tests.cpp` (latest safety fixes).
- 3 independent URDF checks: `src/tap_control/tests/test_urdf_model.py`.
- 2 demos: `src/tap_control/apps/{balance_demo,mpc_demo}.cpp`.
- 16 Python unittests: workspace `tests/`, including inertials, simulation
  description and pose curves. No per-file counts are inferred here.

Main also reports eight dashboard callback tests registered in teleop CTest
(`src/tap_teleop/tests/test_dashboard.py`). Registration alone is not a passing
test result; this sidecar has not rerun that suite.

All entries describe their tested scope and revision, not unconditional current
build approval. Physical results below apply to the recorded configuration and
tested commands, not arbitrary velocities, poses or hardware operation.

### Strict N20 Physical Trial

Inspected artifact: `log/smoke_1789149917/summary.json`, with configuration and
model snapshots alongside it. Domain 97, N20, full-dt integration, damping 1.
Its binary SHA256 is
`38f7a337e6983f8091c00018416a0002591a5715f6e5cd59a0ed5f6760615900`;
`failure` is null. This final safety-fix binary also passed the revised
29-contract ROS suite and was used for the recovery test below.
All guards passed: pitch error <=0.12 rad, hold drift <=0.10 m, tail speed
MAE <=0.025 m/s, yaw-command error <=0.08 rad, pose displacement <=0.15 m
and final pose joint error <=0.08 rad. These thresholds are bounds, not the
measured MAE or joint-error values; the summary does not export those aggregates.

- Hold: 30.005 simulated seconds, max pitch error 0.0169 rad, base X drift
  -0.016211 m. Total hold/drive/pose duration is approximately 67.05 seconds.
- Forward final speed: 0.061 m/s for 0.06 target; stop final speed: 0.001 m/s.
  Forward base X displacement was 0.14983 m over 3.005 seconds.
- Yaw: 0.4523307 rad for 0.45 target, error 0.0023307 rad.
- Moderate 25% squat and 20% rolls passed; peak pose pitch error 0.0574 rad.
  Measured squat joints (LH, LK, RH, RK) were
  `(0.22998, -0.25271, 0.23025, -0.25274)` rad.
- Measured roll-left joints were `(0.16203, -0.20225, 0.16305, -0.04402)`;
  roll-right: `(0.16692, -0.03971, 0.16036, -0.20641)` rad. After returning
  to stand: `(0.11942, -0.04069, 0.11962, -0.04071)` rad. These are measured
  joint positions, not commands; nonzero standing knee sag remains.
- Roll-left base XY travel was approximately 0.066 m, roll-right 0.052 m.
  Uncommanded yaw changes were +0.06809 and -0.14295 rad respectively.
  Passing drift guards does not mean stationary or yaw-decoupled roll. The yaw
  tracking guard covers the commanded yaw stage, not yaw induced by poses.
- Deliberate final e-stop stopped the wheels and the body fell. That stage is
  excluded from balanced duration and does not establish stand-up recovery.

Recovery outcome is recorded below. Full pose range, stronger
disturbances and hardware dynamics remain unverified. Earlier N50/full-dt and
old-dt/2 tuning runs had poor tracking and larger drift; they are superseded for
current status by this strict N20 run, not erased as historical observations.
`log/smoke_1789149303`, preliminary N20 `log/smoke_1789149399` and the earlier
strict `log/smoke_1789149535` are historical; `log/smoke_1789149917` is primary.

### Recovery Failure

Inspected `log/recovery_1789149922/summary.json`, domain 109, final safety-fix
binary `38f7a337e6983f8091c00018416a0002591a5715f6e5cd59a0ed5f6760615900`.
Setup applied e-stop, set pitch 1.2 rad at z=0.25 m, then let gravity settle
the robot to approximately 90 degrees with no subsequent pose resets.
Clearing e-stop and resetting correctly produced Fallen with `estop=0` and
zero wheel/leg commands. The explicit recovery entered StandingUp but timed
out back to Fallen: no sustained 2-second upright recovery in 10.2 simulated
seconds, pitch approximately 1.571 rad and body z approximately 0.122 m
throughout. Controller-inferred contact stayed zero; this is not a measured
wheel contact-force signal. Thus the reset gate is fixed, but the tested
physical stand-up maneuver FAILED. Recovery remains experimental in the GUI,
not ready for normal use or hardware; other falling directions are untested.

`python3 tools/test_ros_contract.py` is an independent black-box ROS sidecar.
It requires a sourced ROS environment and an installed TAP controller, discovers
the executable beneath TAP's own install tree, chooses a separate local DDS
domain and unique namespace, and publishes synthetic observations. It checks
native odometry/pitch, yaw mixing, bilateral pose clamps, startup, stale/malformed
joint input, command expiry and reset under e-stop. It does not launch Gazebo or
test the GUI domain. Exit 2 denotes missing prerequisites, not a test pass.
Latest isolated ROS contract run on 2026-09-12: **29 checks passed**, exit 0,
domain 137, namespace `/tap_contract_82181409a3`. Domains 0, 97 and 109 were
not used. Tested final safety-fix executable SHA256:
`38f7a337e6983f8091c00018416a0002591a5715f6e5cd59a0ed5f6760615900`.
The executable hash was unchanged across the rerun. Earlier 24-contract passes
are historical and do not replace this stronger reset/recovery contract.
The test overrides `sim_pose_topic` with its own namespaced model-pose publisher.
It also verifies unnamed multi-entity pose rejection, single unnamed pose
acceptance, upright disabled reset entering exactly Disabled, fallen disabled
reset entering Fallen with motors off, explicit stand-up accepted after that
reset, and physical e-stop blocking/interruption of stand-up. Clearing physical
e-stop alone does not resume motion. These are synthetic state transitions,
not successful physical stand-up recovery.
The earlier installed-binary reset and bilateral clamp failures pass in this
build. All test-owned processes exited. Reproduce in the sourced TAP ROS
environment with `ROS_DOMAIN_ID=97 python3 -u tools/test_ros_contract.py` (97 is
excluded; the script selects its own domain). These synthetic ROS checks do not
themselves demonstrate physical stand-up or balance; physical evidence is above.

Update each port row with actual implementation path, command, configuration,
result and remaining limits as work lands. Use independent URDF FK for geometry
checks, deterministic core tests for algorithm contracts and ROS tests for
interface/clock behavior before physical trials. Agree quantitative pitch,
drift, duration and saturation bounds for each trial before judging success.

## Desktop Bringup

### Stationary Performance Update

The opt-in `StationKeeper` outer loop and finite dashboard Show workflow now
have separate evidence in [transformation tuning](TRANSFORMATION_TUNING.md).
On final controller `6b74835be5b2d9a1a66b154f2901e1c79bb2582cc6816ba29418b464e2a85655`,
25% squat/roll passed two 32 s cycles at 20 Hz command publication: peak XY
25.743 mm, final residual 0.786 mm. The normal configuration remains station-off.
General/station ROS contracts passed 29/30 checks respectively. GUI Show/Stop
was tested in an isolated synthetic-feedback window, not the existing desktop.
Larger asymmetric poses failed tracking, and the larger squat candidate did
not complete its freshness-gated two-cycle trial. Jumping remains unimplemented.
These updates do not supersede the failed physical stand-up recovery above.

### Earlier Desktop Session

The final desktop session was started paused, then enabled through the actual
dashboard button. `log/desktop_paused.log` records readiness, unpause and
Enabling -> Balance. Rendering and live feedback were inspected in
`log/desktop_live_final.png`. Initial rendering/curve generation on this PC
occasionally delayed joint feedback enough to trip the unchanged controller
watchdog; start the GUI before enabling physics. The dashboard permits at most
2 seconds for the first feedback after an explicit Enable, then restores its
normal feedback timeout; its UI-stall guard remains active throughout.

RViz demonstrates geometry and joint transforms only. Build/demo/unit passes
demonstrate their own scope only. Balance, driving, squat, roll and stand-up each
need separate sustained TAP Gazebo evidence. No hardware capability, motor
calibration or permission to actuate hardware is implied by this migration.
