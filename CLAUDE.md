# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS 2 Jazzy / Gazebo workspace for a six-joint CAD wheeled robot ("TAP"). Control
algorithms are ported from `DangTinhPat/wheel_leg` (mirrored at
`.reference/wheel_leg`, read-only provenance — never edit it or
`/home/dvt/wheel_leg`), replacing the old Upkie geometry fit with native joint
signs and exact CAD-derived whole-body FK. Robot model:
`src/tap_robot_description/description/cad_robot/robot.urdf.xacro`.

**Scope boundary:** the user authorizes algorithm/simulation work only — not
hardware actuation, motor flashing, or deployment. The final deployment target
is an ESP32-S3 N16R8. The permanently fixed physical sensor inventory is only
one MPU6500 and six motor encoders. Physical wheel-contact, force/torque,
range, camera, localization and motor-current sensors are excluded from the
contract and may not become hidden future dependencies. Simulation success
(build, RViz, closed-loop Gazebo) is never
evidence of physical/hardware capability; keep these evidence levels distinct
in any report.

**Mandatory: read `docs/RULES.md`** before adding or changing any
control-path code — it holds every standing project rule, binding on every
session. Rule 1 there is the strictest so far: no control decision may
depend on simulator-internal state with no real-hardware equivalent. The
deployable path is MPU6500 plus six encoders; `sim_pose` and Gazebo contact
telemetry are diagnostic/simulation inputs only and must not silently become
the hardware support or pose estimate.

`docs/RULES.md` and `architecture.md` are protected project-contract files.
Do not modify either one without explicit user permission for that document
change; a normal implementation request is not sufficient authorization.

## Build, run, test

```bash
# Build (after sourcing a ROS 2 Jazzy environment)
colcon build --symlink-install
source install/setup.bash

# Run (no shell-script launchers anywhere in this repo; ros2 launch directly)
ros2 launch tap_robot_description cad_preview.launch.py   # RViz kinematic preview only, no physics
ros2 launch tap_sim sim.launch.py                # Gazebo, starts paused, awaiting /enable
ros2 run gui gui                                 # Dashboard, separate terminal, same ROS domain -
                                                  # its own Launch sim button does the above for you
python3 tools/run_jump.py [rated|manual-curve] [extra ros2 launch args]
ros2 launch tap_sim sim.launch.py controller_config:="$PWD/src/tap_control/config/<profile>.yaml"  # alt profile

# Headless
ros2 launch tap_sim sim.launch.py gui:=false auto_enable:=true

# Tests
python3 -m unittest discover -s tests -v          # top-level Python tests (jump, geometry, ROS contracts)
colcon test --packages-select tap_control tap_teleop gui
colcon test-result --verbose
python3 tools/generate_control_model.py --check
python3 tools/test_ros_contract.py
python3 tools/test_station_ros_contract.py
python3 tools/transformation_smoke.py --config src/tap_control/config/stationary_show.yaml
python3 tools/sim_smoke.py --duration 30 --exercise --poses --max-pitch-error .12
python3 tools/gui_deep_smoke.py --gui            # creates/cleans up its own sim session
python3 tools/gui_deep_smoke.py --gui --drag
python3 tools/gui_reset_standup_smoke.py         # scripted Reset standup end-to-end QA
```

Run a single Python test: `python3 -m unittest tests.test_jump_dynamics -v` (or
`python3 -m pytest tests/test_jump_dynamics.py::TestName::test_method`).

Run a single C++ test after building: the CMake target names in
`src/tap_control/CMakeLists.txt` match `add_test(NAME ... )` entries (e.g.
`build/tap_control/core_tests`, `jump_tests`, `mpc_equivalence_tests`,
`station_keeper_tests`, `deep_squat_tests`) — run the binary directly or
`ctest -R <name>` from the build dir.

Only one TAP simulation may run per ROS domain / Gazebo partition. Before
reopening the robot, stop all existing TAP sim/preview/dashboard processes and
their ROS children (identify by launch ancestry and workspace path — never kill
unrelated or all Python/ROS processes by name); confirm no leftovers before
launching a replacement.

## Architecture

Five colcon packages under `src/`:

- **`tap_robot_description`** — owns the CAD model: meshes, URDF/xacro, mass
  and inertia (`description/cad_robot/{robot.urdf.xacro,PHYSICS.md,masses.yaml,
  mesh_inertials.yaml}`). This is the source of truth other packages derive
  geometry/mass from.
- **`tap_control`** — the control stack, split so the algorithm core stays
  ROS-independent:
  - `core/{include,src}/robot_control/` — plain C++17: `WholeBodyModel` (FK),
    `MpcBalancer` (four-state linear inverted-pendulum acceleration optimizer —
    not full rigid-body/contact-constrained MPC), `JointMotionPlanner` (cubic
    smoothstep position interpolation, not velocity/acceleration-limited),
    `StateMachine` (enable/disable/stop/recovery), `StationKeeper`,
    `JumpController`/`JumpPlan`, `EmbeddedController` (ROS-free path, see
    `docs/EMBEDDED_PERFORMANCE.md`), `WheelFrame`, `GeneratedGeometry` (output
    of `tools/generate_control_model.py`).
  - `ros2_nodes/BalanceControllerNode.cpp` — the ROS wrapper around core.
  - `apps/` — standalone demo/benchmark binaries (`balance_demo`, `mpc_demo`,
    `control_benchmark`, `model_probe`) linked against `tap_control_core`.
  - `config/*.yaml` — per-behavior controller profiles (deep squat, jump
    variants, stationary show, precision pose, etc.), selected via
    `controller_config:=` at launch, not compiled in.
  - `tests/` — C++ unit/contract tests registered as ctest targets, plus
    `test_urdf_model.py`.
  - **`tap_sim`** — Gazebo integration: free-body physics model, simulation-only
  contact telemetry (`src/contact_snapshot.cpp`), bounded actuator plugin
  (`src/bounded_gazebo_system.cpp`, `include/tap_sim/{actuator_bounds,
  actuator_gate,leg_velocity_reference}.hpp`), ros2_control config, launch
  (`launch/sim.launch.py`), worlds (`empty.sdf`, opt-in `jump.sdf`). The
  simulation topic/frame/units contract (joint order, bridge topics, physics
  rates, provisional actuator limits, opt-in jump physics) is authoritative in
  `src/tap_sim/CONTRACT.md` — read it before touching sim wiring.
- **`tap_teleop`** — pure curve-generation library, no ROS node or GUI:
  `pose_curves.py` and `deep_squat.py` (regenerated neutral squat/roll/deep-squat
  curves, config in `config/pose_curves.json`), `show_sequence.py` (Stationary
  Show). Consumed by the `gui` package and by several `tools/*.py` scripts.
- **`gui`** (package and executable both named `gui`, run as `ros2 run gui
  gui` — matches `DangTinhPat/wheel_leg`'s own reference GUI package name by
  request; see `src/gui/README.md`'s collision warning: this machine's
  `~/bin/ros2` wrapper sources `/home/dvt/wheel_leg/install` first, which also
  has a `gui` package, so always source this workspace's own
  `install/setup.bash` yourself before running it) — `gui/dashboard.py`
  (drive/pose Tk GUI, imports `tap_teleop`'s curves), `sim_process.py` (owns
  a `ros2 launch tap_sim sim.launch.py` subprocess directly via
  `tools/owned_process.py`, backs the Launch/Stop sim button), `gz_world.py`
  (the `gz service` calls behind the sim-only
  Reset standup test convenience — see `src/gui/README.md` for the exact
  sequence and its evidence caveat).

ROS interface contract (stable across packages): topics `/cmd_vel`,
`/target_pose` (LH, LK, RH, RK), `/enable`, `/emergency_stop`, `/stand_up`,
`/reset_controller`, `/control_mode`. Joint arrays always use the six-element
order LH, LK, LW, RH, RK, RW with native CAD signs (`+Y` on both sides — never
reintroduce the old mirrored right-side signs). Wheel outputs are rad/s, leg
outputs Nm. Drive commands expire after 0.5 s without updates. Name-map
external joint messages rather than trusting array order.

`tools/` holds the generation/analysis scripts that feed the above:
`generate_control_model.py` (produces `GeneratedGeometry.hpp`), model.
`generate_pose_curves.py`, `estimate_inertials.py`, `audit_squat_geometry.py`,
jump research/optimization scripts (`jump_dynamics.py`, `jump_lqr.py`,
`optimize_jump.py`, `analyze_jump_*.py`, `audit_jump_plan.py`,
`export_jump_plan.py`), simulation smoke tests (`sim_smoke.py`,
`recovery_smoke.py`, `transformation_smoke.py`, `gui_deep_smoke.py`), and ROS
contract checkers (`test_*_ros_contract.py`).

## Expert routing

Domain-specific working rules live in `.agents/skills/` and are more detailed
than this file; read the relevant one before nontrivial work in that area
(and hand off frame/evidence context explicitly for cross-domain work):

| Work | Skill |
| --- | --- |
| CAD, URDF, FK, COM, inertia, contact geometry | `.agents/skills/tap-robot-physics/SKILL.md` |
| MPC, trajectories, control modes, recovery | `.agents/skills/tap-control-expert/SKILL.md` |
| ROS interfaces, Gazebo, launch, GUI integration | `.agents/skills/tap-ros-gazebo/SKILL.md` |
| Tests, regressions, simulation evidence | `.agents/skills/tap-qa-regression/SKILL.md` |
| Balance/landing/fall-recovery theory (humanoid, quadruped, wheel-legged), before designing a new mechanism | `.agents/skills/legged-locomotion-references/SKILL.md` |

`docs/ALGORITHM_MIGRATION.md` tracks port/verification status per algorithm
piece and is required reading before assuming something is already ported or
validated. `docs/UPSTREAM.md` documents source provenance.

## Current validation status (do not overstate)

**Every number below predates 2026-09-15's `pitch_source` default change**
(`sim_pose` → `imu`, per `docs/RULES.md` Rule 1 — see that file and
`docs/LEDGE_DRIVEOFF_LIMIT.md`'s "Known regression" section). General
balance (hold/squat/drive) is close to parity under the new IMU default.
The 20 cm ledge drive-off specifically regressed from 70-88% to 0/10 under
IMU; this was **largely fixed on 2026-09-16** — root cause was a
zero-authority-headroom bug in `MpcBalancer` (the balance-correction signal
was clamped to the same ceiling as commanded cruise speed, leaving no room
to exceed cruise and catch a lean), not the estimator. Fixing it
(`mpc.max_ground_velocity` 0.6→2.0, `mpc.position_cost_scale` 1.5→3.0 to
offset a small Hold-drift side effect) restored **48-60% recovered**
(n=15-25, three independent batches) under the current `pitch_source: imu`
default — see `docs/LEDGE_DRIVEOFF_LIMIT.md`'s "Velocity headroom" section.
Treat everything else below as "what ground truth was capable of," not the
active configuration's current behavior, until re-measured under
`pitch_source: imu`.

- ~67 s N20 sim trial passed hold/drive/moderate poses (25% squat, 20% roll
  only); roll caused up to 0.066 m translation and ~0.14 rad unintended yaw.
- Deep squat (-1.6 rad knee) and Stationary Show (25% squat/roll sequence) are
  simulation-tested; see `docs/DEEP_SQUAT.md`, `docs/TRANSFORMATION_TUNING.md`,
  `docs/QUALITY_AUDIT.md`.
- Stand-up recovery (Tuck/Rise/Settle) failed the tested 90° lying pose
  (timed out to Fallen); other fall directions are untested. Treat recovery as
  a candidate sequence, not proven.
- Driving off a 20 cm ledge, tested on an isolated clean world
  (`terrain:=ledge`, `src/tap_sim/worlds/ledge_test.sdf`) free of the
  confounding nearby heightfield in `rough_training.sdf`'s own spawn pad:
  the stock controller (no landing-specific overrides) recovers 7/10 (70%,
  n=10) at 1.0 m/s, with natural knee crouch-and-relax on touchdown needing
  no added mechanism. The `landing_recovery_test.yaml` overlay built this
  session (MPC landing-window boost + `LandingReflex` pre-crouch) measurably
  made this WORSE (4/10) — currently recommend the stock config, not that
  overlay. The remaining failures are a distinct, chaotic double-pitch-rate
  flip (not a graceful tip). Every event-triggered mechanism tried (damping,
  impedance, gain-window boost, pre-crouch reflex, edge fillet,
  hip-torque/IMU-accelerometer touchdown detection) either did nothing or
  measurably hurt versus the 70% stock baseline. The mechanism that actually
  helped: raising `mpc.fall_pitch_rad` (0.8→1.1) and `mpc.max_ground_accel`
  (6.0→10.0) as a permanent, always-on change (not gated by any
  touchdown-detection signal) — 80% recovered, n=25 across two batches,
  `sim_smoke.py` regression still passes. **Promoted to
  `balance_controller.yaml`'s own defaults on 2026-09-15** after re-running
  `sim_smoke.py` (pass) and `transformation_smoke.py`/`recovery_smoke.py`
  (no regression vs. baseline; see `docs/LEDGE_DRIVEOFF_LIMIT.md`'s
  promotion note for exact numbers and the one acknowledged gap — deep-squat
  QA is GUI-manual and was not re-run for this promotion).
  `src/tap_control/config/landing_authority_test.yaml` is kept only for
  provenance; it is now a redundant no-op against current defaults. The
  earlier conclusion that this ledge was categorically
  unrecoverable was also wrong — an artifact of the heightfield confound,
  not a torque/energy limit. See `docs/LEDGE_DRIVEOFF_LIMIT.md` for full
  evidence.
- Jump is experimental and developed separately from the validated squat
  profile (`docs/JUMP*.md`); flight >10 cm and stable landing are **not**
  validated. `tools/run_jump.py` opts into bounded actuators/measured
  contacts — plain `ros2 launch tap_sim sim.launch.py` is unaffected.
- Embedded (`EmbeddedController`, ROS-free C++17) has host benchmarks only;
  ESP32-S3 timing and hardware integration are future work
  (`docs/EMBEDDED_PERFORMANCE.md`).

When reporting results, distinguish evidence levels explicitly: source
inspected / build / unit-contract tests / RViz / closed-loop Gazebo / hardware
— never inherit or imply a level you haven't actually produced.
