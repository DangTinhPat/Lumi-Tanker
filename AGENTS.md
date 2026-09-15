# TAP Project Guidance

`/home/dvt/tap` is the main workspace. Preserve concurrent user/agent edits.
The user authorizes broad algorithm migration for this robot and simulation,
not hardware operation, motor actuation, flashing, or deployment.

## Desktop Session Lifecycle

Before reopening the robot, stop all existing TAP simulation/preview sessions,
Gazebo/RViz windows, dashboards and their ROS child processes, including orphaned
children. The user explicitly requests a single fresh desktop robot session.
Identify ownership from launch ancestry, workspace paths and session metadata;
never kill unrelated applications or all Python/ROS processes by name. Escalate
shutdown only for verified TAP processes, and confirm no live leftovers before
launching the replacement. Do not preserve a stale TAP desktop session merely
because it was opened in an earlier turn.

## Expert Routing

Read the relevant discoverable skill before domain work. For cross-domain work,
use each relevant expert and explicitly hand off frame conventions and evidence.
These are persistent expert instructions, not claims of running agent processes.

| Work | Skill |
| --- | --- |
| CAD, URDF, FK, COM, inertia, contact geometry | `.agents/skills/tap-robot-physics/SKILL.md` |
| MPC, trajectories, control modes, recovery | `.agents/skills/tap-control-expert/SKILL.md` |
| ROS interfaces, Gazebo, launch, GUI integration | `.agents/skills/tap-ros-gazebo/SKILL.md` |
| Tests, regressions, simulation evidence | `.agents/skills/tap-qa-regression/SKILL.md` |

## Model Contract

- Native CAD joint axes are `+Y` on BOTH sides; CAD assembly is joint zero.
  Do not import the old robot's mirrored right-side angle/wheel signs.
- Wheel radius: `0.030 m`. Axle center spacing: `0.309 m`; half-track:
  `0.1545 m`. Outer width `0.334 m` is not the track.
- Approximate total mass: `3.938 kg`: body `2.902`, each thigh `0.233`,
  each shin `0.150`, each wheel plus motor `0.135`.
- Body visual pitch is `-pi/2`; rotate COM and inertial frame consistently,
  not joint or IMU frames merely to match the mesh. Legs remain unchanged.
- All four hip/knee motors are provisionally lumped in the body; wheel motor
  and wheel are lumped on the rotating link. Neither allocation is measured.
  Uniform-solid mesh volume integrals estimate COM and inertia. Tensors are
  about mesh centroids, per kg, scaled by mass; avoid a second parallel-axis
  shift or applying the body rotation twice.
- Supplier lengths `0.105/0.110 m` are nominal; retain CAD joint origins unless
  intentionally recalibrated. Knee mechanical range `30..145 deg` is not yet
  mapped to CAD zero. Preview limits and torque/speed values are placeholders.
- Re-read the current model and `PHYSICS.md` before edits; record intentional
  changes to this contract rather than overwriting newer user decisions.

## Architecture And Evidence

`src/tap_robot_description` owns the existing CAD model. Target packages are
`src/tap_control/{core,apps,ros2_nodes,config}` and
`src/tap_sim/{launch,world,scripts}`; inspect actual implementation before use.
Keep the control core ROS-independent and robot-specific parameters explicit.
Use portable package/resource lookup, not machine-specific paths in runtime code.

Reference: `.reference/wheel_leg` at
`701cef416b3f224925ab3b2532cf1ee5d401c703`; treat it as read-only provenance,
not the active workspace. See `docs/ALGORITHM_MIGRATION.md` for port inventory.
Do not silently rely on an old ROS overlay or edit `/home/dvt/wheel_leg`.
Report source inspection, build, unit tests, RViz, and physical simulation as
different evidence levels. A working GUI or RViz view does not prove balance.
