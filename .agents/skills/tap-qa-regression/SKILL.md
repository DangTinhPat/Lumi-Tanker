---
name: tap-qa-regression
description: Verify TAP algorithm migration with model, control, ROS contract and closed-loop simulation regression evidence; distinguish builds and RViz from physical capability.
---

# QA And Regression Expert

Read workspace `AGENTS.md`, `docs/ALGORITHM_MIGRATION.md` and current test/build
entrypoints. Inspect implementation before selecting commands; do not invent a
passing launch or copy reference smoke-test thresholds as TAP acceptance limits.

- Model checks: mesh integrity/hash, seven inertial links, six movable joints,
  native +Y axes, mass sum, rotated body COM/tensor and independent FK. Validate
  squat/roll fit error and feasibility throughout their claimed envelope.
- Core checks: forward/yaw mapping and inverse odometry, zero-error MPC,
  acceleration/velocity bounds, reset determinism, finite outputs for supported
  inputs, trajectory endpoints/dt handling, mode transitions and stop priority.
  Exercise asymmetric legs, stale input, held requests and interrupted recovery.
- ROS checks: shuffled/missing joint names, interface units/order, stale IMU or
  orientation, time jumps, pause/resume, watchdog latch/re-enable and correct
  package overlay. Confirm command values, not just topic existence.
- Physical checks, only when execution is authorized: upright hold, drive and
  yaw, pose transitions, squat/roll, disturbance/fall and both stand-up directions.
  Record simulated duration, pitch/drift bounds, saturation, contact behavior,
  mode history and failure reason. Choose and state bounds before judging runs.
  Stand-up timeout, a short noncrashing launch or an RViz pose is not success.
- Maintain evidence levels separately: source inspected, port present, build,
  unit/contract tests, RViz rendering, closed-loop Gazebo, hardware. Pending
  levels remain pending; do not inherit reference robot results.

Report exact commands, versions/configuration, observed results and gaps. Review
only owned diffs; preserve concurrent edits. Skill frontmatter validation checks
discoverability, not algorithm correctness or autonomous expert performance.
