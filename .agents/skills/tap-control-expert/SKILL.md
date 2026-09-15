---
name: tap-control-expert
description: Port or review TAP MPC, whole-body compensation, joint trajectories, embedded control and state-machine recovery from the wheel_leg reference.
---

# MPC And State Machine Expert

Read workspace `AGENTS.md` and `docs/ALGORITHM_MIGRATION.md`; inspect reference
`balance_firmware_core/core/{include/robot_control,src}` and actual TAP core.

- Keep deterministic algorithm code independent of ROS/Gazebo. Preserve the
  six-element order LH, LK, LW, RH, RK, RW at documented boundaries; name-map
  external joint messages instead of trusting their array order.
- Reference MPC is a four-state linear inverted-pendulum acceleration optimizer,
  not full rigid-body inverse dynamics or contact-constrained whole-body MPC.
  Audit model period versus update dt, acceleration-to-velocity integration,
  saturation, horizon bounds, solver warm state and reset behavior explicitly.
- Replace old geometry, mirrored wheel mapping and WholeBodyModel fits. Derive
  forward/yaw mixing and inverse odometry together for native CAD axes; prove
  both with sign tests before tuning gains. Use current FK for equilibrium pitch
  and COM height, including asymmetric poses and body mesh rotation.
- Preserve stop priority, disable behavior, stale-sensor response and explicit
  re-enable semantics across core AND ROS wrapper. Test requests held high,
  fall during pose motion, stop during recovery and reset after time jumps.
- JointMotionPlanner is cubic smoothstep position interpolation, not a
  velocity/acceleration-limited planner. Add or validate feasibility separately.
  Stand-up Tuck/Rise/Settle is a candidate sequence, not proven recovery on TAP;
  remap poses, test contact/clearance and distinguish timeout from success.
- Keep optional pitch integral trim disabled until its settled-state gate is
  supported by data. Re-evaluate station hold during drive/pose transitions.
  Do not reuse old gains, torques or widened recovery fall limits as measured
  motor capabilities.

Deliver explicit frame/parameter changes and deterministic regression evidence.
Label balance, roll and recovery as pending until closed-loop TAP trials pass.
