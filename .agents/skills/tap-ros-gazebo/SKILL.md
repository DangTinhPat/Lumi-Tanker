---
name: tap-ros-gazebo
description: Integrate TAP control with ROS 2, Gazebo, launch files, sensor bridges, controller interfaces and GUI workflows for simulation.
---

# ROS And Gazebo Integration Expert

Read workspace `AGENTS.md` and `docs/ALGORITHM_MIGRATION.md`, then inspect the
actual `tap_control`, `tap_sim` and `tap_robot_description` packages. Their target
layout is a plan, not proof that executables or launch arguments exist.

- Keep preview and free-body simulation distinct. Avoid duplicate joint-state
  publishers or a fixed display mount in physics. Resolve resources using ROS
  package APIs and verify the active package prefix belongs to TAP.
- Define one joint order and frame contract from sensor messages through native
  CAD signs, wheel odometry and actuator commands. Body visual pitch is not an
  IMU correction. Check quaternion validity, timestamp freshness, QoS, clock
  source, timer catch-up, pause/resume and reset semantics.
- Reference wheel output is velocity and leg output is clamped proportional
  effort; damping is delegated to Gazebo joint dynamics. Verify actual controller
  interface types and units, not legacy topic names. Zero velocity is not a
  physical torque-off guarantee, and ideal velocity servos do not validate motor
  torque/current limits.
- Reference may use simulator orientation and pitch-based contact inference.
  Label these simulation-only shortcuts; do not describe them as an estimator
  or physical contact sensor. Preserve watchdogs and explicit enable behavior.
- Port GUI command/status contracts coherently: enable, stop, reset, stand-up,
  drive/yaw, target pose, squat/roll, mode and sensor diagnostics. Keep Gazebo
  world-reset commands simulation-only; reflect unsupported actions honestly.
- When authorized to run simulation, stage resource loading, controller startup,
  fresh observations, command sign checks and then closed-loop trials. Record
  launch arguments and active overlay. Stop only processes owned by that trial.

Hand QA the interface map, clock/contact assumptions and reproducible commands.
Never infer permission for real robot actuation from simulation migration.
