---
name: tap-robot-physics
description: Adapt TAP CAD robot kinematics, URDF, mass properties, COM models and collision geometry when changing the robot model or deriving control geometry.
---

# Robot Physics And Kinematics Expert

Read workspace `AGENTS.md`, `docs/ALGORITHM_MIGRATION.md`, and current
`src/tap_robot_description/description/cad_robot/{robot.urdf.xacro,PHYSICS.md,masses.yaml,mesh_inertials.yaml}`.

- Preserve CAD zero and native +Y axes on both sides. Build FK using actual
  origins, axes and link inertials; do not substitute a mirrored Upkie model.
  Keep visual orientation distinct from body/IMU orientation.
- Use SI units. For rotated inertials, compare tensors in the same frame with
  `R I R^T`; check COM transformation, positive eigenvalues, principal-moment
  triangle inequalities and summed mass. Do not double-count motor mass.
- Validate WholeBodyModel against independent URDF FK across symmetric AND
  asymmetric poses. Derive COM relative to contact/axle geometry, including
  wheel radius. Regenerate coefficients and report sampled range and worst
  errors whenever geometry, mass allocation or body orientation changes.
- For squat/roll generation, require valid root brackets, feasible joint
  configurations and measured FK residuals. Test intermediate combined poses,
  lateral support margin and collision clearance, not only trajectory endpoints.
- Retain the RViz preview mount only for preview. A balance simulation needs
  a free body, valid collision/contact geometry and nonpenetrating spawn pose.
  Prefer simple physical collision shapes where justified; do not mistake STL
  detail or guessed friction for calibrated contact physics.

Deliver model/frame changes, provenance of measured versus assumed parameters,
FK/inertia checks and unresolved mechanical limits to the control expert. Do not
certify hardware dynamics from uniform-density mesh estimates.
