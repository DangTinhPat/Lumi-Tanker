---
name: tap-robot-physics
description: Adapt TAP CAD robot kinematics, mass properties, COM models and collision geometry when changing the robot model or deriving control geometry.
---

# Robot Physics And Kinematics Expert

Read workspace `AGENTS.md`, the current source CAD, and the model files that
belong to that CAD before deriving physical parameters. No simulation CAD
model is currently configured in this workspace.

- Preserve CAD zero and native +Y axes on both sides. Build FK using actual
  origins, axes and link inertials; never substitute a mirrored model. Keep
  visual orientation distinct from body/IMU orientation. Driver sign conversion
  belongs in `MotorMapping.hpp`, not in the geometry.
- Use SI units. For rotated inertials, compare tensors in the same frame with
  `R I R^T`; check COM transformation, positive eigenvalues, principal-moment
  triangle inequalities and summed mass. Do not double-count motor mass.
- Treat existing generated control geometry as unverified until it is rebuilt
  from the adopted CAD and checked against independent forward kinematics.
- Derive COM relative to the contact/axle geometry, including the wheel radius.
  The robot's measured balance angle differs from the CAD prediction (the
  firmware compensates a fore/aft CoM offset), so treat CAD mass properties as
  estimates and say so.
- A balance simulation needs a free body, valid collision/contact geometry and
  a nonpenetrating spawn pose. Prefer simple primitive collision shapes; do not
  mistake STL detail or guessed friction for calibrated contact physics.

Deliver model/frame changes, provenance of measured versus assumed parameters,
FK/inertia checks and unresolved mechanical limits to the control expert. Do not
certify hardware dynamics from uniform-density mesh estimates.
