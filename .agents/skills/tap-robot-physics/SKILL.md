---
name: tap-robot-physics
description: Adapt TAP CAD robot kinematics, mass properties, COM models and collision geometry when changing the robot model or deriving control geometry.
---

# Robot Physics And Kinematics Expert

Read workspace `AGENTS.md` and the current `simulation/model/{tap_robot.xml,PHYSICS.md,rb_cad_manifest.json,mesh_inertials.yaml}`
and `controller/core/include/robot_control/{GeneratedGeometry.hpp,WholeBodyModel.hpp}`.

- Preserve CAD zero and native +Y axes on both sides. Build FK using actual
  origins, axes and link inertials; never substitute a mirrored model. Keep
  visual orientation distinct from body/IMU orientation. Driver sign conversion
  belongs in `MotorMapping.hpp`, not in the geometry.
- Use SI units. For rotated inertials, compare tensors in the same frame with
  `R I R^T`; check COM transformation, positive eigenvalues, principal-moment
  triangle inequalities and summed mass. Do not double-count motor mass.
- `GeneratedGeometry.hpp` is marked as generated from the MJCF, but its
  generator script is no longer in the repository. After any change to
  geometry, mass allocation or body orientation, update the header consistently
  and validate `WholeBodyModel` against independent FK from `tap_robot.xml`
  (`simulation/scripts/validate_model.py`) across symmetric AND asymmetric
  poses. Report the sampled range and the worst errors.
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
