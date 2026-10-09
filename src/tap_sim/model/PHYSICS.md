# GIM6010-8 model frame and physics

Full runtime parameters, friction, actuator equations and evidence are in
[`docs/GIM_SIM_PHYSICS_SPEC.md`](../../../docs/GIM_SIM_PHYSICS_SPEC.md).

This note applies to `ROBOT (GIM6010-8).STEP`. The assembly visual is CAD
geometry. Its STEP file contains neither rigid-body mates nor validated mass
properties; controller geometry therefore uses the explicit measurements and
estimates below, recorded in `gim_mass_properties.yaml`.

## Frames and joints

- World/model axes: +X forward, +Y left, +Z up. The origin is the center of the
  STEP assembly bounding box (source STEP coordinates 1370.7624, 648.5787,
  964.1983 mm).
- Native revolute axes are +Y on both sides. The hip can rotate continuously.
- Hip axes: left `[0, +0.1025, +0.00828] m`, right
  `[0, -0.1025, +0.00828] m`.
- Knee axes: left `[-0.09816, +0.1130, -0.01080] m`, right
  `[-0.09816, -0.1130, -0.01080] m`.
- Wheel axes: left `[0, +0.1327, -0.02988] m`, right
  `[0, -0.1327, -0.02988] m`.
- Both hip-to-knee and knee-to-wheel center distances are 0.1000 m. The wheel
  radius is 0.040 m. STEP wheel-center spacing is 0.2654 m; the previous
  project value 0.309 m is retained as an unresolved conflict and must not be
  mixed into this CAD geometry.
- The CAD assembly knee pose is the 22 degree lower stop. The controller
  coordinate is `q_knee = internal_angle - 22 deg`, so its range is
  `[0, 113 deg]` (`[0, 1.97222] rad`). Encoder/driver zero and mechanical stop
  calibration remain pending.
- The FK hip-to-axle radius spans 38.16 mm at the lower stop to 184.77 mm at
  the upper stop. These are pivot-to-pivot distances, not body height from the
  floor.
- The hip pivot is 8.28 mm above the body-frame origin. With a 40 mm wheel
  radius, the CAD-zero spawn places the body origin 69.88 mm above the ground.
  Body-height IK includes this pivot offset; the simpler leg-height API reports
  hip-to-axle distance only.
- Default simulator spawn and Reset use the `spawn_extended` keyframe:
  hips -56.5 deg, knees +113 deg from CAD zero (135 deg internal angle),
  body origin 0.21649 m above ground, body upright and both wheels on the floor.
  `cad_neutral` remains a separate assembly-reference keyframe, not the spawn.
  The V3 runner's initial lock target matches this extended pose.

## Mass and inertia

Wheel-ground sliding friction is user-specified `mu = 0.8`, applied directly
to both explicit contact pairs (both tangential directions). Each hip/knee
has user-specified Coulomb friction torque `0.1 N m` (`frictionloss`). Their
additional viscous damping is zero; the previous `0.08 N m s/rad` guess is
removed. Wheel-bearing damping and contact softness remain estimates.

Body-ground collision uses the convex hulls of the actual CAD shell parts
50/55 and exposed hip mounts 7/8/31/32, sharing their visual meshes and frames.
The removed guessed 300 mm-long box extended beyond the 160 mm CAD shell and
held the fallen visual 64 mm above the floor. No invisible body box remains.
Both thighs, knee shells, shins and wheel-motor housings now use the convex
hulls of all their CAD components in the same frames as the visuals. The removed
10 mm-radius leg capsules let the knee shell penetrate the floor by 5.6 mm.
Wheels retain cylinders with the specified radius. Hulls fill component cavities;
contact softness still allows brief penetration during impacts.

Adopted mass total is 3.12 kg: body 2.50 kg, each thigh 0.05 kg, each shin tube
0.04 kg, each wheel motor 0.20 kg, and each wheel 0.02 kg. The 0.02 kg wheel
mass is inferred from the previously accepted 0.22 kg wheel-plus-motor total.
The motor stator is grouped with the shin; the wheel alone rotates.

COM and pitch inertia estimates use simple rigid-group approximations recorded
in `gim_mass_properties.yaml` and `GeneratedGeometry.hpp`. The body COM combines
the specified aggregate split with CAD component locations. Thigh, shin, motor,
and wheel inertias use slender-link/cylinder/disc approximations. These are
provisional controller/simulation parameters: no CAD-to-mass solid integration
or physical identification has validated them. No force, current, contact, or
pose telemetry from MuJoCo is a controller sensor.

## Forward and inverse kinematics

The sagittal chain is evaluated with the native +Y rotation convention
`R(q)(x,z) = (cos(q)x + sin(q)z, -sin(q)x + cos(q)z)`. At CAD zero, the
hip-to-knee vector is `[-0.09816, -0.01908] m` and the knee-to-wheel vector is
`[+0.09816, -0.01908] m`. Knee motion adds `q_knee` to the lower-link angle;
there is no old-model knee rebase. FK, axle-plane IK, height IK, COM, and gravity
terms use these same origins and mass groups.

The controller geometry is symmetric in the sagittal plane; left/right lateral
positions come from their CAD joint origins. Wheel track dependent dynamics must
use the measured 0.2654 m CAD spacing until the old 0.309 m contract discrepancy
is resolved.
