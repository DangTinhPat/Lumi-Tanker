# Provisional Mass Properties

Supplier data supplied on 2026-09-12. All masses are kg, lengths m, and inertia
tensor entries kg*m^2. Each thigh/shin/wheel value is interpreted as per side.

| Component | Mass per part | Count | Basis |
| --- | --- | --- | --- |
| Body | 2.902 | 1 | Battery 0.500 + frame 0.100 + electronics 0.750 + four motors of 0.388 |
| Thigh | 0.233 | 2 | 0.160 + 0.073 |
| Shin | 0.150 | 2 | Supplier value |
| Wheel and motor assembly | 0.135 | 2 | User-approved approximate combined mass |
| Total represented mass | 3.938 | | Approximate total under the assumptions below |

Edit `masses.yaml` to update the masses. Xacro scales each inertia tensor with
its link mass, so mass changes do not require regenerating the geometry data.

## Inertia And Centre Of Mass

`mesh_inertials.yaml` is generated from the seven closed STL meshes by
`tools/estimate_inertials.py`, using trimesh volume integrals. Each part is
treated as a uniform-density solid and scaled to its supplied mass. The centre
of mass is the volume centroid, not the bounding-box or vertex average.

Generated tensors are expressed about that centroid in the source mesh axes.
All six independent tensor entries are retained, including products of inertia.
URDF `<origin>` places the tensor at the centroid; no second parallel-axis shift
is applied. Geometry tensors are stored per kg and multiplied by mass by Xacro.
Each geometry record includes the source mesh SHA-256 for verification.

The body visual is rotated -90 degrees about the hip axis (link Y), as requested
by the user. Its COM is rotated by the same transform, and its inertial frame
has pitch -pi/2. Tensor entries remain in that rotated frame; consumers obtain
the link-frame tensor as R I R^T. Leg meshes and all joint frames are unchanged.

These are estimates, not measured CAD mass properties. A hollow body containing
battery, electronics and motors does not really have uniform density. Once
component positions or CAD mass properties are available, replace this estimate.

## Unresolved Physical Details

- Hardware selection update: four leg motors are Steadywin GIM6010-8; the two
  wheel motors are now CJC BL4820M12, superseding the six-GIM selection.
  CJC's supplied page has no mass or rotor inertia. Values in `masses.yaml`
  remain the earlier approximate model. `wheel_motor.yaml` records manufacturer
  reference data only, not runtime actuator limits. See
  `docs/MOTOR_SELECTION.md` at the workspace root for verification requirements;
  do not retain the 0.135 kg wheel-plus-motor assumption as a verified new mass.

- The four hip/knee motors are retained in the body mass exactly as grouped by
  the supplier. If knee motors are carried on the thighs, their masses and
  inertias must be moved there, not counted twice.
- The user approved treating 0.135 kg as the combined wheel and motor mass for
  this approximate model. It is lumped onto the rotating wheel mesh.
  Real stator mass belongs to the supporting link, rotor mass to the wheel;
  the split and motor geometry are not available. Current wheel
  rotational inertia is therefore only a surrogate, not motor/rotor inertia.
- Supplier wheel radius 0.030 matches the mesh. Outer width is 0.334; axle
  centre spacing is 0.309. Neither quantity has been substituted for the other.
- Supplier segment lengths are 0.105 and 0.110; mesh-derived sagittal joint
  distances are about 0.1057 and 0.1103. Joint positions retain the CAD assembly.
- Supplier hip travel is unrestricted and knee travel is 30 to 145 degrees.
  Current preview joint limits are unchanged: conversion of the supplier knee
  angle to the CAD-zero convention is unconfirmed. Do not use preview limits
  or effort/velocity placeholders for hardware control.

The ground mount fixes the body only for RViz. `simulation.urdf.xacro` uses the
same inertials with a free root, primitive collisions, a body-aligned IMU and
ros2_control interfaces. See `src/tap_sim/CONTRACT.md` (workspace-relative) for
simulation actuator and contact assumptions. Wheel velocity actuation is
idealized; motor electrical dynamics and hardware transmissions are not modeled.
Closed-loop evidence belongs in the migration report, not in the inertial data.

## Verification

From the workspace root in the ROS environment (with numpy, trimesh and PyYAML):

```bash
python3 tools/estimate_inertials.py
python3 -m unittest discover -s tests -v
colcon build --symlink-install
./run_preview.sh
```
