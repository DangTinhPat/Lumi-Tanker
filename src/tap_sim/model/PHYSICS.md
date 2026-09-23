# RB R1 native model

`tap_robot.xml` is the canonical simulation model for the supplied
`LAP RAP ROBOT R1 ver 170926.stp`. It is native MuJoCo and intentionally has
no middleware or external simulator dependency.

## CAD and kinematics

- The STEP assembly is millimetres and was saved folded. `rb_cad_manifest.json`
  records the source hash and the pose removed during import.
- The seven moving bodies are body, left/right thigh, shin, and wheel links.
- The joint order is `[LH, LK, LW, RH, RK, RW]`; every hinge axis is native
  `+Y`, including the right side. Joint origins are the CAD origins:
  hip `(0, ±0.125, 0)`, knee `(-0.01410935, ±0.002, -0.10476742)`, and
  wheel `(0.04663076, ±0.0275, -0.1)` metres.
- The mechanical knee encoder zero remains the native `q_knee=0` reference.
  The natural ENABLE reference was intentionally lowered by 30 mm for the
  current operating point: hip `1.0104670525 rad`, knee
  `-1.0682730675 rad`. Every joint remains
  inside its provisional travel; this pose is neither a travel limit, a rest
  pose, nor an actively standing/balance pose. The shin carrier is mounted at
  `-0.45 rad` relative to the former CAD-zero frame and the local hinge axis
  remains `+Y`.
- The STEP knee-assist is component 1133 on the positive-CAD-X side and
  component 1180 on the negative-CAD-X side. The importer splits each CAD
  compound at its fixed/moving interface: the upper mesh follows the thigh,
  the lower mesh follows the shin, and a spatial MuJoCo tendon joins their
  measured endpoint sites. Knee motion therefore changes the assist length
  and passive force; it is not a decorative mesh attached only to the thigh.
- Visual STL meshes are link-local. The body surface is split into four STL
  assets because MuJoCo limits one mesh asset to 200,000 triangles; all
  638,359 body triangles are retained. Collision uses primitive envelopes to
  keep contact solver behaviour stable.

## Mass and inertial evidence

The BOM masses are body `3.371 kg`, each thigh `0.296 kg`, each shin `0.109
kg`, and each wheel/motor `0.220 kg`, totalling `4.621 kg`. Inertias and COMs
are uniform-density estimates from the CAD surfaces in `mesh_inertials.yaml`;
they are not measured hardware dynamics and must not be treated as a physical
certification.

The RB wheel envelope is approximately 40.27 mm radius in the STEP. The native
contact/control nominal is therefore `0.040 m`. The natural ENABLE mechanical
pose is hip
`1.0104670525 rad`, knee `-1.0682730675 rad`, root height `0.148435 m`; it places the wheel
envelopes at the floor while keeping the bent shin clear of the floor. At this pose the assist
tendon length is `0.097053489 m`. The assist is currently disabled in simulation
with stiffness `0 N/m` and damping `0 N·s/m` at the user's request. The CAD
topology and changing length remain authoritative; spring force stays disabled
until a spring-rate/preload measurement from the physical RB is available.

The only simulated sensor contract is one MPU6500-like accelerometer/gyro site
and six joint encoders. The MPU6500 site is centered in the body X/Y plane and
mounted `0.020 m` above the body reference origin, at the upper body surface.
MuJoCo pose, contacts, and constraint forces remain simulation telemetry and
are not exported as controller inputs.
