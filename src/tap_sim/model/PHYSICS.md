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
- The importer writes the body STL pre-rotated `+pi/2` about `Y` and the MJCF
  mounts it without compensation on purpose: the root/IMU frame then matches
  the real MPU6500, which reads pitch 0 with the body horizontal while V3.1
  balances the RB. Because the turn is about the hip axis it only relabels
  angles (the native hip zero is `pi/2` off the STEP hip zero); the encoder
  zero is the calibrated homing anyway. Do not "restore" the STEP body
  orientation: a 2026-09-28 attempt did, broke the IMU convention, and was
  reverted on 2026-09-29.

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

## SIM encoder zero

`power_off_collapse_current` is the SIM zero pose for driver channels
`[LH, LK, LW, RH, RK, RW]`. The native runner subtracts these offsets at the
fake GDS68/BL4820M12 feedback boundary:

`[2.273920000, -1.200000000, -3.578270000, 2.273920000, -1.200000000, -3.178980000] rad`.

The native MJCF and V3.1 controller remain in CAD coordinates; the runner
restores these offsets before simulated feedback reaches the controller.
These SIM offsets do not change firmware or physical encoder calibration.

## Power-off rocker fixture

The native runner starts directly at `power_off_collapse_current`, the
gravity-settled belly-down pose: hips `2.27392 rad`, knees at the MJCF lower
limit `-1.2 rad`, root `z=0.06990 m`, and near-zero body pitch. Body and wheel
collision envelopes support this pose on the floor, so it spawns at the
post-fall state instead of starting elevated and dropping into it. While the
lifecycle is paused the robot counts as held still: the runner only updates
kinematics and the simulated MPU6500 reads +1 g along world up in its own
frame with zero rate, whatever the pose. Hip and knee joints use an
estimated dry-friction limit of `1.2 Nm` each and friction impedance
`0.999 0.9999 0.0001` at the `0.333 ms` physics step. A 120 s gravity-only
regression limits each joint's maximum drift to `0.005 rad`; the measured
model drift is below `0.0016 rad`. These remain simulation estimates,
not measured gearbox breakaway torques;
mechanical limits are likewise provisional, not verified hardware stops.
V3.1's internal zero remains the separate `v3_hang` keyframe
(hip `0.3882 rad`, knee `0 rad`, root `z=0.22856 m`); direct ENABLE is rejected
from the prone pose until a rocking/home-to-hang sequence is implemented.
`power_off_deep_front` remains the earlier elevated/fall-start reference.
Contacts and constraint forces are still
simulation telemetry; the controller must infer support from the MPU6500 and
encoders.
