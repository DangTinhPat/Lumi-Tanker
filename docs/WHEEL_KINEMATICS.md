# Wheel Kinematics and Hop Feasibility

Jump-development update: the historical analysis below is preserved. The
opt-in jump controller now implements absolute wheel-spin observation and the
inverse relative-joint velocity command through `WheelFrame`; legacy behavior
is unchanged. MPC's cart state is the axle, not the body origin, so this change
does not subtract the axle-offset Jacobian from that cart observation. World
body station-keeping remains a separate outer loop. See [jump status](JUMP.md)
for implementation assumptions and physical evidence; the older experiments
below do not validate this newer controller.

## Scope and Evidence

This is a read-only physics analysis recorded on 2026-09-12. It does not
implement compensation, change actuator parameters, or certify a larger motion
envelope. Configuration values below describe the analyzed baseline, not a
requirement to retain those values in future tuning.

Sources:

- `src/tap_robot_description/description/cad_robot/robot.urdf.xacro`: joint
  origins, axes, collision primitives and damping.
- The adjacent `masses.yaml` and `mesh_inertials.yaml`: supplied masses and
  estimated uniform-density COM/inertia, including the rotated body visual.
- `src/tap_teleop/config/pose_curves.json` and
  `src/tap_teleop/tap_teleop/pose_curves.py`: 71 samples per curve and the
  whole-body COM-neutral construction.
- `src/tap_control/ros2_nodes/BalanceControllerNode.cpp`: relative wheel
  odometry, velocity commands, position-only leg effort and contact heuristic.
- `src/tap_control/core/src/MpcBalancer.cpp`: ground-velocity target converted
  to wheel velocity with a fixed differential-drive track.
- `log/smoke_1789149917/samples.json` and `summary.json`: independent model
  translation/heading, measured joints, and controller telemetry. This trial
  used domain 97, 30-second hold, drive/yaw exercises and moderate pose changes.

SHA-256 provenance of files checked when writing this document:

```text
robot.urdf.xacro
8ee4c97fef17d187fe5117ce2c1c126bdb8475799601579840e26c28cc03d09c
pose_curves.json
3d36e983ab73168cd9f955e6d9276a1fc20d0027de1ea1f22b52f0bb552229b3
smoke_1789149917/samples.json
b08a0ac2915f656856dac2b8cefe38a681f642bbd2e525a4bb4987db08661b1a
trial controller binary, from summary.json
38f7a337e6983f8091c00018416a0002591a5715f6e5cd59a0ed5f6760615900
```

The JSON's embedded URDF fingerprint hashes expanded XML, not the source Xacro
file; these are different hashing inputs. Evidence logs are local artifacts and
may not be present in every checkout.

## Frames, Dimensions and Assumptions

Body axes are +X forward, +Y left and +Z up. All six native joint axes are +Y.
Positive wheel rotation rolls forward on either side. Angles are radians,
lengths metres, velocities m/s or rad/s, forces N, torques Nm, and damping
Nms/rad. The body visual's -90-degree rotation does not rotate the body frame
or IMU axes.

For one leg let `h`, `k`, `w` be native hip, knee and wheel joint angles. Define:

```text
r = 0.030                  wheel radius
d = 0.025                  cylinder width
B = 0.309                  level wheel-center track
y_L = +0.1545, y_R = -0.1545
a = (-0.01410935, 0, -0.10476742)
b = (+0.04663076, 0, -0.10000000)

Ry(q) [x,y,z] = [cos(q)*x + sin(q)*z, y, -sin(q)*x + cos(q)*z]
p_i = [0,y_i,0] + Ry(h_i)*a + Ry(h_i+k_i)*b
```

`p_i` is the wheel-center position relative to the body origin. Combining the
three lateral offsets is exact here: the hip, knee and wheel origins contribute
`+/-0.125`, `+/-0.002`, and `+/-0.0275`, and +Y rotations preserve them.
Left/right axle X/Z positions differ when joint angles differ; their axes
remain parallel. The model does not have a single rigid axle under asymmetric
leg motion.

The planar derivation assumes flat ground, wheel contact without longitudinal
slip, zero body roll and cylindrical wheels. Body vertical motion is allowed;
"stationary body" below means zero horizontal forward translation and yaw,
not a prohibition on squat height changes. Arbitrary lateral/roll motions have
additional contact constraints and are not solved by two wheel speeds alone.

## Planar Derivation

Let `theta` be body pitch and `psi` heading. In the horizontal heading frame,
the wheel's forward offset from the body is:

```text
A_i = cos(theta)*p_i.x + sin(theta)*p_i.z
```

All pitch-plane rotations commute, so the absolute wheel spin angle is
`theta + h_i + k_i + w_i`. Its contact point has forward material velocity
equal to wheel-center velocity minus radius times absolute spin rate.
Longitudinal no slip therefore gives:

```text
v_center_i = r*(theta_dot + h_dot_i + k_dot_i + w_dot_i)
v_center_i = V_body - psi_dot*y_i + A_dot_i

w_dot_i = (V_body - psi_dot*y_i + A_dot_i)/r
          - theta_dot - h_dot_i - k_dot_i
```

For stationary translation/yaw set `V_body=psi_dot=0`, retaining the actual
body-pitch and leg-motion terms. Subtracting only `h_dot+k_dot` is insufficient:
it holds wheel absolute spin while allowing the moving axle to drag the body.

An explicit Jacobian avoids differencing noisy axle positions. With
`u=Ry(h)*a`, `v=Ry(h+k)*b`, `p.x=u.x+v.x`, `p.z=u.z+v.z`:

```text
A_dot = J_h*h_dot + J_k*k_dot + J_theta*theta_dot
J_h     = cos(theta)*(u.z+v.z) - sin(theta)*(u.x+v.x)
J_k     = cos(theta)*v.z       - sin(theta)*v.x
J_theta = cos(theta)*p.z       - sin(theta)*p.x
```

Each `J` has units metres per radian. At neutral `h=0.115483095869`, `k=0`,
`theta=0`:

```text
J_h = J_theta = -0.2071508431 m/rad
J_k           = -0.1047070265 m/rad

w_dot_ff = -7.905028*h_dot - 4.490234*k_dot - 7.905028*theta_dot
```

The dimensionless coefficients multiply rad/s. These are a local
linearization, not fixed gains valid through the entire squat curve.

For fixed heading, integration gives an endpoint check:

```text
Delta X_body = r*Delta(theta+h+k+w) - Delta A
Delta w_for_stationary_body = Delta A/r - Delta(theta+h+k)
```

## Full 3D Rolling Constraint

Use this form for appreciable roll rather than adding Euler angle rates.
Let `R` map body to world, `Omega` be body angular velocity in world,
`n` the upward ground normal, and `a_w=R*[0,1,0]` the unit wheel axis.
Let `J_i*q_dot` be the body-frame axle velocity from hip/knee motion only.

```text
t = (a_w cross n)/|a_w cross n|              forward rolling tangent
s = sqrt(1 - (a_w dot n)^2)
rho = -r*(n - (a_w dot n)*a_w)/s
      - (d/2)*sign(a_w dot n)*a_w            center-to-contact offset
v_C = V_body_world + Omega cross (R*p_i) + R*J_i*q_dot
Omega_w = Omega + (h_dot_i+k_dot_i+w_dot_i)*a_w
```

`rho` is the lowest point on an ideal finite cylinder when cambered. At zero
camber the contact is a line; choose its axial center (`sign(0)=0`). Require
`s>0`; a horizontal wheel plane is a degenerate rolling configuration.

The material point at contact obeys:

```text
t dot (v_C + Omega_w cross rho) = 0
t dot (a_w cross rho) = -r

w_dot_i = t dot [V_body_world + Omega cross (R*p_i + rho)
                 + R*J_i*q_dot]/r - h_dot_i - k_dot_i
```

There is no `rho_dot` term: this is the instantaneous material-point velocity,
not the velocity of the geometric contact locus. Use measured angular velocity
and measured joint rates, or coherent trajectory rates, with matched timestamps.

This equation satisfies only the longitudinal constraint. Lateral no-slip and
normal contact impose further conditions. In particular, staggered parallel
wheels cannot execute every requested yaw/roll motion without lateral slip.
Do not interpret feedforward as a guarantee of exact world-pose stationarity
under arbitrary leg trajectories, compliance or changing contact patches.

## Axle Velocity Versus Body Velocity

The analyzed node estimates wheel travel and speed from `r*w` and `r*w_dot`.
Gazebo's wheel velocity interface is relative to the shin, not an absolute
wheel rotation sensor. Thus planar axle odometry requires:

```text
v_axle_mean = r*mean(w_dot + h_dot + k_dot + theta_dot)
V_body = v_axle_mean - mean(A_dot)             when psi_dot=0
```

Absolute angles should be unwrapped and referenced consistently for position
odometry. In 3D, integrate world-frame velocity or use world pose; do not treat
an Euler angle sum as a global wheel-travel coordinate.

If MPC output means ground axle velocity, convert it to relative wheel speed
by subtracting serial angular rates. The body-position anchor must separately
account for the translating axle offsets when producing its axle reference.
If the command already means body velocity, use the complete formula instead.
Do not apply `A_dot` in both places. MPC state position, velocity, reference and
output semantics must agree; otherwise compensation changes the assumed plant.

An outer world-pose loop remains useful: kinematic feedforward cannot remove
slip, COM-model bias, delayed feedback or actual unwanted wheel commands.

## Numeric Verification

For each stage of `smoke_1789149917`, take its first and last telemetry rows.
Compute `A_i` from recorded `joint_positions` and `pitch`, and:

```text
D_i = r*Delta(theta+h_i+k_i+w_i) - Delta A_i
predicted_forward = (D_L+D_R)/2
predicted_yaw = (D_R-D_L)/B
measured_forward = Delta base_x*cos(mean_yaw) + Delta base_y*sin(mean_yaw)
```

This endpoint yaw comparison approximates small roll and modest heading change;
it is not an exact SE(3) reconstruction. Pitch telemetry is rounded, and topic
samples are not perfectly synchronized. The sub-millimetre agreement below
should not be interpreted as measurement accuracy.

| Stage | Predicted forward, m | Measured forward, m | Predicted yaw, rad | Measured yaw, rad |
| --- | ---: | ---: | ---: | ---: |
| Squat | -0.010239 | -0.010463 | -0.000265 | +0.000077 |
| Stand | +0.009719 | +0.009464 | +0.000005 | -0.000259 |
| Roll-left | +0.064900 | +0.065467 | +0.068294 | +0.068086 |
| Roll-right | +0.050794 | +0.050517 | -0.144783 | -0.142948 |
| Stand after roll | +0.007788 | +0.007123 | +0.076443 | +0.075977 |

During roll-left, both relative wheels travel +0.066362 m. Axle offsets change
by +0.008357 m left and -0.007976 m right. Their difference contributes about
0.0529 rad yaw; serial angular rotation contributes about 0.0154 rad. Thus most
uncommanded yaw is axle-origin motion, with a substantial serial-spin term.
The forward drift is mostly real wheel motion, not a fictitious encoder error.

Using nominal curve endpoints instead of measured joints, the integrated
stationary-body wheel corrections from neutral, at constant zero pitch, are:

| Curve amount | Left wheel increment, rad | Right wheel increment, rad |
| --- | ---: | ---: |
| Squat 0.2 | +0.04297 | +0.04297 |
| Squat 0.5 | +0.10949 | +0.10949 |
| Squat 1.0 | +0.22690 | +0.22690 |
| Roll-left 0.2 | +0.37304 | -0.33750 |
| Roll-left 0.5 | +0.88074 | -0.85817 |
| Roll-left 1.0 | +1.57502 | -1.73846 |
| Roll-right 1.0 | -1.78109 | +1.53986 |

These are endpoint integrals, not constant rate commands. The asymmetric values
are planar predictions; use the 3D constraint for precise execution at roll.

## Full-Curve Geometric Clearance

The offline check evaluated every sample of each existing curve. It rotated
the body to equalize wheel-center heights, then placed the cylinder bottoms
on the floor. For roll `phi`, the finite-cylinder support height is
`r*cos(phi)+(d/2)*abs(sin(phi))`, rather than an invariant radius.

For each collision box with world center `c`, rotation `Q` and dimensions `L`,
its lowest height is `c.z - sum(abs(Q[2,j])*L[j]/2)`. Wheel collisions are excluded
from minimum non-wheel clearance, since they are the intended floor contacts.

| Curve endpoint | Body height, mm | COM height, mm | Body clearance, mm | Minimum leg clearance, mm |
| --- | ---: | ---: | ---: | ---: |
| Squat 1.0 | 197.27 | 183.57 | 161.70 | 20.41 |
| Roll-left 1.0 | 213.21 | 195.64 | 164.73 | 21.39 |
| Roll-right 1.0 | 213.23 | 195.21 | 164.22 | 21.41 |

The clearance minima over all 71 samples are at the tabulated endpoints. Full
roll is -0.13261 rad left and +0.13130 rad right. COM heights in this table are
the existing curve evaluator's radius-based values; adding finite-width wheel
support increases full-roll COM height by about 1.4 mm. Body height and box
clearance already include that correction.

Neutral body height is 237.15 mm, neutral COM height 217.30 mm. Full squat lowers
the body 39.88 mm but COM only 33.74 mm because mass-bearing links move relative
to the body. The latter is the relevant stroke for a simple energy calculation.

These checks establish nominal floor clearance only. They do not test
self-collision, mesh protrusions outside simplified collision boxes, actuator
tracking errors, dynamic pitch, real knee-stop conventions, or contact force
distribution. The curve generator's lateral COM margin check is not a complete
collision or dynamic stability test. Estimated uniform-density inertials and
the provisional assignment of four motor masses to the body remain assumptions.

## Why No Hop Is Supported Currently

Analyzed baseline: total mass 3.938 kg, gravity 9.81 m/s^2, leg command limit
3 Nm per joint, joint damping 1 Nms/rad, position gain 40 Nm/rad, and pose slew
0.4 rad/s. The 12 Nm URDF effort field does not make the node command 12 Nm.
The wheel velocity interface is idealized, not a validated motor torque model.

As a screening calculation, use the full squat's COM extension stroke
`s=0.03373572 m` and a symmetric constant-acceleration push starting at rest.
For a ballistic rise `H` above takeoff:

```text
v_takeoff = sqrt(2*g*H)
a_push = v_takeoff^2/(2*s) = g*H/s
F_per_leg = M*(g+a_push)/2
T_push = 2*s/v_takeoff
```

This is a lumped-COM approximation, not inverse dynamics. At deep squat the
knee-to-wheel horizontal lever arm is 0.0705341 m, giving the rough knee thrust
moment `F_per_leg*0.0705341`. Static support alone is about 1.36 Nm per knee
there, versus about 0.67 Nm at neutral. Link gravity, leg angular acceleration,
nonvertical ground force and changing lever arms are not resolved by this
screening estimate.

| Hop rise | Takeoff speed | Push duration | Force per leg | Deep-squat knee thrust moment | Average knee damping moment |
| --- | ---: | ---: | ---: | ---: | ---: |
| 20 mm | 0.626 m/s | 0.108 s | 30.77 N | 2.17 Nm | 7.43 Nm |
| 50 mm | 0.990 m/s | 0.068 s | 47.94 N | 3.38 Nm | 11.74 Nm |

The final column is `b*0.8/T_push`, because the knee must extend 0.8 rad and
`b=1 Nms/rad`. It is an average over the ideal extension, not a claim that this
value and the deep-squat thrust moment peak simultaneously. Even damping alone
exceeds the 3 Nm commanded torque for this proposed profile. The normal pose
path also requires at least `0.8/0.4=2 seconds` for knee extension, far longer
than the ideal push durations. This does not prove that every possible dynamic
maneuver is impossible, but it rules out claiming the requested quick full-squat
hop is supported by the current pose/actuator setup.

Finally, the baseline contact heuristic is body height above 0.10 m and pitch
magnitude below 0.8 rad. An upright airborne robot satisfies both conditions.
It cannot validate takeoff or landing, and wheel odometry during flight is not
ground travel. A credible hop test needs wheel-floor clearance or actual contact
measurements, independent body/COM vertical motion, a deliberate thrust/flight/
landing controller, and verified actuator limits before any success claim.

No jump trial was performed for this analysis. Existing e-stop, fall detection
and sensor watchdogs must remain effective; do not emulate a hop with a pose
reset, gravity change, or a mode transition unaccompanied by physical flight.

## Station-Keeping Roll Failure at Amplitude 0.5

Evidence: `log/transformation_1789196393455153359/samples.json` and
`summary.json`, including the trial's saved configuration and model files.
The run used station keeping, amplitude 0.5, two cycles, 3-second transitions,
2-second dwells, leg gain 40 Nm/rad, 3 Nm command saturation, and 1 Nms/rad
joint damping. Its controller binary SHA-256 was
`91c449c4b8ecf3448a3bb90ada59b9328d57c49f84a968ffdde32f78d346736b`.

The observed failure is persistent LEFT KNEE command saturation and tracking
loss. It is not a general one-radian hip/knee oscillation:

| Measured quantity | Result |
| --- | ---: |
| Maximum left hip target error | 0.029018 rad |
| Maximum left knee target error | 1.003791 rad |
| Maximum right hip target error | 0.034568 rad |
| Maximum right knee target error | 0.056358 rad |
| Peak horizontal displacement | 0.462123 m |
| Final horizontal residual | 0.352725 m |
| Peak pitch-reference error | 0.028734 rad |

The first near-saturated sample (`abs(command)>2.99 Nm`) occurs at simulation
time 23.821 s, in `roll_left_out_dwell`:

- Left knee target -0.4 rad; measured position -0.474970 rad.
- Left knee command +2.994317 Nm; velocity -0.022444 rad/s.
- Left/right hips 0.198462 / 0.255796 rad; right knee -0.011375 rad.
- Body roll -0.071479 rad, pitch +0.010109 rad, speed 0.011359 m/s.

The knee's roughly 0.075 rad error matches the command saturation threshold
`3/40=0.075 rad`. Its damping moment at that instant is only about 0.022 Nm,
so a large instantaneous damping load does not explain the onset.

During the final 0.5 s of that roll-left dwell, mean left knee position is
approximately -0.587 rad, mean velocity -0.101 rad/s, and its command is at
+3 Nm for 100% of those samples. By the neutral-return dwell ending at
30.116 s, mean position is about -0.652 rad; the endpoint velocity is only
+0.0062 rad/s, while command remains +3 Nm. The requested neutral knee is zero.
This is a persistent low-speed tracking failure, not just transition lag.

The first, uncontaminated amplitude-0.5 squat completed before roll failure:
at the dwell ending 15.115 s, measured knees were about -0.451 rad for a
-0.4 rad target, commands about 2.045 / 2.048 Nm, and velocity approximately
zero. The subsequent neutral return completed. After the left roll fails,
the left knee never returns to neutral; later roll-right and cycle-two squat
failures inherit that bad state. They are not independent tests of those poses.

### Static Load Estimate and Uncertainty

A separate offline estimate used the saved link masses/COM, measured body
pitch/roll and joint positions, finite-cylinder contact offsets, vertical
ground forces distributed by lateral COM moment balance, and distal-link
gravity. It ignored horizontal contact forces, angular acceleration and
constraint/servo effects. Its predicted left/right knee moments were:

| Stage endpoint | Static estimate, Nm | Published commands, Nm |
| --- | ---: | ---: |
| Neutral, 10.115 s | 0.688 / 0.591 | 1.632 / 1.626 |
| Squat dwell, 15.115 s | 1.047 / 0.897 | 2.045 / 2.048 |
| Roll-left dwell, 25.116 s | 1.416 / 0.264 | 3.000 / 0.446 |
| Neutral-return dwell, 30.116 s | 1.475 / 0.273 | 3.000 / 0.638 |

The estimate is approximate, particularly near zero camber where ideal-cylinder
contact-point selection is ambiguous. Nevertheless, its knee moment is below
the saturated command. **Observed saturation does not establish fundamental
motor undersizing or prove that gravity alone requires more than 3 Nm.**
The published command and reported joint effort are also not interchangeable:
the stuck left knee's reported effort is about 1.7 Nm while its command is
3 Nm. The effort-interface measurement semantics need checking before treating
that difference as missing actuator torque.

Wheel velocity-servo reactions, longitudinal/lateral contact forces, closed-chain
constraint loading, and accumulated deformation of the commanded posture are
plausible contributors, not diagnosed causes. Remaining pitch balance only
shows that the balance loop follows the changed physical posture; it does not
establish successful transformation tracking or world-position holding.

### Envelope and Next Measurements

The station-enabled amplitude-0.25 comparison is
`log/transformation_1789196286118087091/summary.json`. It passed with peak
horizontal displacement 0.025633 m, final residual 0.001071 m, maximum joint
error 0.062888 rad and peak leg command 2.515500 Nm.

- Retain 0.25 as the demonstrated combined squat/asymmetric envelope.
- Do not enable amplitude-0.5 roll based on pitch balance alone.
- Treat amplitude-0.5 symmetric squat as a separate repeated-cycle validation
  candidate, starting from a clean neutral state. Its initial successful dwell
  is useful evidence but not a complete envelope qualification.
- Slower transitions might change onset, but cannot be assumed to eliminate
  the sustained saturated dwell. Do not infer that increasing gain fixes it.

Instrument the following before attributing the failure to actuator sizing or
raising limits:

1. Per-wheel contact point, normal force and tangential force, including loss
   of contact and slip velocity at the actual contact point.
2. Wheel velocity target versus achieved relative/absolute speed, wheel motor
   reaction torque and the simulator's effort-feedback definition.
3. Per-leg commanded effort, applied motor effort, passive damping moment and
   joint reaction wrench, synchronized with body acceleration and joint motion.
4. Body-anchor/yaw corrections and both axle velocities, to test whether wheel
   commands oppose the geometry required by the changing leg posture.

These are proposed diagnostic measurements. No new instrumentation, actuator
limits, physics parameters or model edits were made for this diagnosis.
