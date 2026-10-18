# Stand-up recovery on the GIM V3 simulator

Updated 2026-10-17, branch `v3.5`. One dashboard **Đứng dậy (từ nằm)** button
now runs all four stages and hands off to `V3Controller` balance. The core owns
all timing; GUI polling no longer sends a delayed `FLIP` command. The current
simulation defaults overlap steps 3 and 4 as described below.

This implementation runs in `tap_mujoco_gim_v3`, which instantiates
`V3Controller`. It does not describe the older `tap_mujoco_native` runner.
The firmware core keeps `StandUp::Config::enabled=false`; only the simulation
runner enables it and applies the fast fold/flip settings below. Firmware
console integration and physical recovery remain unvalidated.

## Sequence and current simulation defaults

| Stage | Behaviour | Exit |
| --- | --- | --- |
| 1. Folding | Hip moves in negative encoder direction by 2.30 rad, at 0.80 rad/s; knee targets remain fixed | Hip command reaches endpoint |
| 2. Kneeling | Powered hip/knee hold; wheel speed command zero | 1.0 simulated second |
| 3. Pitching | IMU lift feedback, Kp 10, Kd 0.08, hip speed cap 3.0 rad/s, travel ±1 rad | Estimated body clearance ≥3 mm and lift progress ≥0.06 rad for 20 ms while moving toward level |
| 3 + 4. Catching | Blend ongoing hip lift into CoM feedback while ramping wheel control over 80 ms | Blend completes, then capture gate below persists 0.02 s |
| Support / Balancing | Preserve recovery pose and initialize LQR; normal height controller raises body toward requested 130 mm | Continuous balance |

`hold_at_kneel=1` retains a diagnostic pause that requires `FLIP`.
`hold_after_flip=1` retains the diagnostic step-3 hold and prevents catch.
Both default to zero in the simulator. Moving phases time out after 20 s each.
The existing pitch guard remains 0.80 rad during the simulator's fold and
0.72 rad thereafter (core fold default is also 0.72 rad).

The starting pose is the configured `home` keyframe. The fold endpoint is an
encoder target, not contact detection. This sequence does not recognize
arbitrary lying poses. Powered joint support is intentional during kneeling;
`DISABLE` releases the legs, whereas a fault retains the existing leg lock.

## Overlapping lift and capture (current revision)

The runner enables `overlap_after_lift=1`; the core flag defaults to false.
Step 3 now exits while the body is still moving. It requires at least 0.06 rad
(3.4°) reduction of absolute pitch from lift entry, and a body-clearance lower
estimate of at least 0.003 m, continuously for 0.02 s. If already within the
0.04 rad level tolerance, only the progress requirement is waived. The body
must be moving toward level; it need not stop there. Missing clearance leaves
it in Pitching until the normal timeout rather than blindly starting capture.

`robot_control::bodyClearance` uses IMU pitch/estimated roll, both measured leg
chains and the native CAD body envelope. Wheel nonpenetration on a flat floor
bounds root height from below; the lowest corner among six enclosing component
boxes bounds the body's bottom. These are calculation bounds, **not additional
collision shapes or supports**. Regenerate `GeneratedBodyEnvelope.hpp` with
`python3 tools/generate_body_envelope.py` after CAD changes. Its source digest
covers the exposed body STL meshes and placements. No live simulator pose,
contact or distance enters the controller.

This is a model estimate, not contact sensing. Orientation error, floor slope,
wheel deformation and unmodeled protrusions can invalidate the clearance
margin. The 3 mm threshold is an initial tuning value, not a measured hardware
clearance guarantee; clearance at transition also does not guarantee that the
shell cannot touch again later during catch.

At early catch entry, the last hip command velocity is retained. An 80 ms
smoothstep blends it into the catch feedback velocity, with the existing
12 rad/s² acceleration bound. Thus hip speed can initially exceed the steady
2 rad/s catch cap while braking from the preceding lift (up to 3 rad/s).
Wheel control ramps from its previous zero-speed command over the same blend.
The LQR handoff gate remains inactive until this blend is complete.

`--standup overlap_after_lift=0` restores the sequential pitch-settle/catch
sequence. Diagnostic `hold_after_flip=1` also keeps its original full leveling
behaviour. Tuning keys: `lift_clearance` (m), `lift_progress` (rad),
`lift_confirm` (s), `catch_blend` (s).

The earlier sequential results below are retained as historical evidence.
They do not validate the new timing and blending.

## Feedback, frames and coordination

Only IMU and encoder-derived states enter the controller. Both CAD joint axes
are native +Y; positive body pitch is nose-down. Driver polarity conversion
remains in the motor mapping. Control geometry and mass properties come from
the existing GIM model; no mass, gravity, friction or collision changes were
made for this catch implementation. See `src/tap_sim/model/PHYSICS.md` for their
estimated provenance and limitations.

The physical lean used for catch is

```
lean = pitch_IMU + comLine(hip_measured, knee_measured)
lean_rate = gyro_y + J_hip * hip_rate + J_knee * knee_rate
```

It has no hanging-pose rebase. The regular V3 virtual angle does have that
rebase, so the balance trim at handoff is seeded to `-com_line_hang_`.
CoM is estimated from the configured mass model and encoders; this is not an
online identification of unknown mass distribution.

Hip command velocity is

```
u_hip = -(4 * lean + 0.2 * lean_rate) / J_hip
        + 4 * body_leveling * pitch + 0.5 * pitch_rate
```

with a 2.0 rad/s steady speed cap, 12 rad/s² acceleration bound, ±1.2 rad travel from
catch entry, and ±0.12 rad command tracking bound. The sign can reverse from
feedback. There is no injected back-and-forth sweep. When body leveling stalls
capture (|lean| > 0.12 rad, |lean_rate| < 0.15 rad/s, and |hip command rate| <
0.25 rad/s for 0.15 s), its multiplier temporarily becomes zero. It returns to one
when |lean| < 0.08 rad.

Wheel control commands an acceleration:

```
a = clamp(35 * lean + 4 * lean_rate, -3, 3)  [m/s²]
wheel_rate = (integrated_velocity + calibrated_wheel_lag * a) / wheel_radius - carrier_rate
carrier_rate = gyro_y + hip_rate + knee_rate
```

Integration has tracking anti-windup (0.15 m/s margin), and wheel commands are
bounded to ±16 rad/s. LQR handles translational braking after handoff.
Catch lead defaults to the configured actuator lag; `catch_lead` explicitly
overrides it for calibration-error experiments. Carrier compensation matters while the hips rotate:
encoder wheel speed is relative to the moving leg. Ground speed is inferred
assuming rolling support; slip and multiple contacts can invalidate that
assumption during catch.

## Capture and handoff

The gate includes a small-angle divergent-mode estimate:

```
omega = sqrt(m * g * h_com_above_axle / I_pitch_at_axle)
capture = lean + lean_rate / omega
```

This follows the capture-point idea in Pratt, Carff, Raibert and Neuhaus,
*Capture Point: A Step toward Humanoid Push Recovery*, Humanoids 2006,
[doi:10.1109/ICHR.2006.321385](https://doi.org/10.1109/ICHR.2006.321385).
The gate is an angular approximation near balance, not a proof of contact or
an optimal whole-body trajectory.

All conditions must persist for 0.02 s (10 controller ticks at 500 Hz):

- |lean| ≤ 0.08 rad and |capture| ≤ 0.06 rad;
- |lean_rate| ≤ 1.5 rad/s and |body pitch| ≤ 0.25 rad;
- |ground speed| ≤ 0.7 m/s;
- hip target tracking error < 0.06 rad.

Passing through pitch zero alone cannot trigger balance. The 0.7 m/s speed
gate admits a rolling capture inside the balancer's wheel-speed authority;
it does not require stopping before LQR takes control. A 0.5 m/s gate blocked
the useful capture window with a rearward CoM shift and let the catch speed
loop saturate. Fall limits and the lean/divergent-mode conditions are unchanged. Handoff preserves
hip/knee commands, the wheel delay history and measured forward speed, and
seeds LQR at the physical CoM equilibrium. The preview velocity planner is also
initialized to the measured rolling speed, with zero initial acceleration,
and then uses its normal bounded acceleration/jerk to plan the stop. The
station anchor follows this stopping plan and reanchors when the robot stops,
as after a DRIVE command. Seeding LQR alone was insufficient: the preview
planner's stale zero overwrote its reference in the handoff tick. A combined
perturbation case then kept oscillating above the 0.08 m/s settling limit;
the corrected handoff reduced its final-window maximum from 0.0916 to
0.0684 m/s without changing the acceptance threshold.

The height controller's normal
power-up ramp is skipped so it cannot overwrite the recovery pose.

Contacts and exact simulator CoM are used only to score the result after a
run. They never trigger transitions or feed the control loop. A hardware
support-confidence observer and robustness to slip/model error remain open.

## Verified overlapping recovery — 2026-10-17

**34/34 recovery scenarios passed**, plus diagnostic hold and ESTOP controls,
using the current simulator defaults with no tuning override. Evidence:
`build/native/gim_physics_evidence/overlap_release/summary.json`. Adjacent logs,
plant variants, `perturbations.json` and SHA256 fingerprints record the run.

```bash
python3 src/tap_sim/tests/gim_standup_test.py \
  --runner build/native/bin/tap_mujoco_gim_v3 \
  --model src/tap_sim/model/gim_dynamic.xml \
  --evidence build/native/gim_physics_evidence/overlap_release \
  --extended --random-cases 12
```

The 22 individual scenarios cover start/settling time, wheel delay/lag from
20/35 to 60/90 ms, forward/backward 1 N pushes for 80 ms, four synthetic sensor
noise seeds, body CoM ±2 mm, body mass/inertia ±5%, wheel friction 0.6–1.0,
and initial hip angle ±0.02 rad. Twelve further fixed-seed cases combine
CoM, mass, friction, hip pose, start time, delay/lag and sensor noise. Plant
perturbations are not supplied to the nominal controller model.

| Metric | Observed over 34 cases |
| --- | --- |
| B4 entry to balance handoff | 0.300–0.478 s |
| Largest backward body pitch after B4 entry | 0.09996 rad / 5.73° |
| Shell contacts after B4 entry | 0 |
| Support after balance handoff | Both wheels only |
| Peak torque request duration | 0.300–0.478 s; configured ceiling 0.600 s |
| Largest final-window speed | 0.06839 m/s; limit 0.08 m/s |

Every scenario runs for 24 simulated seconds, with the final window at
22–24 s: body pitch <0.05 rad, body height 120–140 mm, and exact-model
CoM/axle horizontal error <6 mm. All stay in Balancing after handoff. Torque
stays within 0.20 N·m during the bounded peak and 0.13 N·m outside it. The
ESTOP case interrupts active peak capture, clears the request and wheel
commands, and leaves physics running. The hold control never enters balance.

`RECOVERY_HANDOFF` records the exact controller tick; both planner and LQR
velocity references agree with measured rolling speed within 0.001 m/s.
An earlier check on the next 50 Hz STATE sample falsely rejected three cases:
the robot had already changed speed in the intervening 20 ms. The final run
uses the event at 500 Hz and preserves all physical acceptance thresholds.
Earlier failed tuning batches remain in their own evidence directories.

Additional evidence on the final control core:

- `v3_standup_tests`, `v3_delay_tests`, `v3_tests`, `gim_model_physics`: 4/4
  passed. The handoff unit test checks both rolling directions.
- Normal balance, 20 s: final Balancing, peak body pitch 0.01376 rad,
  peak virtual error 0.01743 rad.
- `pio run -e esp32-s3-v3`: build passed; RAM 158164 B (48.3%), flash
  1120625 B (17.1%). Build and regression logs are copied into the evidence
  directory. No flashing or hardware run.
- Regenerating the body envelope reproduces the checked-in header exactly;
  Python compilation and `git diff --check` passed.

This is 100% of the specified finite simulation set. It does not establish
100% probability over arbitrary CAD, lying poses, floor slopes, slip, sensor
bias/dropout, actuator limits or real hardware. Synthetic noise and mass/contact
parameters remain assumptions. Hardware timing, single-loop firmware and a
planar stand-up test were not validated in this work. The pre-existing
coordination-test failure noted below remains open.

## Recorded evidence — earlier sequential revision

Before the overlap change, the recorded batch used `overlap_after_lift=0`
(the behaviour then implemented). The recorded batch: 24 simulated seconds per scenario, scoring steady
behaviour over t=22–24 s. Evidence is in
`build/native/gim_physics_evidence/standup/summary.json` and adjacent logs.

| Scenario | Balance handoff (sim time) | Catch duration |
| --- | --- | --- |
| Start at 0.2 s | 5.62 s | 1.898 s |
| Start at 0.7 s | 5.22 s | 0.998 s |
| Wheel delay 35 ms, lag 50 ms | 5.14 s | 1.418 s |
| Forward 1 N push for 0.08 s at t=4.1 s | 5.56 s | 1.838 s |
| Backward 1 N push for 0.08 s at t=4.1 s | 4.82 s | 1.098 s |

All five passed, plus diagnostic hold and ESTOP controls. Offline replay found
only wheel contacts at handoff and in the final two seconds, with body height
120–140 mm. Final-window maximum speed was below 0.017 m/s and exact-model
CoM/axle horizontal error below 0.08 mm. These very small errors reflect the
shared ideal mass model, not hardware accuracy. Hip commands reversed in all
five cases. Wheel torque stayed at or below 0.13 N·m per wheel.

Catch can transiently tilt the body backward about 0.60–0.68 rad (34–39°), and
the shell could touch the floor before handoff. Thus that sequential revision
achieved recovery on this model but did not establish a clean shell-free
lift, arbitrary-CAD robustness or a safe hardware motion. Delay 20/35 ms and
60/90 ms tuning trials succeeded earlier; they were not rerun through this
final scored batch and are not counted among the five results.

Other evidence already collected:

- `v3_standup_tests`, `v3_delay_tests`, `v3_tests`, `gim_model_physics`: passed.
- `v3_coordination_tests`: fails its `adaptiveHeightContract` assertion at
  line 120; the same failure was reproduced in an isolated source copy with
  the pre-task controller, stand-up and predictor restored. It remains open.
- `pio run -e esp32-s3-v3`: build passed; RAM 158076 B (48.2%), flash
  1118393 B (17.1%). No flash or hardware run was performed. Hardware timing
  and the single-loop firmware configuration have not been measured here.
- No full-suite or new planar stand-up validation is claimed.

Recorded batch command (from the worktree root):

```bash
python3 src/tap_sim/tests/gim_standup_test.py \
  --runner build/native/bin/tap_mujoco_gim_v3 \
  --model src/tap_sim/model/gim_dynamic.xml \
  --evidence build/native/gim_physics_evidence/standup \
  --cases nominal,start_later,wheel_35_50,push_forward,push_back
```

That command was run before overlap was introduced. On the current source it
uses the new runner default and has not been rerun; saved results describe the
earlier sequential code.

A headless demonstration uses `--start-key home --at 0.2:STANDUP --run-for 24`.
The runner's `--standup key=value,...` overrides map to `applyStandUpSpec` in
`gim_v3_runner.cpp`; `catch_rate` and `catch_wheel_gain` are obsolete and rejected.
Useful current keys include `catch_hip_gain`, `catch_hip_damping`,
`catch_body_gain`, `catch_rate_max`, `catch_hip_accel`, `catch_travel`,
`catch_accel_gain`, `catch_accel_damping`, `catch_speed_gain`, `catch_accel_max`,
`catch_lead`, `catch_wheel_max`, `catch_lean_tol`, `catch_capture_tol`,
`catch_settle`, `kneel_hold`, `hold_at_kneel` and `hold_after_flip`.

The base wheel cap is 0.13 N·m. During Catching the core requests the user's
0.20 N·m peak for at most 0.6 s, ending earlier on Support, Failed or disable.
The simulator adapter applies that cap; `--wheel-torque` overrides both caps.
The core's peak request has a one-second configuration ceiling and cannot
restart inside one catch. This is a software window, not a measured thermal
rating. The physical motor adapter has not been integrated or validated for
this request, and core stand-up remains disabled by default in firmware.

The fold speed was reduced from 1.20 to 0.80 rad/s after the extended tests
exposed forward-pitch guard trips with a +2 mm body CoM shift, -5% body mass,
or a slightly perturbed initial pose. Travel remains 2.30 rad and the pause is
one second. No fall guard was widened. B3–B4 still proceeds rapidly after that
pause. The capture dwell was shortened from 40 to 20 ms after small changes
in initial settling caused the old gate to miss a valid capture window.

Synthetic noise testing uses independent Gaussian sensor perturbations before
the estimator/controller: scale 1 means accelerometer sigma 0.03 m/s², gyro
0.005 rad/s, encoder position 0.0005 rad, leg rate 0.01 rad/s and wheel rate
0.03 rad/s. `--noise-seed` makes a run repeatable; `--sensor-noise 0` preserves
the original exact sensor path. These values are robustness assumptions,
not measurements of the deployed MPU6500 or drivers. Plant CoM/mass/friction
variants live only in the evidence directory, with unchanged controller
geometry. `--allow-model-mismatch` explicitly permits their total-mass change;
the normal model contract remains strict.

## Historical observations

The diagnosis below predates the current catch controller and fold settings.
It is retained to explain the earlier powered-support pose; its values are
not current defaults. Earlier catch laws failed to converge at both 0.13 and
0.20 N·m and were superseded by the feedback/handoff above.

## Front-slot battery, 2.50 rad fold diagnosis

With the 210 g battery moved to the front CAD slot, a headless MuJoCo run from
`home` (`--at 0.2:STANDUP`, `--run-for 15`) used the 2.50 rad fold target and
the normal 0.13 N m wheel cap. Gravity remained `(0, 0, -9.81) m/s²`; the
stationary IMU acceleration magnitude was 9.81 m/s². The body pitched forward
while the hips folded: at 5.0 s, body pitch was +0.701 rad, measured hip was
-0.325 rad, and wheel torque was saturated at -0.13 N m. At 5.20 s, pitch
crossed the 0.72 rad stand-up guard. The controller entered `Failed` at a
measured hip angle of -0.394 rad, well before the planned endpoint of about
-1.194 rad. The body shell and both wheels stayed in ground contact; the
closest shin collision geometry stayed about 16 mm above the plane and the
thigh/knee geometry moved farther from it. Contact geometry was measured by
replaying the native runner's `VIEW` qpos through MuJoCo `mj_forward` and
`mj_geomDistance`; it is diagnostic telemetry, not controller input.

A separate 0.20 N m wheel-cap run also failed during folding, at 4.90 s and
measured hip -0.294 rad. More wheel authority alone did not make this
trajectory kneel. A static hip sweep with the body root fixed predicts shin
penetration at intermediate hip angles, but the free body rotates and rises
instead. The fixed-root sweep therefore cannot certify a kneeling transition.

With the 3.00 rad fold target and the same front-slot battery, the motion still
failed at t=5.20 s, measured hip -0.394 rad, before reaching the new target.
At t=10 s after `Fault`, the whole-body COM projected to world X=40.2 mm,
between the wheel-ground point at 31.6 mm and the CAD shell-ground point at
70.3 mm. The shell contact point lies on the original CAD mesh surface, not in
a gap filled only by the convex hull. The MuJoCo root is free, gravity is
`(0,0,-9.81) m/s²`, there are no equality constraints or mocap bodies, and
physics time advances during this pose.

`Fault` stops the wheels but deliberately leaves `legs_locked` set. The
runner freezes the last leg target and its leg PD/current loop continues to
apply torque, so the static-looking failed pose has powered hip/knee support.
In a matching run with `DISABLE` at t=5.4 s, the leg current commands went to
zero and the robot moved back onto its shell: body pitch fell from about
+0.65 rad at disable to near zero by t=14.9 s, while hip moved from -0.42 rad
to +1.31 rad. The body shell and wheels remained real ground supports; the
experiment isolates the powered fault hold as the reason the failed pose
persists, not an invisible platform or altered gravity.
