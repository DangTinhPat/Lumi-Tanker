# Pose And Jump Quality Audit

## Acceptance And Scope

Simulation only, Gazebo/RViz workspace `/home/dvt/tap`. No hardware actuation,
new motor capability assumptions, scene forces, or relaxed fall limits.
The requested jump remains experimental until height AND landing recovery pass.

This audit targets two consecutive 25% squat/left-roll/right-roll cycles with
peak body-origin XY displacement <=10 mm and final displacement <=2 mm from a
fixed, post-settling world anchor. Pitch-error, roll, joint-tracking, freshness
and emergency-stop checks remain required. Startup displacement is not included
in the post-settling drift metric. These limits do not certify full joint travel.

Jump acceptance separately requires actual bilateral wheel contact loss,
world-COM rise strictly >100 mm after takeoff, controlled touchdown and a 5 s
recovery dwell. The audit requests the same 10/2 mm recovery drift limits.
Jump `Complete` means the sequencer's stationary-posture conditions passed;
it is not an independent height or world-anchor acceptance certificate.

## Changes

- Pose targets cannot take over active/faulted jumping. Pending pose motion
  blocks jump readiness. Real stops and explicit Reset discard latent targets.
  The initial disabled startup handshake preserves the configured stand pose.
- A shared core safety predicate covers ordinary and zero-time jump callbacks.
  Repeated timestamps do not slew pose targets or count extra watchdog cycles.
- Brief airborne/contact gaps during owned jumping preserve the station anchor
  with zero correction. Recovery correction blends in rather than stepping on.
- Landing impedance ramps up from flight stiffness. Measured landing-pose error,
  pitch, pitch rate and joint speed gate recovery. Completion also requires
  low axle and joint velocities, not merely an elapsed timer.
- Optional stable-anchor acquisition prevents the outer position loop from
  capturing its first reference during startup transients. It defaults off in
  existing profiles; precision-pose evaluation uses a 0.5 s stable dwell.
- Whole-body FK reuses hip/knee rotations. The jump sequencer caches its fixed
  neutral pose instead of doing 30 FK evaluations every idle cycle. The varying
  pose solve retains its original iterations and accuracy.
- Shared subprocess-session ownership implements bounded INT/TERM/KILL cleanup
  with PID identity/pidfd checks. Summaries report live processes and zombies.
  No global process-name kills are used. Deliberate new-session escape is outside
  this Linux helper's contract; it requires cgroup containment.
- Harnesses share canonical ROS parameter merging, preserving nested/dotted
  precedence. Snapshots are explicitly YAML-derived, not live parameter dumps.
  Launch arguments no longer leak into the harness's ROS remappings.
- Repeated geometry samples are not stored twice; safety/freshness checks and
  the 20 Hz pose-command scheduler still run normally. Raw input logs remain.
- Bounded-model pose e-stop checks both command targets and fresh applied
  efforts on all six joints. Legacy ideal-wheel tests remain command-only.
- GUI blocks stale Show/manual trajectories after jump ownership or Reset,
  reports pending-pose/readiness reasons, and wraps long jump status text.

## Physical Evidence

The rows below retain failed tests; lower drift at the end does not excuse a
peak-drift or startup failure. Legacy ideal wheel actuation is not proof of the
selected wheel motor's torque-limited behavior.

| Log suffix | Model/profile | Peak XY | Final XY | Result |
| --- | --- | --- | --- | --- |
| `6174139779859` | Legacy station gain 0.8 | 25.456 mm | 0.794 mm | Peak limit failed |
| `6521532568465` | Legacy station gain 2.0 | 12.922 mm | 0.409 mm | Peak limit failed |
| `6698621929766` | Bounded actuators, gain 0.8 | 22.356 mm | 0.226 mm | Peak limit failed |
| `6875851935991` | Bounded, gain 4.0 | Not scored | Not scored | Startup attitude guard |
| `7038047888377` | Bounded, gain 2.5 | Not scored | Not scored | Startup attitude guard |
| `7343492890323` | Bounded, gain 2.5, acquisition dwell | Not scored | Not scored | Startup attitude guard |

These are `log/transformation_178920<suffix>/summary.json` paths. The startup
failures exposed a separate regression: pre-enable Disable could replace the
configured neutral target with initial CAD-zero measurements. Consequently the
failed rows cannot establish that their station gains alone caused instability.
The subsequent startup fix requires its own physical repetitions.

Final controller SHA-256:
`a66a978c74341a14c2086cfc70c2b4dea9d592e2016759c50572910a79bdedd6`.

| Log suffix | Bounded profile after startup fix | Peak XY | Final XY | Result |
| --- | --- | --- | --- | --- |
| `7586437590140` | Gain 2.5, 0.5 s acquisition | 10.408 mm | 0.217 mm | Peak limit failed |
| `7724756565045` | Gain 3.0, 0.5 s acquisition | 9.559 mm | 0.338 mm | PASS, two cycles / 64 s |
| `7923546595258` | Identical cold-start repetition | 9.500 mm | 0.263 mm | PASS, two cycles / 64.004 s |

Both passing runs used `precision_pose.yaml`, rated 120 rpm, 5 Nm leg and
0.13 Nm wheel caps, and local leg velocity-error damping 0.6 Nms/rad.
Maximum absolute pitch was below 0.01554 rad and measured joint tracking error
below 0.02567 rad. Both e-stops sustained zero commands AND fresh zero applied
efforts for 0.2 s. These are moderate poses on flat ground, not deep-squat,
rock-terrain, arbitrary asymmetric pose or jump certification. Startup moves
about 15 mm before the scored anchor; this is not a zero-motion startup claim.

Both passing trials ended with no live owned processes. The first cleanup
snapshot briefly reported its own launcher as a zombie; the second reported
none. A subsequent lifecycle fix explicitly reaps the launcher if it exits
between poll and the process snapshot, covered by a deterministic regression
test. It does not conceal orphan zombies belonging to another parent.

### Jump Recheck

`log/jump_1789208023884785872/summary.json` used the final controller with
`jump_tuning.yaml`, experimental manual motor curve, 0.6 damping and unchanged
5/0.13 Nm caps. Actual bilateral flight lasted 0.252 s. Conservative world-COM
rise was **0.063890 m**, below the required 0.10 m. Landing was observed at
10.980 s, followed by the hard pitch guard at -0.50684 rad (body Z 0.12920 m).
No recovery dwell completed, so low landing drift is NOT established.

E-stop passed fresh zero commands and applied efforts for 0.2 s; cleanup
reported no live processes or zombies. The lifecycle race fix was present in
this trial. Stricter landing gates did not solve the physical landing failure;
the feature remains experimental and unsuitable for a validated demo.

### Deep-Squat Stress Test

`log/transformation_1789208081602166714/summary.json` deliberately tested the
1.6 rad CAD-knee endpoint with the harness's 3 s transition. It FAILED during
the first descent: absolute pitch crossed -0.3 rad, peak displacement reached
0.307 m, and transient joint-target error reached 0.688 rad. No complete cycle
or endpoint hold passed. E-stop and cleanup passed. This transition exceeds
the default 0.4 rad/s command slew capability; it is not the GUI's minimum-11 s
deep-squat trajectory. In this harness joint-error acceptance is checked at
dwell endpoints; transient error is recorded, while attitude guards stop early.

With the same strict 10/2 mm displacement criteria but **11 s transitions**,
`log/transformation_1789208149534518490/summary.json` PASSED two deep-squat
cycles over 55.996 scored seconds: peak XY 2.490 mm, final XY 0.100 mm,
maximum absolute pitch 0.027515 rad, minimum body-origin Z 0.126793 m,
and peak joint-target error 0.049508 rad. Left/right knee minima were
-1.642675/-1.643122 rad, with hip maxima 0.974074/0.974139 rad. The commanded
knee endpoint is -1.6 rad; measured compliance exceeds that target but stays
inside the provisional -1.75 rad guard. No hardware encoder mapping is implied.
Applied-effort e-stop and cleanup passed with no live processes or zombies.

The identical cold-start repetition,
`log/transformation_1789208249182801237/summary.json`, also PASSED two cycles
over 56.005 scored seconds: peak XY **2.111 mm**, final XY **0.062 mm**,
maximum absolute pitch 0.029649 rad, minimum body-origin Z 0.126788 m,
and peak joint-target error 0.050694 rad. Applied-effort e-stop passed and
cleanup reported no live processes or zombies. Thus two cold starts support
the slow symmetric deep-squat profile, not fast deep motion or deep asymmetry.

## Software Verification

- Control CTest: 9/9 suites passed, including jump quality and independent URDF
  kinematics checks. Actuator-bound CTest: 1/1; teleop CTest: 3/3, including
  50 dashboard callback cases.
- Root Python suite: 92 tests passed, including real owned-process escalation,
  unrelated-process preservation and launcher-exit race coverage.
- Final-controller ROS contracts: 51 jump, 29 general and 39 station checks
  passed (119 total), including queued-pose exclusion, startup handshake,
  zero-time guards and delayed anchor acquisition.
- The final lifecycle helper hash is
  `ed152ebe0801d52f49600e7813eadeba3d9dbe67d2167a36807a2fae60a79180`.
  Earlier pose logs snapshot the pre-race-fix helper; their physics is unchanged.

## Performance Evidence

The reproducible microbenchmark is `build/tap_control/control_benchmark`.
One before/after sample on this PC measured mean microseconds per call:

| Operation | Before | After |
| --- | --- | --- |
| Whole-body FK | 0.117 | 0.059 |
| Varying neutral pose | 3.258 | 2.226 |
| Idle jump update | 2.692 | 0.036 |

These are local microbenchmarks, not a whole-simulator speedup or worst-case
real-time guarantee. Geometry is checked independently against URDF transforms.
The original 71,617-row transformation trace had only 28,638 consecutive unique
pose/joint timestamp pairs; deduplication removes redundant storage rather than
downsampling the command stream or bypassing guards.

## Reproduction

After sourcing TAP's installed ROS environment:

```bash
colcon build --symlink-install --packages-select tap_control tap_teleop
ctest --test-dir build/tap_control --output-on-failure
ctest --test-dir build/tap_sim --output-on-failure
ctest --test-dir build/tap_teleop --output-on-failure
python3 -m unittest discover -s tests -q
python3 tools/test_ros_contract.py
python3 tools/test_station_ros_contract.py
python3 tools/test_jump_ros_contract.py
```

Run physical trials sequentially in unused ROS domains; each uses its own Gazebo
partition and refuses existing traffic. The precision profile must be paired
with the bounded model and velocity-error gain:

```bash
python3 tools/transformation_smoke.py --domain 124 \
  --config src/tap_control/config/precision_pose.yaml --amplitude .25 --cycles 2 \
  --max-xy .01 --max-residual .002 --launch-arg jump_model:=true \
  --launch-arg jump_leg_speed:=12.566370614359172 \
  --launch-arg jump_leg_velocity_damping:=0.6
python3 tools/jump_smoke.py --domain 125 --jump-leg-velocity-damping .6 \
  --config src/tap_control/config/jump_tuning.yaml --jump-leg-speed 28.274333882308138 \
  --max-xy .01 --max-residual .002
python3 tools/transformation_smoke.py --domain 125 \
  --config src/tap_control/config/precision_pose.yaml --motions squat \
  --deep-squat-knee 1.6 --squat-amplitude 1 --cycles 2 --transition 11 \
  --max-xy .01 --max-residual .002 --min-height .115 \
  --max-absolute-pitch .3 --max-joint-error .12 \
  --launch-arg jump_model:=true \
  --launch-arg jump_leg_speed:=12.566370614359172 \
  --launch-arg jump_leg_velocity_damping:=0.6
```

## Remaining Limits

- Fixed-heading wheel balancing cannot independently correct lateral position.
  Repeated pose tests and per-axis residuals are necessary; do not promise zero
  drift for arbitrary asymmetric poses or terrain.
- Post-Reset deep recovery outside the GUI's validated manual/return envelope
  is blocked. Measured emergency travel thresholds are not authorization for
  an unvalidated recovery trajectory. Do not widen bounds or clip observations
  merely to make Neutral available.
- GUI verification here is callback/state testing and source sizing review,
  not a new live desktop rendering test. The user's existing desktop simulation
  is not stopped or replaced by these headless trials.
- Mass distribution, motor torque-speed curves and inertias remain approximate.
  RViz appearance and software test passes do not establish hardware capability.
