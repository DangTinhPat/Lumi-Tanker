# Driving off a 20 cm ledge: current findings (second correction)

## Known regression: `pitch_source: imu` default (2026-09-15)

**Every number below this point was measured with `pitch_source: sim_pose`
(ground truth), which was the default at the time.** As of 2026-09-15,
`pitch_source: imu` is the new default (`docs/RULES.md` Rule 1 - no
control decision may depend on simulator ground truth). This is a
deliberate, user-approved tradeoff, not an oversight:

- General balance (standing, squat, roll, driving on flat ground) with the
  IMU path (a real Kalman filter, `state=[angle,gyro_bias]`, verified
  against the canonical `TKJElectronics/KalmanFilter` reference) is close
  to sim_pose parity - `sim_smoke.py`'s "hold" stage passes cleanly with
  `kalman_r_measure: 0.1`; "squat" is marginal (~0.120-0.121 rad against
  the test's 0.12 bound, itself an arbitrary threshold calibrated against
  ground-truth precision, not a real hardware-derived requirement).
- **This 20 cm ledge drive-off test specifically regressed hard: 0/10
  (0%) recovered with the IMU default**, down from sim_pose's 70-88%
  documented below. Every trial shows a real, physical full flip (peak
  pitch 1.8-2.6 rad), not a small estimation error.
- **Update, 2026-09-16: largely resolved, root cause was not the IMU
  estimator.** See "Velocity headroom: the actual root cause" below - the
  real cause was a zero-authority-headroom bug in `MpcBalancer` present
  under *either* pitch source, that just happened to be masked under
  sim_pose (whose other margins were generous enough to absorb it) and
  fully exposed under imu (whose margins were already tighter). Fixing it
  (`mpc.max_ground_velocity` 0.6 to 2.0) restored recovery to **48-60%**
  under the current `pitch_source: imu` default, n=15-25 across three
  independent batches - without touching the estimator at all.
- **Decision (user, 2026-09-15): keep `pitch_source: imu` as the default
  anyway, accept this landing regression as a known, separate, tracked
  problem to solve later** - not a reason to fall back to ground truth.
  Rule 1 compliance is the higher priority; landing-specific IMU
  robustness is future work, scoped separately from general balance.
- Before resuming ledge-drop-recovery work, re-derive any new baseline
  under the current `pitch_source: imu` default, not the sim_pose numbers
  below - they are no longer the active configuration's behavior, only a
  historical/reference point for what ground truth was capable of.

## Correction history

1. An earlier version of this document concluded the 20 cm
   `rough_arena_spawn_pad` ledge drive-off was **never recoverable**. That
   was wrong — an artifact of `rough_training.sdf`'s heightfield starting
   only 0.10 m past that pad's edge (see "Confound #1" below), not a real
   physical limit.
2. After building an isolated clean world (`terrain:=ledge`,
   `src/tap_sim/worlds/ledge_test.sdf`) and an initial small batch (n=6-8)
   looked promising (~83-87% recovery, and one A/B run appeared to show the
   `LandingReflex` mechanism was harmless), a **larger, statistically
   meaningful batch (n=10 each) reversed that reading**: recovery rate on
   the stock controller settled at 70%, and the combined landing-recovery
   config (`landing_recovery_test.yaml`) actively made it worse (40%). Small
   early samples (n=6-8) were not reliable here; this is the corrected,
   larger-sample picture. Get more samples than that before trusting a delta
   in this specific test again — see "Why sample size matters" below.

## Current findings (closed-loop Gazebo, isolated clean world, n=10 each)

`src/tap_sim/worlds/ledge_test.sdf`: `ground_plane` (top z=0) plus one static
`ledge_pad` (top z=0.20 m, 0.9x0.9 m) plus a quarter-round fillet (radius
0.03 m) on the departure edge only, and nothing else. Robot spawns centered
on the pad; `tools/clean_ledge_smoke.py` drives it off at 1.0 m/s (500 Hz
IMU / 1 kHz control, this session's current rates) and measures recovery
over up to 10 s.

| Condition | Recovered | Notes |
| --- | --- | --- |
| Stock controller (no landing-specific overrides) | **7/10 (70%)** | Best known result. Successful landings show natural, unprompted knee compliance: crouches to about -0.26 rad within ~20 ms of losing contact, smoothly relaxes over ~0.5-0.6 s, pitch stays under 0.15 rad throughout - this comes from the existing torque-limited effort control alone, no added mechanism. |
| `landing_recovery_test.yaml` (kp/kd landing-window boost + `LandingReflex` pre-crouch, all from this session's work) | **4/10 (40%)** | Measured regression versus stock. Root cause of the regression not isolated (the config bundles several terms - `leg_kd`, `landing_kp_boost`, `landing_kd_boost`, `landing_ground_accel_boost_m_s2`, `landing_reflex.airborne_knee_crouch_rad` - together). |

**Current recommendation: use the stock controller with no landing-specific
overlay.** It is simpler and measurably better than everything tried on top
of it so far.

## The remaining, unresolved failure mode

Both conditions show the same distinct bimodal signature on their failing
trials: peak pitch RATE at touchdown is 7.7-9.5 rad/s, roughly double the
4.0-5.2 rad/s of recovered trials, and the outcome is always a complete,
exact 180-degree flip (`peak_pitch_rad == pi`), not a graceful partial tip.
This is consistent with the wheel occasionally catching the departure
edge's corner and picking up an extra rotational impulse from that fulcrum,
rather than a smooth, simultaneous both-wheels drop-off - a chaotic,
contact-timing-sensitive event rather than a torque/tuning-limited one.

A quarter-round fillet (radius 0.03 m, matching wheel radius) was added to
the departure edge specifically to test this hypothesis. An 8-trial batch
suggested a large improvement (7/8 vs. an earlier 5/8 on a sharp corner);
a larger 10-trial batch (part of the same run summarized above) came back
at 70%, statistically indistinguishable from no fillet at all once real
sample sizes are used. **The fillet's effect, if any, has not been
established** - it is left in place (it is physically reasonable and
strictly removes a real sharp-edge artifact) but should not be cited as a
proven fix.

## Real-world research consulted, and what it did and did not fix here

Ground truth for this class of robot: **[Ascento: A Two-Wheeled Jumping
Robot](https://arxiv.org/abs/2005.11435)** (ETH Zurich, ICRA 2019) detects
ground contact from a **hip-joint torque threshold**, not pose/height, then
resumes stabilizing control with a virtual spring-damper for the landing.
**[Upkie](https://github.com/upkie/upkie)** (the robot `MpcBalancer` in this
codebase is itself ported from) has its own published contact-estimation
work testing 3 cm and 12 cm drops. **[CTBC](https://arxiv.org/pdf/2509.02986)**
(real hardware, LimX Tron1) and a **[blind-stair-climbing RL
paper](https://arxiv.org/abs/2402.06143)** (tested on Ascento, ANYmal on
Wheels, Cassie, Unitree Go1; 90-100% success on 18 cm steps) both use learned,
not hand-designed, policies.

Implemented and tested Ascento's specific technique: `torque_touchdown_enabled`
(`BalanceControllerNode.cpp`, off by default, a pure widen-only OR onto the
existing pose-based `state_.floor_contact` - it can only ever make contact
register EARLIER than the pose heuristic alone, never later, and only while
already reading airborne, so it cannot change grounded/normal-operation
behavior). Requires both hip torque >= 2.5 Nm AND hip velocity <= 1.0 rad/s
(a genuine stall/obstruction signature, not just "torque is high", which also
happens during any fast deliberate pose change given `kp=40`/`cap=3 Nm`
saturates past 0.075 rad of error). Tested in isolation (no other landing
mechanism enabled) via `src/tap_control/config/landing_torque_touchdown_test.yaml`,
n=10: **5/10 (50%) recovered - worse than stock's 70%,** not the improvement
the technique's own paper would suggest. Plausible reason: this only
accelerates when the MPC starts actively commanding wheel torque during
contact, and in the chaotic high-pitch-rate snag failures specifically,
acting earlier during an already-violent, still-settling impact may feed
control effort in at the wrong moment rather than helping - untested,
not confirmed, but consistent with the direction of the result.

## IMU accelerometer touchdown detection - also tried, also did not help

Per the user's specific request to get more out of the IMU: the raw
(unfiltered) accelerometer specific-force magnitude was wired in as a third,
independent touchdown-confirmation signal (`imu_touchdown_enabled`,
`BalanceControllerNode.cpp`/`onImu` - `last_accel_magnitude_m_s2_`, same
widen-only-while-airborne safety pattern as the torque one, off by default).
This is the standard free-fall/impact detection principle (accelerometer
magnitude near 0 in free fall, a sharp spike on impact) and is materially
different from and lower-latency than both the pose heuristic and the hip
torque threshold. A real trace confirmed the raw accelerometer does show
large, real, physically-grounded transients on contact events (spikes into
the hundreds of m/s^2 for one or two 500 Hz samples, decaying over roughly
100-200 ms as the contact settles) - a legitimate signal, not sensor noise.

Tested in isolation (`landing_torque_touchdown_test.yaml`'s sibling
`landing_imu_touchdown_test.yaml`, `imu_touchdown_accel_mps2: 20.0`), n=10:
**5/10 (50%) - the same rate as the hip-torque approach, still worse than
stock's 70%.**

Three separate touchdown-detection improvements (pose+torque,
pose+hip-torque-with-velocity-gate, pose+IMU-accelerometer) all land at
approximately the same ~50% rate, consistently below the 70% stock baseline.
This is no longer attributable to "wrong sensor" - it looks like the
intervention *type* itself (making the balance controller react earlier
during the chaotic high-pitch-rate snag event) is the problem, regardless of
which sensor triggers it: reacting with an MPC control law tuned for
near-equilibrium correction, DURING a still-violent, highly transient,
nonlinear event, plausibly injects torque at the wrong moment and can
amplify rather than damp the chaotic rotation. The stock controller's
behavior - not reacting with fresh control authority until the pose signal
settles enough to read as grounded - may be *inadvertently protective* for
exactly this reason, rather than merely being "slow." **Do not enable any of
`torque_touchdown_enabled` / `imu_touchdown_enabled` as a default without a
new idea for what to DO once contact is confirmed sooner - detecting it
earlier alone has now failed three times.**

## Roll/yaw axes checked directly - not the missing lever here

Per a direct user question ("have we used all 6 IMU axes"): `angular_velocity.x`
(roll rate) was never read anywhere in `BalanceControllerNode.cpp` (not even
validated for finiteness) before this session; `angular_velocity.z` (yaw
rate) was stored into `state_.imu.yaw_rate_rad_s` but never consumed by any
control logic; `linear_acceleration.y` was only used inside the inactive
(`pitch_source_ != "sim_pose"`) accelerometer-pitch fallback. Real gyro
telemetry was captured directly (not derived from Euler-angle pose math,
which is genuinely degenerate near pitch=pi and produced the earlier
misleading `peak_roll_rad == pi` readings in `clean_ledge_smoke.py`) across
4 fresh trials: a Fallen trial showed peak roll rate of only 0.136 rad/s
against a peak pitch rate of 9.415 rad/s, while a *recovered* trial showed
1.781 rad/s of roll rate - i.e. roll rate does not track pass/fail here.
**Roll is not the missing lever for this specific failure mode**; the
earlier `peak_roll_rad` numbers were a measurement artifact, not a real
physical roll event. Pitch rate remains the genuine discriminator.

## Fused accel+pitch-rate hard-impact classifier, with delay instead of acceleration

Per the user's push to make better use of the accelerometer, and following
real fall-detection literature ("Evaluation Of IMU-Based Pre-Impact Fall
Detection Algorithms", PMC6412321 - combines acceleration + angular velocity
+ tilt for 100% detection accuracy with ~400 ms lead time; "Contact
detection with multi-information fusion", quadruped work reaching 99.3%
accuracy at 4-5 ms delay by fusing multiple signals, not one threshold): a
**fused** classifier was added (`hard_impact_delay_enabled`,
`BalanceControllerNode.cpp`) requiring BOTH the raw accel spike AND
`|state_.imu.pitch_rate_rad_s| >= 6.0 rad/s` simultaneously - a materially
different design from the three single-signal detectors above.

Critically, its *response* is the opposite of all three earlier attempts:
instead of widening `floor_contact` to react SOONER, a detected hard impact
forcibly holds `floor_contact` false (delays MPC re-engagement) for
`hard_impact_settle_s` (0.15 s default) after the last tick the fused
condition was true, letting the chaotic rotation damp passively before
active control resumes - directly testing the hypothesis raised by the
three prior results (reacting earlier during the chaotic snag consistently
measured worse).

Result, n=10 (`landing_hard_impact_delay_test.yaml`): **7/10 (70%) -
statistically identical to the stock baseline, and the first landing
mechanism tried this session that did not measurably regress it.** This
neither proves nor disproves the "delay, don't accelerate" hypothesis on its
own (a tie at n=10 is also consistent with the fused condition simply not
firing during these particular trials' failures - not yet checked from the
telemetry) but it is a qualitatively different outcome from every
reaction-acceleration attempt, all of which measurably hurt. Worth
following up with: (a) confirming from `accel_mag`/telemetry (now exposed in
`publishMode`'s status string, see below) whether the fused condition
actually fires during the 3 remaining failures in this batch; (b) a larger
n; (c) tuning `hard_impact_settle_s` longer if the condition does fire but
0.15 s isn't enough dead time for the rotation to damp.

**Also added per the user's request to make acceleration itself observable**:
raw accelerometer magnitude is now published every tick as `accel_mag=` in
`/control_mode`'s status string (`BalanceControllerNode::publishMode`), so it
can be watched/logged directly rather than only inferred indirectly.

## First genuine improvement: widen the reaction window itself, always-on

Every mechanism above operated inside a FIXED, narrow decision window: given
the chaotic snag's ~8.7-9.5 rad/s peak pitch rate and the active
`mpc.fall_pitch_rad` threshold of 0.8 rad, the controller has only
roughly 75-90 ms from touchdown before `Fallen` is declared and all torque is
cut - every earlier attempt tried to act faster or slower *inside* that fixed
window. This one instead widens the window and the wheels' authority to use
it, unconditionally (not gated by any touchdown-detection signal, sidestepping
the "which sensor/when to trigger" question entirely):

```yaml
mpc.fall_pitch_rad: 1.1       # was 0.8 (active default in balance_controller.yaml)
mpc.max_ground_accel: 10.0    # was 6.0 (active default)
```

(`src/tap_control/config/landing_authority_test.yaml`.) Both parameters were
already exposed as ROS parameters with these "0.8"/"6.0" values as the real
active production defaults - not the C++ struct's own compile-time defaults,
which are different and unused at runtime here. Neither had been tested as a
blanket, always-on change before; only as a landing-window-*gated* boost
layered on top of the same base 6.0 (`landing_ground_accel_boost_m_s2`,
which measurably did not help, see above) - conflating "not enough authority"
with "authority arrives at the wrong time." This change removes that
confound by making the extra authority permanent instead of event-triggered.

**Result, two independent batches, n=25 total: 20/25 (80%) recovered -
a real, reproducized improvement over the stock baseline's 70%.** (Batch 1:
8/10; batch 2: 12/15.) `sim_smoke.py`'s 30 s balance/drive/pose/e-stop
regression still passes with this config
(`python3 tools/sim_smoke.py --controller-config
src/tap_control/config/landing_authority_test.yaml ...`; `sim_smoke.py` was
extended with a `--controller-config` flag this session specifically to
allow this kind of test). This is the first mechanism tried across this
entire investigation that measurably beat, not just tied or hurt, the stock
baseline.

**Promoted to production default 2026-09-15.** The broader validation this
section called for was run:

- `sim_smoke.py --duration 30 --exercise --poses --max-pitch-error .12
  --controller-config landing_authority_test.yaml`: pass (`failure: null`).
- `recovery_smoke.py`: fails with "No sustained 2s upright recovery" — but
  this is the pre-existing 90° lying-pose stand-up failure documented
  elsewhere in this repo (CLAUDE.md), reproduced identically without any
  config override too (the script has no config-override flag, and
  `stand_up.fall_pitch_rad: 2.0` is a separate namespace from
  `mpc.fall_pitch_rad` regardless). Not a regression from this change.
- `transformation_smoke.py` (Stationary Show): the default `--freshness 0.15`
  produced a "Stale pose" failure identically with and without the override
  (an environment real-time-factor flake, not config-dependent); re-run with
  `--freshness 0.3` on both sides gave peak_yaw_error_rad 0.0835 (with
  override) vs. 0.0839 (baseline) — statistically indistinguishable, so the
  already-known roll-induced yaw (~0.14 rad, exceeding this script's strict
  0.06 rad default bound regardless of this change) is unaffected.
- Deep-squat QA (`docs/DEEP_SQUAT.md`) is a GUI-driven manual procedure, not
  automated; not re-run for this promotion. `mpc.fall_pitch_rad` and
  `mpc.max_ground_accel` are Balance-mode wheel/fall parameters, not squat
  pose-envelope parameters, so the risk of interaction is low, but this is a
  real, explicitly-acknowledged gap in this promotion's evidence, not a
  confirmed pass.

`balance_controller.yaml`'s own `mpc.fall_pitch_rad` and `mpc.max_ground_accel`
are now `1.1` and `10.0` (see comments there). `landing_authority_test.yaml`
is kept only for provenance/reproducing the original isolated A/B test; it is
now a redundant no-op against current defaults, not a required override.

## LandingImpedance (knee-only) re-measured fresh on the promoted baseline (2026-09-15)

The current `LandingImpedance` revision (knee-only gain-scheduled critical
damping, `src/tap_control/config/landing_impedance_test.yaml`) carried a code
comment claiming "75% combined" and "measured better than stock," but by its
own admission had "NOT yet been measured fresh in closed-loop Gazebo" against
the current config - it predates the `fall_pitch_rad`/`max_ground_accel`
promotion above. Re-measured with `tools/clean_ledge_smoke.py` on the now-
promoted baseline (two batches after a tool bug - see below - forced a
restart, so `enabled` is 41 trials total, not a clean 25):

| Condition | Recovered | n |
| --- | --- | --- |
| Baseline (promoted `fall_pitch_rad=1.1`/`max_ground_accel=10.0`, no other override) | **22/25 (88%)** | 25 |
| + `landing_impedance_test.yaml` layered on top | 14/16, then a fresh 21/25 = **35/41 (85%)** | 41 |

**Statistically tied with the new baseline, not a reproduced improvement.**
This does not promote `landing_impedance.*` into `balance_controller.yaml`'s
defaults - consistent with this document's standing rule that only a clearly
reproduced win (not a tie, per "Why sample size matters" below) gets
promoted. The `LandingImpedance.hpp` header comment's "measured better than
stock" claim should be read as referring to the old, narrower 70%-baseline
condition it was actually tested against, not the current default.

**Tooling bug found and worked around, not yet fixed in the tool itself**:
`tools/clean_ledge_smoke.py` assigns `ROS_DOMAIN_ID` starting at 191 and
incrementing by 1 per trial with no wraparound (`domain = 191`, `domain += 1`
per trial, around line 224). A single invocation running both `baseline` and
`enabled` conditions at `--trials 25` needs 50 domain IDs (191-240), which
exceeds Fast-RTPS's effective domain ID ceiling (~232) and crashes with
"Calculated port number is too high" partway through the second condition.
Worked around here by running each condition as a separate invocation
(`--skip-baseline` / `--skip-enabled`), which resets the domain counter to
191 each time and stays under the ceiling for batches up to about 40 trials.
For any future batch larger than that, or when a single invocation needs to
cover both conditions with wraparound, the tool itself should mod the domain
counter into a safe range instead of incrementing unbounded.

## Honest overall status

Every *reaction-accelerating* mechanism tried this session - passive damping
increase, active qdot-feedback impedance, edge-triggered MPC gain-window
boost, `LandingReflex` pre-crouch, a departure-edge fillet, Ascento-style
hip-torque-threshold contact detection, and IMU-accelerometer-threshold
contact detection - either showed no effect or measurably **hurt** compared
to the stock controller's 70% baseline. The one *reaction-delaying* mechanism
tried (the fused accel+pitch-rate hard-impact classifier above) came back
statistically tied with stock (70%) - not proven better, but the first one
that did not measurably hurt. This is a real, useful negative result, not a string of failed
attempts to hide: it suggests the remaining ~30% chaotic-flip failure mode is
not close to any of the hand-designed levers tried so far, and the published
literature's own best results on materially the same problem (90-100% on an
18 cm step) come from **learned, not hand-tuned, control** (asymmetric
actor-critic PPO trained in Isaac Gym on thousands of simulated trials) - a
different, much larger scope of work than anything attempted here. If higher
reliability than 70% is required, that RL-based approach (or a great deal
more hand-tuning with a much larger, statistically rigorous trial budget per
change than this session used) is the realistic next step, not another
single hand-picked heuristic.

Not yet tried: isolating which single term in `landing_recovery_test.yaml`
causes its regression; a much larger fillet or a full bevel on all four
edges; whether the ~75 ms window between the fast touchdown transient and
`fall_pitch_rad` (0.65 rad) being crossed is simply too short for any
bounded torque budget to arrest (in which case this is a genuine, if
narrower and rarer than first wrongly concluded, physical limit); an actual
RL training pipeline.

## Why sample size matters here

This is a chaotic, contact-timing-sensitive system: an 8-trial batch has a
wide binomial confidence interval around any true rate near 60-80% (an
observed 7/8 or 5/8 are both quite compatible with the same underlying
~70% rate). Do not conclude an improvement or regression from an n<10 batch
in this specific test without either a much larger n or a clearly
non-overlapping result.

## Confound #1 (superseded root cause, kept for provenance)

The original test used `rough_arena_spawn_pad` in `rough_training.sdf`,
whose heightfield starts only 0.10 m past that pad's edge, at a surface
height close enough to the robot's own ride height that the robot's base
stayed at approximately normal standing height almost immediately after
"leaving" the pad - i.e. it never free-fell the intended 0.20 m onto open
ground. A high-resolution `/joint_states` trace showed the base height
staying flat at ~0.23 m for ~0.35 m of onward travel post-"edge", with pitch
climbing gradually before an eventual, separate fall further into the
heightfield's own terrain - a *"loses balance driving fast over rough
terrain"* failure, not a landing-impact failure. This is why the earlier
constant-damping, active-impedance, and edge-triggered gain-boost attempts
showed no effect on that test: it wasn't testing what they were built to
address.

## Evidence-level status

- **Source inspected / build / unit-contract tests**: `LandingReflex` is
  unit-tested (13 assertions) and proven a true no-op in `EmbeddedController`
  at default (zero) gains - the regression above is not from a broken
  no-op guarantee, it is from measured behavior once the gains are turned on.
- **Closed-loop Gazebo, isolated ledge, n=10 each**: the load-bearing result
  in this document - stock 70%, landing-recovery overlay 40%, real (not
  cosmetic) roughly-2x pitch-rate bimodality in the remaining failures on
  both.
- **Closed-loop Gazebo, rough-terrain drive-fast disturbance rejection**:
  still a separate, unsolved, and barely-investigated problem if
  `terrain:=rough` at speed remains a real goal - do not conflate it with
  the ledge-drop question above.
- Nothing hardware-related was touched or claimed.

## Real ground-truth contact telemetry, opt-in and observational only (2026-09-15)

Prior sections above rely entirely on the pose+IMU `floor_contact` heuristic
(`BalanceControllerNode.cpp`) for anything that reads "grounded" in this
document, because Gazebo's real per-wheel contact sensing was only ever wired
up for the separate `jump_model` path (`run_jump.sh`), not the plain
`run_sim.sh`/ledge-test path everything above was measured on.

Wired real contact telemetry into the ledge-test path too, additively:

- New launch arg `measure_contacts:=true` (independent of `jump_model`, so it
  does not also swap in `BoundedGazeboSystem`/lower actuator limits like
  `jump_model` does) enables the wheel contact sensors on
  `src/tap_sim/worlds/ledge_test.sdf` (now loads `gz-sim-contact-system`)
  without changing anything else.
- Bridges the sensor's own `_raw` gz topic directly (`sim.launch.py`) rather
  than through `tap_sim::ContactSnapshot`
  (`src/tap_sim/src/contact_snapshot.cpp`), which was observed to
  intermittently fail to load in this environment ("undefined symbol:
  removexattrat" - a `libsystemd` version mismatch in the conda/micromamba
  env, unrelated to that plugin's own code). The existing
  `BalanceControllerNode` subscription (`jump.left_contacts_topic` /
  `jump.right_contacts_topic`, already always-on regardless of `jump.enabled`)
  needed no code change - it already consumed exactly this topic name and
  message type.
- Verified end-to-end with a real drive-off-the-ledge trial
  (`ROS_DOMAIN_ID`/`GZ_PARTITION`-isolated, `owned_process` pattern): 9876
  `/jump_status` samples collected, 8390 showed real
  `left_contact=1`/`right_contact=1` from actual Gazebo collision data (not
  simulated/mocked), 5303 within the freshness window - confirming the whole
  pipeline (world plugin -> sensor -> bridge -> subscription -> status
  string) delivers genuine ground-truth per-wheel contact.

**This is deliberately observational only - it does not feed
`state_.floor_contact` or any control decision.** Three earlier attempts in
this document to make `floor_contact` react to a better/faster sensor
(hip-torque threshold, IMU-accelerometer threshold, and the pose heuristic's
own tuning) all measurably hurt or tied at best; wiring real contact into the
*decision* path would very likely repeat that same failed experiment with a
different sensor. This capability instead exists so future investigation can
compare the pose heuristic's accuracy against real ground truth, or feed a
kinematics-estimated `Fz`/friction-cone term (a fundamentally different kind
of use than "trigger a reaction sooner") - not to change any current
production behavior. Confirmed zero effect on the default path: `sim_smoke.py`
and a fresh `clean_ledge_smoke.py --skip-enabled` batch both behave
identically with `measure_contacts` left at its default `false`.

## Track B items 2-4: friction cone, critical damping, Fz state (2026-09-16)

Follow-on to the overhaul plan's Track B (giving the MPC genuine force/
contact awareness it structurally lacked - see claim #1/#4 in the original
4-point critique). Track A (authority promotion, LandingImpedance
reconciliation) and Track B item 1 (contact telemetry infrastructure) were
already done; this closes out items 2-4.

**Item 3 (critical leg damping from real mass, done):** `leg_kd` was
`0.0` since the initial commit - unexamined, not a measured or tuned value.
`tools/generate_control_model.py` gained `leg_inertia()`: parallel-axis-
theorem rotational inertia about the hip axis (thigh+shin+wheel) and knee
axis (shin+wheel) from the real mesh-derived mass/COM/inertia data already
in `GeneratedGeometry.hpp`, at the CAD-zero neutral pose. `BalanceControllerNode`
now defaults `leg_kd` to critical damping for the deployed `leg_kp` against
the larger (hip) inertia: `Kd = 2*sqrt(Kp * max(I_hip, I_knee))` ≈ 1.31 at
`leg_kp=40`, replacing the unmotivated `0.0`. **A real bug was caught and
fixed before promotion**: the first version of `leg_inertia()` anchored the
wheel's parallel-axis distance at the knee joint directly, silently dropping
the knee-to-wheel joint offset (~0.11 m) from the FK chain - understated
both inertias (worst on I_knee, off by ~3x). Caught by
`tests/test_leg_inertia.py`'s independent from-scratch cross-check at
hip=knee=0, not by inspection. Verified after the fix: 15/15 C++ unit tests,
`sim_smoke.py --exercise` (hold/drive/yaw all 0.012-0.032 rad, well under
the 0.12 bound), unchanged from the pre-change baseline.

**Item 2 (friction-cone ceiling, done, deliberately static):**
`MpcBalancer::Config::wheel_ground_mu` (default 1.0, matching every world's
`ground_plane` surface `<mu>`) caps `solveHorizon`'s acceleration authority
at `mu * gravity_m_s2` in addition to the existing tunable
`max_ground_accel`: a wheel rolling without slip cannot deliver more
tangential acceleration than `mu*g` regardless of commanded torque or mass
(both `F=ma` and `F_friction=mu*m*g` scale with mass, so it cancels) - a real
physical bound nothing previously enforced. At the current defaults
(`max_ground_accel=10.0`, `mu*g=9.81`) this trims authority by under 2%,
so it does not undo Track A's promoted authority increase. Deliberately
**not** gated on live/measured contact force, even though real per-wheel
contact telemetry now exists (the section above): every prior attempt in
this document to react to touchdown *faster* via a sensor threshold (hip
torque, IMU accelerometer, the pose heuristic itself) measurably hurt or
tied at best. A live-Fz-scaled version of this bound would need the same
n>=25 evidence-gated comparison those attempts got before shipping, not just
first-principles reasoning that it should help - not implemented here.
Verified: 15/15 C++ unit tests, `sim_smoke.py --exercise` unchanged, and a
fresh `clean_ledge_smoke.py --skip-enabled` n=5 batch still runs cleanly
(0/5, unchanged from the existing 0/10 IMU-era baseline - no crash, no new
failure mode, nothing left to regress since that baseline is already zero).

**Item 4 (extend MPC state with Fz, investigated, not implemented as
originally scoped):** the plan's smallest version was "add Fz as a measured
parameter that scales the feasible max_ground_accel in real time." Reading
`MpcBalancer::update()` closely first: `solveHorizon` (the whole
ground-acceleration QP, including item 2's new cap) is **only ever called
while `state.floor_contact` is true**; the moment contact is lost, `update()`
takes a completely different branch (`commandeded_ground_velocity_m_s_`
low-passes to 0, the position anchor just follows the wheel) and never asks
the QP for an acceleration at all. The real, structural
"no-ground-contact-means-no-authority" property item 4 was meant to add
already exists, as a simpler and already-proven binary gate rather than a
continuous Fz-scaled one - and per item 2's own reasoning, making that gate
*more* reactive/continuous is exactly the class of change this document has
repeatedly found to hurt, not help. Closing this item as "investigated,
existing architecture already provides the intended property" rather than
building a new mechanism to duplicate it.

## Track B YAW/ROLL (3-axis article) reconfirms, does not overturn, the earlier roll/yaw finding (2026-09-16)

A user-supplied article proposed a Z/ROLL/YAW/PITCH task-allocation matrix
for landing (symmetric leg compression for Z, differential leg length for
ROLL, differential wheel torque via gyro-Z for YAW, wheel rolling/capture
point for PITCH). Implemented in full: `MpcBalancer::Config::yaw_rate_feedback_kp`
closes the wheel-differential loop on real `state.imu.yaw_rate_rad_s`
(previously open-loop, commanded-yaw-only); `LandingImpedance` was rewritten
from one shared phase state machine to two independent per-side ones driven
by new `left_floor_contact`/`right_floor_contact` (real per-wheel contact
sensors), so an asymmetric touchdown can soften only the side that actually
landed. Both are structurally sound and unit-tested (17/17 assertions,
including two new tests proving the per-side split works as designed).

Measured with both enabled, n=10 clean ledge trials: **0/10 recovered,
peak_roll_rad still exactly pi in all 10** - identical to the pre-change
baseline. Root-caused first to `EmbeddedController`'s Fallen-mode
short-circuit (`writeSafeStop()` + immediate return the instant
`|pitch| > fall_pitch_rad`, before `runBalance()`/`LandingImpedance::update()`
can act again). To isolate that variable, `mpc.fall_pitch_rad` was pushed to
3.0 rad (way past the 1.1 default) with YAW/ROLL still enabled: still
**0/10 recovered**, and - critically - 3 of the 10 trials never even
triggered Fallen (`final_mode=Balance` throughout) yet **still** showed
`peak_roll_rad == pi`. A cutoff that never fired cannot be the explanation
for those 3 rows, which forced a second look at what `peak_roll_rad` is
actually measuring.

Traced the real rotation axis directly from the raw pose quaternion
(`diag_rotation_axis.py`, axis-angle decomposition, not Euler angles) through
a live fall: `axis=(+0.007, +1.000, -0.003)` essentially unchanged for the
entire tumble, angle growing past 90 deg (123-147+ deg observed in one
window). **The fall is a single-axis rotation about Y (pitch), full stop.**
`peak_roll_rad == pi` is `clean_ledge_smoke.py`'s
`asin(2*(w*y-z*x))`-based pitch extraction hitting its +/-90 deg domain
limit and being forced into the gimbal-lock representation of the
remaining rotation as `roll=pi, yaw=pi` - a decomposition artifact of a
range-limited Euler formula, not a physical lateral or twisting motion.
This independently reconfirms, via a different method (raw quaternion axis
vs. the existing section's gyro-rate comparison above), the **same**
conclusion this document already reached at "Roll/yaw axes checked directly"
before this session's Track B/article work started: roll is not, and never
was, the missing lever for this specific failure mode. The YAW-rate and
per-side ROLL mechanisms built from the article therefore behaved exactly as
the project's own prior evidence predicted they would - not a wasted
implementation, but confirmation that this ledge-driveoff failure has no
real roll/yaw disturbance component to correct. Both mechanisms are kept
(structurally sound, default-off/no-op: `yaw_rate_feedback_kp=0.0`,
per-side impedance collapses to the old shared behavior when both sides see
identical contact/velocity, which is the common case) as validated
infrastructure for a failure mode where an actual lateral disturbance exists
(uneven terrain, one-sided obstacle) - not promoted as a fix for this one.

**What remains as the real, narrowed cause of 0% recovery here:** (1) the
Fallen-mode instantaneous, ungraduated torque cutoff at `fall_pitch_rad`,
which the n=10 fall_pitch_rad=3.0 experiment shows does not even fully
explain it alone (0/10 either way); combined with (2) the fall itself being
a genuine large-angle (>90 deg, confirmed past 147 deg) single-axis pitch
tumble - well outside the small-angle linear-inverted-pendulum regime
`MpcBalancer` is built around - meaning even a controller given unlimited
time post-threshold has, per this same n=10 experiment, not been shown able
to arrest it with the current pitch-axis authority alone. Neither a 3-axis
decomposition nor a smarter cutoff threshold addresses a controller that
is not modeled for the angle range the failure actually occurs in.

## Velocity headroom: the actual root cause (2026-09-16)

Prompted by the user asking why a robot that balances fine still falls
driving off a step, and told to concentrate effort on it. The previous
section's "unlimited time still can't arrest it" conclusion turned out to be
an artifact of testing the wrong variable, discovered via a chain of
closed-loop experiments unrelated to pitch-axis modeling or the estimator:

**1. A general, ledge-independent instability was found first.** Driving
*straight*, no ledge, no terrain at all, at a commanded 1.0 m/s, reliably
tips the robot over after roughly 2 s - reproduced twice independently.
Tested with `pitch_source: sim_pose` (ground truth, removing the IMU
entirely) to rule out the estimator: the fall still happened, identically.
Debug trace at the fall onset showed `mpc_cmd` (commanded ground
acceleration) pegged at `9.81 m/s^2` (the friction-cone ceiling) and
`wheel_vel` pegged at exactly `0.6 m/s` (`max_ground_velocity`) for the
*entire* window - the QP was continuously asking for maximum correction and
getting none of it, because the integrated correction signal
(`commanded_ground_velocity_m_s_`) is hard-clamped to `max_ground_velocity_m_s`
- the *same* ceiling the commanded cruise speed itself is clamped to. When
the target cruise speed sits at or near that ceiling (0.6-1.0 m/s in every
test in this document, including the ledge one), there is **zero authority
headroom left for the balance loop to exceed cruise speed and actually catch
a lean** - the moment any real disturbance needs the wheel to go faster than
cruise to get back under the falling CoM, the correction is silently
discarded by the clamp, and the robot free-falls as an unforced inverted
pendulum from that point on (matching the observed smooth, unresisted growth
first to a few tenths of a radian, then runaway).

**Confirmed the mechanism, not just the symptom**, by keeping the cruise
target fixed at 0.6 m/s and only raising the ceiling (`max_ground_velocity`
to 1.5, sim_pose): the robot drove stably the entire 8 s test. This rules out
"it's just driving faster now" as an alternative explanation - the fix is
headroom, not speed.

**2. Four narrower, event-gated versions of the fix were tried first and
none worked as well as an always-on change** (this codebase has hit this
exact pattern before - see the fall_pitch_rad/max_ground_accel promotion
above - and it repeated here):
  - Gate on `|pitch_rate| > 1.5 rad/s`: 0/10 on the ledge test. By the time
    pitch_rate spikes, floor contact is often already lost (airborne - no
    ground correction is possible regardless of ceiling), so the headroom
    unlocks after the window that needed it.
  - Gate on nonzero commanded forward velocity: 0/10. The drive command is
    already back to 0 (explicit stop published on crossing the edge) before
    the actual touchdown impact.
  - Gate on the existing `landing_active` post-touchdown window
    (edge-triggered on real `floor_contact` false->true): 0/10 alone,
    combined with the forward gate: still 0/10.
  - Gate on real measured wheel velocity (catches the coasting-on-momentum
    gap between "drive command already zero" and "actually airborne"): 1/10
    - better than the others but still far short of the always-on result.
  - **Always-on `max_ground_velocity: 2.0`, no gating at all: 6/10 (n=10),
    12/25 (48%, n=25), 8/15 (53%, n=15) across three independent batches**,
    under the real `pitch_source: imu` production default.

**3. The always-on change had one measurable side effect**, closing the
loop on the honest-reporting obligation this document has kept throughout:
a wider ceiling also let `Hold`'s own already-tight position drift regress
just past `sim_smoke.py`'s 0.10 m bound (0.10-0.11 m, noisy around the
bound at first). Raising `mpc.position_cost_scale` from 1.5 to 3.0
(a plain, landing-unrelated retune of the existing position-tracking
penalty) restored real margin (0.077-0.083 m, two repeats) without touching
anything landing-specific, and was re-verified not to undo the ledge
recovery gain. `squat`'s own pre-existing marginal failure (~0.121 rad
against the 0.12 bound) was confirmed present in the unmodified baseline
too (0.1208 rad) - unrelated to this change, not fixed here.

**Promoted to `balance_controller.yaml` defaults**: `mpc.max_ground_velocity`
0.6 to 2.0, `mpc.position_cost_scale` 1.5 to 3.0. Verified: 20/20 C++ unit
tests, `sim_smoke.py --exercise` (hold/forward/backward/yaw all pass with
real margin; squat's pre-existing marginal failure unchanged), and the
20 cm ledge drive-off recovering 48-60% (n=15-25, two independent batches)
under `pitch_source: imu` - up from the 0% this document spent the previous
sections trying to explain. The remaining ~40-50% of failures still show
the full-flip signature (`peak_pitch` 3.2-4.1 rad, gimbal-locked
`peak_roll_rad == pi`, see the Track B section above) - not yet understood,
a candidate for the next investigation.
