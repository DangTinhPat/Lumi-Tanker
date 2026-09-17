# Project rules (mandatory)

Standing rules for this codebase, binding on every session and every piece
of work — not suggestions to weigh against convenience. Each rule has its own
section below; add new mandatory rules as their own numbered section rather
than folding them into this preamble. If a rule and a one-off task
instruction conflict, ask the user rather than silently picking one.

---

## Rule 1: No ground truth in the control path

**No control decision may depend on simulator-internal state that has no
real-hardware equivalent.** This applies to every new feature, every bug fix,
every tuning change, from this point forward — not just landing/contact work.

"Ground truth" here means any signal Gazebo can hand the controller for free
because it has perfect internal knowledge of the world, but that a real robot
could never read directly off a physical sensor — the clearest example in
this codebase today is `sim_pose` (the controller's default `pitch_source`:
exact body orientation read straight from the physics engine's model pose,
not estimated from any sensor).

### Absolute framing (user, 2026-09-16)

The robot's entire operating process — everything that decides a torque,
velocity, or pose command — must run **completely on the planned hardware
sensor data (MPU6500 accelerometer/gyro and six motor encoders), with zero
involvement of ground truth**. Support/contact is an estimate derived from
those signals, not a direct physical contact measurement in the current
inventory. Gazebo is nothing more than the physics environment the sensors are
simulated inside of; it is not itself a sensor and its internal model state
must never reach a control decision, directly or indirectly.

`sim_pose` (or any other ground-truth read) is permitted in exactly one
role: **an offline diagnostic signal, read only by a debugging/analysis
script, to compare against what the real-sensor path estimated, so a real
bug can be found and fixed.** The moment a ground-truth value is read by
`BalanceControllerNode` (or any other node that issues a real command) and
used for anything the robot *does* — not just observes for a human to
read afterward — that is a violation of this rule, full stop, regardless of
whether the feature is "opt-in," "off by default," or "scoped as future
work." There is no accepted resting state short of zero.

### Why this rule exists

The user's stated goal (2026-09-15) is to keep this codebase as close as
possible to what a real robot deployment will look like, specifically so
that porting to real hardware later requires minimal rework. Every control
decision built today on a ground-truth shortcut is a decision that will have
to be re-derived, re-tuned, and re-validated later against noisy, biased,
latent real sensors instead — silently accumulating debt that only surfaces
at the worst possible time (on real hardware, not in sim).

### What's allowed vs. not allowed

**Allowed** — anything with a real physical sensor behind it, even if the
current implementation happens to run in simulation:
- IMU: accelerometer (3-axis specific force) and gyro (3-axis angular
  velocity) — real hardware will have this.
- Joint encoders: position and velocity per joint — real hardware will have
  this.
- Motor current / torque sensing is outside this robot's hardware contract and
  must not be introduced as a hidden control dependency.
- Wheel odometry derived from the above.
- Anything computed *from* the above via a real estimator (a complementary
  filter, an EKF, FK/IK) — the computation can run in sim or on hardware
  identically; what matters is that its *inputs* are things a real sensor
  could produce.

**Not allowed** — anything that only exists because the code is talking to a
physics engine:
- `sim_pose` / any direct TF or model-state read from Gazebo for
  orientation, position, or velocity used in a control decision.
- Any Gazebo contact sensor value used as if it were a real force/torque
  sensor reading, when the target hardware has no such sensor (see
  `docs/LEDGE_DRIVEOFF_LIMIT.md`'s contact-telemetry section for the current,
  explicitly-scoped exception: it is wired in as *observational telemetry
  only*, not fed into any control decision, precisely to avoid violating
  this rule while still being useful for sim-side investigation).
- Any "free" quantity (exact mass, exact CoM, exact velocity) pulled from the
  simulator's internal state at runtime rather than from a sensor-backed
  estimate or a design-time constant (CAD mass/inertia data, e.g.
  `masses.yaml`, is fine to use as a *known constant* the way a real robot's
  designer would also know it at design time — the line is "measured live
  from the sim" vs. "known in advance from the CAD/BOM").

### Current status (updated 2026-09-16 — corrects earlier stale text)

**`pitch_source` now defaults to `"imu"`** (since 2026-09-15, commit
`0ca725a`), not `sim_pose` — the earlier version of this section was out of
date. The permanently fixed hardware sensor inventory is only one MPU6500
(3-axis accelerometer + 3-axis gyroscope) and six motor encoders (four leg
 joints and two wheel joints). Physical wheel contact sensors, force/torque
 sensors, range sensors, cameras/localizers and motor current/torque feedback
 are permanently excluded from this robot's hardware contract and control
 design. Therefore the current Gazebo
 per-wheel contact stream (`any_wheel_contact_[0]`/`[1]`) is simulation
 telemetry, not a real hardware sensor, and must not be treated as deployable
 touchdown truth. The deployable path must infer a support hypothesis from
 MPU6500 and encoder data, with explicit uncertainty and persistence.

This corrects the earlier documentation claim that the default config already
used real per-wheel contact sensors. Any existing simulation result that feeds
Gazebo contact into `state_.floor_contact` is simulation-path evidence only
until it is replaced by the hardware-equivalent support observer.

**Two active violations remain, and per the Absolute framing above neither
is acceptable as "scoped future work":**

1. **Station Keeper** (`station_keeper.enabled`, off in
   `balance_controller.yaml` by default, on in some other profiles) reads
   `last_sim_pose_x_m_`/`last_sim_pose_y_m_` (Gazebo ground-truth world
   position) directly as its x/y feedback, regardless of `pitch_source`.
   A real robot has no such signal; this needs a real-sensor substitute
   (e.g. wheel-odometry + gyro-yaw dead reckoning) before it can be
   considered compliant.
2. **Jump** (`jump.enabled`, off by default, experimental) hard-requires
   `pitch_source == "sim_pose"` at construction and throws otherwise — it
   cannot currently run on the real-sensor IMU path at all.

Both are real, open defects to eliminate, not documented debt to leave in
place — see project memory `project_imu_only_direction.md` for status
tracking. Until fixed, any number gathered with either feature enabled is
ground-truth-assisted evidence, not evidence about the real-sensor
controller's own behavior, and must be labeled as such when reported.

### How to apply this rule going forward

Before adding or changing any control-path code:

1. Ask: "what sensor, that a real TAP robot will actually have, produces
   this value?" If the honest answer is "none, it's just what Gazebo already
   knows," stop and flag it — don't add it as a silent dependency.
2. If a required real sensor is excluded from this robot's hardware contract
   (for example a wheel force/torque sensor), do not fake it with a simulator
   shortcut and do not propose it as a future dependency. Derive an estimate
   only from the MPU6500 and six encoders, or explicitly scope the feature as
   simulation-only and say so in its evidence level.
3. When reporting results, state clearly whether the tested configuration
   used `sim_pose` (ground truth) or a real-sensor-backed path — do not let
   a `sim_pose`-based result stand in for evidence about the real-sensor
   path's behavior, since they are not the same controller in practice.
4. This rule does not block *using* simulation to develop and test — it
   blocks the *controller itself* from depending on simulator omniscience.
   The simulator is allowed to know everything; the controller is not.
5. Ground truth is always fine in a standalone diagnostic script (e.g. this
   session's `diag_*.py` scratchpad tools, or `clean_ledge_smoke.py`
   reading `/tap_robot/pose` to score a trial) that only *reads and reports*
   — never in anything that publishes back into the running node's own
   command path. If a debugging script's ground-truth read is ever wired
   into a topic/parameter the control node itself consumes to act, it has
   crossed from diagnostic to control-path and this rule applies.

---

## Rule 2: Analyze leg/motor load across both legs together, never one leg in isolation

**Any torque, load, or motor-capacity analysis must account for both legs
sharing the robot's weight symmetrically. Never compute or reason about a
single leg carrying the full load, and never design a mechanism around a
single-leg-touchdown or single-leg-support scenario** unless the user has
explicitly described a real one-legged event — TAP has no normal operating
mode (standing, driving, jumping, landing) where only one leg bears the
robot's weight while the other carries none.

### Why this rule exists

The user corrected this directly (2026-09-16) after an early static-torque
hypothesis implicitly reasoned about one leg's motor holding up more than
its real symmetric share. Static and dynamic load calculations (holding a
pose, sizing `leg_max_torque_nm`, landing-impact absorption, GIM6010-8
headroom checks) are only physically meaningful when both legs' real,
simultaneous load share is used - a one-leg-only assumption silently
produces a number for a scenario that does not occur, invalidating whatever
conclusion follows from it (an apparent torque shortfall or surplus that
isn't real).

### What's allowed vs. not

**Allowed**: computing one leg's required torque *using that leg's correct
symmetric share* of total robot weight (e.g. `total_mass/2 * g` per leg for
a two-legged double-support stance), then noting the other leg is
identical by symmetry - this is the correct, already-used method (see
`docs/LEDGE_DRIVEOFF_LIMIT.md`'s velocity-headroom section and this
session's GIM6010-8 static-torque check, both real examples).

**Not allowed**: sizing a mechanism, torque cap, or safety margin against a
hypothetical single-leg-bears-everything load, or reasoning about landing/
jump/touchdown as if one leg could independently absorb an impact the real
robot would always distribute across both. The per-side floor-contact
telemetry (`left_floor_contact`/`right_floor_contact`, see
`docs/LEDGE_DRIVEOFF_LIMIT.md`'s Track B section) exists to detect brief,
millisecond-scale *timing* asymmetry in when each wheel touches down - not
evidence that one leg ever bears the full load alone. Do not conflate the
two.

### How to apply this rule going forward

1. Before computing any torque/force number, ask: "does this assume one
   leg supports weight the real robot would split across two?" If yes,
   halve it (or use the correct fraction) before drawing a conclusion.
2. When citing per-side contact-timing differences as motivation for a
   mechanism, keep the framing to *timing/reaction*, not *load-bearing
   fraction* - the robot's weight is still shared, only the touchdown
   instant may differ by milliseconds.
3. If the user ever does describe a genuine one-legged scenario, treat it
   as an explicit, separately-scoped exception, not license to revert to
   single-leg reasoning elsewhere.

---

## Rule 3: The canonical landing test is a 2.5 m/s command off a 50 cm ledge

**Every landing/fall-recovery mechanism (impedance, reflex, roll correction,
MPC changes, fall-handling changes - anything touching what happens between
losing and regaining floor contact) must be evaluated against this exact
scenario before any claim of improvement or regression is made:** the robot
starts balanced on flat ground, drives forward with a **2.5 m/s command**, and
drives straight off a ledge of height **h = 50 cm**, landing on the lower
level with no other maneuver in between.

The `2.5 m/s` value is the requested command, not a promise that the current
actuator model can realize it. The test must separately record the
encoder/observer-derived forward velocity at liftoff, wheel-speed saturation
and slip. A saturated trial must be reported as **2.5 m/s command / lower
realized liftoff speed**, never as a measured 2.5 m/s landing trial.

### Why this rule exists

The previous 1 m/s / 50 cm scenario exposed a physical edge-clearance limit:
at the observed realized speed the robot lost wheel support while the knee
was still over the ledge and could strike the ledge edge. The user therefore
changed the canonical experiment to a 2.5 m/s command (2026-09-17) so the
robot's forward momentum and whole-body clearance are tested explicitly. This
does not waive actuator limits; it makes saturation and the resulting
realized speed first-class evidence instead of hiding them behind the command
value.

### The scenario, precisely

1. Flat ground, robot enabled and settled in `Balance` mode at the default
   stance (`stand_up.stand_pose_rad`).
2. Drive command: forward velocity target **2.5 m/s** (`/cmd_vel` or the
   equivalent test-harness publisher), zero yaw rate - straight line, no
   turning.
3. Ledge height **h = 0.50 m**, sharp drop (see
   `tools/generate_ledge_world.py`'s `generate(height)` - the world this
   session's tooling already builds), robot drives off it under its own
   commanded velocity (not dropped/teleported).
4. Landing is unassisted otherwise - whatever the controller's own real-time
   pitch/roll/contact response does is exactly what is being measured.

The controller does not receive `h`, ledge geometry, simulator pose or a
direct contact/force signal. The planned deployable sensor set is only one
MPU6500 and six motor encoders. `h = 0.50 m` is a property of the test world
and offline scoring, never a control input.

### Operational success definition

The test harness must keep publishing the commanded forward velocity through
the edge crossing, flight, touchdown, handoff and post-handoff hold. It must
not publish a zero command immediately after the front wheel crosses the edge:
that would turn the experiment into "fall, stop and recover" instead of
"land and continue driving". The command remains **2.5 m/s forward, 0 rad/s
yaw** unless the trial is terminated by a declared safety timeout. The
realized encoder-derived speed remains a separate measured quantity.

The trial is successful only when the robot does all of the following:

1. Detects the support-loss/flight interval from the MPU6500 and encoder path.
2. Forms a persistent, sensor-derived support hypothesis after the landing
   event. An IMU shock, one encoder transient, Gazebo contact message or one
   sampled contact-like pulse alone is not touchdown proof.
3. Reduces pitch and pitch rate into the predeclared handoff window without
   entering `Fallen` or requiring simulator recovery.
4. Re-enables normal wheel balance only after the declared sensor-derived
   support and attitude conditions are true, then returns to `Balance`/`Drive`.
5. Tracks the still-active forward command using encoder-derived wheel
   motion, and remains stable for the declared post-handoff hold interval.

`recovered` therefore means **ready to continue driving**, not merely "the
robot has touched the lower surface" or "the body has not yet fallen". The
following acceptance values must be frozen and recorded before each headline
trial batch (and must not be changed after seeing its outcomes):

- support persistence: the sensor-derived support confidence must remain above
  its declared threshold continuously for at least **0.10 s**;
- attitude handoff: `|pitch| <= 0.22 rad` and `|pitch_rate| <= 0.55 rad/s`;
- wheel-balance reactivation: support persistence is satisfied, the attitude
  handoff window is satisfied, the controller is not in `Fallen`, and the
  wheel-balance output is demonstrably active; Gazebo contact is not an
  admissible gate;
- post-handoff hold: the above conditions must remain satisfied for **2.0 s**;
- continued motion: the 2.5 m/s command remains active and the
  encoder-derived forward wheel velocity must remain within **±20% of the
  measured pre-liftoff speed**, subject to a separately reported saturation
  or slip flag. A pre-liftoff speed below 2.5 m/s is not silently promoted to
  a 2.5 m/s physical result.

The 0.10 s, 0.22 rad, 0.55 rad/s and 2.0 s values remain the canonical
evaluation thresholds for the current 50 cm test. The velocity threshold is
now relative to the measured pre-liftoff speed as stated above. A mechanism may
use different internal debounce/gain thresholds, but any different scoring
threshold must be justified and frozen before the batch; changing it after a
failure is a new experiment, not a recovery.

Gazebo model pose, model velocity and contact streams may be recorded by an
offline scorer to establish the actual edge-crossing and touchdown timeline,
to diagnose collision/solver artefacts and to explain a failure. They must be
labelled **offline diagnostics** and must never be consumed by the controller,
the support observer, the landing FSM or the recovery gate.

### What's allowed vs. not allowed

**Allowed**: testing OTHER heights/speeds as exploratory diagnostics to
understand a mechanism's behavior across a range (this session did this
extensively and it produced real insight - e.g. "medium heights work, 50cm
fails" was itself discovered this way) - but any headline claim of
improvement, regression, or a promotion candidate for `balance_controller.
yaml`'s own defaults must cite a real run of THIS exact scenario, not a
different height/speed substituted for convenience.

**Not allowed**: reporting a result from a different height or speed (e.g.
20 cm, or a vertical drop with no forward velocity) as evidence for or
against a 50 cm/2.5 m/s-command landing mechanism's effectiveness. The 20 cm ledge
drive-off has its own, separately-tracked evidence in
`docs/LEDGE_DRIVEOFF_LIMIT.md` and is a genuinely easier case (already
70-80% recovered by the stock/promoted config) - it does not stand in for
the 50 cm case this rule targets.

### How to apply this rule going forward

1. Use `tools/ledge_height_smoke.py --heights 0.50 --trials <n>
   --drive-speed 2.5 --controller-config <profile.yaml>` (or an equivalent real launch of the
   same scenario) as the standard evidence-gathering command. The harness is
   compliant only when it continues publishing the 2.5 m/s command after the
   edge and scores the sensor-only handoff criteria above; a harness that
   stops at the edge is an exploratory failure-injection diagnostic and cannot
   produce headline recovery claims.
2. Before launching the batch, record the exact controller profile, random or
   world seed, sensor rates, the five acceptance values above, and the meaning
   of every logged support/velocity field. Log the sensor-derived flight and
   support estimate, support confidence/persistence, landing phase, wheel
   balance active state, commanded and encoder-derived wheel velocity, pitch,
   pitch rate, roll, and torque/command saturation. Log Gazebo pose/contact
   separately under an explicit `offline_diagnostic` label.
3. Report `n` (trial count), `recovered` count/rate, failure reason and phase
   history, plus peak pitch, pitch rate and roll, support confidence,
   commanded/actual wheel velocity and saturation. A reduced peak angle
   without an actual `recovered=True` is real, reportable progress, but must
   not be presented as equivalent to a recovery.
4. When isolating one mechanism's contribution (as opposed to reporting a
   combined-configuration result), disable every other experimental
   mechanism and change exactly one variable per run, per this session's own
   practice after the capture-point-hip regression was found this way (a
   mechanism that helps in isolation can still make a combined configuration
   worse - always re-verify after combining, not just individually).
5. Before promoting anything to `balance_controller.yaml`'s own defaults,
   also re-run `sim_smoke.py`/`transformation_smoke.py`/`recovery_smoke.py`
   per the existing promotion discipline in CLAUDE.md's validation-status
   section - a win on this scenario alone is necessary but not sufficient
   for promotion.

---

## Rule 4: Simulation is a verification environment; ESP32-S3 is the deployment target

**The current project phase is simulation, but the final control target is an
ESP32-S3 N16R8. Every control algorithm, estimator, WBC/MPC extension and
landing mechanism must therefore be planned from the beginning so that a
realistic, deterministic implementation can run on the microcontroller.**
ROS 2 and Gazebo are adapters and verification tools, not foundations that the
embedded controller will depend on.

### Why this rule exists

Features that work only because of ROS callbacks, Gazebo ground truth, dynamic
memory, runtime configuration parsing or an unbounded desktop QP solver create
false progress: they may pass simulation while requiring a second algorithm to
be written for the ESP32. The simulation path must exercise the same portable
control core and the same sensor-shaped inputs that the embedded target will
eventually use.

### What's allowed vs. not

**Allowed**:

- ROS 2, Gazebo and simulator ground truth in adapters or offline diagnostic
  tools, provided ground truth never enters a control decision.
- Host-side visualisation, logging, replay, parameter sweeps and reference
  solvers for analysis.
- A portable C/C++ control core using fixed-size storage, bounded iteration
  counts and deterministic execution; single-precision floating point is
  acceptable when its timing and numerical error are measured on the target.
- Simulation sensor bridges that produce the same logical MPU6500 and six
  encoder fields expected from real hardware. Contact, force/torque and motor
  current streams may be logged for offline diagnosis, but are not hardware
  inputs. The excluded sensors may not be added as a control-path escape hatch.

**Not allowed in the deployable control path**:

- ROS 2, Gazebo APIs, simulator pose/contact ground truth or simulator-only
  state as an input to a control decision.
- Unbounded allocation, unbounded loops, runtime YAML/JSON parsing or a QP/
  optimisation backend whose worst-case execution time is unknown.
- Treating desktop CPU speed, a 1 kHz simulation loop or a successful RViz/
  Gazebo visualisation as evidence that the algorithm is feasible at the
  embedded control rate.
- Designing a WBC/MPC that requires the excluded force/torque, contact, range,
  camera, localization or motor-current signals, even as a proposed future
  control-path dependency. Such a feature is simulation-only unless it can be
  reformulated using only the MPU6500 and six encoders.

### Embedded architecture contract

1. Keep the deterministic algorithm in `src/tap_control/core`, independent of
   ROS 2 and Gazebo. `src/tap_control/ros2_nodes` and `src/tap_sim` are sensor,
   actuator and test adapters; an ESP32 HAL/driver layer will replace them.
2. Represent configuration as validated compile-time/default structs or a
   generated static configuration for firmware. Desktop YAML profiles may
   override those values during simulation but must not become a runtime
   firmware dependency.
3. Give every estimator, FSM, MPC/WBC solve and safety path a bounded memory
   footprint and a measured worst-case cycle time. Record the target rate,
   stack/heap use, solver iteration cap and numerical failure behaviour.
4. Use only the planned hardware-equivalent inputs in the deployable core:
   MPU6500 accelerometer/gyroscope and six motor encoders, plus quantities
   derived from them such as pitch, angular rates, FK and wheel odometry. A
   support/contact value must be an explicitly uncertain estimator from those
   inputs; Gazebo contact, force/torque and pose data may score a trial
   offline but may not silently substitute for a missing hardware signal.
5. Keep the six-joint order and CAD frame/sign contract identical across the
   simulator, host tests and firmware adapter. Any conversion belongs at an
   adapter boundary and requires sign/regression tests.
6. Treat host build/tests and Gazebo closed-loop runs as algorithm evidence;
   treat ESP32 timing, memory, numerical and I/O tests as deployment evidence.
   Neither evidence level alone certifies physical capability.

### How to apply this rule going forward

Before adding a feature, answer: "Which ESP32 task executes this, what are its
inputs, its bounded worst-case cost and its hardware-equivalent sensors?" If
those answers are unknown, the feature remains a diagnostic/prototype path and
must not be presented as deployable control. WBC/MPC design reviews must state
the actuator command interface (torque/current/velocity), contact sensing
assumption, solver bound, update rate and fallback behaviour before tuning
gains.

---

## Rule 5: Protected rules and architecture documents

**An agent must not modify `docs/RULES.md` or `architecture.md` without an
explicit permission from the user in the current task.** Reading, auditing,
quoting and proposing a patch are allowed; writing, reordering, deleting or
rewriting either document is not implicitly authorized by a normal code,
architecture or documentation task.

The permission must be clear enough to identify the intended document and
scope. A user request to implement a feature does not by itself authorize
changing the project rules or architecture contract. If implementation appears
to conflict with either document, the agent must stop the conflicting change
and ask the user, rather than silently weakening or rewriting the contract.

When permission is granted, the agent must keep the change focused, preserve
unrelated user edits, report the exact files/sections changed, and distinguish
rule changes from implementation changes in the final report.

<!--
Add future mandatory rules below as their own "## Rule N: <short name>"
section, following Rule 1's structure (the rule itself, why it exists, what
counts as compliant vs. not, any known existing violations and how they're
being tracked, and how to apply it going forward). Keep CLAUDE.md's pointer
to this file up to date if the summary line there needs to change.
-->
