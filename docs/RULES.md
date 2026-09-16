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
velocity, or pose command — runs **completely on real-sensor data (IMU,
joint encoders, per-wheel contact), with zero involvement of ground truth**.
Gazebo is nothing more than the physics environment the real sensors happen
to be simulated inside of; it is not itself a sensor and its internal model
state must never reach a control decision, directly or indirectly.

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
- Motor current / torque sensing, if the target hardware has it.
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
date. Under the default config, pitch (`BalanceControllerNode::onImu`, a
Kalman filter over real accelerometer+gyro) and floor contact
(`any_wheel_contact_[0]`/`[1]`, real per-wheel contact sensors) are both
real-sensor-only for every path `balance_controller.yaml` itself exercises
(hold, drive, squat, the 20 cm ledge landing) — verified by reading
`onImu`/`onSimPose`'s early-return guards and confirmed 2026-09-16 while
root-causing the ledge landing regression (see
`docs/LEDGE_DRIVEOFF_LIMIT.md`'s "Velocity headroom" section).

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
2. If a real sensor doesn't exist on the current design (e.g., no
   force/torque sensor on the wheels), don't fake it with a simulator
   shortcut — either derive an estimate from sensors that do exist (see
   `docs/LEDGE_DRIVEOFF_LIMIT.md`'s Fz-estimation discussion) or explicitly
   scope the feature as simulation-only and say so in its evidence level.
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

<!--
Add future mandatory rules below as their own "## Rule N: <short name>"
section, following Rule 1's structure (the rule itself, why it exists, what
counts as compliant vs. not, any known existing violations and how they're
being tracked, and how to apply it going forward). Keep CLAUDE.md's pointer
to this file up to date if the summary line there needs to change.
-->
