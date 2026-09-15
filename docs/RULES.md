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

### Current known violation — not fixed yet, tracked deliberately

`BalanceControllerNode`'s `pitch_source` parameter defaults to `"sim_pose"`
today, and `station_keeper.enabled` / `jump.enabled` both hard-require
`pitch_source == "sim_pose"` at construction (they throw otherwise). This is
the single largest violation of this rule in the codebase and is the
existing baseline that essentially all of Track A/B's QA evidence
(`sim_smoke.py`, `clean_ledge_smoke.py`, `transformation_smoke.py`) was
gathered against.

This is **known, intentional debt, not an oversight** — switching it out is
scoped as its own future track (see project memory
`project_imu_only_direction.md` / the memory index) specifically *because*
it is large enough to need its own from-scratch QA pass, not something to
fold silently into unrelated work. The existing IMU-based complementary-
filter path (`BalanceControllerNode::onImu`) is real-sensor-only and already
exists in the codebase, but has never been run as the primary path for any
committed evidence so far — treat any number gathered with `sim_pose` active
as provisional once this migration happens, not as final validation of the
IMU path's own behavior.

**Attempted 2026-09-15, reverted — real blocker found, not just config.**
Flipping `pitch_source` to the IMU path and generalizing `floor_contact` to
use real contact sensing (`anyWheelContactFresh()`/`any_wheel_contact_`,
built on Track B.1's `measure_contacts` telemetry, which now defaults `true`
and is wired into all three worlds specifically to support this) both built
and passed every static/unit/contract test cleanly. Closed-loop Gazebo did
not: with the IMU path actually driving the MPC, `sim_smoke.py`'s stationary
"hold" stage showed the wheel runaway-driving at up to 0.6 m/s (direction
flipped between runs — oscillatory, not a fixed drift) and pitch error
growing to ~0.12 rad within about a second of standing still. Isolated the
cause before giving up: measured the complementary filter's own static bias
while the robot was independently stabilized by `sim_pose` (so the
measurement wasn't itself corrupted by the instability) — only ~0.017 rad,
too small to explain the failure. Tried compensating that bias
(`pitch_offset_rad`) and slowing the accelerometer low-pass (0.15s → 0.4s,
to better reject MPC-driven linear acceleration bleeding into the gravity
estimate) — neither fixed it; same ~0.12 rad failure, same oscillatory
direction-flip signature. This points to a genuine closed-loop
instability specific to combining this exact complementary filter with this
robot's MPC dynamics, not a one-parameter calibration issue.

**Reverted `pitch_source` to `sim_pose` as the shipped default** rather than
leave a known-broken default in place — re-confirmed the revert regresses
nothing (`sim_smoke.py` clean, `clean_ledge_smoke.py` 8/10 matching Track
A's established 80% baseline exactly, all unit/contract tests pass). Every
other piece of scaffolding from this attempt was kept as real, working
infrastructure ready for whenever the estimator itself gets fixed: the
`floor_contact` generalization, `measure_contacts` defaulting on with all
three worlds supporting it, and `jump`/`station_keeper`'s configs now
explicitly pinning `pitch_source: sim_pose` themselves (so they stay correct
regardless of the base default).

**What's actually needed to finish this migration** (the real next step, not
a config flip): proper closed-loop tuning or redesign of the pitch
estimator itself — likely a real EKF fusing gyro + accelerometer with an
explicit motion/acceleration model (rather than the current simple
low-pass-for-gravity complementary filter, which assumes near-static
specific force and has no way to distinguish "tilted" from "accelerating"
beyond a soft alpha blend), evaluated with its own dedicated closed-loop
test campaign before it's trusted as the default. This is real control/
estimation engineering, not a rules-compliance checkbox.

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

---

<!--
Add future mandatory rules below as their own "## Rule N: <short name>"
section, following Rule 1's structure (the rule itself, why it exists, what
counts as compliant vs. not, any known existing violations and how they're
being tracked, and how to apply it going forward). Keep CLAUDE.md's pointer
to this file up to date if the summary line there needs to change.
-->
