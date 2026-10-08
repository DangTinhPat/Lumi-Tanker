# Project rules (mandatory)

Standing rules for this codebase, binding on every session and every piece of
work — not suggestions to weigh against convenience. Each rule has its own
section below; add new mandatory rules as their own numbered section rather
than folding them into this preamble. If a rule and a one-off task
instruction conflict, ask the user rather than silently picking one.

The deployed controller is the v3.3 firmware (`src_fw/src/v3/V3Controller.cpp`
on the ESP32-S3, see `architecture.md`). The native MuJoCo simulator and its
older `EmbeddedController` are verification tools; they are not what the robot
runs. These rules apply to both unless a rule says otherwise.

---

## Rule 1: No ground truth in the control path

**No control decision may depend on simulator-internal state that has no
real-hardware equivalent.** This applies to every new feature, every bug fix,
every tuning change — not just contact or landing work.

"Ground truth" means any signal a simulator can hand the controller for free
because it has perfect knowledge of the world, but that a real robot could
never read directly off a physical sensor: MuJoCo `qpos`/`qvel`, body pose,
contact lists, constraint forces, exact mass or CoM read at runtime, the
phase of a scripted scenario.

### Absolute framing (user, 2026-09-16)

The robot's entire operating process — everything that decides a torque,
velocity, or pose command — must run **completely on the planned hardware
sensor data (MPU6500 accelerometer/gyro and six motor encoders), with zero
involvement of ground truth**. Support or contact is an estimate derived from
those signals, not a direct physical contact measurement. The simulator is the
physics environment the sensors are simulated inside of; it is not itself a
sensor and its internal state must never reach a control decision, directly or
indirectly.

Ground truth is permitted in exactly one role: **an offline diagnostic signal,
read only by a debugging/analysis/scoring script, to compare against what the
real-sensor path estimated.** The moment a ground-truth value is read by the
controller (or any code that issues a command) and used for anything the robot
*does* — not just observes for a human to read afterward — that is a violation
of this rule, regardless of whether the feature is "opt-in," "off by default,"
or "future work." There is no accepted resting state short of zero.

### Why this rule exists

The user's stated goal (2026-09-15) is to keep this codebase as close as
possible to what a real robot deployment looks like, so that moving from
simulation to the robot requires minimal rework. A control decision built on a
ground-truth shortcut has to be re-derived, re-tuned and re-validated later
against noisy, biased, latent real sensors — debt that surfaces on hardware.

### What is allowed vs. not allowed

**Allowed** — anything with a real physical sensor behind it, even when the
implementation runs in simulation:

- IMU: accelerometer (3-axis specific force) and gyro (3-axis angular
  velocity) — one MPU6500.
- Joint encoders: position and velocity per joint — four leg joints (GDS68
  drivers over CAN) and two wheel joints (BL4820 drivers over UART).
- **Leg motor Iq-based torque estimate, leg CAN driver only** (2026-09-18
  exception — see "Leg motor current-based torque exception" below). Allowed
  **only** as a secondary, corroborating signal in a support/contact estimator
  that also requires independent IMU and encoder/kinematic evidence to agree.
  It must never be the sole or primary gate for a control decision and must
  pass the same persistence/debounce discipline as every other signal.
  Wheel motor current/load (`docs/actuatal_driver/WHEEL_MOTOR_PROTOCOL.md`)
  remains excluded: its `Load` register has an unverified, uncalibrated unit.
  A standalone force/torque sensor (strain gauge, load cell, etc.) remains
  permanently excluded.
- Wheel odometry derived from the above.
- Anything computed *from* the above by a real estimator (complementary
  filter, EKF, FK/IK, `ImuStateEstimator`): the computation may run in
  simulation or on hardware identically; what matters is that its *inputs*
  are things a real sensor could produce.
- Design-time constants known in advance from CAD/BOM (mass, CoM, inertia in
  `src/tap_control/core/include/robot_control/GeneratedGeometry.hpp` and
  `src/tap_sim/model/mesh_inertials.yaml`), the way the robot's designer also
  knows them. The line is "measured live from the simulator" vs. "known in
  advance from the CAD/BOM".

**Not allowed** — anything that exists only because the code talks to a
physics engine:

- Any direct read of simulator pose, velocity, orientation, or model state
  used in a control decision.
- Any simulator contact or force value used as if it were a real force/torque
  or contact sensor when the target hardware has none. Contact, effort and
  pose streams may be logged as **observational telemetry** (the native
  runner's `VIEW|` channel, offline scorers) and never fed to the controller.
- Any "free" quantity (exact mass, exact CoM, exact velocity) pulled from the
  simulator's internal state at runtime instead of from a sensor-backed
  estimate or a design-time constant.

### Hardware inventory

The permanently fixed deployable sensor inventory is one MPU6500 and six motor
encoders, plus — only under the exception below — the four leg joints' own
CAN-driver Iq-based torque estimate. Physical wheel contact sensors,
standalone force/torque sensors, range sensors, cameras/localizers used for
balance, and wheel motor current/load are excluded from the control design.
(The optional person-follower under `blue_object_vision/` uses a camera on a
host PC and only sends drive commands over the serial console; it is not part
of the balance loop.)

### Leg motor current-based torque exception (user, 2026-09-18)

Verified against the driver protocol documents in `docs/actuatal_driver/`
(`GDS68_DRIVER.md`, `WHEEL_MOTOR_PROTOCOL.md`):

- The four leg joints' CAN driver (SteadyWin GDS68, ODrive-based firmware)
  measures FOC quadrature current (Iq) as a normal part of closed-loop motor
  control — not an added sensor — and reports `torque ≈ Iq × Kt` in the MIT
  feedback frame, at the same rate as position/velocity. This is real driver
  behaviour, not a simulator shortcut.
- The two wheel joints use a different driver (UART, CJC BL4820M12) whose
  `Load` register is not usable. The wheel is also where ground reaction first
  acts, so this exception does **not** give a direct read of ground contact
  force; it gives an indirect, Jacobian-mediated view from the hip/knee side.
- Precedent: CAD/BOM inertials are accepted as design-time constants; the leg
  CAN driver's Kt and Iq measurement are analogous — a property of the robot's
  own hardware, not information borrowed from a simulator's omniscient state.

**Binding conditions** (all required):

1. Never the sole gate. A leg-torque-residual signal may only raise or lower
   confidence in a support/contact hypothesis that also independently requires
   IMU evidence and encoder/kinematic evidence to agree (the multi-signal
   persistence design of `FlightObserver`).
2. Residual, not raw value. Any control-path use must compare measured torque
   against an expected torque computed from the commanded motion and known
   leg mass/inertia, not threshold the raw instantaneous value — raw Iq is
   dominated by self-motion and gearbox-friction terms.
3. No wheel extension. The exception covers the four leg (hip/knee) CAN joints
   only; it is no licence to use wheel motor current/load or to claim a direct
   ground-normal-force measurement at the wheel.
4. No new physical sensor. Only data the GDS68 driver already produces as part
   of its existing closed-loop control.
5. Evidence labelling. Report whether a given trial's decision used this
   signal and do not present a leg-torque-assisted result as equivalent to one
   derived from MPU6500 + encoders alone until the signal is validated on the
   robot.

### How to apply this rule going forward

Before adding or changing any control-path code:

1. Ask: "what sensor, that the real robot actually has, produces this value?"
   If the honest answer is "none, it is just what the simulator knows," stop
   and flag it — do not add it as a silent dependency.
2. If a required sensor is excluded from the hardware contract, do not fake it
   with a simulator shortcut and do not propose it as a future dependency.
   Derive an estimate from the MPU6500 and six encoders, or scope the feature
   as simulation-only and say so in its evidence level.
3. When reporting results, state whether the tested configuration used any
   ground-truth signal or only the sensor-shaped path.
4. This rule does not block using simulation to develop and test; it blocks
   the *controller itself* from depending on simulator omniscience. The
   simulator may know everything; the controller may not.
5. Ground truth is always fine in a standalone diagnostic script that only
   *reads and reports*. If a diagnostic's ground-truth read is ever wired into
   anything the controller consumes, it has crossed into the control path and
   this rule applies.

---

## Rule 2: Analyze leg/motor load across both legs together, never one leg in isolation

**Any torque, load, or motor-capacity analysis must account for both legs
sharing the robot's weight symmetrically. Never compute or reason about a
single leg carrying the full load, and never design a mechanism around a
single-leg support scenario** unless the user has explicitly described a real
one-legged event — TAP has no normal operating mode (standing, driving,
crouching, height change) where only one leg bears the weight.

### Why this rule exists

The user corrected this directly (2026-09-16) after an early static-torque
hypothesis implicitly reasoned about one leg's motor holding more than its real
symmetric share. Static and dynamic load calculations (holding a pose, sizing
a torque cap, GIM6010-8 headroom checks, feed-forward torque tables) are only
meaningful when both legs' simultaneous load share is used; a one-leg
assumption silently produces a number for a scenario that does not occur.

### What is allowed vs. not

**Allowed**: computing one leg's required torque using that leg's correct
symmetric share of total robot weight (`total_mass/2 * g` per leg in
double-support), then noting the other leg is identical by symmetry — the
method used by `WholeBodyModel::bodySupportTorque`.

**Not allowed**: sizing a mechanism, torque cap, or safety margin against a
hypothetical single-leg-bears-everything load. Per-side differences (roll
offsets, left/right timing) are *timing/reaction* effects, not evidence that
one leg bears the full load; do not conflate the two.

### How to apply this rule going forward

1. Before computing any torque/force number, ask: "does this assume one leg
   supports weight the real robot splits across two?" If yes, halve it (or use
   the correct fraction) before drawing a conclusion.
2. If the user describes a genuine one-legged scenario, treat it as an
   explicit, separately-scoped exception, not licence to revert to single-leg
   reasoning elsewhere.

---

## Rule 3: Retired — canonical landing test (2026-10-08)

This rule required every landing/fall-recovery mechanism to be evaluated
against a 2.5 m/s command off a 50 cm ledge in a retired simulation setup. The
robot no longer has a jump, landing or ledge-drive-off function, the simulation
harness (`tools/ledge_height_smoke.py`, `tools/generate_ledge_world.py`) is gone
and the supporting documents were removed. The rule is retired by the user's
decision of 2026-10-08; the number is kept so that older references stay
unambiguous. The controller code for jump and landing that remains in
`src/tap_control/core` is unused by the robot; if such a function is revived,
a new rule with a scenario that exists in the current tooling must be written
first.

---

## Rule 4: Simulation is a verification environment; ESP32-S3 is the deployment target

**Every control algorithm, estimator and safety mechanism must be planned so
that a realistic, deterministic implementation runs on the ESP32-S3 N16R8.**
The v3.3 firmware (`firmware_esp32/`, `src_fw/`) already runs on the robot.
Simulators, desktop GUIs and log tools are adapters and verification tools,
not foundations the embedded controller depends on.

### Why this rule exists

Features that work only because of desktop libraries, simulator ground truth,
dynamic memory, runtime configuration parsing or an unbounded solver create
false progress: they pass simulation while requiring a second algorithm for
the ESP32. The simulated path must exercise the same portable control code and
the same sensor-shaped inputs the robot uses.

### What is allowed vs. not

**Allowed**:

- Simulator ground truth in adapters and offline diagnostic tools, provided it
  never enters a control decision (Rule 1).
- Host-side visualisation, logging, replay, parameter sweeps and reference
  solvers for analysis (`tools/`, `src/gui`, `src_fw/tests`).
- A portable C/C++ control core with fixed-size storage, bounded iteration
  counts and deterministic execution; single precision is acceptable when its
  timing and numerical error are measured on the target.
- Scalar runtime tunables over the serial console (`SET <key> <value>`) that
  override compile-time defaults, each with a safe range and a way to revert.
- Simulation sensor bridges that produce the same logical MPU6500 and
  six-encoder fields the hardware provides.

**Not allowed in the deployable control path**:

- Simulator APIs, simulator pose/contact ground truth, or simulator-only state
  as an input to a control decision.
- Unbounded allocation, unbounded loops, runtime YAML/JSON parsing, or an
  optimisation backend whose worst-case execution time is unknown.
- Treating desktop CPU speed, a kHz simulation loop or a good-looking viewer as
  evidence that the algorithm is feasible at the embedded control rate.
- A controller that requires an excluded signal (force/torque, contact, range,
  camera, localization, wheel motor current) as a control-path dependency.
  Such a feature is simulation-only unless reformulated on the MPU6500 and six
  encoders.

### Embedded architecture contract

1. Deterministic algorithms live in portable C++17 without desktop
   dependencies: the robot controller in `src_fw/src/v3`, shared models and
   estimators in `src/tap_control/core`, the motor protocol layer in
   `src/tap_control/hal`. ESP32 peripherals are reached only through
   `firmware_esp32/` adapters.
2. Configuration is validated compile-time/default structs. Console `SET`
   overrides are allowed; desktop config files must not become a firmware
   dependency.
3. Every estimator, FSM and safety path has bounded memory and a measured
   worst-case cycle time (the firmware reports per-tick timing in its `SENS`
   line). Record the target rate, task, and numerical-failure behaviour.
4. Use only hardware-equivalent inputs in the deployable core. A
   support/contact value must be an explicitly uncertain estimator from them.
5. Keep the six-joint order `[LH, LK, LW, RH, RK, RW]` and the CAD frame/sign
   contract identical across the simulator, host tests and firmware. Any
   conversion belongs at an adapter boundary (`MotorMapping.hpp`) and needs
   sign regression tests.
6. Treat host builds/tests and simulator runs as algorithm evidence; treat
   firmware timing, memory, I/O and robot logs as deployment evidence. Neither
   alone certifies physical capability. Robot time is scarce: new behaviour
   ships behind a default-off or revertible `SET` flag and is validated in
   simulation first.

### How to apply this rule going forward

Before adding a feature, answer: "Which ESP32 task executes this, what are its
inputs, its bounded worst-case cost and its hardware-equivalent sensors?" If
those answers are unknown, the feature remains a diagnostic/prototype path and
must not be presented as deployable control.

---

## Rule 5: Protected rules and architecture documents

**An agent must not modify `docs/RULES.md` or `architecture.md` without an
explicit permission from the user in the current task.** Reading, auditing,
quoting and proposing a patch are allowed; writing, reordering, deleting or
rewriting either document is not implicitly authorized by a normal code,
architecture or documentation task.

The permission must be clear enough to identify the intended document and
scope. A request to implement a feature does not by itself authorize changing
the rules or architecture contract. If implementation appears to conflict with
either document, stop the conflicting change and ask the user, rather than
silently weakening or rewriting the contract.

When permission is granted, keep the change focused, preserve unrelated user
edits, report the exact files/sections changed, and distinguish rule changes
from implementation changes in the final report.

---

## Revision history

- 2026-10-08 (user-authorized restructure): rewritten against the current code.
  Removed the retired-simulator wording, the `sim_pose` status section and the two
  "active violations" (Station Keeper and Jump read `sim_pose`; no code reads
  `sim_pose` any more), the references to retired documents, and retired
  Rule 3. Fixed driver-document paths (`docs/actuatal_driver/`). Rule 4 now
  states that the firmware exists and that console `SET` tunables are allowed.
  Rule 1's allowed/not-allowed lists, the Iq exception and its five binding
  conditions, Rule 2 and Rule 5 keep their substance unchanged.

<!--
Add future mandatory rules below as their own "## Rule N: <short name>"
section: the rule itself, why it exists, what counts as compliant vs. not,
known violations and how they are tracked, and how to apply it. Keep the
pointer in CLAUDE.md up to date if its summary line needs to change.
-->
