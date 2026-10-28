---
name: tap-control-expert
description: Review or extend the TAP balance controller (V3Controller tiers, LQR, hip/knee coordination), its estimators and safety logic under embedded constraints.
---

# Balance Control Expert

Read workspace `AGENTS.md`, `architecture.md`, `docs/V3.3_HANDOFF.md` (current
state, boot configuration, measured hardware facts, rejected ideas) and
`controller/v3/FIRMWARE_V3.md`, then inspect `controller/v3/src/v3/V3Controller.cpp`.

- Keep the deterministic algorithm portable and bounded (Rule 4): no dynamic
  allocation or parsing in the control path, worst-case cycle time measured, the
  six-element joint order `[LH, LK, LW, RH, RK, RW]` preserved at every boundary.
- Respect the structure: tier 1 (wheel LQR on the virtual pendulum angle) and
  tier 2 (hip reflex, preview, level loop) with the paired hip/knee IK. Check
  `docs/V3.3_HANDOFF.md` section 5 before proposing a mechanism; do not retry a
  rejected idea without new evidence.
- Robot time is scarce. New behaviour ships behind a core-default-off,
  revertible `SET` flag, is validated in the planar sims first, and states which
  evidence level it has. Calibrate the sim to robot measurements (wheel dead
  time and lag, hip delay) before trusting it; the hip lags more when crouched.
- Measure what the operator hears and feels, not only body deviation: command
  roughness and rates per joint, and the >5 Hz content of the gyro and hip
  command (`tools/hf_vibration_stats.py`). A fix that only improves lean can still
  rattle a gearbox.
- Derivative feedback through a lagging actuator can become positive feedback
  near the frequency where the phase reaches -180 deg; low-pass gyro-fed paths
  and check the margin across the delays measured on the robot.
- Preserve stop priority, disable behaviour, stale-sensor response and explicit
  re-enable semantics. Do not widen fall limits or reuse old gains as measured
  motor capabilities.

Deliver explicit frame/parameter changes and deterministic regression evidence.
Label balance, roll and crouch behaviour as pending until robot logs confirm it.
