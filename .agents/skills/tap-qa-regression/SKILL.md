---
name: tap-qa-regression
description: Verify TAP controller changes with model, host-test, planar-sim and robot-log evidence; distinguish builds and viewers from physical capability.
---

# QA And Regression Expert

Read workspace `AGENTS.md` and the current test/build entrypoints
(`architecture.md` section 6, `docs/V3.3_HANDOFF.md` section 7). Inspect the
implementation before selecting commands; do not invent a passing run or copy
thresholds from another project as TAP acceptance limits.

- Model checks: `src/tap_sim/scripts/validate_model.py` (mesh integrity, seven
  inertial links, six movable joints, native +Y axes, mass sum, rotated body
  COM/tensor, independent FK).
- Host checks: `ctest` (five failures predate current work and are listed in the
  handoff; new failures are regressions), the `v3_*` tests, and the `TIER_*`
  planar sims in `v3_tier_tests`. For a refactor, build the old and new firmware
  and compare the ELF symbol tables; a flag-off path must behave like the old code.
- Add the metric that would have caught the defect you fixed (for example knee
  command rate or gyro >5 Hz rms) so a test can fail for it, and include a
  control case with the fix disabled that proves the test sees the problem.
- Firmware checks: `pio run -e esp32-s3-v3` and the single-loop environment,
  flash/RAM size, the boot `V3 TUNE` lines.
- Log checks (only logs the user recorded): `tools/tr_rest_stats.py`,
  `tools/hip_chatter_stats.py`, `tools/hf_vibration_stats.py`. Compare sessions
  at equal disturbance levels and say how many windows back a claim.
- Hardware tests run only when the user asks. Choose and state bounds before
  judging a run; a short noncrashing run or a viewer pose is not success.
- Maintain evidence levels separately: source inspected, build, unit/contract
  tests, planar firmware sims, MuJoCo runs, robot logs. Pending levels remain
  pending.

Report exact commands, versions/configuration, observed results and gaps. Review
only owned diffs; preserve concurrent edits. Skill frontmatter validation checks
discoverability, not algorithm correctness.
