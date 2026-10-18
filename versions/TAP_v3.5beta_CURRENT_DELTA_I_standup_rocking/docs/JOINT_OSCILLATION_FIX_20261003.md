# Joint oscillation fix checkpoint — 2026-10-03

> **Báo cáo lịch sử (03/10).** VMC gối và hybrid suspension được nhắc ở đây đã bị gỡ khỏi firmware và native ngày 04/10; tiếng lạch cạch hết sau đó. Trạng thái hiện tại: `docs/V3.3_HANDOFF.md` (5.40, 5.45, 5.47).

## Status

**Not a completed hardware fix. The stability prototype was not uploaded.**
At the user's subsequent explicit request, the telemetry-only ESP32 V3 firmware
was uploaded successfully; SHA256
`0fbd76dc5dea75cbb224e9e23430ca7bf5e9b823e19a293e6f6440edd44e5d8b`.
Boot verified MPU6500 PASS, RTOS, Idle/enable=0, vki=.50 unchanged. No motor
start or motion command was issued during upload/verification. Both normal
and single-loop PlatformIO builds passed; the normal RTOS build was flashed.
GUI restored after the serial verification connection was closed.
The user clarified that shaking begins after a period of stationary balance.
This fits accumulating posture/support states, but does not prove which loop
starts the fast mechanical oscillation.

The VMC stability correction remains HOST ONLY behind
`TAP_VMC_STABILITY_HOST_PROTOTYPE`, with `Config::Vmc::stability_fix=false`
by default. ESP32 cannot enable it. This contains a failed robustness gate:
the best tested setting has no new falls, but three cases exceed the user's
10% body-angle degradation criterion. Current firmware control behavior is
preserved; extra TR diagnostics are the only firmware-facing change.

## Implemented experiment

- Clamp core spring Kp to the same 12 Nm/rad cap as LegLock BEFORE computing
  integral learning. Driver-side defensive cap remains in place.
- Conditional integration accounts for the complete gravity + skyhook +
  integral feedforward limit, allows unwinding, and avoids accumulating toward
  unreachable knee reference limits.
- Clear the floor-support integral when leaving soft support mode.
- Add per-side knee target and integral to controller output; TR adds `kmv`,
  `kref`, `ki`, `kd`, `kv`. These are core/CAD signs. Existing `legs` are in
  driver signs. `kmv=0` means no fresh VMC output; the motor may retain its
  previous locked command. No new sensor dependency.

An experiment removing feedback-derived knee velocity feedforward was rejected
and reverted: it produced a new fall in the high-hip-inertia variant. The final
code retains the original velocity feedforward law. Hip gains, level, preview,
roll, suspension and firmware defaults were not retuned.

## Evidence and rejected trials

Independent pre-edit simulator binary: `/tmp/tap-joint-oscillation-baseline`.
Logs/results: `log/joint_fix_20261003/` (ignored runtime artifacts).

Each robustness matrix contains 84 cases: heights 84, 120, 176.6 mm;
stand, 4 N push, 8 N push, and drive/brake 0.55 m/s; seven existing actuator
variants. Simulation duration is 24 s/case. Baseline matches boot settings
`TR_HEXP=1.5 TR_TVGLOW=.7 TR_ROLL=1 TR_VMC=1 TR_SUSPENSION=1`.

- Baseline: 11 falls /84.
- Stability correction at original `vki=.5`, original feedforward: 13 falls,
  **3 new falls**, six nonfallen cases degrade >10%.
- Correction + planned-height-only velocity feedforward: 11 falls, **1 new
  fall**, hence reverted.
- Slower integral, weaker suspension, reduced hip lead and deadband trials
  did not satisfy the full acceptance gate; no parameter change promoted.
- Correction with `vki=.2`, original feedforward: 10 falls /84, **0 new falls**,
  but three nonfallen cases degrade >10%:
  - 84 mm, 4 N, wheel lag .10: 0.1479 -> 0.1697 rad body peak.
  - 176.6 mm, 8 N, soft hip: 0.0996 -> 0.1192 rad.
  - 176.6 mm, drive .55, extra wheel delay: 0.0135 -> 0.0152 rad.

The last setting reduces effective stiff-mode integral gain from the legacy
40/.5=80 to capped 12/.2=60 Nm/(rad s). It is a measured candidate, not an
accepted new firmware default. Results are `ki02_matrix.json`; earlier files
refer to interim unguarded host experiments. To reproduce after gating, enable
`TR_VFIX=1` in addition to the recorded parameters.

Commands for one candidate trial:

```sh
TIER_DIAG=1 DIAG_H=.12 DIAG_F=8 DIAG_VAR=0 DIAG_DUR=24 \
TR_HEXP=1.5 TR_TVGLOW=.7 TR_ROLL=1 TR_VMC=1 TR_SUSPENSION=1 \
TR_VFIX=1 TR_VKI=.2 build/tsplit-host/bin/v3_tier_tests
```

New deterministic contracts cover capped core gain, bounded integral under
feedforward saturation, unwinding, clearing support state, gravity consuming
the feedforward budget and the per-side reference. Focused V3 tests pass.
Full ctest had 23/28 pass: core_allocation, jump, jump_quality, lqr and v3_tier
failed. The first four were independently reproduced from an isolated archive
of HEAD (4d20893) without this patch; v3_tier stops at the already established
V15 held/preview assertion. V24/V32 are not reached. GUI or builds do not prove
physical balance.

## Remaining work

The fast hip feedback and slow suspension/level loops still require an accepted
closed-loop correction. Merely reducing gains improved some tests and harmed
others, so it is not justified as a complete cure. The simulator has not
reproduced the observed stationary ~8 Hz mode. Faster passive per-side target,
encoder-rate and integral recording on the robot is needed to separate driver
tracking/gear motion from accumulated controller offsets. There is no claim
that the prototype eliminates physical shaking.

Guard verification: four selected legacy-mode closed-loop results match the
pre-edit binary exactly, and four enabled prototype results match the recorded
ki=.2 experiment exactly (`guard_parity.json`). Final focused ctest: 7/7 pass.
