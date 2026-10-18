# Joint oscillation diagnosis, 2026-10-03 evening

> **Báo cáo lịch sử (03/10).** VMC gối và hybrid suspension được nhắc ở đây đã bị gỡ khỏi firmware và native ngày 04/10; tiếng lạch cạch hết sau đó. Trạng thái hiện tại: `docs/V3.3_HANDOFF.md` (5.40, 5.45, 5.47).

Source inspected at commit 4d20893. Hardware evidence: existing GUI log
`/home/dvt/tap_logs/tap_20261003_190409.log`; no hardware commands issued in this investigation.

## Observations

Two balancing sessions: 30.112–86.698 s (ended fallen/pitch limit),
103.458–174.932 s (ended sensor feedback lost). No logged height or tuning
commands; only GET, zero DRIVE and two START commands.

In 144.816–164.781 s, requested height is exactly 176.6 mm. Nominal pose
height spans 171.2–184.6 mm; this is a command/model quantity, not measured
physical body height. Suspension spans -5.4..+8.0 mm. Hip tier offset spans
-0.0898..+0.0937 rad (10.5 deg peak-to-peak), while slow level spans
-0.0062..+0.0499 rad and trunk authority stays 1.00. IMU rate RMS is
0.767 rad/s. Windowed FFT of IMU rate at 144.734 and 155.031 s peaks at
8.06 and 8.20 Hz. First session develops peaks around 6.4, 9.5 and 10.7 Hz.
These are apparent frequencies at approximately 24–25 Hz telemetry; faster
motion may alias, so they do not identify a motor resonance conclusively.

Driver-frame encoder ranges in the 20 s window: LH 0.240, LK 0.421,
RH 0.095, RK 0.018 rad peak-to-peak. These are NOT all native CAD signs;
AppV3 applies driver_sign before passing them to the controller. Roll height
correction spans -17.3..-3.3 mm. Asymmetry merits per-side target/error
recording, but differing ranges alone do not prove a bad motor or wrong sign.

## Mechanisms established by source inspection

1. updateTrunk remains active at rest, with no event gate, hip deadband zero,
   60 ms rate lead and lead_lpf_hz=0 (unfiltered virtual rate). Encoder joint
   rates contribute to virtual rate. Hip response moves the joints that feed
   this rate back; actuator/feedback delay can sustain a coupled oscillation.
   At 8 Hz, a 60 ms derivative lead has amplitude factor about 3 relative to
   the same sinusoidal angle. This is a mechanism, not a measured phase proof.
2. coordinateTrunkAxle deliberately couples knee target to hip response.
   Hip chatter therefore also commands knee movement to preserve height.
3. Hybrid suspension moves the nominal height from slow vertical-speed error;
   VMC applies high-band torque, gravity and an additional knee-error integral.
   The observed slow height excursions are directly explained by sus, despite
   unchanged htgt. The shared estimator uses IMU plus encoders and assumes
   supported-leg kinematics; its error can drive both paths.
4. Concrete core/actuator inconsistency: updateVmc computes spring gains and
   integrates error using uncapped kp (stiff default 40, soft up to 60), while
   LegLock caps transmitted kp at 12. Observed kkp is 12 throughout this
   window. Thus ki=0.5 s is not referenced to the actual spring gain; clipping
   the integral at +/-3 Nm does not resolve that mismatch. Its causal share
   in this run cannot be measured because raw kp and per-side integral are
   absent from TR.
5. LegLock selects one message mode per joint: hips POS, knees MIT with VMC.
   Source inspection found no simultaneous POS and MIT target send to the same
   knee in sendTargets. Competing feedback objectives remain possible even
   with one command sender.

## Next discriminating checks and proposed correction order

- Record per-side final q_ref, encoder q, rate, transmitted kp/kd, integral,
  skyhook and gravity components, feedback age and deadline overruns. Use
  buffered acquisition faster than TR to resolve the fast motion without
  adding serial work to the control loop.
- Correct VMC to use the effective actuator spring gain before integral
  calculation, and add saturation-aware integration with regression coverage.
- Separate quiet posture from transient hip response; current firmware lacks
  the previously planned split. Event gating must be validated, not enabled
  from the failed host prototype.
- Isolate position suspension, VMC and trunk in controlled one-variable trials
  before tuning. Disabling trunk also disables its level loop, so that test
  does not retain the gimbal behavior. VMC mode changes require DISABLE first.
- Audit per-side tracking and communication for the pronounced asymmetry.
  Feedback loss at 174.966 s is a separate recorded failure; source/log do not
  establish whether it caused earlier oscillation or followed it.

No control code edited, no flash, and no new physical trial performed.
