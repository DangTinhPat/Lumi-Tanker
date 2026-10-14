# IMU tipping capture — withdrawn 14 October 2026

# Status: **withdrawn after the operator reported substantially worse physical
behavior**. Firmware source and GUI profile have been returned to the documented
12:44 geometric-catch revision; this file records the rejected experiment only.

Branch `fw`. The IMU detector described below was built and flashed once, but
was **not physically validated as a successful capture algorithm**. User requested reverting the preceding pre-catch experiment and
building a more active catch around the physical tipping point, using IMU
instead of depending on potentially wrong CAD COM. The prior bidirectional
Rise slew and gyro-dominant StandUp estimator are reverted. Its deployment and
failed robot trials remain recorded in `STANDUP_PRECAPTURE_REVIEW_20261014.md`.
Robot/old GUI still use that withdrawn `a289bed6…` image until a requested upload.

## Why the last trials did not balance

Source `/home/dvt/tap_logs/tap_20261014_134907.log`, sanitized snapshot
`/tmp/tap_134907-review.log`, parsed `/tmp/tap_134907-rows.json`.
Both executed attempts fail in StandUp, with zero wheel command throughout;
there is no Balancing segment or Normal outcome. First attempt 22.229→24.883 s,
second 62.788→65.378 s; both fault `stand-up failed` at rear CAD lean about
−0.5 rad. Nearby motor feedback is present, driver axis errors zero, leg bus
about 23.9 V. This evidence points to missed control handover, not a new wheel
UART fault. Operator intervention timestamps remain unknown.

In trial 1, pitch reaches about +0.453 rad, gyro then changes −0.06→−0.28
rad/s as CAD lean changes +0.0486→+0.0175 rad (24.656–24.699 s). Before enough
qualified dwell fits inside the narrow geometric corridor, gyro/COM rate grows
and that corridor is left. At 24.737 s gyro is −0.76 rad/s; at 24.861 s −5.27.
Trial 2 shows the same sequence (65.149–65.231 s): gyro 0→−0.30→−0.82.
The wheel controller is never enabled. A synthetic test of angle estimation or
reference rates alone did not exercise this irreversible tipping transition.

`tr_rest_stats` finds zero standing windows; `hf_vibration_stats` finds no
Balancing data. There is no basis for reporting improved balance RMS.

## Detector and handover

`StandUpTippingCapture` accepts only bias-corrected body gyro, both hip/knee
encoder positions/rates, their commands, fresh feedback and known lift phase.
It has no CAD lean, COM, Jacobian, inertia, current or contact input.
The evidence is a **tipping/support hypothesis**, not a contact sensor or proof
that the true COM is exactly above the axle. A hand can generate a similar
signature; test interventions must be marked.

| Stage | Operation |
| --- | --- |
| Search | Preserve the original lift. After both hip encoders complete the 28° first segment, accumulate actual positive gyro rotation during paired folding. |
| Armed | At least 0.08 rad of lift rotation at gyro >0.10 rad/s has been observed. Wait for the first negative departure while hips are still folding. |
| Brake | Gyro <−0.10 rad/s, filtered gyro derivative <−0.5 rad/s², fresh/tracking paired hips and settled knees. Latch and hold the current hip command immediately; do not chase the CAD goal. |
| Caught | Negative departure remains continuously qualified for 14 ms, with body rate inside ±0.7 rad/s. Activate the existing bounded wheel arrest at the latched pose. |
| Rejected | Rate becomes too large, or no qualified event within 120 ms of braking. Fail with wheels disabled by V3; do not resume repeated rocking in this candidate. |

Hip command tracking ≤0.06 rad per side, left/right core hip separation
≤0.04 rad, hip rate ≤1.2 rad/s; knees track within 0.02 rad and rate
≤0.20 rad/s. These are provisional qualification thresholds from the observed
trajectory scale, **not calibrated mechanical or motor capabilities**.
Filtered angular acceleration uses a 20 ms one-pole filter; it is not an
actuator derivative feedback loop. Short gyro pulses, absent lift, one stuck
hip, moving knee, stale/invalid inputs and excessive body rate cannot authorize
the catch. Invalid samples clear dwell and expire an active brake candidate.

The 14 ms event qualification replaces the geometric 50 ms catch dwell only
under this new profile: its purpose is to fit inside the observed gyro departure
window. The original static CAD catch remains available as the flag-off control.
Firmware's accepted 20 ms TorqueEnable-to-speed delay is preserved. The wheel
stabilizer already computes its first bounded command in the handover tick;
this work changes the trigger, not wheel gains or protocol.

CAD still supplies the original search hip goal, existing fall checks, virtual
pendulum/LQR model and post-catch geometry. The new trigger does **not** require
the COM corridor, COM rate or proximity to the trimmed CAD hip goal. Existing
fall/stale/disable guards, eight-second stand timeout, wheel speed/saturation
envelope, post-catch governor, first 28° segment and knee trajectory remain.
This is reduced dependence at the switching point, not a CAD-free whole robot.

Framework reference: Garone, Di Cairano and Kolmanovsky, *Reference and command
governors for systems with constraints: A survey on theory and applications*,
Automatica 75 (2017), [DOI](https://doi.org/10.1016/j.automatica.2016.08.013).
Crossref title/authors/DOI were checked. The specific gyro detector is our
testable TAP hypothesis, not a port of that paper or an invariant-set proof.
No capture-point constant is derived from the unverified replacement CAD.

## Frames and reversible configuration

Body +X forward, +Y left, +Z up; positive pitch/gyro about +Y is nose-down.
The two recent trials lift with positive gyro, then tip with negative gyro.
Core hips include homeHip; LH driver sign −1, RH +1. Core knees use LK −1,
RK +1. Joint order remains LH/LK/LW/RH/RK/RW. No frame/physical parameter changes.

Core `predictive_capture` defaults false; `SET catchpredict 1` requires
`capturegov` and Idle/Fault. `SET catchpredict 0` selects the previous catch.
GUI checkbox "Bắt tâm theo IMU" selects the flag in the eight-command STANDUP
profile; toggling it/opening the GUI does not actuate anything. HOME confirmation
is still required. SET acknowledgement is short, preventing a GET dump during
the STANDUP command burst. Opening the USB port loses runtime SET overrides.

TR adds `scap` (stage enum 0–4), `scacc` (filtered gyro derivative) and `scblock`:
1 invalid/stale, 2 hip tracking/rate/pair, 4 knee motion/tracking, 8 body speed,
16 first lift incomplete, 32 waiting for lift evidence, 64 waiting for departure,
128 candidate confirmation or rejection. Buffer increases 832→896 bytes to
retain existing fields. Controller computation uses fixed scalar state and
two-side arrays, with no parsing, allocation, desktop or extra sensor. ESP32
cycle time for this profile is not measured yet.

## Evidence and limitations

- New `v3_standup_approach_tests` exercises event qualification, missing lift,
  one stuck/lagging hip, moving knee, invalid feedback, fast departure, short
  gyro pulse, candidate deadline, original first lift and V3 stop/stale handling.
- A scalar encoder-response stimulus uses a prescribed log-shaped gyro turn,
  noise, transport delays 4/20/50/100 ms plus 15 ms joint response. At 4/20/50 ms,
  command latches at 2.428 s and Caught at 2.440 s for deliberately wrong CAD
  lean 0.2/0.6/1.0 rad and invalid geometric catch rate. All nine agree; the
  three 100 ms cases fail closed at the explicit tracking limit. This is an
  input/contract test, **not a physics or successful-balance simulation**.
- V3 integration checks latched pose preserved through the actual transition,
  wheel cap and immediate DISABLE, with actual encoder-response feedback.
- Host CTest selected five targets: 5/5 passed, 121.95 s; full existing StandUp
  tests include provisional planar post-catch regressions. Final refinements
  to invalid-candidate handling/telemetry and the integration test also passed
  four focused targets, 1.81 s. Final pending-stage telemetry and distinct
  rejection reason passed those four targets again, 2.04 s. Build outputs
  `/tmp/tap-tip-host-build.out`.
- GUI py_compile and mock eight-command profiles both selected/unselected pass.
- Detector-only replay of the two robot trajectories, linearly interpolating
  25 Hz TR gyro and signed encoders to ~2 ms, reports estimated catches at
  24.676 and 65.179 s, about 0.207/0.199 s before the recorded Faults. Inputs
  use measured encoder finite differences, not absent driver velocity samples.
  Harness `/tmp/tap-tip-replay.cpp`, input `/tmp/tap-tip-replay.csv`. These are
  **counterfactual trigger timings**: latched hips and wheel arrest would change
  the trajectory, so replay cannot establish autonomous balance success.
- There is no verified replacement CAD/full shell-contact plant. An initial
  experimental COM-reference governor on the provisional free-axle plant did
  not capture those synthetic starts; it was removed when the user requested
  an IMU event mechanism. No pre-catch rigid-body success is claimed. No MuJoCo
  or new robot result exists for this revision.

Physical acceptance must mark touch/power interventions, confirm Search→Armed→
Brake→Caught on the first turn, measure command/encoder response and actual
TorqueEnable-to-wheel response, then compare repeated unassisted capture and
post-catch gyro RMS at the same initial pose. Detecting a turn does not by
itself demonstrate sufficient wheel torque, correct gain/lead or contact.

Final firmware command:
`/home/dvt/.platformio/penv/bin/pio run -e esp32-s3-v3 -e esp32-s3-v3-single-loop`.
Both succeeded (9.082/9.316 s); RAM 158540/327680 bytes, RTOS flash
1127573/6553600 bytes, single-loop flash 1126213/6553600 bytes.
Output `/tmp/tap-tip-build.out`. RTOS SHA256
`bd3c2da5a183be5c93ee54dcf967557a81e6f1895b6c80f7445d6f4a060ed67a`;
single-loop SHA256
`08440f88d25fbc4950325b1af0c5e254fb86625f374cf068a8dcde934e375035`.
`git diff --check` was clean before the initial deployment. No motor command,
commit or push was performed during that deployment.

## Deployment session — 14 October 2026, 14:28 local

- Uploaded RTOS image SHA256
  `bd3c2da5a183be5c93ee54dcf967557a81e6f1895b6c80f7445d6f4a060ed67a` to the
  ESP32-S3 at `/dev/ttyUSB0`; esptool reported `Hash of data verified` for the
  application and all data partitions, then reset the board successfully.
- Reopened the TAP serial GUI. It reports visible, connected, firmware ready,
  mode `Idle`; log: `/home/dvt/tap_logs/tap_20261014_142850.log`.
- At first inspection the log reports `can_legs=0`, `uart_wheels=0`,
  `homed=0`, and both wheel buses silent. No START/STANDUP or motor command was
  sent. Robot feedback/hardware readiness therefore remains unconfirmed.

## User-requested rollback — 14 October 2026, 14:43 local

- Removed the tipping detector, its `catchpredict` control, its GUI checkbox,
  and its telemetry. Also removed the later encoder-tracking and pose-window
  qualification that was added after the 12:44 firmware revision.
- Restored the 12:44 behavior documented in `STANDUP_CAPTURE_REVIEW_20261014.md`:
  geometric StandUp catch, reachable trimmed-goal qualification, and the earlier
  arrest/verify/adjust/observe governor with bounded progress renewal.
- The exact 12:44 binary was not retained in the workspace. The restored source
  was rebuilt, so the uploaded image is a behavioral restoration rather than a
  byte-identical copy of SHA256
  `486dcc4a9f656040686a50f3c77a5de011622723f5ec8d4af1dc3e1ebd746275`.
- Rebuilt RTOS image SHA256:
  `b4d8c10040d1bfa278fa9bc65c93ea5f36276e128620f2118239884dca1d00a6`.
  Upload succeeded with esptool `Hash of data verified` and RTS reset.
- GUI is visible and connected; current log is
  `/home/dvt/tap_logs/tap_20261014_144312.log`. Initial boot telemetry showed
  `can_legs=0`, `uart_wheels=0`, `homed=0`, and silent wheel buses. Later in the
  same session, feedback became fresh and GUI/log changed to `Balancing` with
  `can_legs=1`, `uart_wheels=1`, `homed=1`. I sent no START/STANDUP command and
  have not issued a stop or other robot command during that transition.
- Later log transitions show two StandUp attempts: 23.884→Balancing 28.839 s,
  then Fault at 69.792 s with `stand-up capture: wheel speed outside envelope`;
  and 90.617→Balancing 97.745 s, then the same Fault at 98.182 s. The GUI
  currently reports `Fault` with CAN and UART feedback present. These are
  operator-session robot outcomes, not a claim that the restored revision works.
