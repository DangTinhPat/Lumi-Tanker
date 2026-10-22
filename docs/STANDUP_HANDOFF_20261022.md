# Catching timeout: preparation before stopped handoff

Source: miniforce worktree, parent 9a2da75. No flash or robot commands in this task.

## Recorded evidence

Two successful sessions: tap_20261022_192736 and 193258. Four terminal Catching
failures: 193015, 193102, 193147, 193544. The robot reaches knee target ~0.903 rad
but cannot hold every handoff predicate for 200 ms; 193015, 193147 and 193544
explicitly end in phase timeout. 193102 recording ends near the timeout.

In the full-prediction, unsaturated, fully raised samples, wheel tracking error
exceeds 4 rad/s in 51.7–56.7% of the four failing recordings, versus 20.0–27.6%
of the successful recordings. Command acceleration lead is 75 ms; at 5 m/s²
and wheel radius 0.04 m this alone contributes 9.375 rad/s to the command.
Removing that known term for diagnosis leaves maximum errors 1.667–2.294 rad/s
in the failures. See `evidence/standup_handoff_20261022/log_stats.txt`.

This identifies the immediate transition blocker and the dominant command term.
It does **not** establish healthy motor latency, identify a hardware defect,
or show that all possible failures have one cause. In particular, session
193525 faults in Folding on the pitch limit and is a different failure.
SU hip_rate is commanded rate, not measured hip rate. Logs are ~25 Hz and
cannot reconstruct every 500 Hz gate reset. Baseline estimates use rounded TR
commands/measurements paired by log timestamp; they are not a closed-loop replay.

Reproduce with `python3 tools/standup_handoff_stats.py <logs...>`; defaults
assume this build's 75 ms lead, 40 mm radius and terminal knee >=0.902 rad.

## Change

Core-default-off `handoff_prepare` / `SET standprep 0|1` (Idle only).
Enabled at boot only for `esp32-s3-v3-1840-roll`, the current miniforce candidate.
Regular, single-loop and donor comparison profiles retain the old policy.

For the stopped, measured-braking path only, require the pre-existing raw
capture/floor/posture gates, valid clearance, calm measured body and hip rates,
completed knee rise, and both raw and filtered speed <=0.08 m/s. Ramp the
acceleration-lead multiplier from 1 to 0 over 200 ms. If calm is lost, restore
lead over the same bounded ramp. The wheel speed integral and acceleration
controller continue to operate. The full command is still checked against
both actual wheel measurements with the original 4 rad/s tolerance; require
full preparation and then the original uninterrupted 200 ms dwell. No timer
forces Balance. Rolling handoff retains its prior gates. Folding, lift,
Balance gains, hip/roll settings and frame conventions are unchanged.

Why not just subtract lead from the tracking gate: the command integral is
already clamped within a tracking margin of measured ground speed. Using that
alone would make the tracking gate much less informative. Instead this change
prepares the real command and retains the measured tracking test.

SW telemetry adds `prep` (0..1) and `lead_s`. To revert, DISABLE then
`SET standprep 0`; resetting the ESP restores the build's boot default.

## Validation and limits

- Six focused CTests pass: v3_standup_tests, v3_addwifi_balance_tests,
  motor_feedback_tests, hop_sequence_tests, motion_profile_tests,
  plan_scurve_tests. The new stand-up regression prescribes noisy captured
  motion: disabled policy stays in Catching; enabled policy completes preparation
  and the unchanged dwell. Bad wheel tracking, clearance, body/hip rates,
  rolling speed and capture lean still block handoff. A separate test verifies
  knee-rise progress is not deadlocked by preparation and loss of calm restores
  the original lead. These are synthetic contracts, not robot capability tests.
- ESP32 roll candidate, regular and single-loop builds pass. Candidate RAM 166296 bytes,
  flash 1505381 bytes. Candidate binary SHA256:
  `9eaef1d74ab4f0925d287c7acf2394384c7a03c75ecda6e0c3470c9f7a19dd9a`.
- `TIER_ADDWIFI_COMPARE=1 TIER_ADDWIFI_PROFILE=0 .../v3_tier_tests`:
  24/24 released balance/height planar cases do not fall. This diagnostic does
  not run the floor stand-up sequence or validate the new preparation policy.
- Physical effect of reducing lead during the final stopped Catching window,
  command roughness and repeated stand-up success remain pending robot logs.
  There is no new dynamic allocation, history buffer or unbounded loop in the
  control path; target worst-case timing has not been measured after this edit.

Build: `/home/dvt/.platformio/penv/bin/platformio run -e esp32-s3-v3-1840-roll
-e esp32-s3-v3-single-loop`. Host build directory `/tmp/tap-1840-addwifi-host`.
Deployment follow-up: the stopped-handoff candidate was OTA-flashed and
`standprep=1` verified. The subsequent Wi-Fi diagnostic build was also
OTA-flashed; its SHA256 is
`0e11ef0431fa74139cab807f60dc0b44197cc6f7cfc9d83342b2cdd09dfcdaa8`.
The patched GUI runs from the miniforce worktree. No autonomous motion test
was commanded by the agent; repeated physical stand-up success is not certified.
