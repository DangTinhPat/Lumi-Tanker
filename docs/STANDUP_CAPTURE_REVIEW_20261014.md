# Capture review after the 12:22 robot session

Branch `fw`, uncommitted working tree. Source log:
`/home/dvt/tap_logs/tap_20261014_122220.log`, snapshot through 194.419 s
(last DISABLE; later Idle telemetry excluded). Deployed artifact during these
trials: RTOS SHA256
`a4828d0b26ab9e869639b43b4624ccc4fd627a3c31367629c9282c9960e55c80`.
The corrections described below were flashed at the user's request at 12:44
on 2026-10-14. Their physical balance performance is **not robot-validated**.

## Robot evidence

| Attempt | StandUp start | Wheel balance start | End | Result |
|---|---:|---:|---:|---|
| 1 | 45.181 s | 51.874 s | 81.879 s | Governor deadline; no Normal |
| 2 | 119.209 s | none | 127.209 s | StandUp timeout at 8 s |
| 3 | 148.573 s | 154.513 s | 184.517 s | Governor deadline; no Normal |

| Window after catch | Attempt 1 | Attempt 3 |
|---|---:|---:|
| Body-rate RMS, first 2 s | 0.974 rad/s | 0.751 rad/s |
| Body-rate RMS, 2–10 s | 0.948 rad/s | 0.604 rad/s |
| Sent wheel-speed RMS, 2–10 s (mean pair) | 6.42 rad/s | 4.26 rad/s |
| Measured wheel-speed RMS, 2–10 s (mean pair) | 2.80 rad/s | 2.01 rad/s |
| Hip command during 2–10 s | fixed 0.5786 rad | fixed 0.5757 rad |
| Hip command at start/end of 10–30 s | 0.5786 → 0.3986 rad | 0.5757 → 0.4407 rad |

CAN-leg/UART-wheel freshness flags stayed 1 in all 401 sampled SENS records
across these attempts. Minimum bus-voltage reports ranged 23.90–24.15 V.
This does not exclude events between samples or identify available motor torque.
WBUS samples around the first capture showed drvmode=1, sync=1, no driver error
or write timeout, and feedback age 0–1 ms. The driver interface was not changed.

For windows [52,61.87] and [154.6,164.5] s, dominant gyro frequency from a
Hann-windowed FFT was 2.601/2.801 Hz. Least-squares sine/cosine fits at those
frequencies (with offset and linear trend) gave sent/measured speed amplitudes
8.01/3.29 and 4.52/1.81 rad/s: gain 0.411/0.402, phase −97.8°/−100.3°.
These are **closed-loop command/feedback observations**, confounding servo
lag, measurement filtering, load and possible manual intervention. They do not
separately identify physical transport delay, actuator time constant or Kt.
The lead still comes from the previous 0.05–0.09 s schedule. Blindly increasing
gain/lead from this spectral fit is not a validated fix.

`hf_vibration_stats.py` on fixed-hip windows [53.874,61.874] and
[156.513,164.513] s: gyro >5 Hz RMS 0.217/0.148 rad/s; commanded hip >5 Hz
content 0. Body/wheel oscillation therefore exists before posture movement.
Hip variance attribution returns NaN because that command is constant; it must
not be interpreted as an identified hip feedback contribution. TR is only
25 Hz; faster motion can alias.

Figure: `/home/dvt/tap_logs/reports/capture_20261014_122220.png` plots the
first fixed-hip oscillation window; motor velocity still follows the previous
command direction after the sent command has reversed.

## Deterministic defects and corrections

### Reachable catch target

The bisection solves zero CAD lean, then subtracts the built-in 0.0396 rad hip
trim and extra 1° fold. The resulting delivered lean is about −0.053 rad,
while `standextra` required |lean| <0.025 rad around zero. A tracking robot
can sit at its requested goal and never qualify; successful attempts depended
on a subsequent disturbance entering the narrow band.

With `capturegov` enabled, StandUp now accepts either the original axle band or
the model lean at the actual trimmed goal. The goal residual must fit the same
inner band; absolute lean must still remain inside the configured outer
0.0654 rad corridor. Both body/COM rates remain <0.7 rad/s for ≥0.05 s;
the hip-goal proximity check remains. The legacy flag-off path is unchanged.
This aligns the handshake with its selected geometry; quiet is not proof of
floor contact. No CAD constants, wheel gains or first-lift 28° trajectory changed.
New trace fields `scphase`, `sclean`, `sctgt` expose phase and target consistency.

### Observation and progress budget

Observe already requires 0.5 s of uninterrupted quiet and a completed response
interval. The old path discarded that qualification and spent another 0.5 s
in Verify before each 0.015 rad step. The new path retains it; any subsequent
disturbance still clears qualification before another step can start.

The original 30 s budget remains for unqualified/stalled episodes. A quiet,
observed reduction of body error ≥0.01 rad may renew 3 s of progress time, with
an absolute 45 s ceiling. Wrong response, travel cap, speed/rate/lean limits,
step 0.015 rad, rate 0.04 rad/s and acceleration 0.20 rad/s² remain unchanged.
Command motion alone never renews the deadline. This prevents a healthy late
recovery from being discarded while still bounding endless oscillation.

## Verification

- Model-derived synthetic stalled goal: flag-off times out at 8.002 s with
  lean −0.0529; new path catches at 0.348 s with lean −0.0570. Independent cases
  with excessive body/COM rates or lean outside the outer corridor stay off.
  This verifies transition logic, not physical balance or support.
- Synthetic 10 s oscillatory arrest followed by delayed, healthy body response:
  original duplicate-dwell/fixed-deadline control fails at 29.994 s with body
  error −0.1650 rad. New path reaches Normal at 31.534 s, body −0.0706 rad.
  Steps/rates remain bounded. Noisy nonresponse does not renew the deadline;
  persistent improvement cannot exceed the absolute ceiling.
- Selected host contracts/regressions: `v3_capture_governor_tests`, `v3_tests`,
  `motor_feedback_tests`, 3/3 passed (1.72 s).
- Full `v3_standup_tests` passed (122.95 s), including flag-off controls,
  reachable-target checks, existing safety/trajectory contracts and diagnostics.
  Passing the executable does not turn printed diagnostic failures into success.
- Planar V3 diagnostics at deployed 0.09 s lead: 6/9 cases reached Normal
  (wheel lag 130/200 ms × hip delay 4/20/50 ms), body-rate RMS after 2 s
  0.013–0.014 rad/s, final body about −0.067 rad. Three 35 ms wheel-lag cases
  still timed out at 30 s, RMS about 0.173 rad/s. The diagnostic now runs
  through 46 s so renewed progress deadlines are not hidden. These use
  provisional geometry and ideal sensors; none certifies the real motor.
- Firmware RTOS and single-loop builds passed. RTOS RAM 158244/327680 bytes,
  flash 1123037/6553600 bytes; single-loop flash 1122237/6553600 bytes.
- Revised RTOS artifact SHA256:
  `486dcc4a9f656040686a50f3c77a5de011622723f5ec8d4af1dc3e1ebd746275`.
  This is the artifact deployed in the subsequent operator-requested upload.

Remaining issue: the fixed-pose wheel loop visibly oscillates in the robot
logs. These handshake/progress corrections do not establish that it is stable.
Physical actuator identification and comparative operator-recorded MIN trials
remain required. Source/test review did not operate the hardware. The subsequent
user-requested deployment is recorded below; no commit or push was performed.

## User-requested deployment (2026-10-14 12:44)

RTOS upload succeeded in 22.394 s with `Hash of data verified`; SHA256 matches
the revised artifact above. The old GUI session had no live members before
reopening. New GUI PID/session 31422 is the sole owner of `/dev/ttyUSB0`,
connected/firmware ready, mode Idle. Log:
`/home/dvt/tap_logs/tap_20261014_124455.log`.
CAN-leg/UART-wheel feedback were absent at opening. No START/STANDUP or HOME
confirmation was sent automatically. GUI STANDUP retains the seven-command
`capturegov` profile. Physical validation remains pending.
