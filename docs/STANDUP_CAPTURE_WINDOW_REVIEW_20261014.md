# Capture qualification review — 14 October 2026

Branch `fw`. This revision is built and host-tested, **flashed at the user's
subsequent request at 13:30; physical performance is not robot-validated**.
The robot-log evidence below belongs to the previous
`486dcc4a9f656040686a50f3c77a5de011622723f5ec8d4af1dc3e1ebd746275`
prototype. No motor commands, serial-port reopening, flash, commit or push
were performed during the analysis task. The subsequent authorized deployment
is recorded at the end of this document.

## Evidence and intervention boundary

Sources: `~/tap_logs/tap_20261014_124455.log` and
`~/tap_logs/tap_20261014_125431.log`. Sanitized snapshots remove NUL/invalid
bytes; analysis uses TR at about 25 Hz, SENS and V3 fault messages. Later
Fault telemetry was checked and contains no subsequent StandUp attempt.
The operator confirmed intervening by switching motor power and/or holding
or pushing the robot. Individual intervention timestamps were not identified.
These are observed outcomes, **not an autonomous success-rate experiment**.

| Session | StandUp starts [s] | Balance starts [s] | Normal [s] | End/reason [s] |
| --- | ---: | ---: | ---: | --- |
| 124455 | 14.975 | 17.947 | 40.170 | 287.094 wheel feedback lost |
| 124455 | 315.915 | 318.891 | 343.550 | 357.615 wheel feedback lost |
| 124455 | 405.969 | 412.901 | — | 414.826 wheel feedback lost |
| 124455 | 440.492 | 447.512 | — | 450.558 governor response fault |
| 124455 | 498.775 | 503.705 | — | 516.108 wheel feedback lost |
| 125431 | 28.500 | 33.654 | 55.023 | 96.813 operator DISABLE |
| 125431 | 132.064 | 134.905 | — | 153.601 wheel feedback lost |
| 125431 | 188.920 | 191.965 | — | 196.917 governor response fault |
| 125431 | 241.827 | 247.054 | — | 270.827 wheel envelope fault |

Two additional requests at 390.180/394.296 s in session 124455 were refused
because sensors were not ready; they never ran the trajectory.
Three of the nine executed episodes reached Normal. Five ultimately ended
with wheel feedback loss, including two that had already reached Normal.
The fault diagnostics were `imu=1 wheels=0 legs=1 ... axis=ok` at the fault
tick. `Iq=0` is expected in this POS-only profile, not the fault cause.

Four wheel-loss events show supply decay in surrounding SENS samples:
23.92→6.54 V, 16.28→9.35 V, 23.91→6.29 V and 16.72→6.63 V.
The 516.108 s event has only 23.40/23.31 V nearby bus readings and does not
establish a power-off cause by itself. Those voltages are leg-driver reports,
not an independent wheel-supply measurement; the stale wheel records cannot
identify the lost supply or physical disconnect. Power/intervention events
must not be scored as spontaneous controller instability.

The Normal intervals lasted approximately 247, 14 and 42 s before those
endpoints. Body-rate RMS in those intervals was 0.070, 0.058 and 0.078 rad/s.
The longest interval's body pitch ranged −0.093…−0.056 rad. Static final hip
commands were approximately 0.346, 0.341 and 0.333 rad. These show that the
previous supervisor can sometimes reach a quiet pose; they do not establish
repeatable autonomous capture, particularly with unmarked hand intervention.

## Confirmed software holes

### Endpoint pitch mistaken for a sustained wrong response

The previous governor captured `abs(body_pitch)` at Verify→Adjust and compared
one body-pitch sample when Observe's 0.5 s interval expired. The threshold was
only 0.01 rad. That endpoint can be an extremum of a small oscillation while
the mean pose remains unchanged or improves.

| Fault | Pre-step mean, last 0.5 s [rad] | Observe mean [rad] | Last pre-fault pitch [rad] | Body-rate RMS before/after [rad/s] |
| --- | ---: | ---: | ---: | --- |
| 124455 / 450.558 s | −0.35333 | −0.35525 | −0.375 | 0.043 / 0.137 |
| 125431 / 196.917 s | −0.25675 | −0.24708 | −0.268 | 0.041 / 0.152 |

The first mean worsened by only 0.00192 rad; the second improved by
0.00967 rad. Neither window establishes a sustained mean-error increase
>0.01 rad. Both show increased rate energy, which is a reason to **hold and
observe**, not to declare an instantaneous wrong hip response or authorize
another step. Exact 500 Hz latched pitch samples are absent from 25 Hz TR;
the table is a sampled-window comparison, not a reconstruction of those ticks.
Feedback was fresh, driver diagnostics healthy, and bus reports about
23.91–23.92 V in both windows.

Plot: `~/tap_logs/reports/capture_response_20261014.png`. The red line uses
the sampled pre-step mean minus 0.01 rad as a reference; the exact old latched
threshold is unavailable.

### Command completion mistaken for mechanical completion

Observe started when the internally generated hip command reached its endpoint.
There was no per-side encoder tracking/stationarity condition. The mean of
two hips also hides an opposing error or one lagging side. A 0.015 rad step
is comparable with delivered-motion delay/backlash, so observing immediately
at command completion can measure an unfinished pose.

The revision passes both hip positions and rates in the existing core frame
from V3Controller through PostStandUpBalance. No sign/zero conversion is
introduced. Each encoder must be within 0.008 rad of the command, each rate
<=0.06 rad/s, and each position span <=0.004 rad over the complete 0.5 s
qualification. Rate alone is insufficient: slow drift can fit the noise
allowance. The rate floor is provisional; logged held-pose SENS hip-rate RMS
medians were about 0.020–0.023 rad/s and maxima 0.033–0.048 rad/s in the
longer Normal intervals. SENS RMS does not specify a per-sample noise bound.

### Absolute quiet limits permit progress while energy grows

At 267–270 s in session 125431 the body-rate envelope grows from roughly
0.05 rad/s to >2 rad/s. A new adjustment begins around 267.925 s while energy
is already increasing, but still below the 0.20 rad/s absolute quiet ceiling.
Hip command is static at 0.3565 rad after about 268.504 s, yet oscillation
continues growing until the existing wheel-speed envelope faults. Voltage
remains about 23.71–23.82 V and wheel feedback fresh.

The operator's intervention means this is not proof of a self-excited wheel
mode or a hip-caused instability. It nevertheless exposes a supervisor hole:
absolute thresholds do not require retention of the pre-step stability level.
The revision pauses Adjust/Observe when EWMA body/virtual-rate RMS exceeds
max(0.10 rad/s, 1.5×the qualified pre-step window RMS). A pause holds the
current hip command; it never changes wheel gains or reverses repeatedly.
This cannot stabilize an intrinsically unstable fixed-pose wheel loop.

## Final behavior and limits

Before/after body errors and progress use time-weighted pose-window means.
A stationary mean-error increase >0.01 rad still fails as Response. A pitch
span >0.03 rad or endpoint >0.01 rad from its window mean instead discards
qualification and waits at the same pose. Disturbance, held indication,
missing/trailing hip feedback or excessive energy clears the pose window.
The next step requires a complete qualified window at the delivered pose.
Normal requires every body-error sample in that window within ±0.08 rad;
a brief crossing into tolerance no longer completes transfer.

`cgrms` and new `cgblock` continue updating in Normal while the accepted leg
pose stays fixed. Previously Normal returned before updating RMS, so its
telemetry could appear quiet during a later disturbance. Existing V3 stop,
fall, stale-feedback and wheel-envelope guards remain responsible for safety.
`cgblock` bits: 1 motion/lean/speed, 2 held, 4 hip feedback/tracking,
8 hip moving/drifting, 16 rate-energy growth, 32 nonstationary pitch window.

The 0.015 rad step, 0.04 rad/s rate, 0.20 rad/s² acceleration, 0.35 rad travel,
30 s initial / 45 s absolute deadline and progress-only extension stay in place.
The first-rise 28° trajectory, knee trajectory, accepted half-duplex wheel
protocol, wheel gains/lead and CAD parameters were not changed by this revision.
All changes stay behind existing core-default-off `capturegov`; flag-off
PostStandUpBalance behavior is unaffected by the new encoder fields.

The control path uses fixed-size scalar/window statistics and two-side arrays,
with no allocation, parsing, desktop dependency or extra sensor. Its work is
bounded per tick. ESP32 timing of this revision remains unmeasured because it
was not deployed in this task.

## Verification

- Host build completed for the governor, stand-up, V3 and feedback targets.
- Selected CTest suite: 4/4 passed, 134.95 s (`v3_capture_governor_tests`,
  `v3_standup_tests`, `v3_tests`, `motor_feedback_tests`). The final refinement
  of the sample-comparator control case was rebuilt/retested: 1/1, 0.02 s.
- New contracts cover logged endpoint-excursion shapes, a single-sample control
  that rejects those tails, resumed qualification after a quiet response,
  missing feedback, opposing per-side errors/rates, a stalled hip, slow pose
  drift, energy growth below old absolute limits, and tolerance crossings.
  Persistent wrong response/travel/timeout/invalid-input contracts still pass.
  Normal updates RMS without moving the accepted pose. Sampled log shapes
  are resampled with synthetic hip tracking: supervisor tests, **not a full
  closed-loop robot replay**.
- Planar V3 run with deployed 0.09 s lead:
  `v3_standup_tests 0.09 capture`. All nine diagnostic contracts completed:
  six 130/200 ms wheel-lag cases × 4/20/50 ms hip delay reached Normal,
  post-2 s rate RMS 0.013–0.014 rad/s, final body about −0.067 rad. Three
  35 ms wheel-lag cases still timed out at about 30 s, RMS 0.173–0.174 rad/s.
  Expected bounded failures are reported as failures to achieve balance,
  even though the diagnostic contract passes. Provisional plant geometry,
  ideal sensor signals and those lag values are not calibrated to the new motor.
- RTOS and single-loop PlatformIO builds passed. Both use 158308/327680 bytes
  RAM; flash 1125045 and 1123917 / 6553600 bytes respectively.
- RTOS SHA256:
  `6886d6e4d06eb22bdeb39c695dc42e3b85c12fa1316c67e4906a78bfbcedc2d2`.
  Single-loop SHA256:
  `fd25e8f9762ae416a6a822c1e78c3e125fc66bc96c2c94537434472db801ff30`.
- `git diff --check` passed. No MuJoCo/model verification was claimed.

Next evidence needed: comparative MIN trials with power left on and intervention
times marked, per-side hip tracking and `cgblock` logged, plus command/encoder
response of the new wheels at a fixed pose. Transport freshness is not motor
servo lag. The old 0.05–0.09 s lead schedule was derived for different observed
wheel dynamics; its stability margin on the new wheels is still unresolved.
Gain/lead changes should follow actuator identification and delay/noise sweeps,
not a stronger command chosen from a manually disturbed failure window.

## User-requested deployment — 13:30

- User explicitly requested flashing and reopening the GUI after this review.
- Old owned GUI session 32016 was stopped using verified PID/session/start-time
  metadata and `owned_process` identity helpers. No live session members remained;
  the USB port was confirmed free before flashing. No other TAP runner/dashboard
  or vision session was found in the process inventory.
- `pio run -e esp32-s3-v3 -t upload --upload-port <CP2102 by-id path>` succeeded
  in 22.368 s. Esptool reported `Hash of data verified` and reset the board.
  RTOS SHA256 remains `6886d6e4d06eb22bdeb39c695dc42e3b85c12fa1316c67e4906a78bfbcedc2d2`.
- New visible GUI PID/session 35019 is the sole `/dev/ttyUSB0` owner. Status:
  connected, firmware ready, Idle. Boot reports TWAI PASS, UART READY, MPU6500
  PASS and `rtos=1`. Runtime SET overrides were reset; GUI STANDUP selects the
  existing seven-command `capturegov` profile when the operator presses it.
- New log: `~/tap_logs/tap_20261014_133001.log`. At opening, CAN-leg and
  UART-wheel feedback were absent (`can_legs=0 uart_wheels=0`, voltage NaN).
  Only the GUI's read-only GET was sent; no START/STANDUP/HOME confirmation
  or motor actuation was initiated. Comparative physical testing remains pending.
