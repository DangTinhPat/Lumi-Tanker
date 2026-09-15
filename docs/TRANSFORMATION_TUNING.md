# Transformation Tuning

## Requested Behavior

Perform joint transformations while maintaining balance and limiting drift.
Validation is simulation-only; retain existing stop and sensor watchdogs.

## Baseline Findings

- Position anchoring is disabled in the current MPC configuration.
- The station-hold gate requires quiet bent knees and a dwell, excluding active
  transformations and small asymmetric poses.
- The previous moderate roll trial remained balanced but translated about
  0.066 m and changed heading unintentionally. Balance alone is insufficient.

## Acceptance Plan

Measure world-frame position, heading, body attitude, measured joint tracking
and actuator commands through repeated cycles, including the return to neutral.
Compare peak displacement, residual displacement and cumulative drift against
the unchanged baseline. Establish a small verified envelope before increasing
amplitude or speed. Do not claim zero drift from wheel odometry alone, suppress
fall detection, reset simulator poses during a performance, or weaken watchdogs.

## Implemented Controls

- `StationKeeper` is an opt-in, simulation-only world-position/heading outer
  loop supplying bounded velocity corrections to the existing MPC. Raw operator
  drive commands remain separate and move the anchor; corrections do not.
- Forward error is projected onto the current heading. Pure lateral error is
  reported but cannot be cancelled by a fictitious sideways wheel command.
- Stop, reset, Fallen and latched sensor loss clear the anchor. Transient sensor
  loss suppresses correction while preserving the anchor; operator motion during
  that gap requests fresh capture. Both cases passed final-binary ROS contracts.
- `ShowSequence` provides a finite neutral/squat/neutral/left/neutral/right/
  neutral sequence using quintic curve amplitude, explicit stop/resume and no
  automatic looping. Geometry tests are distinct from physical validation.
- `sim.launch.py` accepts `controller_config` overrides layered over the default
  configuration. `config/stationary_show.yaml` enables the outer loop without
  duplicating the base controller parameters. Normal startup remains opt-out.

## Gazebo Evidence

Independent fixed-origin measurements use `tools/transformation_smoke.py`.
Acceptance bounds: peak XY 40 mm, residual 15 mm, peak heading error 0.06 rad,
pitch-reference error 0.12 rad, roll 0.3 rad, body height at least 0.14 m and
settled joint error at most 0.08 rad. No simulation pose resets during a show.

On binary `91c449c4b8ecf3448a3bb90ada59b9328d57c49f84a968ffdde32f78d346736b`:

| Trial | Outcome | Peak XY | Final XY | Peak heading |
| --- | --- | --- | --- | --- |
| 25%, 3 s transitions, 2 cycles, station off | Fail | 287.18 mm | 287.18 mm | 0.08460 rad |
| 25%, 3 s transitions, 2 cycles, station on | Pass | 25.63 mm | 1.07 mm | 0.03612 rad |
| 50%, same timing and loop | Fail | 462.12 mm | 352.73 mm | 0.04364 rad |

Artifacts: `log/transformation_1789196622415082295` (station off),
`log/transformation_1789196286118087091` and
`log/transformation_1789196393455153359`. All collected 64 simulated seconds
and passed final e-stop zero-command checks. The 50% failure reached the 3 Nm
leg command bound with peak joint tracking error 1.004 rad. Remaining upright
was not sufficient to pass. The original 25% station-off run also exceeded
position/heading limits, but ended early on timestamp incoherence and is not
a complete paired baseline (`log/transformation_1789196024224109038`); the
corrected-harness station-off run in the table supersedes it for comparison.

### Final Binary

`log/transformation_1789196940027971572` passed two complete cycles, 64.004 s,
with the installed `stationary_show.yaml` profile layered over the base config.
This trial publishes command pairs at 20 Hz simulated time with no catch-up
bursts; the earlier A/B trials above published at the measurement-loop rate.
The final controller SHA256 is
`6b74835be5b2d9a1a66b154f2901e1c79bb2582cc6816ba29418b464e2a85655`.

- Peak XY displacement: 25.743 mm; final residual: 0.786 mm.
- Peak heading error: 0.03632 rad; final heading residual: 0.0000645 rad.
- Peak pitch-reference error: 0.00607 rad; minimum body height: 0.22811 m.
- Peak joint tracking error: 0.07455 rad; final error: 0.04079 rad.
- Peak leg command: 2.679 Nm. Final e-stop zero-command check passed.
- Thresholds remained unchanged. No in-show world-pose resets were used.

Build and all seven registered CTest suites passed. Final-binary synthetic ROS
verification passed 29 general contracts and 30 station-specific contracts,
including nonfinite world XY input, transient stale capture, operator override,
latched stale stop and no automatic re-enable. Core checks: 58 existing and 51
StationKeeper checks. The 16 independent model tests also passed.

Actual Tk Show/Stop buttons were tested using synthetic feedback on isolated
DDS domain 217, not the live robot domain. Screenshots are under
`log/dashboard_isolated_render_current/`. This verifies rendering and GUI
callbacks, separately from the headless Gazebo physical trial above.
The final GUI callback suite has 22 checks, including forward clock gaps over
0.3 s, backward time, stale feedback, operator intervention and held-pose stop.
Ten sequence tests cover finite completion and current-URDF geometry/speed.

### Larger Squat Candidate

`log/transformation_1789197302457610877` tested squat 50%, rolls 25%, 5 s
transitions, 20 Hz commands and the final controller. It completed one 44 s
cycle with 0.433 mm cumulative residual, then stopped in cycle two on pose
age 0.153 s, exceeding the unchanged 0.150 s freshness limit. Collected show
time was 67.375 s; no posture/position bound violations had been recorded,
and final e-stop zeroing passed. This is an INCOMPLETE two-cycle qualification,
not a release pass. The GUI therefore stays at 25% for every motion.

Earlier 5 s-transition trials stopped near their first cycle boundary while
the harness computed full trace summaries inside its subscription loop. Those
computations now run only after shutdown to avoid starving sensor callbacks.
The later failure above happened beyond that boundary; it is not erased or
relabelled a pass by the harness correction. No freshness thresholds were widened.

These results do not certify deep squat, larger asymmetric poses, jumping,
rough terrain or hardware. See `WHEEL_KINEMATICS.md` for wheel/leg coupling and hop feasibility:
the current 3 Nm limits, 1 Nms/rad joint damping and pose slew do not support
the analyzed light hop, and pose-based contact inference cannot detect flight.
