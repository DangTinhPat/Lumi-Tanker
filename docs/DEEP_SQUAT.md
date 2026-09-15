# Deep Squat Validation

## Coordinate Limits

The supplier confirmed that 30..145 degrees are encoder readings, not the
included angle between links. Total encoder travel is 115 degrees (2.00713 rad).
The encoder reading at CAD joint zero and the encoder/native sign relationship
are not yet calibrated. These readings therefore do not define native URDF
endpoints. No profile here is a hardware travel-limit calibration.

The existing pose curves stop at native knee -0.8 rad and common hip +0.54280
rad. The normal controller caps hip at +0.6 rad. URDF limits of +/-1.57 hip
and +/-2.1 knee are provisional simulator limits, not measured motor travel.

## Geometry Before Trials

Read-only CAD analysis found wheel/thigh mesh overlap near native knee -1.9 rad
and beyond; primitive simulation collisions do not represent this interference.
Do not interpret an uncollided primitive model at -2.1 as physical feasibility.
At -1.8, sampled wheel/thigh separation is about 7.8 mm, not a complete collision
certificate. Common-hip neutral roots must follow the branch connected to rest.

| Knee, rad | Neutral hip, rad | Body height, m | Leg-floor clearance, m |
| --- | --- | --- | --- |
| -0.8 | 0.543 | 0.197 | 0.0204 |
| -1.0 | 0.650 | 0.183 | 0.0194 |
| -1.2 | 0.758 | 0.167 | 0.0185 |
| -1.4 | 0.868 | 0.149 | 0.0176 |
| -1.6 | 0.979 | 0.131 | 0.0169 |
| -1.8 | 1.094 | 0.111 | 0.0164 |

Below about knee -1.4517 the estimated effective pendulum length reaches the
current MPC minimum of 0.15 m. Body height passes below the existing 0.14 m QA
threshold around knee -1.5037. Larger angles require a fresh model/guard review,
not just wider pose sliders. Masses, inertia and CAD assembly remain unchanged.

## Candidate And Acceptance

The first opt-in candidate used hip maximum +0.8, knee minimum -1.2 and
station keeping. It did not change motor torque, damping, sensor timeout,
fall limits or MPC gains. The final profile is recorded below.

Test symmetric squat alone, from neutral through descent, dwell, ascent and
neutral dwell for two complete cycles. Use 8 s quintic transitions for knees
up to -1.2, keeping nominal peak speed below the GUI's 0.3 rad/s limit.
Retain peak XY 40 mm, return residual 15 mm, yaw 0.06 rad, pitch-reference error
0.12 rad, body height at least 0.14 m and settled joint error at most 0.08 rad.
Passing a descent without returning to neutral is not a pass. The existing
25% Show workflow is unchanged.

## Predeclared Deeper Candidates

Only proceed after the preceding depth passes. A geometry audit with hip held
at the neutral command, bilateral knee sag of -0.08 rad and absolute pitch
swept through +/-0.12 rad gives minimum body heights of 0.1404 m at target
-1.4 and 0.1218 m at target -1.6. Effective MPC lengths under this sag estimate
are 0.1470 and 0.1298 m. Sampled wheel/thigh gaps are 38.6 and 19.7 mm;
no contained mesh vertices were detected, but this is not a collision certificate.

Before any trial at those depths, choose:

| Target knee | Hip cap | MPC minimum length | QA minimum body height | Absolute pitch cap |
| --- | --- | --- | --- | --- |
| -1.4 | 1.0 | 0.14 m | 0.14 m | 0.12 rad |
| -1.6 | 1.05 | 0.125 m | 0.115 m | 0.12 rad |

Use 11 s quintic transitions, 3 s dwells and two complete cycles. Other QA
bounds remain as above, including 0.08 rad settled joint error. Preserve the
controller's 0.10 m pose-based contact threshold and 3 Nm command limit.
The lower MPC bound follows the shorter modeled pendulum; the lower body-height
guard is declared from geometry before testing, not adjusted after a failure.
At -1.7 with the same sag, sampled wheel/thigh gap is only 9.8 mm and the body
is approximately 0.112 m above ground; defer that depth. The geometry audit is
reproducible with `tools/audit_squat_geometry.py` and its regression tests.

## Tested Profile

`src/tap_control/config/deep_squat.yaml` now selects native knee -1.6 rad,
hip cap +1.05 rad, station feedback on and MPC minimum length 0.125 m.
The neutral endpoint hip is approximately +0.97921 rad. This is the deepest
released simulation envelope, not a claim of calibrated maximum encoder travel.
No CAD, mass, inertia, joint damping, torque or contact-threshold changes were made.

Staged squat-only trials completed two down/up cycles at each requested depth:

| Native knee depth | Log directory | Result |
| --- | --- | --- |
| 0.4 rad | `transformation_1789198550036040328` | Pass |
| 0.6 rad | `transformation_1789198641098862173` | Pass |
| 0.8 rad | `transformation_1789198735383437158` | Pass |
| 1.0 rad | `transformation_1789199071073370731` | Pass |
| 1.2 rad | `transformation_1789199161677314418` | Pass |
| 1.4 rad | `transformation_1789199306603045832` | Pass |
| 1.6 rad | `transformation_1789199429460244843` | Pass |

Directories are under `log/`; each preserves effective config, curve and binary
provenance. The preliminary 1.6 trial had a single 10 ms, 3 Nm saturation on
return, none during deep dwell. Measured knees reached about -1.6657 rad,
including approximately 0.0657 rad load sag; measured hips reached 0.9791 rad.

Final regression: `log/transformation_1789199824809555575`, source profile above,
11 s quintic transitions, 3 s dwells, 20 Hz commands, two cycles (62.004 s).
Controller SHA256:
`2560b042dfc6d35ba672e3a46dfbf74b2fe39fb4cda22bb226a1f55f5dd3a8bf`.

- Peak XY displacement 9.320 mm; final residual 0.0455 mm.
- Minimum body height 0.12465 m, versus approximately 0.236 m upright.
- Peak absolute pitch 0.03579 rad; peak reference error 0.00335 rad.
- Peak joint tracking error 0.07726 rad; final error 0.04079 rad.
- Peak leg command 2.895 Nm, below the unchanged 3 Nm bound.
- Final e-stop zero-command verification passed; no in-motion pose resets.

An independent final-binary repeat in `log/transformation_1789199925259160136`
also passed 62.004 s: peak XY 9.391 mm, final residual 0.00465 mm, minimum body
height 0.124649 m and peak leg command 2.903 Nm. Neither final-binary run
saturated the leg command bound. Capability telemetry confirmed hip cap 1.05
and knee minimum -1.6 throughout the trials.

Build and all nine CTest suites passed, plus 27 independent model/harness tests.
Deep controller checks: 29; GUI callback checks: 37. The final binary also
passed 29 general and 30 station ROS contracts. Actual Tk controls were tested
with isolated synthetic feedback at 650x660; screenshots live under
`log/deep_gui_final_16_20260912_145737/`. GUI rendering/callback evidence is
separate from the physical Gazebo trace above.

## Running

From the workspace root, with no other simulation in the same ROS domain:

```bash
./run_sim.sh controller_config:="$PWD/src/tap_control/config/deep_squat.yaml"
ros2 run gui gui --deep-squat-knee 1.6
```

After Enable and neutral feedback, select Experimental motions and Deep squat.
Amplitude 1 commands the tested endpoint. Hip and knee follow one coordinated
quintic curve in simulation time; full travel takes at least 11 s. Hold squat
holds the last commanded pose while balance continues. Neutral pose returns
along the same curve before handing back to normal controls. Workflow switches
also require the coordinated return; manual drive and stand-up cannot take over
a deep pose. Fault/clock interruptions hold the trajectory without auto-resume.

Deep mode verifies the reported controller pose bounds and fresh station/body
feedback. It holds for absolute pitch over 0.12 rad, body height below 0.115 m,
or joint tracking error over 0.15 rad sustained for 0.3 simulated seconds.
This does not replace the controller watchdog or physical contact sensing.
The CLI rejects depths beyond 1.6; the optional geometric generator is not a
hardware motion API. Real encoder calibration and larger asymmetric poses
remain outside the tested scope.

The user desktop was reopened with this profile and GUI argument, initially
paused. Readiness is recorded in `log/desktop_deep_sim.log`; the visible robot
and Deep squat/Hold squat controls were inspected in `log/desktop_deep_open.png`.
