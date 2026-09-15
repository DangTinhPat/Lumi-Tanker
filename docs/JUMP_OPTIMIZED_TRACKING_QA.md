# Optimized Jump Tracking: Development Evidence

2026-09-12, `/home/dvt/tap`. Simulation only. **Not an accepted jump skill.**
Neither >=10 cm conservative measured rise nor stable low-drift landing has
been demonstrated by this implementation. Default precision-pose and GUI
profiles are unchanged; the new profiles require explicit experimental opt-in.

## Implementation

- `tools/jump_dynamics.py`: ADAM 0.5.0 URDF dynamics, symmetric sagittal
  coordinates `[body_x, body_z, pitch, hip, knee, wheel]`; native +Y both sides.
- `tools/optimize_jump.py`: offline CasADi/IPOPT midpoint trajectory optimization
  with torque-speed, rolling contact, clearance, position and rate constraints.
- `tools/jump_lqr.py`: offline time-varying LQR, including the local 0.6 Nm per
  rad/s leg velocity loop. Runtime does not run ADAM, IPOPT, CasADi or SciPy.
- `JumpPlan.hpp`: fixed-capacity Hermite sampler, analytical bounds validation,
  immutable reference and gain data. Runtime feedback uses 36 multiply-adds
  plus interpolation and filtering. No allocation in the tested update path.
- Optional planned `JumpController` mode, explicit ownership of leg and wheel
  outputs, tighter stationary admission, contact-event guards and flight clock
  retiming. Stop, stale-input, pitch and measured joint-travel guards remain.

The runtime currently uses Gazebo body X/Z and finite-difference velocities.
It is **not an ESP32 estimator or hardware-ready controller**. The reduced
planner does not model asymmetric 3D impacts, tire compliance or roll control.
Both wheels remain capped at 0.13 Nm and legs at 5 Nm. Trials use the experimental
190-to-270 rpm torque taper, not the 120 rpm rated profile. This is not a claim
of verified motor peak or thermal capability.

## Physical Trials

All directories below are under `log/`, with executable/configuration hashes,
expanded URDF, controller sources, contact events and recorded measurements.

| Trial | Outcome |
| --- | --- |
| `jump_1789221852819278087` | Joint tracking without full-state feedback: pitch guard before takeoff. |
| `jump_1789222431519884674` | No request sent: initial stationary-ready gate not met. |
| `jump_1789222507351876609` | TVLQR: 0.08893334 m conservative COM rise, 0.308 s airborne; late contact, compression stroke exhausted, body-height stop. |
| `jump_1789223165267577923` | Retiming: 0.08551301 m conservative rise; early contact, then feet lost contact again, measured joint-travel stop. |
| `jump_1789223371811082648` | Compression-margin candidate: contact lost too early in thrust; timing guard stopped. No qualified jump. |
| `jump_1789223552690817224` | No jump: strict interpolation validator rejected floating-point extremum roundoff during startup. |
| `jump_1789223912629144989` | Final candidate loads; tighter stationary admission not reached, `crouch_tracking_timeout`. No takeoff. |

The controller's asynchronous `apex_gain` is not the acceptance measurement.
Use the harness conservative takeoff bracket and coherent COM reconstruction.
An offline 0.125 m solution exists but has not been validated in Gazebo.

Raw optimizer candidates and source snapshots are archived in
`log/jump_optimizer_20260912/`. Expanded source URDF SHA256:
`7720f72660e5bc2c8d1cc3717ac145502ad9f1ccc942270b1b36d94fd6f50af8`.
The separate PC environment is `/home/dvt/.local/share/tap-jump-optimizer`;
dependency versions are in `tools/jump_optimizer_requirements.txt`.

## Remaining Technical Work

The current planner imposes nearly zero foot velocity and continuous generalized
velocity at touchdown. Actual early/late impacts violate that assumption.
Changing the clock is not a substitute for a dynamically feasible new reference;
retiming also invalidates the exact nominal TVLQR linearization. In trial 4 the
reference jumped forward about 50 ms at contact and subsequently withdrew the
feet. Do not promote this mechanism to a stable default.

An impact-aware extension should solve separate pre/post-impact velocities:
`M(q)(v+ - v-) = J(q)^T impulse`, with supported unilateral/friction impulse,
non-increasing kinetic energy and no instantaneous motor impulse. Landing must
start from that resulting state and retain enough feasible compression stroke.
It also needs an impact-aware feedback transition and early/late-contact tests.
These changes are recommendations, **not implemented capabilities**.

The optional landing-margin optimizer reserves knee travel, bounds body angular
velocity and maintains a normal-force floor before release. Its convergence is
not evidence that it tolerates actual impact, latency or actuator saturation.

## Verification

- Full core CTest: 13/13 passed on the final binary, SHA256
  `d2b1ad23906edf46b04c859a80b3e570f76b6dcdc1fbfd8fb5ecc73b8da5c6a9`.
- Synthetic jump ROS contracts: 54/54 passed, including invalid plan startup
  rejection. These do not prove valid-plan physical tracking.
- ADAM dynamics tests: 9 passed; TVLQR tests: 7 passed; exporter tests: 11 passed.
- Full planned feedback allocation branch is asserted exercised by tests.
- The final candidate's tighter stationary admission prevents repeating an
  insufficiently settled takeoff. It does not yet converge to a ready state.
  At 17.084 s the body pitch was 0.00189 rad but wheel-derived speed was
  -0.027 m/s. Preparation station keeping needs separate correction.
- Final physical squat regression passed: `transformation_1789223998799210493`,
  two full-depth cycles, 56.005 s measured motion, peak XY 0.00282195 m,
  residual XY 0.00001044 m, peak absolute pitch 0.0279215 rad, minimum body
  height 0.126791 m. Stop commands and applied-effort zero checks passed.
  This used `precision_pose.yaml`, the unchanged rated 120 rpm jump plant,
  knee depth 1.6 rad, 11 s transitions, 10 mm peak / 2 mm residual limits.
  It proves this squat regression only, not jump or asymmetric terrain skills.
- No repeated automatic jump demonstration is enabled. All simulation sessions
  used owned-process cleanup, with no live descendants remaining in their logs.

The sampler's interior-position arithmetic permits only a coefficient-scaled
double-epsilon evaluation tolerance. Node bounds and rate limits remain hard;
this fixes a roughly 7e-15 rad arithmetic rejection, not physical overshoot.
