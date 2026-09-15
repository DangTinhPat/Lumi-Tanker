# Phase Optimization: Controlled Experiments

2026-09-12. Simulation only, no hardware, actuator caps or default GUI changes.
This supplements `JUMP_OPTIMIZED_TRACKING_QA.md`. The requested >=10 cm jump
with stable landing remains unachieved.

## Changes

- Offline optimizer accepts `--crouch-knee` in [-1.6,-0.6) rad. Previously
  preparation depth was fixed at -1.6, preventing a controlled comparison.
- Optional `--monotonic-thrust` constrains knee extension over entire Hermite
  thrust segments using derivative Bezier control points. This is an experiment,
  not an assumption that countermovement is always undesirable.
- `audit_jump_plan.py` measures positive/negative bilateral motor work,
  near-limit torque fraction, joint rates, compression travel, COM velocity
  and a sticking-impact energy diagnostic. It does not certify landing.
- Exporter option `--precision-preparation` reuses precision_pose's station
  gains/dwell and 11 s descent. It leaves thrust/flight/landing references and
  motor caps unchanged. Default exporter behavior remains unchanged.

## Offline Comparison

Same URDF, target 0.105 m, 40/80/80 intervals and landing-margin constraints:

| Initial knee | Thrust duration | Positive thrust motor work | Positive flight motor work |
| --- | --- | --- | --- |
| -1.6 rad | 0.1068 s | 8.326 J | 1.789 J |
| -1.4 rad | 0.1822 s | 8.855 J | 4.682 J |

The -1.4 solution subsequently lowers almost to -1.6 during thrust; it is not
a demonstrated efficiency improvement. Its takeoff horizontal COM velocity
also reaches the allowed -0.025 m/s boundary. Therefore it was not promoted.
The monotonic -1.4 variant failed to converge within 500 iterations, with
constraint violation 0.05455. No profile was exported from that failed solve.
This solver result does not prove physical infeasibility or global optimality.

Artifacts: `log/jump_optimizer_20260912/{crouch14.json,
crouch14_monotonic.json,crouch-comparison.json}`. Source snapshots are alongside.
The audit uses midpoint work quadrature; mesh convergence is not established.

## Gazebo Isolation Tests

1. `log/jump_1789229060113740801`: -1.4 candidate, original 3 s preparation
   and default station gain. Failed `crouch_tracking_timeout`, no takeoff.
2. `log/jump_1789229206179300647`: identical planned trajectory with precision
   preparation. Passed stationary admission and entered Thrust, then lost
   contact at plan 0.035 s; `plan_contact_timing` safely stopped the trial.
   No qualified jump or recovery. This is one preparation improvement, not
   repeated preparation reliability or a successful jumping skill.

Both tests used 5 Nm legs, 0.13 Nm wheels, existing experimental speed taper,
10 mm peak/2 mm residual acceptance limits. No reset/external wrench was used.
Owned-process cleanup reports no live descendants. Runtime binary unchanged:
`d2b1ad23906edf46b04c859a80b3e570f76b6dcdc1fbfd8fb5ecc73b8da5c6a9`.

## Research And Next Dependencies

Re-read the primary repository READMEs on this date:

- https://github.com/silvery107/wheeled-bipedal-jumping : Zhuang et al.,
  *Height Control and Optimal Torque Planning for Jumping With Wheeled-Bipedal
  Robots*, ICARM 2021, DOI 10.1109/ICARM52023.2021.9536196. Supports considering
  height and energy jointly; Webots-specific implementation is not copied.
- https://github.com/BinghengNUS/SUSTech_NeZha : Chen et al., *Underactuated
  Motion Planning and Control for Jumping With Wheeled-Bipedal Robots*, RA-L
  2021, DOI 10.1109/LRA.2020.3047787. Paper-associated provenance confirmed;
  no new full-text or source-archive inspection is claimed here.

Preparation, thrust and landing must be evaluated separately. Preparation now
has a candidate tested setting; thrust still needs contact-feasible closed-loop
tracking under latency and torque saturation. Flight/landing still require the
impact-aware velocity reset and compression continuation described in the
previous report. The diagnostic impact projection added here is NOT that
controller implementation. Do not claim those phases have been optimized.

Verification: 12 exporter unit tests and 3 audit unit tests passed. No C++ or
default squat code changed in this iteration. Prior core/squat evidence is
historical, not a new run in this iteration.
