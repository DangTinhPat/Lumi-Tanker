# Landing Controller Implementation Audit

2026-09-12. Partial implementation of [the research design](JUMP_RESEARCH.md).
**The requested stable jump above 0.10 m is still NOT achieved.** These changes
are opt-in experiments, not a validated demonstration or a hardware controller.

## Implemented

- `landing_world_frame`: solve the planar COM-to-axle alignment in world X,
  accounting for measured body pitch and native +Y axes on both legs. Applied
  during Flight only; continuously re-aligning during Land worsened oscillation
  in an ablation trial and was removed. This is kinematic alignment, not a
  feasible capture-state or momentum optimizer.
- `thrust_world_frame`: map the upward reaction with a pitch-rotated foot
  Jacobian. Finite-difference tests cover both legs and both pitch signs.
  The old body-frame feedforward remains the default for comparison.
- `thrust_hip_offset_rad`: a smooth, bounded stance hip-reference correction,
  at most 0.15 rad, inside the original provisional joint travel.
- `thrust_wheel_control` / `thrust_wheel_speed_m_s`: optional bounded rolling
  speed reference during Thrust (at most 0.35 m/s). ROS applies the existing
  carrier-rate conversion. This is an experimental reference, not a closed-loop
  centroidal momentum regulator. Flight and faults release the override.
- `landing_compression`: a contact-triggered bend/recovery sequence, with low
  initial position stiffness and the existing local actuator damping. It is
  NOT task-space force impedance. The target bend and duration are validated,
  there must be time left to settle, and tracked pose, low rates and contact
  still gate recovery. Timing alone cannot report completion.
- `smooth_phase_transitions`: cubic Hermite flight/landing references seeded
  with the previous commanded position and rate, rather than resetting to a
  potentially displaced encoder sample. At the exact boundary the incoming
  rate is retained; subsequent rates use bounded target differences. Position
  and speed saturation take priority over polynomial continuity. Terminal
  velocity remains zero; nonzero optimal touchdown rates are NOT implemented.
- Conflicting flight hip-integrator/reference modes are rejected. New dynamic
  pose options require the full existing provisional envelope in the ROS
  profile; compression targets are checked against the profile too.
- Offline world-COM angular momentum diagnostics include both rigid-body spin
  and orbital terms using archived URDF inertials. Full-rate measurements are
  preferred when present; current logs use differentiated, normalized,
  sign-unwrapped quaternions. Tests cover frames, inertial rotations, native
  signs, source timestamps, translation invariance and analytic examples.

Runtime changes are in `JumpController.{hpp,cpp}` and `BalanceControllerNode.cpp`.
The normal/precision/demo profiles do not enable the new options. Development
profiles are `jump_development.yaml` and `landing_development.yaml`; the latter
is a lower-stroke diagnostic, not a 10 cm candidate. Compression remains OFF in
the final high-stroke experiment after failed comparisons, but its opt-in
implementation and tests remain available. No automatic repeated jumps were
enabled. No actuator plugin, mass, inertia, contact setting, motor torque/speed
envelope, watchdog or safety threshold was increased or relaxed.

## Gazebo Evidence

All directories are under `log/`; each archives the actual parameters, source,
binary and model. Heights are conservative COM rises from measured takeoff.
All trials used the existing 5 Nm legs / 0.13 Nm wheels and manual experimental
speed curve, with 10 mm peak XY and 2 mm residual objectives unchanged.

| Jump directory | Main comparison | Rise | Result |
| --- | --- | ---: | --- |
| `jump_1789217222787183612` | World alignment during Flight and Land | 84.85 mm | Pitch guard |
| `jump_1789217354186916967` | +0.10 rad thrust hip correction | 85.78 mm | Pitch guard |
| `jump_1789217420092545473` | +0.15 rad hip correction | 86.34 mm | Pitch guard |
| `jump_1789217546135214851` | -0.15 m/s thrust wheel reference | 84.26 mm | Pitch guard after contact |
| `jump_1789217620552714619` | World alignment in Flight only | 84.44 mm | Pitch guard |
| `jump_1789217679939149255` | Increased attitude feedback within 5 Nm cap | 84.79 mm | Pitch guard |
| `jump_1789217894225883852` | Contact-triggered compression | 85.42 mm | Pitch guard |
| `jump_1789217954000786568` | -0.07 m/s wheel reference | 85.64 mm | Pitch guard |
| `jump_1789218007710160646` | -0.11 m/s wheel reference | 85.25 mm | Pitch guard |
| `jump_1789218166478775675` | Hermite phase references | 85.27 mm | Pitch guard |
| `jump_1789218305843247202` | World-up thrust Jacobian | 85.14 mm | Pitch guard |
| `jump_1789218377743676976` | Lower-stroke experiment | 52.37 mm | Pitch guard before confirmed landing |
| `jump_1789218508162898882` | Hermite/world thrust without compression | 85.16 mm | Pitch guard |
| `jump_1789218763317496560` | Final functional build repeat | 82.64 mm | Pitch guard; stop and cleanup passed |

One intermediate trial (`jump_1789217546135214851`) reached the first Land-phase
sample at pitch +0.116 rad with COM/axle X mismatch about 0.08 mm, compared with
the earlier ~47 mm mismatch. It still fell after impact. This is evidence of
improved instantaneous placement, NOT evidence of stable landing or low drift.
Phase-edge values follow contact debounce and use asynchronous sensor packets;
they are not exact impact-state or contact-wrench measurements.

The lower-height trial also failed. Consequently no stable lower-hop capability,
repeated-jump capability or five-second recovery is claimed from this audit.
The current core remains a bounded trajectory tracker, not the full optimal
control architecture proposed in the research document.

## Verification

Final functional controller SHA256:
`56924912968788485d62f9eac130427837cfa82d938ea1e00ca12b1cbbddc310`.

- CTest: 11/11 passed, including 137 synthetic jump checks, MPC equivalence,
  model checks and no-allocation checks. The allocation test enables world
  landing, compression and Hermite references through all synthetic phases.
- Jump-related Python suite: 57 passed, including 14 momentum diagnostics tests.
- Existing jump ROS contracts: 51/51 passed on the final binary.
- Focused landing ROS contracts: 35/35 passed. Wheel reference units/signs, flag behavior,
  contact-driven compression, zero stop outputs and invalid parameters checked
  by `tools/test_landing_ros_contract.py`. These are synthetic interface tests.
- On this desktop build `sizeof(JumpController) = 352` bytes. No heap allocation
  occurs in the tested sequence. MCU worst-case execution is not measured.
- Moderate pose regression `transformation_1789218650435293803`: PASS, two
  squat/left-roll/right-roll cycles, 64.005 scored seconds, peak XY 9.667 mm,
  residual XY 0.379 mm. Owned live process leftovers: none.
- Deep squat regression `transformation_1789218876056907531`: PASS, two -1.6 rad
  squat cycles with 11 s transitions, 56.004 scored seconds, peak XY 2.576 mm,
  residual XY 0.062 mm, minimum body Z 0.126793 m. Owned live leftovers: none.
  These regressions use the precision pose profile, not the experimental jump
  settings. After the final trials, no live TAP simulation or contract-test
  controller processes remained.

## Remaining Work

The offline constrained multibody trajectory optimizer, feasible nonzero
touchdown velocity/momentum targets, online touchdown-time prediction and a
properly coordinated task-space impact controller are still missing. Fixed
timing, zero terminal joint rates and finite-gain wheel tracking do not yet
produce a recoverable touchdown. Individual kinematic/feedback improvements
must not be described as solving that coupled dynamic problem.

Next implementation should optimize initial/terminal momentum, foot velocity
and contact force jointly using a validated dynamics library, then verify on
the unchanged Gazebo plant. Do not continue to promote hand-tuned gain trials
or increase actuator ratings to make the acceptance report pass. The research
references are design guidance, not performance evidence for TAP.
