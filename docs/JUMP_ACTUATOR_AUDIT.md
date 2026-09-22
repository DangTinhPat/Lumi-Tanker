# Jump Actuator Audit

2026-09-12. Offline source inspection and archived-data reduction only. No
simulation, gain tuning, core edits or motor-limit changes. Applied the physics,
control, ROS/Gazebo and QA expert skills in `.agents/skills`; these are functional
review roles, not claims of separate running agents. Frame handoff: SI, native
relative +Y joints on BOTH sides, six-element LH/LK/LW/RH/RK/RW order; four-element
leg commands omit wheels. Evidence handoff: plugin-applied effort, not requested
effort, is used for mechanical work.

## Strongest Finding

**The demonstrated high-stroke takeoff depends heavily on positive work above
120 rpm and is already knee-torque-limited. It is not evidence for the rated GUI
profile.** In the final repeat, knees saturate for 80% of Thrust and reach
24.88 rad/s (237.6 rpm); 5.11 J of positive leg work occurs above 120 rpm.
The next actionable task is actuator-consistent trajectory feasibility, including
the local velocity servo and knee-release event, on unchanged limits. Increasing
force requests or feedback gains cannot create torque beyond this envelope.
Do not treat unused hip torque capacity as freely transferable vertical power.

## Envelopes And Command Semantics

- `rated_120rpm`: 5 Nm active leg cap, positive-work cutoff 12.56637 rad/s.
  Per-motor positive power approaches 62.83 W below cutoff, then is zero.
- `manual_curve_experimental`: 5 Nm through 19.89675 rad/s (190 rpm), linear
  taper to zero at 28.27433 rad/s (270 rpm). Maximum positive power is 99.48 W
  per leg motor at the plateau edge, NOT 5 Nm times 270 rpm. At 24.878 rad/s,
  the instantaneous motoring cap is about 2.03 Nm. Braking remains bounded to
  5 Nm in both profiles; external overspeed is allowed, not clipped away.
- `run_jump.sh:4-28` defaults to rated and pairs the selected controller/model
  cutoff; both options use local velocity gain D=0.6 Nm/(rad/s).
  `precision_pose.yaml:7` also selects rated. The dashboard's opt-in/Jump button
  only authorizes/publishes a request (`dashboard.py:198,292`); it does not select
  a faster motor curve. Ordinary `run_sim.sh` instead defaults to the legacy,
  non-jump plant (`sim.launch.py:56`), not a bounded rated jump validation.
- Actual active leg torque is `envelope(effort_cmd + D*(qd_ref-qd))`.
  Node clipping happens BEFORE this local feedback, so command saturation and
  applied saturation differ. Wheels take rad/s commands and apply
  `clamp(0.02*(clamp(qd_cmd,+/-41.88790)-qd),+/-0.13)` Nm with a positive-work
  cutoff. A wheel command is not torque. Sources:
  `src/legacy/ros2_gazebo_sim/include/tap_sim/actuator_bounds.hpp:7`,
  `src/legacy/ros2_gazebo_sim/src/bounded_gazebo_system.cpp:105,168`, and
  `src/tap_control/ros2_adapter/BalanceControllerNode.cpp:1112,1150`.

## Measured Thrust

Primary archive: `log/jump_1789218763317496560/`; controller SHA256
`56924912968788485d62f9eac130427837cfa82d938ea1e00ca12b1cbbddc310`.
Verified manifest hashes for its controller binary, expanded URDF, and archived
JumpController, ROS node, actuator bounds and plugin source. Configuration:
manual envelope, D=0.6, crouch -1.6 rad, release target -0.95 rad, nominal
thrust duration 0.08 s, requested total force 300 N, knee force mode enabled.
Sources: `effective.yaml`, `manifest.json`, `samples.json`, `summary.json` there.

Deduplicated joint-source samples give 0.100 s labelled Thrust (10.615-10.715 s,
4-6 ms steps). Saturation means >=99% of fixed 5/0.13 Nm cap. Work below is
left-hold integration of reported active torque times relative measured rate.

| Joint | Peak abs rate rad/s | Peak abs effort Nm | Cap time ms | Net work J | Positive work J |
| --- | ---: | ---: | ---: | ---: | ---: |
| LH | 13.578 | 4.012 | 0 | 0.194 | 0.305 |
| LK | 24.878 | 5.000 | 80 | 3.408 | 3.872 |
| LW | 24.492 | 0.130 | 40 | -0.139 | 0.006 |
| RH | 13.561 | 4.020 | 0 | 0.225 | 0.309 |
| RK | 24.843 | 5.000 | 80 | 3.599 | 4.064 |
| RW | 23.974 | 0.130 | 40 | -0.138 | 0.006 |

Leg net/positive/negative work: 7.426 / 8.549 / -1.123 J. Of positive leg work,
5.112 J (59.8%) occurs above 120 rpm. Each knee has 15 ms of positive-work
samples above the 190 rpm taper onset. Peak sampled total active joint power
is 196.9 W; average net leg power is 74.3 W. Passive damping estimates
`integral(b*qd^2 dt)` are 0.778 J legs plus 0.049 J wheels and are NOT included
in reported effort. These are joint-work estimates, not battery energy or a
closed whole-body energy balance. Quadrature sensitivity on the same intervals:
trapezoidal power gives 7.484 J net leg work; trapezoidal torque times measured
joint displacement gives 7.585 J, versus 7.426 J with left-hold power. This
spread is not a bound on timing or model error.

Across all 14 runs listed in `JUMP_LANDING_IMPLEMENTATION.md`, controller and
plant manual cutoffs agree. The 13 high-stroke runs show peak knee rates
23.88-25.41 rad/s, net leg work 7.75-8.27 J except final repeat 7.43 J,
and 4.51-5.11 J positive leg work above 120 rpm. The lower-stroke diagnostic
still reaches 25.64 rad/s, with 5.17 J net leg work and 3.77 J above 120 rpm.
All failed landing guards. Historical rated run `jump_1789204699669857176`
has only 13.18 mm conservative rise, 3.136 J net leg work, peak knee rate
14.336 rad/s, and no sampled 99%-cap saturation in its 0.100 s Thrust.
Its trajectory/controller differ: this is NOT a controlled profile ablation.

Final conservative COM rise is 82.64 mm, takeoff bracket 10.704-10.712 s.
This historical result used a superseded mass allocation and is not current
model actuator-sizing evidence. Stance lifting, internal motion, braking,
losses and recoverable landing also matter. Full-thrust phase labels include
contact debounce after physical release; the reported negative work is not all
a pre-takeoff loss.

## Model/Command Inconsistencies And Limits

1. **Force mode is also a maximum-speed servo.** Archived `JumpController.cpp:286`
   sets knee stiffness to zero but commands 28.27433 rad/s until the measured
   knee crosses -0.95 rad, then zero velocity and zero force feedforward.
   At rest D contributes 16.965 Nm before the 5 Nm cap; at 20 rad/s it still
   contributes 4.965 Nm. Therefore the Jacobian force request is not the applied
   contact force, even with position stiffness zero. After release, zero effort
   command is not torque-off: D requests braking. At joint stamp 10.705 s,
   LK has +18.536 rad/s and -5 Nm applied, while the sampled effort command is
   zero and contacts are absent. The knee reaches -0.785 rad at 10.700 s despite
   the -0.95 release target. A planner must represent this event and bounded
   braking, not assume position/rate commands form one smooth feasible path.
2. Controller `jump.profile` and plant `leg_speed_limit` are separate settings.
   The wrapper pairs them, but raw launch arguments can mismatch them. Archive
   both and validate agreement before interpreting results. No such mismatch
   was found in the 14 audited landing runs. URDF velocity=1000 is deliberately
   a solver guard, not a motor rating; do not infer available power from it.
3. Samples combine asynchronous phase/contact/command streams; primary archive
   maximum source spread is 7 ms. Plugin effort is the last written torque,
   while joint velocity is read from physics (`bounded_gazebo_system.cpp:134`).
   Thus a sampled torque/rate pair can appear outside the instantaneous taper
   without proving an enforcement violation. Work and boundary timing are
   approximate; physics-write torque/rate/reference telemetry is the next
   instrumentation requirement. No reference-velocity history is archived here.
4. `PHYSICS.md` still describes ideal wheel actuation generically; that applies
   to the legacy plant, not this bounded jump plugin. Its substantive hardware
   cautions still apply: the archived mass allocation is superseded, all four
   leg motors are lumped in the body, wheel/stator/rotor inertia is a surrogate,
   knee mechanical limits
   are unmapped, and voltage/thermal/electrical/transmission dynamics are absent.
   Neither experimental envelope nor these traces certify hardware capability.

## Reproduce Without Simulation

Run from `/home/dvt/tap` (Python standard library). This prints the primary
table plus positive work above 120 rpm and passive damping work per joint;
change `p` to any archived directory for comparisons.

```python
import json, math
from pathlib import Path
p = Path('log/jump_1789218763317496560')
unique = {}
for s in json.loads((p / 'samples.json').read_text()):
    unique.setdefault(s['source_stamps']['joints'], s)
rows = sorted(unique.items())
for side in ('left', 'right'):
    for joint in ('hip', 'knee', 'wheel'):
        name = f'cad_{side}_{joint}'
        cap, damping = (0.13, 0.001) if joint == 'wheel' else (5., 0.02)
        duration = vmax = emax = sat = work = positive = above = loss = 0.
        for (t, s), (u, _) in zip(rows, rows[1:]):
            if s['phase'] != 'Thrust':
                continue
            dt = u - t
            v, e = (s['joints'][k][name] for k in ('velocities', 'efforts'))
            power = v * e
            duration += dt
            vmax, emax = max(vmax, abs(v)), max(emax, abs(e))
            sat += dt * (abs(e) >= 0.99 * cap)
            work += dt * power
            positive += dt * max(power, 0.)
            above += dt * max(power, 0.) * (abs(v) >= 4 * math.pi)
            loss += dt * damping * v * v
        print(name, *(round(x, 6) for x in
              (duration, vmax, emax, sat, work, positive, above, loss)))
```

## Optimizer Review (Read-Only)

2026-09-12. Reviewed `tools/optimize_jump.py` SHA256
`b8a1dcce27920d0032dd863f429e9f058047cb88616d63b03dc0aa93a4390236`.
Sagan's `tools/jump_dynamics.py` appeared during review; reviewed SHA256
`154654e763f6d826359058328872f5108f1611a26ec3e338c6aabc4df5eb30df`.
Line references below are to that optimizer revision. No solver, simulation or
ADAM tests were run; verification was source inspection and standalone algebra.

- **P1: Contact work is not constrained where force acts (78,90-101).**
  Contact position/velocity constraints are at nodes; `J(q_mid)*v_mid` is not
  zero merely because endpoint rolling/height constraints hold. Consequently
  `F.T*J*v_mid` can add artificial energy in stance. Even exact node tangency
  does not fix nonlinear midpoint drift. An illustrative scalar constraint
  `z=0.1*cos(theta)`, theta 0.3 to 0.7 rad over 0.02 s, admits exact endpoint
  tangency and midpoint position integration yet midpoint normal velocity
  0.00638 m/s: 300 N would do 0.0383 J of spurious work in that interval.
  Fix the contact transcription at force evaluation points; independently
  check contact work, flight COM ballistic momentum, and
  `delta(T+mg*z_com) - integral(2*tau.dot(v_joint)-P_passive) dt`.
  Require mesh-refinement/dense-trajectory residual checks, not just IPOPT's
  algebraic constraint residual, before calling a candidate feasible.
- **P1: Motor feasibility is midpoint-only and unbound to the plant (53,102-111).**
  The revised linear leg facets are correct for the MANUAL envelope inside
  +/-28.27433 rad/s, but a 12 to 27 rad/s interval admits constant 5 Nm at its
  19.5 rad/s midpoint while the endpoint cap is only 0.761 Nm. Enforce the
  envelope over the reconstructed interval, not only its mean rate. Explicitly
  select/archive/validate rated versus manual cutoff and passive damping
  against the supplied model: the current input accepts a rated/legacy URDF
  while optimization unconditionally uses manual limits and jump damping.
  Hard joint-speed bounds additionally exclude permitted external overspeed;
  label them trajectory restrictions, not the plugin's physical speed clamp.
- **P2: Landing inherits the wrong rolling integration constant (65,72-77).**
  `axle_x-R*(pitch+hip+knee+wheel)` is constant within each grounded rolling
  phase, not across flight. Reusing `contact_start` artificially constrains
  airborne wheel rotation and can reject valid landings. Anchor the second
  rolling phase to its own touchdown configuration while retaining the actual
  COM/axle displacement objectives. Continuous velocity with `Jtd*v` near zero
  (80-83) is an intentional impactless subset, not an energy-erasing reset;
  infeasibility of this subset does not establish motor-limited impossibility.
  General impact landing needs an explicit impulse/momentum and energy-loss
  model, not merely a relaxed touchdown velocity tolerance.
- **P2: Wheel inverse omits saturated-servo branches (109-111).** At relative
  speed 50 rad/s, command 41.88790 produces valid -0.13 Nm braking in the
  plugin, but `v+tau/0.02=43.5` violates this optimizer's bound. Model an explicit
  bounded velocity command and the exact saturated servo/cutoff, or its complete
  torque-reachability set. The current inverse is exact only off saturation
  and a conservative restriction otherwise; it does not add positive work.
- **P2: Accepted height range contains an impossible constraint (61,191).**
  The hard 1.7 m/s takeoff ceiling allows at most 0.147299 m ballistic rise,
  yet CLI accepts 0.15 m (requires 1.71552 m/s). Make input validation and the
  stated planning envelope consistent; this is not an actuator-rating change.

Correct contracts, confirmed against the newly available model: contact
`Jx=d(axle_x)/dq-0.03*[0,0,1,1,1,1]`, `Jz=d(axle_z)/dq`, positive upward
`Fz`, and `+J.T*F` on the RHS have the native +Y signs. `force` is bilateral
TOTAL force, so do not multiply it by two again. Per-side motor torque and
passive damping DO need the factor two in the reduced coordinates. Positive
passive damping on the LHS dissipates
`2*.02*(hip_rate^2+knee_rate^2)+2*.001*wheel_rate^2` W; ADAM bias excludes it.
Do not add local servo gain 0.6 as passive friction: `torque` must mean net
active applied torque, with a separate realizable command reconstruction for
the runtime tracker. An 80,601-point algebraic grid found no mismatch between
the revised leg facets and the plugin manual envelope within their speed box.
The 300 N reaction ceiling and impactless landing remain planning assumptions,
not measured motor limits or proof of stable recovery.
