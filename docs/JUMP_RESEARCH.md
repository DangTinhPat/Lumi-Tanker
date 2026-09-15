# Jump And Landing Research

Reviewed 2026-09-12. Research and design recommendation only; no new physical
success is claimed. Current TAP evidence remains approximately 8.5 cm COM rise
with failed landing. Runtime configuration was not changed for this review.

Subsequent opt-in implementation and failed physical trials are documented in
[the landing implementation audit](JUMP_LANDING_IMPLEMENTATION.md). That audit
distinguishes completed components from the still-missing dynamics optimizer.

## Sources Read

1. Klemm et al., **Ascento: A Two-Wheeled Jumping Robot**, ICRA 2019.
   https://arxiv.org/abs/2005.11435
   Full text, sections IV.A-B: stability-gated preparation, phased jumping,
   contact-triggered restoration of balance, virtual spring-damper landing.
   Its linkage and physical torsion springs differ from TAP. Its demonstration
   does not validate TAP's actuator capability or positional drift.
2. Chen et al., **Underactuated Motion Planning and Control for Jumping with
   Wheeled-Bipedal Robots**, 2020 preprint / RA-L.
   https://arxiv.org/abs/2012.06156
   Full text, sections III-V, especially equations (9)-(11): W-SLIP reduced
   dynamics, bounded stance planning, ballistic flight, angular-momentum
   conservation, terminal leg length AND velocity constraints at touchdown.
   Demonstrated in V-REP; not a hardware result for TAP. The template's single
   flight shape DOF is not the DOF count of TAP's articulated mechanism.
3. Dinev et al., **Modeling and Control of a Hybrid Wheeled Jumping Robot**,
   IROS 2020. https://arxiv.org/abs/2003.01440
   Full text: nonlinear model-based planning over preflight, flight and
   post-touchdown phases with different contact constraints. The optimized
   motion extends the prismatic leg before landing to absorb impact.
   Its prismatic template is not directly interchangeable with TAP's CAD legs.
4. Secondary implementation inspected (README and `src/python/main_total.py`):
   https://github.com/rbonuomo/UR_HybridWheeledJumpingRobot
   Student implementation, not identified as the paper authors' official code.
   Uses CasADi/acados SQP_RTI and phase-dependent contact/input/state constraints.
   Its main loop advances to the solver's next predicted state; that alone is
   not an independent Gazebo plant validation. No code was copied or executed.
   Do not import its masses, wheel radius, torque bounds or solver runtime into
   TAP. Check license before any future code reuse.

## Evidence From TAP

Reanalyzed `log/jump_1789216663751564324` using
`python3 tools/analyze_jump_trace.py log/jump_1789216663751564324`.

- Measured conservative rise: 0.08499656 m. Ballistic fit gives initial vertical
  COM speed about 1.2944 m/s and acceleration -9.8212 m/s^2.
- First Flight-phase sample: body pitch +0.0662 rad, reported pitch rate
  +1.038 rad/s, estimated horizontal COM velocity +0.143 m/s.
- First Land-phase sample: body pitch -0.2050 rad, pitch rate -0.58 rad/s;
  COM minus axle world X = -0.04676 m. Wheels are already ahead of the COM.
- These phase samples follow contact debounce, not exact impact instants;
  velocities are local fitted estimates from asynchronous samples.
- At the recorded flight duration of about 0.26 s, holding horizontal COM
  displacement below 10 mm suggests takeoff speed below roughly 0.038 m/s.
  This is a screening calculation, not a full peak body-XY acceptance test.

The code currently blends into a static body-frame COM-neutral landing pose.
That is not a world-frame touchdown target for a pitched, rotating robot.
Hip reaction feedback redistributes angular momentum between links, but cannot
remove whole-robot angular momentum in free flight. Flight joints also cannot
cancel the COM's existing horizontal velocity without an external force.
The thrust Jacobian currently uses body-frame angles for an intended upward
reaction; the world-frame transform must be derived and tested before changing
its implementation. This is a concrete audit item, not a proven sole cause.

## Recommended Design

### 1. Plan The Full Maneuver Offline

Use a validated multibody library and the existing CAD inertials to optimize
a sagittal symmetric jump first. Include leg masses, pitch/shape coupling,
wheel rolling, unilateral contact, friction and impact. Preserve native +Y
axes on both sides. Optimize thrust timing, hip/knee trajectories and terminal
states jointly, rather than maximizing the force request alone.

Enforce the existing 5 Nm leg / 0.13 Nm wheel limits and speed-dependent torque
envelope, provisional travel, body-ground clearance and contact constraints.
Include separate decision phases for takeoff, flight, compression and recovery.
Use a constrained solver such as CasADi/IPOPT on the PC; Gazebo remains the
independent test plant. Do not optimize against height alone.

For a 10 cm ballistic COM rise, vz must exceed sqrt(2*g*0.10) = 1.4007 m/s.
An initial planning target of 1.50 m/s provides a nominal 11.5 cm margin, but
is not yet known feasible within TAP's envelope. Penalize horizontal takeoff
velocity and whole-robot pitch angular momentum; body pitch rate alone is not
an adequate momentum state. Report infeasibility instead of increasing ratings.

### 2. Predict Touchdown And Coordinate Shape

At measured takeoff, estimate COM state and angular momentum using IMU,
joint positions/velocities and the rigid-body model. Use Gazebo truth only as
an explicitly labeled QA oracle while developing the deployable estimator.

Generate a bounded flight reference with touchdown time, leg length, leg
length rate, body attitude and wheel position objectives. Minimize touchdown
horizontal velocity and wheel/COM placement error together. For near-stationary
landing, target support near the COM; for residual velocity, solve a feasible
capture trajectory rather than forcing exact static alignment.

Preserve position AND velocity continuity when entering Flight. Current
position-only smoothstep blending can introduce a velocity discontinuity at
liftoff. Validate a polynomial/Hermite reference against torque, speed and travel
limits, including its initial measured velocity. Reject infeasible references.
Do not integrate unlimited hip correction until the feet are unreachable.

### 3. Absorb Impact Before Returning To Stand

Prepare the legs before predicted contact, leaving sufficient compression
stroke. On measured contact use bounded task-space impedance, conceptually
Fz = mg + kz*(z_ref-z) + dz*(vz_ref-vz), mapped through a verified world-frame
Jacobian and constrained by torque, friction and unilateral support.
Coordinate this with the plugin's existing local damping; do not stack two
unaccounted velocity servos. Virtual compliance does not create extra torque
or physical spring energy.

Estimate required absorption energy from touchdown velocity, including link
rotation; m*vz^2/2 is only the translational component. Account for gravitational
work during compression. If available stroke/torque cannot absorb the energy,
revise the planned touchdown state before running the trial.

Enable wheel balance on valid ground contact with bumpless state transfer and
anti-windup. Match wheel rolling speed to the planned ground-relative wheel
motion rather than assuming zero wheel angular speed is always a soft landing.
Retain the original station anchor; blend position recovery only after impact
compression and balance recovery. A return to the anchor must not conceal a
large transient excursion.

### 4. Keep The Embedded Core Small

Export verified trajectory coefficients and scheduled gains to the portable
C++ core. Runtime work: small fixed-size FK/Jacobian operations, state
estimation, polynomial evaluation, bounded feedback and contact state machine.
No online large NLP, ROS, Gazebo dependency or heap allocation in the control
tick. Measure worst-case execution on ESP32-S3 later; desktop timings cannot
certify MCU performance. No hardware actuation is authorized by this study.

## Verification Sequence

1. Unit-test frames, Jacobians by finite differences, flight momentum accounting,
   trajectory continuity/limits, unilateral force, saturation and stop/reset.
2. First demonstrate lower-height jumps with stable landing; do not conflate
   those with the >10 cm target. Isolate flight-placement versus impact-control
   changes using the same archived takeoff candidate.
3. Optimize for >10 cm and validate in unchanged bounded-actuator Gazebo.
   Require height from measured takeoff COM, no safety trip and 5 s recovered
   balance. Keep peak XY <=10 mm and residual <=2 mm as acceptance objectives.
4. Require at least 10 independent passing starts before a finite repeated-jump
   demonstration. Then test parameter uncertainty, delayed/asymmetric contact
   and small initial-state errors separately, preserving safety/watchdogs.
5. Rerun moderate/deep transformation regressions on the final binary. Never
   attribute reference robots' performance or an optimizer's feasible solution
   to TAP until independent physical tests pass.

This review identifies a defensible architecture and specific code weaknesses;
it does not establish that stable >10 cm jumps have already been implemented.
