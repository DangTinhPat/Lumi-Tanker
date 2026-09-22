# Experimental Jump Development

The subsequent [pose/jump quality audit](QUALITY_AUDIT.md) records newer safety,
pose-ownership, landing gates, performance and regression results. Historical
trials and binary hashes below remain historical evidence, not a current jump
pass. Bounded-actuator pose passes do not validate flight or landing.

## Scope And Acceptance

Simulation only, Gazebo + RViz; this document records historical jump work
against a superseded CAD model.
The selected four GIM6010-8 leg motors and two BL4820M12 wheel motors are not
fully characterized. No hardware operation, automatic repeated jumps or salto
is enabled by this work. The requested target is strictly greater than 0.10 m
world COM rise AFTER actual takeoff, not body rise from crouch to extension.

Acceptance requires independent wheel contact observations, a sustained flight
interval, measured whole-robot COM apex relative to takeoff, controlled landing
and subsequent stationary balance. A height pass with a fall is a failed jump.
Source inspection, unit tests, launch validation, and RViz rendering do not
constitute physical jump evidence. Final simulation results are recorded below
when available; no prior squat result is inherited as jump validation.

## Research Used

- Klemm et al., **Ascento: A Two-Wheeled Jumping Robot**, ICRA 2019,
  DOI https://doi.org/10.1109/ICRA.2019.8793792,
  full text https://arxiv.org/abs/2005.11435. Section IV-B describes successive
  retract, trigger, extract, fly and land phases, overriding ordinary driving.
  Stabilization remains active during retraction, flight uses retracted-leg
  impedance, and contact detection triggers landing damping and stabilization.
  The paper uses hip torque thresholds for contact; TAP will use simulated
  collision observations. Ascento's parallel elastic mechanism, 40 Nm peak hip
  motors and different geometry make its numeric settings inapplicable here.
- MIT **Cheetah-Software**, inspected commit
  `c71c5a138d3e418cc833e94e25357ceea8955daa`:
  https://github.com/mit-biomimetics/Cheetah-Software/blob/c71c5a138d3e418cc833e94e25357ceea8955daa/user/MIT_Controller/Controllers/BackFlip/FrontJumpCtrl.cpp
  uses precomputed position/velocity/feedforward torque trajectories and
  phase-specific impedance/landing posture. TAP does not copy the quadruped's
  trajectory files, iteration counts, torque multipliers or disabled safety
  checks. The transferable idea is explicit motion ownership and distinct
  takeoff/flight/landing control, not a stronger ordinary squat command.

Sources retrieved and source code/full paper inspected 2026-09-12. This is
an adaptation of control principles, not an implementation of either complete
external controller or a claim to reproduce their experiments.

## Actuator And Geometry Caveats

GIM nominal torque is 5 Nm, nominal speed 120 rpm and published maximum speed
420 rpm. **11 Nm is stall torque, not an allowed jumping peak.** A rectangular
5 Nm envelope through 420 rpm is not established by the datasheet. Motor torque
at speed, power supply sag, transient duration and regenerative braking remain
unverified. CJC's 0.130 Nm has no specified duty rating; its 12 V branch must
not be assumed compatible with the GIM supply without power-system design.

Further visual inspection of manual page 6 revealed an embedded characteristic
plot omitted by text extraction: the red curve is approximately 200 rpm at
5.1 Nm and 285 rpm near zero torque. Thus the rated-speed screen below is NOT
the complete motor capability. A graph-informed experimental torque rolloff
is preferable to the unverified 420 rpm rectangular case. The plot lacks supply
voltage, temperature and duty conditions, so even a conservative graphical
fit remains provisional until matched to the purchased variant and supply.

`tools/analyze_jump_envelope.py` screens a symmetric COM-neutral path from
native knee -1.6 to zero using the existing CAD FK. It accounts for gravitational
work and joint speed, but neglects rotational kinetic energy, link dynamics,
losses and the need to allocate torque to pitch stabilization. It is an
optimistic scalar screen, NOT full inverse dynamics or a general impossibility
proof for all motions.

At the current mass and geometry the path has 89.695 mm of COM stroke:

| Hypothetical per-joint box | Optimistic sampled ballistic rise |
| --- | --- |
| 5 Nm, speed <=120 rpm | 47.7 mm |
| 5 Nm, speed <=190 rpm | 105.7 mm |
| 5 Nm, speed <=420 rpm | 293.2 mm |

The last row assumes unverified torque availability far above rated speed.
No row predicts achieved Gazebo or hardware performance. These results
show why the earlier constant 110 mm lever-arm estimate alone is insufficient:
near leg extension the vertical Jacobian shrinks, limiting vertical velocity.
Actual encoder 30..145 degree endpoints are still not mapped to CAD zero;
the simulation knee range is not a hardware travel approval.

## Verification Status

The newer [landing implementation audit](JUMP_LANDING_IMPLEMENTATION.md) records
opt-in world-frame placement, wheel coordination, compression and Hermite
experiments. They have NOT produced a stable >10 cm jump; historical results
below are not superseded by a new passing demonstration.

The experimental implementation is present. The requested >0.10 m jump with
controlled landing has NOT passed acceptance for the selected motor combination.
Best measured rise is approximately 0.08634 m, followed by a failed landing. No validated jump
demo or hardware-ready controller is delivered by these results.

### Bounded Extension Development

The isolated `src/tap_control/config/jump_development.yaml` enables the optional
`jump.thrust_knee_force_control` branch. Despite its name, this is a bounded
velocity-driven push with Jacobian feedforward, not a measured ground-force
feedback controller. It removes the knee position spring while that foot is
grounded in Thrust, requests bounded extension speed, and cuts that side's thrust
feedforward at its measured extension endpoint. A per-side latch prevents a
second push after recoil; reset and a new accepted jump clear the latch. Flight
restores knee impedance. Measured contacts, safety stops and actuator watchdogs
remain authoritative; reaching the extension endpoint never proves takeoff.

The knee endpoint may remain bent, and the landing pose may be more extended
than the release pose. Both stay inside the existing provisional CAD envelope.
This permits early release and leaves compression travel for landing. No masses,
inertias, contact parameters, torque caps or motor speed curves were increased.
The requested 300 N reaction is still limited by the plugin's 5 Nm leg and
0.13 Nm wheel envelopes. Normal/demo configurations do not enable this branch.

| Evidence directory under `log/` | Conservative COM rise | Outcome |
| --- | ---: | --- |
| `jump_1789215803435078574` | 0.05319 m | Measured-speed reference; landing pitch guard |
| `jump_1789215950202788465` | 0.05645 m | 300 N request; landing pitch guard |
| `jump_1789216118435939239` | 0.07158 m | Bounded extension speed; landing pitch guard |
| `jump_1789216202718639422` | 0.07312 m | Flight hip feedback; low-body guard |
| `jump_1789216290217657029` | 0.07165 m | -0.65 rad release endpoint; low-body guard |
| `jump_1789216365273152157` | Not airborne | Pre-jump readiness failed |
| `jump_1789216408625648636` | 0.08464 m | -0.95 rad endpoint; landing pitch guard |
| `jump_1789216542871977404` | 0.08473 m | Latched release; landing pitch guard |
| `jump_1789216614567412841` | 0.08487 m | More extended landing pose; landing pitch guard |
| `jump_1789216663751564324` | 0.08500 m | Independent repeat of final configuration; landing pitch guard |

The last two rows used controller SHA256
`9d1914a0550a0d511ddf0426bc821196ba786e6798c0f53dc0a5add575a4412e`.
Each trial archives its own source/configuration/binary hashes. These are failed
jump trials, not evidence of five-second recovery or of the requested 10 mm
peak XY / 2 mm residual bounds. All listed trials cleaned up their owned live
processes. No repeat-jump demo is enabled by this work.

Source-level verification includes 94 synthetic jump checks (endpoint release,
per-side gating, recoil latch, reset and stop) and the no-allocation core test.
The Python jump description/launcher/envelope/measurement suite passed 43 tests.
`tools/analyze_jump_trace.py` supplies offline FK, sampled actuator work and
ballistic diagnostics for flight logs; it is not a force sensor or a capability
certificate. Its flight analysis requires takeoff/landing evidence.

Reproduce the isolated experiment from a sourced TAP overlay:

```bash
python3 tools/jump_smoke.py --domain 125 \
  --config src/tap_control/config/jump_development.yaml \
  --jump-leg-velocity-damping .6 --jump-leg-speed 28.274333882308138 \
  --max-xy .01 --max-residual .002
```

The remaining control problem is coordinated takeoff momentum and touchdown
foot placement, not simply a larger force request. These results do not prove
that 10 cm is physically impossible with the selected motors, and do not
validate operation on hardware or ESP32-S3.

Preflight baseline `log/jump_1789202806723857487` failed before any jump request:
the torso fell to 0.13841 m while all six applied actuator efforts remained
zero despite leg effort requests reaching +/-3 Nm. The disabled-to-enabled
transition spanned only 5 ms of simulation time but 671 ms wall time during
startup, exceeding the initial 100 ms gate handshake. Neither wheel contact
stream delivered packets. This is integration failure, not evidence that the
motors lack strength. The harness stopped the run; emergency-stop zeroing
passed and the trial's owned processes were removed. Gate startup semantics
and contact routing must be corrected before jumping.

Those two integration faults were subsequently corrected: physics contact
snapshots now include empty airborne heartbeats, and disabled startup may wait
for an explicit enable without being confused with an enabled-stream timeout.
`log/jump_1789203080139232077` observed both contact streams and nonzero bounded
actuation, but failed pre-jump settling. No jump was requested.

Further bounded-actuator preflight trials (all without jump requests):

| Log | Local active velocity damping | Outcome |
| --- | --- | --- |
| `jump_1789203419961264063` | 0.1 Nms/rad | Pitch guard at +0.517 rad |
| `jump_1789203472071111489` | 0.3 Nms/rad | Pitch guard at -0.508 rad |
| `jump_1789203568474231912` | 0.6 Nms/rad | Upright for the 8 s settling window; readiness dropped at its boundary |

The last trial ended near pitch 0.01275 rad, hip 0.110 rad and knee -0.016 rad,
without actuator saturation. It is not a passed baseline: the required readiness
dwell failed. Receipt ordering of contact timestamps and `/clock` is under
investigation. Local active damping is part of the bounded controller torque,
not an unlimited extra passive brake. Passive leg damping is separately 0.02.

Jump-mode URDF velocity values are now generous 1000 rad/s numerical solver
guards, NOT motor ratings. The custom actuator enforces the motor's positive-work
cutoff and torque rolloff without instantaneously clipping physical joint
velocities. This avoids unmodeled constraint braking during launch/impact.

Baseline `log/jump_1789204002667156478` PASSED: 8 s settling and 15 s stationary
hold with local velocity-error gain 0.6 Nms/rad, wheel-frame compensation and
the corrected contact-clock handling. During the 15 s hold, peak COM XY drift
was 0.455 mm, final drift 0.126 mm and maximum absolute pitch 0.012935 rad.
Applied-effort emergency-stop zeroing passed. Binary:
`fa81219f8594770386751b5cc139871d34ce305762a519073f94877e127e7a15`.
This is a bounded-actuator balance baseline, NOT jump validation. A subsequent
reference-loss latch fix was subsequently built and tested; later flight trials
use that plugin and verify applied-effort emergency stopping.

### Measured Flight Trials

These are failed development trials, not approved demo settings. The strict
measurement uses full-world URDF COM and independent collision heartbeats.
Clock startup and the flight detector have dedicated regression tests so a
paused startup is not confused with flight, and intermediate contact samples
cannot erase a genuine last-grounded measurement.

| Log | Profile and change | Post-takeoff COM rise | Result |
| --- | --- | --- | --- |
| `jump_1789204699669857176` | Rated, -1.2 knee, hip attitude 4/0.4 | 0.01318 m | 0.112 s airborne; landing pitch guard |
| `jump_1789204821813033040` | Manual curve, -1.6 knee, 0.12 s thrust, 120 N requested force, hip attitude 20/3 | 0.06269 m | 0.268 s airborne; landing pitch guard |
| `jump_1789204896721389082` | Same deep thrust, hip attitude 4/0.4 | 0.05944 m | 0.256 s airborne; pitch guard at landing |
| `jump_1789205038347230696` | Deep thrust plus integrated flight hip reference | 0.06073 m | Pitch improved, but low-body guard before measured landing |
| `jump_1789205832822611013` | Measured-knee hip coordination, 0.12 s thrust, -0.8 landing knee | 0.06831 m | 0.264 s airborne; landing pitch guard |
| `jump_1789205893776013481` | Coordinated hips, 0.08 s thrust, -0.4 landing knee, attitude 20/3 | 0.06665 m | 0.216 s airborne; landing pitch guard |

These trials retain final actuator bounds of 5 Nm per leg and 0.13 Nm per wheel;
120 N is a desired total reaction, not an externally applied world force or
a promise that the actuators deliver it. Emergency-stop applied-effort zeroing
passed in each. None passed the stationary recovery test or the height target.
The deep configuration is isolated in `jump_tuning.yaml`, not selected by the
normal launcher. Optional integrated flight hip reference and coordination of
thrust hips to measured knee angles remain under evaluation. They default off
outside that development profile. Actual joint travel is independently guarded
while jumping: hip [-0.65, 1.10] rad and knee [-1.75, 0.30] rad, around the
narrower commanded envelope. These are provisional model guards, not a mapped
hardware encoder range or a collision certificate. The deepest measured knee
in the 5.94 cm trial was -1.64329 rad; no positive-knee overshoot was observed.

Node `ca4f856b20eea207c32d81ac25fbcc431e701aa14e3a2c984b3abe4f7dd25db0`
passed 31 jump, 29 general and 30 station synthetic ROS checks. Those checks
prove interface/state handling, not successful physical flight or landing.

### Historical Verification Snapshot

Installed node SHA256:
`c0fb38746a39651df2113c1b6927a1ccd9b7c214206cde64b40565c0e8ed547a`.
Bounded actuator plugin SHA256:
`8eb0367bb48568a7407c2fc74f0e65ca2669303840d86c4d694482889bf96a8e`.

- Control build and all 8 control CTest suites passed, including 81 synthetic
  jump checks. Root Python suite: 71 passed. Actuator CTest: 1 passed. Teleop
  CTest: all 3 suites passed. These are software tests, not physical acceptance.
- Final ROS contracts: jump 40, general 29, station 30, all passed. They include
  Fault surviving Enable, explicit Reset, simulated-time handling, rejection of
  unsupported torque budgets, and the -1 ms timestamp numerical boundary.
- Final bounded-actuator baseline `jump_1789205716599204504` passed 8 s settling
  plus 15 s hold: peak COM XY drift 0.400 mm, final drift 0.289 mm, maximum
  absolute pitch 0.013065 rad. Applied-effort emergency stopping passed.
- `jump_1789205766860272786` was rejected before jumping because readiness
  briefly dropped within the required final 1 s settling dwell. This failure
  is retained; a repeat produced the 6.83 cm flight, not a successful landing.
- `jump_tuning.yaml` retains the 6.83 cm candidate. The flight hip integrator
  is disabled there because its trial improved attitude but left the legs
  poorly placed for landing. No automatic repeated jumping is enabled.

The principal remaining work is a dynamically feasible coordinated takeoff and
landing trajectory, with controlled angular momentum and touchdown impedance.
Static COM-neutral geometry and extra attitude gain alone did not solve it.
Actual motor torque-speed/power and moving-mass allocation remain uncalibrated;
these failures do not establish a general impossibility proof for the hardware.

## Experimental Entry Points

After building and sourcing this workspace, the separate paused simulator
entry points keep controller and actuator speed cases paired:

```bash
./run_jump.sh rated
./run_jump.sh manual-curve
```

Run only one in a given ROS domain / Gazebo partition. `rated` uses the
120 rpm positive-work cutoff. `manual-curve` uses the provisional 5 Nm plateau
through 190 rpm, decreasing linearly to zero at 270 rpm, informed by the
manual's plotted curve. Braking is torque-bounded, not an instantaneous speed
reset. Neither entry point means the requested jump has passed validation.
Both wrappers select local velocity-error gain 0.6 Nms/rad. The node publishes
stamped relative leg velocity targets; Gazebo computes `effort + D*(qd_ref-qd)`
before final torque/speed-envelope saturation. Outside jumps the reference is
zero. This control feedback is not added to the passive mechanical damping.
Ordinary `run_sim.sh` defaults to the unchanged legacy actuator setup.
The jump-enabled ROS adapter currently requires the plugin's fixed total caps
of exactly 5.0 Nm and 0.13 Nm. Other configured budgets are rejected, including
lower ones, because clipping the node's position torque alone would not bound
the plugin's subsequent velocity-feedback torque to that lower budget.

The dashboard's explicit `--experimental-jump` option exposes the single-shot
request. Actual sensor/contact freshness, stable neutral stance and controller
readiness are required. `/jump_request` is a Bool rising edge, not a sustained
velocity command. `/jump_status` reports the phase and failure reason. During
jump ownership, ordinary pose/drive/show controls must not override the motion;
disable and emergency stop remain available.
Fault requires explicit Reset; Enable alone cannot clear it. Repeated callbacks
at unchanged simulation time do not advance jump trajectories or consume a
pending request. Emergency stop and stale/travel guards remain active.
