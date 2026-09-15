# Dashboard And Deep Squat QA

## Scope

The reported desktop failure log (`log/desktop_1789219155/sim_retry.log`)
contains pitch around 0.351 rad and subsequent EmergencyStop transitions.
It does not record operator slider targets or actuator-gate state, so it cannot
establish whether the original collapse began with an unintended manual pose,
a feedback/watchdog failure, or another physical disturbance. Do not call the
following usability and feedback fixes a proven diagnosis of that incident.

Changes are restricted to Tk/ROS GUI and tests; actuator caps, watchdogs,
controller gains and robot physics are unchanged.

- Jump once is always visible, with a separate default-off experimental opt-in.
- Jump status is observed even when opt-in is off, preserving controller pose
  ownership if another authorized simulator client initiates a jump.
- Manual joint scales are locked in coordinated pose modes. Deep squat has a
  nonnegative depth control and retains its existing 11-second full-depth plan.
- GUI drains up to 32 ROS callbacks with a 4 ms budget per 20 ms tick, instead
  of four per 50 ms. Joint/clock/motor displays retain only the latest sample;
  control-mode and jump-phase queues retain transitions.
- Tk Scale configuration writes its variable even on a state-only change.
  Such GUI refreshes are explicitly excluded from operator-change callbacks.
  The first new test caught this regression as a held trajectory, not a fall.

## Real GUI Simulation

`tools/gui_deep_smoke.py` constructs the actual Dashboard and runs its normal
Tk timer, subscriptions and publishers. It selects the workflow and changes
the same Tk variables used by sliders. This tests GUI callbacks/physics, not
physical mouse dragging or real motors. Both GUI and Gazebo rendering are on.
The harness uses an owned simulation, an isolated ROS domain/Gazebo partition,
and loopback Gazebo transport on this PC. It never attaches to real hardware.

Admission: five simulation seconds of neutral settling, then GUI readiness.
Scored bounds: 10 mm peak XY, 2 mm residual XY (latest harness), body Z >=
0.115 m, absolute pitch <= 0.3 rad, knee depth at least -1.55 rad. Any GUI hold,
stale/disarmed controller, missing return or timeout fails the run.
Startup settling displacement is not included in the scored drift.

`log/gui_deep_1789219680094471916`: PASS, two lower/return cycles,
1,314 samples, body Z minimum 0.126838 m, peak pitch 0.026817 rad,
peak XY 1.797 mm, computed final XY residual 0.0041 mm, peak commanded/measured
joint error 0.04675 rad. This run preceded adding the explicit residual gate;
the trace independently satisfies that gate. Cleanup reports no live leftovers.

Jump execution and stable landing are not claimed by these squat tests.

`log/gui_deep_1789219807378765021`: PASS with `--gui --drag`, two cycles
retargeting depth from 1 to 0.25 and back to 1 before each full-depth hold.
1,565 samples, minimum body Z 0.126838 m, peak pitch 0.027105 rad,
peak XY 1.805 mm, final XY residual 0.0019 mm, maximum joint error 0.04649 rad.
Explicit residual gate passed; cleanup reports no live leftovers.

Dashboard callback/widget tests: 54 passed, including actual Tk creation with
the Jump once button visible and disabled by default, ownership observation
without opt-in, bounded feedback draining, and Scale-refresh callback isolation.
