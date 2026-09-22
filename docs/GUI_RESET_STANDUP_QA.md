# Dashboard Reset Standup QA

## Scope

The dashboard's **Reset standup (sim-only)** button (`src/gui/gui/
dashboard.py`) lets a fallen or badly posed robot be recovered for testing
without restarting the whole Gazebo simulation. It is a simulation-only test
convenience: it teleports the Gazebo world pose via `gz service`, which no
hardware equivalent exists for. It is not evidence of physical recovery
capability, and it does not change the validated status of stand-up recovery
recorded in `docs/ALGORITHM_MIGRATION.md` (still FAILED for the tested
90-degree lying pose) — see `.agents/skills/tap-ros-gazebo/SKILL.md` ("Keep
Gazebo world-reset commands simulation-only; reflect unsupported actions
honestly") and `.agents/skills/tap-qa-regression/SKILL.md` ("do not inherit
reference robot results").

The sequence, in `Dashboard._reset_standup_gz_phase`/`_advance_reset_standup`:

1. Pause the world (`/world/empty/control`, proven in
   `src/legacy/ros2_gazebo_sim/scripts/activate_controllers`).
2. Publish `/emergency_stop=true`.
3. Teleport the robot base to whatever pose is currently in the dashboard's
   **Spawn X/Y/Z** fields (`/world/empty/set_pose`, proven in
   `tools/recovery_smoke.py`), identity orientation. These fields default to
   `x=0 y=0 z=0.2372` for `terrain=none` (matching `sim.launch.py`'s
  `spawn_z` default and `src/legacy/ros2_gazebo_sim/CONTRACT.md`'s documented wheel-ground
   height) or `x=-4.6 y=0 z=0.4372` for `terrain=rough` (the flat test pad
   in `rough_training.sdf`), and are read fresh on every `Reset standup`
   click — not fixed at launch time — so a custom position (e.g. edited
   after driving the robot somewhere specific) is honored without
   relaunching Gazebo. Unpause immediately — see "No settle-and-verify step"
   below for why.
4. Hand off to the dashboard's own existing button methods, in order:
   `reset()` (publishes `/reset_controller`) → `clear_estop()` → `neutral()`
   (polled every tick; only clears `pose_commands_blocked` once measured
   feedback is fresh and in bounds — the existing safety gate is reused, not
   bypassed) → `enable()` (publishes `/enable=true`). This calls `enable()`,
   not the separate experimental `stand_up()` recovery sequence:
   `StateMachine::update` (`src/tap_control/core/src/StateMachine.cpp`) latches
   `EmergencyStop` unconditionally whenever the last `/enable` publish was
   `false` (`request_disable`), regardless of `/stand_up` or tip angle, so only
   an explicit `/enable=true` can exit it — and since the teleport already
   placed the base upright, no Tuck/Rise/Settle sequence is needed here.
5. Confirm 2 continuous seconds of `Balance` mode with `abs(pitch) < 0.2 rad`
   before reporting done; a ~10 s timeout on step 4's neutral wait, a ~15 s
   timeout on this confirm step, and any gz service failure, fail the sequence
   instead of hanging.

### No settle-and-verify step (and no gravity-zero)

Earlier iterations of this sequence tried to step the paused world in bounded
increments after teleporting, checking `/control_mode`'s reported pitch/pose_z
before declaring the base "settled" and handing off to the controller — first
under real gravity, then (after that toppled the robot before handoff, see
below) with gravity zeroed for the settle window, matching the wheel_leg
reference GUI's own reasoning for that step.

Both were abandoned after real end-to-end runs against this workspace's
Gazebo Sim 8.10 build:

- **Real gravity during settle**: a plain `set_pose` only moves the base
  link, so after a fall the legs keep whatever angles they ended up at.
  Stepping physics with real gravity active against that mismatched leg
  configuration immediately toppled the robot again (`pitch` swinging
  1.57 -> -0.58, `pose_z` sinking to 0.036 m, i.e. on the ground) before the
  controller ever regained control.
- **Zeroing gravity for the settle window**: `gz service -l` confirmed
  `/world/empty/set_physics` (`gz.msgs.Physics`) exists, and a direct manual
  test (`gz service -s /world/empty/set_physics ... --req 'gravity {x: 0.0
  y: 0.0 z: 0.0} ...'`) replied `data: true` — but a teleported test body
  still fell from `z=0.5` to `z=0.309` in the following 200 ms of stepped
  physics, i.e. at essentially normal `-9.81` gravity speed. The service
  accepts the request but does not change runtime gravity on this build, so
  it cannot be used for this purpose here.

The sequence therefore does not attempt to verify a "settled" upright base at
all: it trusts `set_pose`'s own `data: true` reply, unpauses immediately, and
lets the real, actively-running controller (once `enable()`d) attempt to
recover from whatever leg configuration remains. Step 5's confirm timeout
reports failure honestly if that does not succeed — this is a genuine
platform/physics limit, not a bug to route around, and the feature does not
claim to guarantee recovery from every fall (see "Evidence" below for how
severe a fall this was actually verified against).

## Unit tests

`src/gui/tests/test_dashboard.py` (`ResetStandupGateTest`,
`ResetStandupHandoffTest`, `OnModeLastPitchTest`) and `src/gui/tests/
test_gz_world.py` mock every ROS/gz call and drive the gate logic and the
`_advance_reset_standup` state machine directly. They do not open a window,
initialize a ROS node, launch Gazebo, or call the real `gz` CLI. Run via
`colcon test --packages-select gui` or directly with `python3 -m
unittest discover -s src/gui/tests -p "test_*.py" -v`.

## Real GUI + Gazebo simulation

`tools/gui_reset_standup_smoke.py` follows `tools/gui_deep_smoke.py`'s
template: it constructs the actual `Dashboard`, drives it through its normal
Tk timer, and owns its Gazebo subprocess (isolated `ROS_DOMAIN_ID`/
`GZ_PARTITION`, `tools/owned_process.py` for launch/cleanup). It forces a
fallen precondition via `/world/empty/set_pose` — a moderate `0.9` rad tip
(just over the `0.8` rad `mpc.fall_pitch_rad` threshold in
`balance_controller.yaml`), deliberately milder than `tools/recovery_smoke.py`'s
own `1.2` rad near-full-topple fixture, since this feature targets ordinary
testing mishaps, not the hardest fall case — then estops, waits 0.5 s, and
calls the real `Dashboard.reset_standup()`, asserting it reaches `'done'` with
the sustained-upright condition from step 5 above **twice in the same
simulation session**, which is the specific requirement this feature exists
for (no Gazebo restart between test cycles).

Pass/fail: both cycles must reach `reset_standup_stage == 'done'` within their
stage deadlines (60 s for startup, 20 s to observe the forced fall, 60 s for
the reset-standup sequence itself per cycle); a `reset_standup_error`, a
simulator exit, or a stage deadline miss fails the run. Cleanup must report no
live process-group leftovers.

Run: `python3 tools/gui_reset_standup_smoke.py` (after sourcing the workspace;
creates and cleans up its own simulation session).

### Fixed: calling Reset upright while already balanced used to tip the robot

Every scripted test of this feature (the smoke tool above) starts from a
**forced fallen precondition**. A manual real-GUI check found the other
case — the robot already standing normally in `Balance` mode, Reset upright
clicked with no fall at all (should be a near no-op: teleporting to the pose
it already occupies) — reproducibly failed: pitch grew to 0.5-1.6 rad during
the sequence and it ended in `EmergencyStop`/`Fallen` instead of confirmed
`Balance`.

**Root cause, found by reading the actual mode-transition log
(`Control mode: ...` lines, only emitted on real transitions) during a
reproduction**: the old sequence unpaused immediately after teleporting,
then handed off to a multi-tick main-thread state machine
(`awaiting_reset` -> `awaiting_neutral` -> `awaiting_enable`) that called
`reset()` then `clear_estop()` -- both call `disable()` internally, which
publishes `/enable=false`. `BalanceControllerNode`'s `enable` subscription
sets `command_.request_disable = !msg->data`, so publishing `enable=false`
sets `request_disable=true`, and `StateMachine::update`'s very first check
is `if (state.emergency_stop || command.request_disable) enter(EmergencyStop)`
-- meaning every `disable()` call re-enters EmergencyStop immediately,
regardless of the controller having briefly re-enabled moments earlier. The
observed real log trace was `Balance -> Disabled -> Balance -> EmergencyStop`
within about 400 ms: the sequence's own bookkeeping calls fought its own
progress and lost.

**Fix**: `_reset_standup_gz_phase` now publishes `reset_controller`, clears
estop, and enables directly (plain thread-safe `Bool` publishes) BEFORE
unpausing -- mirroring the wheel_leg reference GUI's actual structural
principle (it never resumes real-time physics until after publishing its
own equivalent of enable, via bounded paused `multi_step` calls). The
main-thread handoff was simplified to a single `'confirming_sync'` stage
that only syncs `self.enabled`/`self.estopped` bookkeeping -- it no longer
calls `reset()`/`clear_estop()`, since teleporting always sets an upright
orientation regardless of whether this was a trivial no-op reset or a real
fall, so there is nothing left for those calls to accomplish once the
background phase has already published enable.

**Verified real-GUI-and-Gazebo (not just unit-mocked), isolated sim
session**: from `Balance`, pitch 0.017 rad, calling `reset_standup()` now
completes with `reset_standup_stage == 'done'` and a **peak pitch of only
0.029 rad for the entire sequence** (final pitch 0.017 rad, i.e. it returns
to the exact same standing state it started from) -- matching the
"quay về chỗ cũ để chạy lại" (return to the same spot to run again)
behavior this was supposed to provide. Before the fix, the identical
scenario measured peak pitch 0.5-1.6 rad and ended disabled.

Separately, recovering from a genuine severe fall (a forced 90-degree tip)
was also re-tested after this fix and still fails -- it gets stuck in
`Fallen` mode and the sequence times out honestly rather than hanging or
crashing. This is not a new regression: `docs/ALGORITHM_MIGRATION.md`
already documents stand-up recovery failing the tested 90-degree lying pose
before any of today's changes. Root cause not fully chased down, but a
plausible mechanism was observed: `StateMachine::update`'s unconditional
`if (tipped) { enter(Fallen); return; }` check runs before the switch
statement's own `case Fallen: if (request_enable) enter(Enabling)` recovery
path can ever execute, so as long as the CONTROLLER's own cached pitch
reading has not yet caught up to a just-completed teleport (plausible while
the world is paused, if the bridged pose topic only updates on physics
steps), a single `enable=true` published during that stale window is
silently discarded with no later retry. Fixing this properly is a separate,
harder problem than the one reported this session and was intentionally
left alone rather than risking another blind change to a working case.

### Evidence

Four real runs against this workspace's Gazebo Sim 8.10 build
(`log/gui_reset_standup_1789355821590924300` through
`log/gui_reset_standup_1789357148226147173`), used to develop and debug the
sequence above, consistently verify the **choreography**: pause →
`/emergency_stop=true` → `/world/empty/set_pose` teleport → unpause →
`reset()` → `clear_estop()` → `neutral()` (correctly parks until
`pose_commands_blocked` clears, then advances) → `enable()` → `confirming`
(correctly reads `/control_mode` and detects both success and failure)
execute in the right order with the right arguments every time, including
after the fixes recorded in "No settle-and-verify step" above.

The **outcome** in every run so far, including the final `0.9` rad / `0.5` s
settle configuration described here
(`log/gui_reset_standup_1789357148226147173/result.json`): the base teleports
and the handoff proceeds correctly, but the legs are already significantly
bent (knee around `-0.51` to `-0.57` rad on both sides, well past the
`[-0.8, 0.2]` operating envelope's midpoint) by the time control resumes —
this happens essentially instantly on teleport, not gradually, so shortening
the settle wait from 3 s to 0.5 s did not change it. `enable()` briefly
reaches `Balance` (or skips straight through), pitch then diverges further
(observed swinging through 1.57, -0.33, -0.77 rad across the four runs), and
the controller's own watchdog re-enters `EmergencyStop` before 2 continuous
seconds of small-pitch `Balance` is ever observed, so `reset_standup_stage`
correctly reports `'failed'` with `"timed out waiting for Balance mode
(last: EmergencyStop)"` and the tool exits non-zero.

**This is an honest, expected result, not a bug in the GUI feature**: it is
consistent with — not contradicting — this repo's existing, separately
recorded finding that general fall recovery is unsolved
(`docs/ALGORITHM_MIGRATION.md`: stand-up recovery "FAILED tested lying-90-degree
recovery"). Reset standup was never meant to solve that problem; it reliably
performs the world-pose teleport and the reset/enable handoff (the part a GUI
button can actually own), and then hands off to the same balance controller
whose recovery capability is tracked and limited elsewhere in this repo. A
future pass here would mean the underlying MPC/whole-body controller has
gotten better at recovering from this specific leg configuration — it would
not mean the GUI choreography changed, since that part is already verified.
Do not read a future PASS from this tool as new stand-up-recovery evidence
without also updating `docs/ALGORITHM_MIGRATION.md`.
