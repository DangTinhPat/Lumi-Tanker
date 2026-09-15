# TAP dashboard and sim lifecycle

Executable: `ros2 run gui gui`. Curve/trajectory generation (native pose
order, squat/roll/deep-squat tables) lives in the separate `tap_teleop`
package and is imported here; see `src/tap_teleop/README.md` for that math.

**Package name collision warning:** this machine's `~/bin/ros2` wrapper
sources `/home/dvt/wheel_leg/install/setup.bash` (if present) before running
the real `ros2` — and that workspace also installs its own `gui` package
(the reference GUI this one was adapted from). Always `source
install/setup.bash` **from this TAP workspace** yourself (last, or instead of
using that wrapper) before `ros2 run gui gui`, or you may silently launch
wheel_leg's GUI instead of this one. There is no safety wrapper script here
by design — this naming was chosen to match wheel_leg's own `ros2 run gui
gui` exactly, and the collision risk that comes with it is accepted.

## Simulation lifecycle

The "Launch sim" / "Stop sim" button in the Simulation panel directly builds
and owns a literal `ros2 launch tap_sim sim.launch.py` subprocess itself
(`gui/sim_process.py`, via `tools/owned_process.py`'s session-verified
`start_owned_process`/`cleanup_group`) — matching the wheel_leg reference
GUI's own pattern exactly (it shells out to `exec ros2 launch
balance_firmware_core sim.launch.py`), rather than shelling out to a
separate wrapper script or an inline ros2cli Python invocation. Requires
`install/setup.bash` to already be sourced in the shell `ros2 run gui gui`
was started from (the usual precondition for this workspace, see the
collision warning above) — the subprocess inherits that environment, so the
launched sim resolves `tap_sim` correctly even though this machine's
`~/bin/ros2` wrapper also layers in `/home/dvt/wheel_leg/install`.
`run_sim.sh` itself is untouched and still used standalone by
`tools/*_smoke.py` and manual headless runs; the GUI just no longer calls it.
A plain test cycle needs only `ros2 run gui gui` — no separate terminal.
Closing the window stops a sim launched this way; a sim started independently
(by hand, or by a QA tool) is never touched, because `cleanup_group` refuses
to act on any process it did not spawn itself.

The **Terrain** dropdown next to Launch sim selects `sim.launch.py`'s
`terrain` argument (`none` or `rough` — see "Rough training terrain" below);
it locks while a sim is running. Other `run_sim.sh`/`sim.launch.py` arguments
— Stationary Show, Deep Squat, jump profiles, precision pose — still need the
two-terminal `./run_sim.sh <args>` + `ros2 run gui gui` flow; a general
argument-passthrough UI is a documented follow-up, not built yet.

The **Spawn X/Y/Z** fields next to Terrain hold the position used both by
Launch sim (as `spawn_x`/`spawn_y`/`spawn_z` launch args) and by Reset
upright (as its teleport target, see below) — so both always agree on
"home" for whatever terrain is selected. Picking a Terrain value refills
these fields with that terrain's documented default (`none`: `0 0 0.2372`;
`rough`: `-4.6 0 0.4372`, matching `sim.launch.py`'s own defaults), but they
stay editable afterwards: change them to spawn somewhere else, or to move
Reset upright's recovery point to wherever testing needs it (e.g. onto the
rough terrain's step pad, or past it) without relaunching Gazebo. An invalid
(non-numeric or non-finite) value blocks both Launch sim and Reset upright
with a status message instead of silently falling back to a default.

Before launching, the dashboard refuses if a ROS graph already publishes
`/clock` on this domain (can't safely determine whether it's owned by another
TAP session), rather than guessing and risking a double-launch.

Launch sim always requests `auto_enable:=true`, so `activate_controllers`
publishes `/enable=true` itself as soon as controllers are ready -- a plain
test cycle needs no separate manual Enable click. The dashboard's own
`self.enabled` bookkeeping (which gates Drive and the other enabled-only
controls) is synced automatically once the sim log shows the "Running"
milestone (see `_tick_sim`), not just when the Enable button is clicked.
The Enable/Disable/Clear E-stop/Reset controller buttons in the row below
the header still exist for manual use afterward (e.g. after an intentional
Disable or E-stop).

The **Reset upright (sim-only)** button lives in the Simulation panel itself
(next to Launch sim), not further down the window, since recovering from a
fall and continuing a test cycle is exactly what that panel is for.

## Reset upright (sim-only test convenience)

**This is a simulation-only shortcut for iterating on tests, not evidence of
physical recovery capability.** It pauses the Gazebo world, e-stops, teleports
the robot's base back to whatever pose is currently in the **Spawn X/Y/Z**
fields described above (`/world/empty/set_pose`, identity orientation —
defaulting to `x=0 y=0 z=0.2372` for `none` terrain, matching
`sim.launch.py`'s `spawn_z` default and `src/tap_sim/CONTRACT.md`'s
wheel-ground height), unpauses, then hands off to the real controller through
the dashboard's
existing `Reset controller` → `Clear E-stop` → `Neutral pose` → `Enable`
sequence — reusing the existing `pose_commands_blocked` safety gate rather
than bypassing it — and confirms 2 continuous seconds of `Balance` mode with
small pitch before reporting done. It calls `Enable`, not the separate
experimental `Stand up` recovery sequence: the teleport already places the
base upright, and the controller's `EmergencyStop`/`Fallen` modes only exit
via an explicit `/enable=true` regardless of `/stand_up`.

It uses only the two `gz service` calls already proven working elsewhere in
this repo (`tools/recovery_smoke.py`, `src/tap_sim/scripts/
activate_controllers`): `/world/empty/control` (pause) and
`/world/empty/set_pose` (teleport). A `/world/empty/set_physics` gravity-zero
step (matching the wheel_leg reference GUI, to give the base a weightless
settle window before the legs' unchanged, possibly bent angles can act on
it) was tried and measured to have **no effect on runtime physics** on this
Gazebo Sim 8.10 build — the service replies `data: true` but the robot keeps
falling at normal gravity speed — so it is not used. **A plain `set_pose`
only moves the base link**: the legs keep whatever angles they had when the
robot fell, and whether the controller then recovers from that leg
configuration once `Enable`d is a real open question this feature does not
control — see `docs/GUI_RESET_STANDUP_QA.md` for the measured gravity-zero
investigation and real end-to-end runs, including ones where the teleport and
handoff both executed correctly but the controller did not recover.

Requires Experimental motions and a fresh sim clock; blocked while a jump,
Show, or Deep squat is in progress, or while a previous reset-standup attempt
is still running. All status text this feature produces is prefixed
`[SIM-ONLY]`, and success is reported as "test convenience only, not evidence
of physical recovery" — per `.agents/skills/tap-ros-gazebo/SKILL.md` ("Keep
Gazebo world-reset commands simulation-only; reflect unsupported actions
honestly") and `.agents/skills/tap-qa-regression/SKILL.md` ("do not inherit
reference robot results"). See `docs/GUI_RESET_STANDUP_QA.md` for the scripted
end-to-end evidence and its pass/fail thresholds.

## Dashboard behavior

The native pose order is `[cad_left_hip, cad_left_knee, cad_right_hip,
cad_right_knee]`, radians, all axes +Y. Left and right use the same signs.
The provisional GUI envelope is hips `[-0.6, 0.6]`, knees `[-0.8, 0.2]`.
Negative knees shorten this CAD robot. At knee zero the computed neutral hip
is approximately `+0.1154831` rad, not zero. These limits are operational
estimates, not the supplier's yet-unmapped mechanical limits.

`/cmd_vel` (Twist) is published at 20 Hz; hold a direction button to move.
Release, pointer leave, focus loss, disable, and E-stop publish zero velocity.
A UI delay above 0.3 seconds or controller feedback older than 1 second
disables control. The controller must also enforce its own command timeout:
a dead GUI cannot publish a stop. The GUI never periodically re-enables.

Other output topics are `/target_pose` (Float64MultiArray, four angles),
`/enable`, `/emergency_stop`, `/stand_up`, `/reset_controller` (Bool).
Feedback is `/control_mode` (String) and `/motor_commands` (JointState).
The manual `Reset controller` button only requests a controller reset; it
does not reposition the robot — only the separate, explicitly-labeled Reset
standup button teleports the world pose, and only as described above.
Clearing E-stop leaves control disabled. Experimental stand-up can be
explicitly requested while disabled or fallen; it activates the local
keepalive and sends the recovery trigger without clearing E-stop. Fallen,
disabled, and emergency stop feedback clear local drive enable; only the
telemetry `estop=1` flag latches E-stop until the user explicitly clears it.
EmergencyStop mode alone does not relatch a cleared stop, since the FSM also
uses it for watchdog stops.

The header displays only mode, pitch, pitch reference, CoM height, sensor
freshness, and wheel speed. Motor command efforts are displayed as torque
limits, not measured motor effort.

Manual small poses are limited to 0.15 rad around neutral unless experimental
motions are selected. Squat, body roll, wiggle, and stand-up are experimental:
FK-neutral does not establish dynamic balance, ground-contact feasibility,
motor capacity, or collision clearance. Pose targets slew at 0.3 rad/s.

Runtime dependencies: ROS 2 rclpy, std_msgs, geometry_msgs, sensor_msgs,
rosgraph_msgs, ament_index_python, Tk, tap_robot_description, and
tap_teleop (for curve generation). Build uses ament_cmake and
ament_cmake_python. Offline validation:

```bash
python3 -m unittest discover -s src/gui/tests -p test_dashboard.py -v
python3 -m unittest discover -s src/gui/tests -p test_gz_world.py -v
python3 -m unittest discover -s src/gui/tests -p test_sim_process.py -v
colcon test --packages-select gui
colcon test-result --verbose
```

The mocked dashboard/gz_world/sim_process tests are registered with CTest
when `BUILD_TESTING` is enabled. They do not open a window, initialize a ROS
node, launch Gazebo, or call the real `gz` CLI.

Optional simulation-only deep squat profile:

```bash
ros2 run gui gui --deep-squat-knee 1.6
```

See `src/tap_teleop/README.md` for the full deep-squat trajectory contract
(gates, timing, GUI cap) — unchanged by this package split.
