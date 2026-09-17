# Lumi-Tanker Architecture

This document describes the implementation currently present in this workspace.
It is an architecture map, not a claim that every controller mode or landing
strategy has been validated on hardware. The reference `wheel_leg` workspace is
provenance only; it is not part of the runtime path. The current implementation
is being verified in ROS 2/Gazebo, but the deployment target is an ESP32-S3
N16R8 and the deployable control path is restricted to the hardware contract
in Section 2.4.

## 1. System Overview

Lumi-Tanker is a six-joint wheel-legged robot. The system is split into five
layers:

```text
CAD meshes, URDF, inertias, collision model
                    |
                    v
Gazebo / real sensor interfaces ---> BalanceControllerNode (ROS 2 adapter)
                                      |
                                      v
                              EmbeddedController (ROS-free core)
                                      |
             +------------------------+-------------------------+
             |                        |                         |
        normal balance          landing stack              jump stack
        StateMachine + MPC      flight/contact FSMs         JumpController
             |                        |                         |
             +------------------------+-------------------------+
                                      v
                         wheel and leg actuator commands
```

Runtime ownership:

| Area | Package/path | Responsibility |
| --- | --- | --- |
| Robot description | `src/tap_robot_description` | CAD meshes, URDF/Xacro, inertials and preview model |
| Control core | `src/tap_control/core` | Deterministic FK, MPC, state machine, trajectories and landing logic; no ROS/Gazebo dependency |
| ROS adapter | `src/tap_control/ros2_nodes/BalanceControllerNode.cpp` | Sensor assembly, timestamp/freshness checks, parameter loading, command publication and status |
| Simulation | `src/tap_sim` | Free-body Gazebo model, ros2_control, bounded experimental actuators, diagnostic contact telemetry and worlds |
| Teleoperation | `src/tap_teleop` and `src/gui` | Pose curves, dashboard commands, simulation lifecycle and simulation-only reset convenience |
| Offline tools | `tools/` | Model generation, smoke trials, traces, audits and jump-plan tooling |

## 2. Physical Model Contract

### 2.1 Frames and signs

- CAD assembly is joint zero.
- All six native joint axes are ROS `+Y`, including both right-side legs. The
  right side is not mirrored by negating its joint angles in software.
- Body visual and body inertial data are rotated by `-pi/2` about the body/hip
  `Y` axis. This is a mesh and inertial-frame convention, not an IMU-frame
  correction and not a joint-axis rotation.
- The planar controller uses body-frame `x` for fore/aft and `z` for height.
  Yaw, roll and angular rates are supplied by the IMU; support/contact is a
  sensor-derived hypothesis, not a direct hardware measurement in the current
  inventory. The current whole-body FK is sagittal.

### 2.2 Geometry and mass

| Quantity | Current value/status |
| --- | --- |
| Wheel radius | `0.030 m` |
| Axle centre spacing | `0.309 m` |
| Half-track | `0.1545 m`; this is not the `0.334 m` outer width |
| Approximate total mass | `3.938 kg` |
| Body mass | `2.902 kg` |
| Each thigh / shin | `0.233 / 0.150 kg` |
| Each wheel plus motor | `0.135 kg`, provisional lumped estimate |

The seven link inertias and COMs are uniform-density mesh estimates. The body,
motor mass allocation, wheel rotor inertia, mechanical limits and actuator
ratings are not fully measured. The authoritative assumptions are in
[`PHYSICS.md`](src/tap_robot_description/description/cad_robot/PHYSICS.md).

### 2.3 Joint order

The canonical internal and external named order is:

```text
0 LH  cad_left_hip
1 LK  cad_left_knee
2 LW  cad_left_wheel
3 RH  cad_right_hip
4 RK  cad_right_knee
5 RW  cad_right_wheel
```

ROS joint messages are name-mapped. Consumers must not rely on an arbitrary
incoming array order. Leg pose commands use the four-element order
`[LH, LK, RH, RK]`; wheel commands use `[LW, RW]`.

### 2.4 Hardware sensor and actuator contract

The planned physical hardware currently available to the controller is:

| Hardware | Available signal | Use / limitation |
| --- | --- | --- |
| MPU6500 | 3-axis accelerometer + 3-axis gyroscope | Pitch/roll/rates, specific-force and flight/impact evidence; no absolute height or terrain identity |
| Four leg motor encoders | Position and derived velocity | Leg FK, Jacobians, pose and motion residuals; no direct leg load measurement |
| Two wheel motor encoders | Position and derived velocity | Wheel odometry and wheel-motion residuals; no direct wheel support measurement |

The following are permanently excluded from this robot's hardware and control
contract: wheel contact switches, force/torque sensors, motor current/torque
feedback, range sensors, cameras, GPS and external localization. They must not
be introduced later as hidden dependencies for landing, WBC, MPC or recovery.
Motor command mode and measured actuator dynamics remain a separate hardware-
integration item; the simulation's leg-effort and wheel-velocity interfaces are
not proof that the final motors expose those same interfaces.

Consequences for landing:

- Absolute ledge height and vertical position are not observable from this
  permanently restricted inventory.
- Touchdown/support is an inferred hypothesis from IMU + encoder consistency,
  with confidence and persistence; it is not a boolean physical measurement.
- An accelerometer shock cannot distinguish a wheel touchdown from a body/edge
  collision by itself.
- A contact-aware WBC requiring measured contact wrench is permanently outside
  the hardware scope. A model-based leg impedance/WBC may use an estimated
  support mode, but must fail conservatively when confidence is low.
- Gazebo contact and pose are allowed only for offline scoring/diagnosis and
  must never enter the deployable core's command decision.

## 3. Sensor and Command Data Flow

`BalanceControllerNode` receives asynchronous ROS messages, stores the latest
validated sample, and runs the core from a simulation-time timer. The core only
sees `RobotState` and `ControlCommand` from
[`Types.hpp`](src/tap_control/core/include/robot_control/Types.hpp).

### 3.1 Inputs

| Topic | Type | Use |
| --- | --- | --- |
| `imu` | `sensor_msgs/msg/Imu` | Gyro pitch/yaw/roll rates, raw specific-force magnitude and pitch estimation |
| `joint_states` | `sensor_msgs/msg/JointState` | Named joint position and velocity; wheel odometry is derived here. Simulation effort fields are diagnostic only because motor current/torque feedback is not in the hardware contract |
| `left_wheel_contacts` / `right_wheel_contacts` | `ros_gz_interfaces/msg/Contacts` | Gazebo-only diagnostic contact telemetry; not a deployable sensor input |
| `/tap_robot/pose` | `tf2_msgs/msg/TFMessage` | Dedicated simulation/offline diagnostic pose stream; never a deployable control input |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Forward velocity and yaw-rate requests; expires after `cmd_vel_timeout_s` |
| `enable` | `std_msgs/msg/Bool` | Enable/disable request |
| `emergency_stop` | `std_msgs/msg/Bool` | Immediate software emergency-stop latch |
| `stand_up` | `std_msgs/msg/Bool` | Experimental stand-up request |
| `reset_controller` | `std_msgs/msg/Bool` | Reset controller state; it does not teleport or repair the robot |
| `target_pose` | `std_msgs/msg/Float64MultiArray` | `[LH, LK, RH, RK]` target in radians |
| `jump_request` | `std_msgs/msg/Bool` | Rising-edge request for the opt-in JumpController path |

The production pitch source is MPU6500-backed. `sim_pose` is an explicit
simulation/offline diagnostic option, not a physical estimator. The current
ROS adapter still assembles a legacy `floor_contact` field from Gazebo contact
telemetry for simulation experiments; the hardware-faithful target path must
replace that field with a support observer derived only from MPU6500 and the
six encoders. Stale or malformed IMU/encoder data fails closed and commands a
safe stop.

The IMU callback calculates raw specific-force magnitude from the three linear
acceleration components. It also latches a short fused high-acceleration/high-
pitch-rate impact event so a one- or two-sample impact is visible to the slower
control tick. The pitch estimator uses gyro integration plus a Kalman
angle/bias filter and rejects or downweights accelerometer tilt during
free-fall, fast rotation and large linear acceleration.

Wheel odometry is the average of the two named wheel positions/velocities,
converted with radius `0.030 m` and the configured common wheel polarity. It is
not simulator pose odometry.

### 3.2 Outputs

| Topic | Type / order | Use |
| --- | --- | --- |
| `wheel_effort_controller/commands` | `Float64MultiArray [LW, RW]`, rad/s | Normal wheel velocity references |
| `leg_effort_controller/commands` | `Float64MultiArray [LH, LK, RH, RK]`, Nm | Leg PD plus bounded feedforward torque |
| `motor_commands` | `sensor_msgs/msg/JointState` | Debug command snapshot, six named joints |
| `control_mode` | `std_msgs/msg/String` | Machine-readable mode, timing, sensor, MPC and landing status |
| `jump_status` | `std_msgs/msg/String` | Jump phase, readiness, contact and plan status |
| `actuator_enable` | `std_msgs/msg/Bool` | Experimental jump actuator gate |
| `leg_velocity_reference` | `sensor_msgs/msg/JointState` | Experimental jump leg velocity references |

The ROS node computes non-jump leg torque as:

```text
tau = leg_kp * kp_scale * (q_ref - q)
    - leg_kd * kd_scale * qdot
    + landing_feedforward
```

The result is clamped by the configured provisional torque budget. Zero wheel
velocity is not equivalent to physical torque-off; the simulation actuator
contract owns the actual gate behavior.

## 4. Control-Cycle Architecture

The control timer performs this logical sequence:

1. Measure timer period and clamp the effective `dt`; do not integrate a whole
   delayed `/clock` gap as one normal control step.
2. Reject stale, missing or malformed MPU6500/joint data. Publish safe outputs
   while the watchdog is not satisfied.
3. Assemble `RobotState`, including the sensor-derived support hypothesis, IMU
   impact event and wheel odometry. Gazebo contact is retained only as
   diagnostic telemetry in the simulation adapter.
4. Update the optional `JumpController` path and its contact/status data.
5. Call `EmbeddedController::update()` for normal balance, pose motion,
   landing or stand-up recovery.
6. Publish wheel commands, leg commands, debug commands and status.

The simulator is configured for 1000 Hz physics, 1000 Hz ros2_control, 500 Hz
IMU and about 250 Hz model/contact observations. A 1 kHz simulation loop is a
physics-limited approximation; it is not evidence for the target ESP32-S3
embedded rate of about 5 kHz. The core uses fixed-size arrays and bounded MPC
storage, but ROS callbacks and DDS transport remain asynchronous and must be
measured separately from core execution time.

## 5. Embedded Control Core

### 5.1 `WholeBodyModel`

`WholeBodyModel` computes exact planar FK from the current four leg encoder
angles and the generated CAD geometry. It provides:

- whole-body COM fore/aft offset and height;
- pitch-axis inertia about the axle;
- axle height difference;
- equilibrium body pitch for the current leg pose;
- leg height, fore/aft position and their hip/knee Jacobians.

The Jacobians are used by landing controllers to solve a coordinated leg
extension/compression target while limiting unwanted fore/aft motion.

### 5.2 `MpcBalancer`

The MPC is a reduced-order wheeled inverted-pendulum controller, not a full
rigid-body, contact-constrained whole-body optimizer. Its state is:

```text
x = [ground_position, relative_body_pitch, ground_velocity, pitch_rate]
u = ground_acceleration
```

It builds a finite-horizon linear model, solves a bounded quadratic objective
with fixed projected iterations, and integrates the first acceleration into a
ground-velocity command. The model is scheduled from the live whole-body COM
height within configured limits. It also provides:

- forward velocity and differential yaw mixing;
- position/velocity and acceleration bounds;
- static friction-cone acceleration ceiling `mu*g`;
- optional nonlinear gravity correction for large pitch errors;
- optional station hold after a settled bent-pose dwell;
- a bounded post-touchdown landing window for wheel authority and leg gain
  scaling.

While support is absent, wheel-ground acceleration authority is physically
unavailable. The landing stack shapes the legs during this interval and the
MPC resumes/catches only after validated support, subject to its fall logic.

### 5.3 Whole-body compensation

The live FK equilibrium pitch becomes the MPC pitch reference, low-pass filtered
and bounded. This allows a pose change or leg sag to shift the COM without
forcing the wheels to chase a stale upright reference. Integral trim exists but
is disabled by default and is gated on a genuinely settled bent pose.

### 5.4 `JointMotionPlanner`

Pose changes use a cubic smoothstep interpolation of all six joint targets.
This planner is position interpolation, not a complete velocity/acceleration-
limited trajectory planner. The ROS node additionally slews incoming pose
targets and clamps them to the provisional operating envelope.

### 5.5 `StateMachine`

The normal mode FSM is:

```text
Disabled -> Enabling -> Balance <-> Drive
                         |
                         v
                    PoseChange

Balance / Drive / Fallen --stand_up--> StandingUp -> Balance or Fallen
Any active mode --disable/e-stop--> EmergencyStop
Large supported pitch error ---------------------> Fallen
```

Modes are `Disabled`, `Enabling`, `StandingUp`, `Balance`, `Drive`,
`PoseChange`, `Fallen` and `EmergencyStop`. A confirmed airborne flag suppresses
the generic pitch-based Fallen transition during a ledge flight interval. The
default Fallen behavior is a safe stop; keeping legs active and the special
landing-fallen handoff are explicit experimental options.

### 5.6 StationKeeper and WheelFrame

`StationKeeper` is an optional outer proportional loop for stationary
simulation demonstrations. It anchors world `x/y/yaw` and returns bounded
forward/yaw corrections while the operator is not driving. It is reset when
world-pose feedback is inactive or invalid. This loop consumes simulation pose
feedback and has no hardware localization equivalent in the current stack.

`WheelFrame` provides the planar no-slip axle odometry and the carrier-rate
correction used by the experimental jump path. It keeps wheel rotation, leg
carrier motion and body pitch conventionally separate. It must not be confused
with `StationKeeper`: the former is local wheel-frame kinematics, while the
latter is simulation world-position feedback.

## 6. Landing Architecture

Landing is event-driven and must be deployable from the planned MPU6500 plus
six encoders. It does not estimate an absolute ledge height from those sensors.
Relative flight time, specific force, angular rates and encoder consistency are
useful; terrain height and direct support force are not observable without an
additional range, contact or force sensor. The current Gazebo contact bridge
is a simulation diagnostic path, not the final landing sensor.

The deployable observer exposes a support hypothesis with confidence and
persistence, rather than pretending that a physical boolean contact signal
exists. A shock is only an impact candidate. Support becomes usable for the
wheel controller only after the post-impact IMU and encoder evidence remains
consistent for the configured confirmation interval.

There are several cooperating phase machines. Their names must be kept
distinct:

### 6.1 FlightObserver: support and event classification

`FlightObserver` is the support/flight observer and the source of the
`airborne` and support-confidence decisions used by the state machine and
`LandingController`:

```text
Grounded
  -> PartialSupport
  -> TransientUnloading
  -> BallisticFlight
  -> CollisionCandidate
  -> TouchdownCandidate
  -> SupportCandidate
  -> ReadyToDrive
                 |
                 +-> NoSupportTimeout
```

Operational behavior:

1. A brief support gap below the debounce interval remains transient. This is
   intended to prevent many short steps from becoming repeated landings.
2. Persistent support loss exposes airborne state.
3. Low IMU specific force, near zero during free-fall, raises ballistic
   confidence after its own debounce.
4. A high acceleration or latched hard-impact event while airborne creates
   `CollisionCandidate`; it is not automatically touchdown. A body or ledge
   collision can produce the same shock without a wheel support wrench.
5. A post-impact support hypothesis remains `TouchdownCandidate`/`SupportCandidate`
   until IMU and encoder evidence persists long enough. A Gazebo contact pulse
   is not deployable confirmation.
6. `downward_velocity_m_s` is a relative ballistic propagation initialized at
   the first missing-contact sample. It is not a height measurement.
7. `NoSupportTimeout` prevents an unsupported robot from being considered
   recoverable flight forever and returns the system toward normal fall/recovery
   handling; it must never be interpreted as grounded.

The current simulation contact bridge may deliver nonempty contact reports but
not guaranteed empty heartbeats because the snapshot plugin is bypassed on
some environments. Stale contact is therefore unknown, not proof of flight.

### 6.2 LandingController: global four-phase landing behavior

`LandingController` is the main phase coordinator:

```text
Grounded -> Flight -> Impact -> Recovery -> Grounded
                  ^       |       |
                  +-------+-------+  (support lost again)
```

#### Grounded

Normal MPC and leg posture control run. No landing offset or landing impedance
is applied unless a profile explicitly enables it.

#### Flight

The wheels have no ground authority, so wheel balance is suspended. The leg
target is shaped toward a touchdown posture after a configurable edge-clear
delay. Extension is calculated from `0.5*g*t^2`, transformed through the real
leg Jacobian, and bounded in task space and joint space. Optional bounded
pitch-rate hip placement may shape sagittal attitude. Targets are slew-limited.

#### Impact

Impact is created only after a minimum flight time and a persistent,
sensor-derived support hypothesis. IMU shock is a supplementary impact cue,
not sufficient to convert an edge or body strike into touchdown. The controller
captures the current per-side leg height and can apply a bounded Cartesian
virtual spring-damper:

```text
F = K * (x_ref - x) + D * (v_ref - v)
tau = J^T * F
```

Height is the primary axis; fore/aft force is optional and should remain small
because hip force can inject pitch disturbance. After support confidence is
high enough, wheel MPC is allowed to catch concurrently with leg impact
damping. A contact-force WBC is not claimed because force/torque sensing is not
in the hardware contract.

#### Recovery

Recovery waits for impact motion to quiet, or for a hard timeout, then blends
landing authority back to normal. Loss of support during Impact or Recovery
returns to Flight. A sufficiently stable recovery reaches `ReadyToDrive`, then
normal `Balance/Drive` decisions resume only after the wheel handoff gate.

### 6.3 LandingReflex: bounded target offsets

`LandingReflex` changes the leg target rather than only changing gains. Its
optional mechanisms are:

- symmetric airborne knee crouch;
- pitch-rate hip counter-lean latched at liftoff;
- velocity-matched active knee extension using `g*t^2` and FK Jacobians;
- differential left/right hip response to measured gyro roll rate;
- optional capture-point hip placement, using launch velocity and COM height
  latched at liftoff rather than noisy post-touchdown velocity;
- hold and blend of offsets through the catch window.

All these gains default to zero. A deep crouched starting pose is required for
meaningful extension travel; a near-straight leg is already close to its reach
limit. The summed target is re-clamped by the embedded controller.

### 6.4 LandingImpedance: per-side gain scheduling

`LandingImpedance` runs an independent phase machine for each side:

```text
Grounded -> Airborne -> Impact -> Rebound -> Grounded
```

It normally leaves the hip unchanged and applies the scheduled impedance to
the knee. During Impact, knee position stiffness can be reduced and damping
raised so the leg dissipates energy instead of fighting a fixed position. Once
compression has bottomed out, gains blend back to normal. Optional energy
sizing uses measured touchdown knee velocity, available hard-limit stroke and
an effective knee inertia; it is disabled unless all required physical
parameters are explicitly configured.

The left and right phases are independent so one wheel may absorb an impact
while the other is still airborne. A real impact gate can use measured leg
compression velocity, raw accelerometer shock or the latched IMU event. Default
profiles keep this mechanism inactive or experimental unless a landing YAML
profile enables it.

### 6.5 Landing and command composition

On each valid core update, the effective leg target is composed in this order:

```text
nominal commanded pose
  + LandingReflex offset
  + LandingController flight offset
  -> operating-envelope clamp / stand-up pose handling
  -> MPC nominal leg gain
  -> LandingImpedance per-side gain scale
  -> LandingController impact feedforward torque
  -> final leg torque clamp
```

The wheel command remains owned by MPC except when the explicit jump path owns
the pose. During Flight the landing layers may keep legs active while the wheel
authority is suspended. At touchdown the wheel catch window and leg impact
layers overlap deliberately, but their authority is bounded and phase-gated.

### 6.6 Landing phases in a drive-off test

For the canonical commanded `1 m/s`, straight drive-off from a `50 cm` ledge,
the intended interpretation is:

```text
Balance/Drive
  -> TransientUnloading       short edge gap, no landing yet
  -> BallisticFlight          persistent support loss + low specific force
  -> CollisionCandidate       edge/body shock, still no wheel support
  -> TouchdownCandidate       support hypothesis is forming
  -> SupportCandidate         IMU/encoder evidence persists
  -> Impact/Catch              landing controller active
  -> Recovery                 impact motion decays / timeout fallback
  -> ReadyToDrive              pitch/rate stable, wheel handoff permitted
  -> Balance/Drive              command resumes and remains stable
  -> NoSupportTimeout/Fallen   no reliable support evidence / safety termination
```

The `50 cm` value belongs to the test world geometry. It is not fed to the
sensor-only core as a known height. A landing trial is operationally successful
only when the robot infers stable support, re-enables wheel balance, returns to
`Balance`/`Drive`, resumes the forward command and remains stable for the
declared post-handoff hold interval. Touching the lower surface or avoiding an
immediate crash is not sufficient.

## 7. Experimental Jump Path

The explicit jump path is separate from passive drive-off landing. When
`jump.enabled` is false, it does not alter normal balance behavior.

`JumpController` phases are:

```text
Idle -> Crouch -> Thrust -> Flight -> Land -> Settle
                                      |         |
                                      v         v
                                  Complete    Fault
```

It owns a single requested jump, validates readiness, generates bounded leg
targets/velocities and optional feedforward, observes measured contacts, and
requires a grounded and settled window before completion. `JumpPlan` is an
offline immutable Hermite-interpolated trajectory with explicit bounds and
optional feedback gains; validation checks interpolation limits, not motor
feasibility or physical landing success.

The current ROS jump integration has stronger simulation assumptions than the
normal landing path: jump profiles require explicit simulation pose/contact
configuration, and the ROS jump estimator currently receives simulator pose
inputs. It must therefore be reported as experimental simulation control, not
as a sensor-complete hardware jump controller.

The bounded simulation actuator path applies provisional effort/speed envelopes
and a gate handshake. It is useful for controlled experiments, but the ideal
wheel velocity and provisional motor limits do not represent measured hardware
electrical dynamics.

## 8. Simulation Architecture

`tap_sim/sim.launch.py` starts, in order:

1. Gazebo Sim with `empty.sdf`, `rough_training.sdf`, `ledge_test.sdf`,
   `jump.sdf`, or a generated ledge world.
2. `robot_state_publisher` from `simulation.urdf.xacro`.
3. A free-root `tap_robot` spawn.
4. `balance_controller_node` with the workspace controller configuration.
5. `activate_controllers`, which manages paused startup and the enable gate.

The simulation URDF keeps the free body and uses primitive collision geometry:
rotated body box, slender leg boxes and 30 mm wheel cylinders. The preview URDF
has a fixed display mount and is only for RViz/kinematic inspection.

The normal simulator uses wheel velocity commands and leg efforts. The opt-in
`jump_model` selects the bounded Gazebo hardware plugin; `measure_contacts`
enables wheel contact sensors independently of that actuator model. All worlds
share the CAD signs and provisional inertial data.

Simulation-only conveniences include world reset/teleport from the GUI and
`sim_pose` feedback. They cannot be used as evidence of hardware balance,
contact estimation or stand-up capability.

## 9. Safety and Failure Handling

- Invalid core inputs produce a safe stop.
- Missing/stale sensor data is fail-closed.
- E-stop has priority and clearing it does not by itself re-enable motion.
- Disable clears motion requests and stops commands.
- Commanded drive velocity expires after the configured timeout.
- Contact shock alone never authorizes touchdown or wheel balance.
- Airborne support loss suppresses premature generic pitch-based Fallen
  classification, but a no-support timeout and normal fall paths remain.
- Default Fallen behavior disables all motion. `keep_legs_active_when_fallen`
  and landing-fallen handoff are explicit experimental overrides.
- No code path in this documentation authorizes motor actuation, flashing,
  deployment or hardware operation.

## 10. Evidence and Known Limits

Evidence levels are separate:

1. Source inspection and static model checks.
2. Core build and deterministic unit/contract tests.
3. ROS interface/freshness tests.
4. RViz rendering, which proves geometry display only.
5. Closed-loop Gazebo, which proves only the tested model/configuration.
6. Hardware trials, currently not authorized and not claimed.

Current implementation evidence includes the core/ROS landing observer tests
and a successful build/test integration. A canonical 50 cm closed-loop smoke
trial has launched, crossed the edge, and still ended in `Fallen`; this is not
landing success. The landing trace also needs observer fields recorded in the
CSV, and the `Drive -> Balance` transition/reset behavior during active flight
remains an identified integration item. Stand-up recovery is experimental and
the tested lying-pose recovery has failed.

Relevant verification entry points:

```bash
cmake --build build/tap_control -j2
ctest --test-dir build/tap_control --output-on-failure
python3 -m unittest discover -s tests -v
colcon build --symlink-install --packages-select tap_robot_description tap_control tap_sim
python3 tools/ledge_height_smoke.py --heights 0.50 --trials 1 --trace-rate-hz 100
```

The exact scope, thresholds, model assumptions and historical trial artifacts
are maintained in [`docs/ALGORITHM_MIGRATION.md`](docs/ALGORITHM_MIGRATION.md),
[`docs/RULES.md`](docs/RULES.md),
[`src/tap_sim/CONTRACT.md`](src/tap_sim/CONTRACT.md), and the CAD
[`PHYSICS.md`](src/tap_robot_description/description/cad_robot/PHYSICS.md).
