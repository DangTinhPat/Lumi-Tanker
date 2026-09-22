# Lumi-Tanker architecture

This document describes the current canonical runtime and separates it from
the retained ROS 2/Gazebo migration code. It is an implementation map, not a
claim of hardware validation. No real robot, motor, firmware flash, or ESP32
deployment is authorized by this repository.

## 1. Canonical runtime

The active path is native C++ MuJoCo plus a small Python/Tk operator GUI:

```text
Python/Tk GUI
    │ line-based stdin/stdout IPC
    ▼
tap_mujoco_native
    ├── MuJoCo free-body physics
    ├── MPU6500-like IMU synthesis
    ├── six encoder synthesis
    ├── optional GDS68 leg-torque feedback mock
    └── tap_control_core
          ├── ImuStateEstimator
          ├── FlightObserver
          ├── TorqueResidualEstimator
          ├── CMF support/flight decision
          └── balance, landing and safety controllers
```

The root [`CMakeLists.txt`](CMakeLists.txt) intentionally builds only
`tap_control_core` and `tap_mujoco`. It does not discover, link, launch, or
exchange messages with ROS 2 or Gazebo.

| Area | Current location | Responsibility |
| --- | --- | --- |
| Deterministic controller | `src/tap_control/core` | ROS-independent state, FK, MPC, IMU estimator, CMF and landing logic |
| Native simulator | `src/tap_mujoco` | MuJoCo model, fixed-rate runner, actuator dynamics and sensor boundary |
| Sensor/actuator boundary | `src/tap_mujoco/scripts/mujoco_sensor_actuator_backend.py` and the native runner | Noise, bias, latency, dropout, quantization, saturation and GDS68 feedback mock |
| Native operator GUI | `src/gui/gui/native_dashboard.py` | Tk controls/diagnostics and the MuJoCo 3-D viewer; it communicates only through line IPC |
| Native GUI process client | `src/gui/gui/native_backend.py` | Starts/stops the runner and serializes operator commands |
| CAD source | `src/tap_robot_description/description/company_rb` | Authoritative company RB meshes, inertial data, CAD signs and URDF/Xacro parity source |
| Pose/offline tools | `src/tap_teleop`, `tools/` | Offline curve generation, audits and regression tooling |
| Retained migration code | `src/legacy/ros2_gazebo_sim` | ROS 2/Gazebo adapter and historical simulation only; outside canonical build |

`src/gui/gui/dashboard.py` and `src/gui/scripts/gui` are compatibility entry
points for the native dashboard. They do not import `rclpy`.

## 2. Hardware-shaped contract

The deployable algorithm is deliberately restricted to the planned hardware:

- one MPU6500: three-axis accelerometer and three-axis gyroscope;
- four leg motor encoders and two wheel motor encoders;
- the four leg drivers' GDS68 current-derived torque estimate, only as a
  secondary corroborating signal when its provenance is enabled and valid.

There is no force/torque sensor, wheel contact switch, range sensor, camera,
GPS, external localization, or wheel load/current signal in the contract.
MuJoCo pose, contact, constraint force, and simulator state are telemetry for
scoring and visualization only. They are not copied into `RobotState` and do
not make decisions in CMF, landing, MPC, or the safety FSM.

The six-joint order is fixed:

```text
[LH, LK, LW, RH, RK, RW]
```

The native CAD axes are `+Y` on both sides. Right-side signs must not be
mirrored as an old-robot compatibility shortcut. The model contract is:

| Quantity | Value/status |
| --- | --- |
| Wheel radius | `0.040 m` |
| Axle centre spacing | `0.309 m` |
| Half-track | `0.1545 m` |
| Approximate mass | `4.621 kg`, provisional CAD/BOM estimate |
| Body visual/inertial pitch | `-pi/2`, applied consistently to body visual and inertia |

The full mass, COM, inertia, motor-lumping and mechanical-limit caveats remain
in [`PHYSICS.md`](src/tap_mujoco/model/PHYSICS.md). The MJCF and checked-in
CAD manifest are the active native geometry source.

## 3. Native simulator boundary

`src/tap_mujoco/model/tap_robot.xml` contains a free-root robot, ground plane,
CAD visuals, bounded primitive collisions, inertias, six actuators and raw
sensor channels. The model's current physical assumptions are recorded in
`model_manifest.yaml`.

The runner configures the model timestep to the requested loop rate. The
current normal target is 3 kHz and the stress target is 4 kHz; sensor packets
are independently generated at 500 Hz. The sensor boundary applies:

- accelerometer and gyro white noise, bias and random walk;
- quantization, packet latency and occasional dropout;
- encoder position/velocity noise, quantization and dropout;
- command latency and first-order actuator lag;
- leg torque saturation, wheel torque saturation and the measured/provisional
  wheel speed envelope;
- Coulomb/stiction/viscous friction terms;
- optional delayed, biased and noisy GDS68-like leg torque feedback.

These are exposed simulation assumptions, not calibration certificates. They
must be replaced by measured hardware parameters when available.

The GUI receives two separate channels:

```text
STATE|...   sensor-derived controller telemetry
VIEW|...    simulator pose used only for drawing the debug view
```

`VIEW` is intentionally not parsed into the controller. A GUI that renders a
correct pose therefore does not prove state estimation, balance, or landing.

## 4. Control and CMF data flow

For each native step:

1. The simulator advances physics and applies bounded actuator dynamics.
2. Delayed/noisy sensor packets become available when their simulated latency
   expires.
3. `ImuStateEstimator` consumes MPU6500 samples and maintains pitch, rates,
   specific-force magnitude, startup qualification and impact evidence.
4. Named encoder samples update joint state, wheel odometry and motion
   consistency. Missing or stale samples fail closed.
5. `TorqueResidualEstimator` processes only valid leg torque-feedback samples
   and the commanded/measured joint motion. It estimates a bounded normal-load
   hypothesis; it is never a standalone contact gate.
6. `FlightObserver` fuses IMU and encoder evidence, with torque residual as
   optional corroboration, into support/flight phases.
7. `EmbeddedController` applies the state machine, MPC, landing stack and
   bounded motor commands.
8. Periodic `STATE` telemetry is emitted for inspection; `VIEW` is emitted
   separately for visualization.

No stage reads MuJoCo `qpos`, body pose, contacts, constraint forces, or hidden
ground-truth phase for control. The landing scenario may alter simulator
state to create a ledge/drop event, but only subsequent sensor packets enter
the controller.

## 5. CMF and landing

CMF means the constrained combination of `FlightObserver` and
`TorqueResidualEstimator`:

```text
MPU6500 + six encoders ──► FlightObserver ──► primary phase hypothesis
                                  ▲
valid GDS68 leg torque ──► TorqueResidualEstimator
                           secondary corroboration only
```

The observer phases include grounded/partial support, unloading, ballistic
flight, collision candidate, touchdown candidate and no-support timeout.
Debounce and persistence are measured in sensor-observation time, not blindly
in control-loop iterations. One-wheel support may be reported as partial
support while the landing controller waits for a stable handoff.

Landing authority is phase-gated:

```text
nominal pose
  + LandingReflex / flight extension
  + LandingController target and feed-forward
  -> operating-envelope clamp
  -> per-side LandingImpedance scheduling
  -> bounded leg torque
```

During ballistic flight, wheel-ground acceleration authority is not assumed.
After a sensor-derived support hypothesis persists, the controller blends
impact handling, recovery and wheel-balance handoff. A touchdown candidate is
not equivalent to proven physical contact, and a successful observer trace is
not by itself proof that the robot can recover dynamically.

The current canonical landing profile is the controlled `v = 1.5 m/s`,
`h = 0.30 m` scenario in
[`landing_30cm_cmf_1p5.yaml`](src/tap_control/config/landing_30cm_cmf_1p5.yaml).
The scenario uses height and velocity only to configure the simulator event;
they are not fed to CMF as measurements. The known result remains that CMF
phase detection is repeatable, while recovery success is not yet established.

## 6. Command and IPC contract

The native GUI sends line commands to the child process. Launching the GUI also
opens a MuJoCo 3-D viewer loaded from the same `tap_robot.xml`; the C++ child
remains the only physics/control owner and the viewer mirrors only the
debug-only `VIEW` snapshot:

```text
ENABLE
DISABLE
RESET
ESTOP
CLEAR_ESTOP
DRIVE <velocity_m_s> <yaw_rate_rad_s>
POSE <LH> <LK> <RH> <RK>
STAND_UP
QUIT
```

The child owns the kHz loop. Tk runs at a human-scale refresh rate and never
acts as the control scheduler. `ESTOP`, stale sensor data, invalid samples and
expired commands force safe outputs. Clearing e-stop does not implicitly
resume motion. `DISABLE` is a recoverable safe stop: it clears drive/pose
requests and requires a fresh `ENABLE`; it is distinct from the latched
`ESTOP` path. `POSE` is a transaction and is retired automatically when the
core joint planner reaches its endpoint. `RESET` resets MuJoCo state, sensor
queues/timestamps, estimator/controller state and command latches while
keeping the external sensor clock monotonic.

## 7. Legacy ROS 2/Gazebo boundary

`src/legacy/ros2_gazebo_sim` contains the old `tap_sim` ROS package, Gazebo
worlds, `ros2_control` plugin, launch file and controller activation helper.
`src/tap_control/ros2_adapter/BalanceControllerNode.cpp` is the old ROS 2
adapter retained for migration comparison. Their package identities and
internal plugin names remain unchanged where required by old launch files.

They are not part of the root native build. Their telemetry may be used for
historical debugging, but Gazebo contact/pose/effort must not be described as
hardware evidence or allowed to leak into the native CMF path.

The repository therefore still contains ROS/Gazebo text and code by design,
but it is isolated and explicitly named as legacy. “ROS-free runtime” means
the canonical native build and execution graph, not that historical source
has been silently deleted.

## 8. Verification levels

Evidence must be reported separately:

1. source/path audit;
2. native CMake build and deterministic core tests;
3. MuJoCo model, sensor-boundary and native-runner contracts;
4. multi-rate CMF and landing scenario tests;
5. GUI/IPC smoke tests;
6. closed-loop simulation outcomes;
7. hardware tests, which are currently out of scope.

Useful native commands are:

```bash
cmake -S . -B build/native -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
cmake --build build/native -j2
ctest --test-dir build/native --output-on-failure
python3 src/tap_mujoco/native_landing_contract_test.py \
  --runner build/native/bin/tap_mujoco_native \
  --model src/tap_mujoco/model/tap_robot.xml
python3 tools/tap_native_gui.py
```

The GUI and `VIEW` channel are usability evidence only. Passing native tests
does not establish motor calibration, contact-force accuracy, or physical
landing recovery.
