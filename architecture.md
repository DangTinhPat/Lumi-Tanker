# Lumi-Tanker architecture

This document maps the repository as it is today and states which part runs on
the robot and which part is only a verification tool. It is an implementation
map, not a claim of hardware validation: simulation and host tests are evidence
for the models they exercise, and robot logs are the only evidence about the
physical robot. Operating hardware, flashing firmware or sending motor commands
requires an explicit request from the user.

## 1. Two deliverables that share a core

| | Robot firmware (v3.3) | Native simulator |
| --- | --- | --- |
| Purpose | Balances and drives the physical RB v2 | MuJoCo model + GUI for inspection and experiments |
| Controller | `V3Controller` (`src_fw/src/v3`) | `V3Controller` in `tap_mujoco_gim_v3`; `EmbeddedController` (`src/tap_control/core`) in the older `tap_mujoco_native` |
| Runs on | ESP32-S3 N16R8, FreeRTOS | Host PC, MuJoCo 3 |
| Build | PlatformIO env `esp32-s3-v3` | CMake target `tap_mujoco_gim_v3` (`TAP_BUILD_SIMULATOR=ON`, needs MuJoCo) |
| Operator | `tools/tap_serial_control_gui.py` over USB serial | `tools/tap_native_gui.py` over line IPC |

**The native simulator does not run `V3Controller`.** It drives the older
`EmbeddedController` (MPC, landing and jump logic) which the robot does not use.
The firmware's control law is exercised on the host by the two-body planar
plant in `src_fw/tests/LegPlant.hpp` (the `v3_*_tests`, in particular
`v3_tier_tests`). Results from the MuJoCo runner therefore say nothing about
`V3Controller`, and vice versa.

Shared code: the rigid-body model `WholeBodyModel` and generated CAD geometry
(`GeneratedGeometry.hpp`), `ImuStateEstimator`, `LqrBalancer`, the tuned LQR
configuration (`TunedConfig`, used by `defaultV3Config()`), and the motor
protocol layer in `src/tap_control/hal`.

## 2. Repository map

```text
firmware_esp32/        ESP32-S3 adapter: main.cpp, AppV3 (tasks, console, logging),
                       Esp32Bus (TWAI/UART), Mpu6500Spi
src_fw/
  include/tap_fw/      BoardConfig (pins, rates, hardware facts), ImuMount, MotorFeedback,
                       BusTransport, FastFormat, v3/ (V3Controller, LegLock, LegHeight, ...)
  src/                 implementations; v3/ holds the controller
  tests/               host tests and the planar plant (LegPlant.hpp)
  FIRMWARE_V3.md       firmware reference (states, SET keys, log fields)
src/tap_control/
  core/                portable C++17: WholeBodyModel, ImuStateEstimator, LqrBalancer,
                       TunedConfig, EmbeddedController and its native-only pieces
  hal/                 MotorFrameCodec, MotorMapping, MotorTransport (GDS68 CAN, BL4820 UART)
  apps/, tests/        demos and core tests
src/tap_sim/           MuJoCo runner (native/), RB v2 MJCF (`gim_*.xml`) + mass properties + PHYSICS.md (model/),
                       sensor/actuator drivers, model physics tests
src/gui/               Tk dashboard + 3-D viewer for the simulator, command router, vision client
tools/                 serial GUI, log analysis (hip_chatter_stats, hf_vibration_stats,
                       tr_rest_stats, fit_leg_servo), tier-sim helpers, leg-jog bench firmware
blue_object_vision/    optional person detector/tracker used by the follower (host camera)
docs/                  V3.3_HANDOFF.md (current state), RULES.md, NATIVE_RUNTIME.md,
                       actuatal_driver/ (GDS68 and wheel-motor protocols), dated investigations
source_assets/         supplier datasheets
```

`docs/V3.3_HANDOFF.md` is the authoritative record of the firmware's current
state, boot configuration, measured hardware facts, rejected ideas and open
problems; this document does not duplicate it.

## 3. Hardware and sensor contract

- Board: custom ESP32-S3-WROOM-1 N16R8 PCB. Console on UART0 via CP2102 at
  921600 baud; optional Dabble gamepad over BLE.
- IMU: one MPU6500 (3-axis accelerometer and gyro) on SPI. It is mounted
  rotated 90° (chip Y forward, chip X right); `ImuMount.hpp` maps chip axes to
  the body frame before the estimator.
- Leg joints: four SteadyWin GDS68 drivers (GIM6010-8, 8:1) on CAN at
  500 kbit/s, node IDs and signs in `docs/actuatal_driver/GDS68_MOTOR_MAPPING.md`
  and `MotorMapping.hpp`. The firmware runs all four in the driver's position
  mode; legs lock at the pose they had at power-up (that pose is joint zero).
- Wheels: two CJC BL4820M12 motors on UART at 1 Mbaud, speed command and
  encoder feedback.
- Deployable sensor inventory: one MPU6500 and six motor encoders. The leg
  drivers' Iq-based torque estimate is allowed only under the conditions of
  Rule 1 in `docs/RULES.md`. Simulator pose, contact and constraint data never
  enter a control decision.

Fixed conventions:

- Joint order `[LH, LK, LW, RH, RK, RW]`; native CAD axes are `+Y` on both
  sides and are not mirrored. Driver sign conversion happens only in
  `MotorMapping.hpp`.
- Wheel radius `0.040 m`, axle spacing `0.2654 m` (half-track `0.1327 m`).
- Mass `3.12 kg` (body 2.50, each thigh 0.05, each shin tube 0.04, each wheel
  motor 0.20, each wheel 0.02). Mass, CoM and inertia are CAD/BOM estimates (see
  `src/tap_sim/model/gim_mass_properties.yaml`); the firmware compensates a
  measured CoM offset (`SET cdx`). See `src/tap_sim/model/PHYSICS.md`.
- Height is the hip-to-axle distance, 188.6 mm (legs hanging) down to 78 mm.

## 4. Firmware v3.3

### 4.1 Runtime

A FreeRTOS control task on core 1 runs the 500 Hz loop; serial, BLE and
logging run on core 0 (`-DTAP_V3_SINGLE_LOOP=1` selects a single-loop fallback
environment). Leg commands are sent as one batch of four at about 333 Hz;
GDS68 position feedback arrives at about 96 Hz. State machine:
`Idle → Locking → Ready → Balancing`, with `Fault` on any safety trip.
`DISABLE`, `ESTOP`, a lost serial link (0.5 s), a pitch beyond 0.6 rad, wheel
runaway, sensor loss or leg/pose inconsistency stop the wheels; the legs stay
locked until `DISABLE`.

### 4.2 Control structure (`V3Controller`)

```text
IMU pitch/gyro + hip/knee encoders + wheel encoders
        │ virtual pendulum angle θv = pitch + CoM line angle(measured legs) − hang value
        ▼
 safety: fall, runaway, held/lifted detection, θv vs legs consistency
        ▼
 Tier 1 (wheels): LQR on θv with a gain table over pendulum length, position anchor,
                  balance trim, height-adaptive feedback lead
 Tier 2 (hips):  reflex (lean residual + lead + gated body damping) + manoeuvre preview
                  + slow IMU level loop ("gimbal")  →  dhip
 Coordination:   paired hip/knee IK keeps the axle on a horizontal plane; the knee
                  term follows through a low-pass (kfhz); height profile (rate/accel/jerk);
                  position-mode suspension; optional per-side roll (off at boot)
        ▼
 Output → AppV3 → LegLock (GDS68 position targets) + wheel speed commands → CAN/UART
```

Gains, boot overrides and the evidence behind each are listed in
`docs/V3.3_HANDOFF.md` (section 3); every override has a `SET` key that reverts
it on the robot without reflashing. Opening the serial port resets the ESP32
and clears runtime `SET` overrides.

### 4.3 Operator interface and logs

`tools/tap_serial_control_gui.py` sends `START`, `GO`, `DRIVE`, `HMOVE`,
`DISABLE`, `ESTOP`, `SET`, `GET`, `REC` and writes every line to
`~/tap_logs/`. The firmware emits `SENS` (5 Hz health/timing), `TR` (25 Hz
controller trace), `CONTROL` and `V3 TUNE` lines; field meanings are in
`src_fw/FIRMWARE_V3.md`. `REC` captures an 8 s, 250 Hz RAM trace for fast
phenomena that the 25 Hz trace cannot show.

## 5. Native simulator

`tap_mujoco_gim_v3` (`src/tap_sim/native/gim_v3_runner.cpp`) loads
`src/tap_sim/model/gim_dynamic.xml`, the RB v2 (GIM6010-8) model from
`ROBOT (GIM6010-8).STEP` (free-root body, six `+Y` hinge joints, CAD visuals,
convex-hull collisions, one MPU6500-like IMU, six encoders), and runs the
firmware's `V3Controller`. The older `tap_mujoco_native` runner takes its model
by `--model` and has no checked-in RB v2 default. The runner steps physics at 3 kHz (4 kHz stress
target) and produces sensor packets at 500 Hz with noise, bias, quantization,
latency, dropout, actuator lag, torque saturation and friction; leg torque
feedback can be mocked. Only these sensor packets reach the controller.

The GUI talks to the child process over line IPC. Commands: `ENABLE`, `DISABLE`,
`RESET`, `ESTOP`, `CLEAR_ESTOP`, `STOP`, `DRIVE <v> <yaw>`, `POSE <LH> <LK> <RH> <RK>`
(retired by `POSE_DONE`), `HEIGHT <m>`, `STAND_UP`, `DANCE START|STOP`, `QUIT`.
Telemetry has two channels:

```text
STATE|...   sensor-derived controller telemetry
VIEW|...    simulator pose, used only to draw the 3-D view
```

`VIEW` is never parsed into the controller, so a correct picture proves nothing
about estimation or balance. The child owns the fast loop; Tk is only a
human-rate client. `ESTOP`, stale data and expired commands force safe outputs.

`EmbeddedController` also contains flight/landing logic (`FlightObserver`,
`TorqueResidualEstimator`), a jump controller and a station keeper. The robot
does not use them; they stay in the shared core only because `TunedConfig`
and the native runner depend on the same library.

## 6. Verification levels

Report evidence separately; none certifies the physical robot on its own:

1. source/path audit;
2. host build and unit tests (`ctest`);
3. planar two-body firmware sims (`v3_tier_tests`, selected by `TIER_*`
   environment variables; method in `docs/V3.3_HANDOFF.md` section 7);
4. MuJoCo native runner and GUI/IPC smoke tests;
5. firmware build and size/timing (`pio run -e esp32-s3-v3`);
6. robot logs from real sessions.

`ctest` currently has five failures that predate this document
(`core_allocation_tests`, `jump_tests`, `jump_quality_tests`, `lqr_tests`, and
`v3_tier_tests`, which stops at its V14 assertion); they are tracked in
`docs/V3.3_HANDOFF.md`. Commands:

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON -DTAP_BUILD_SIMULATOR=OFF
cmake --build build/native -j4
ctest --test-dir build/native --output-on-failure
~/.platformio/penv/bin/pio run -e esp32-s3-v3
python3 tools/tap_serial_control_gui.py
```

With MuJoCo installed, drop `-DTAP_BUILD_SIMULATOR=OFF` and run
`python3 tools/tap_native_gui.py` for the simulator.
