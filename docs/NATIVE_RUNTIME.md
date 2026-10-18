# Native TAP runtime

The simulator is a detachable native stack. It verifies the model and the
shared core; it does **not** run the robot firmware's `V3Controller` (see
`architecture.md` section 1).

```text
Tk/Python GUI  -- line IPC -->  tap_mujoco_native (C++/MuJoCo)
                                      |
                                      +-- tap_control_core (EmbeddedController)
                                      +-- MPU6500 / encoder sensor model
                                      +-- bounded actuator model
```

Build and run from the repository root (MuJoCo 3 must be installed, or set
`MUJOCO_ROOT`; use `-DTAP_BUILD_SIMULATOR=OFF` to build only the host tests):

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON
cmake --build build/native -j4
ctest --test-dir build/native --output-on-failure
python3 src/tap_sim/scripts/validate_model.py --model src/tap_sim/model/tap_robot.xml
python3 tools/tap_native_gui.py --rate 3000
```

Runner options (`tap_mujoco_native --help`):

| Option | Meaning |
| --- | --- |
| `--model FILE` | MJCF to load (required) |
| `--rate 1000..4000` | physics/control rate in Hz; 3000 is normal, 4000 the stress setting |
| `--sensor-rate N` | sensor packet rate in Hz (default 500, at most `--rate`) |
| `--run-for SEC` | stop after SEC seconds of simulated time |
| `--seed N` | sensor-noise seed |
| `--no-gds68` | disable the mocked GDS68 leg-torque feedback |
| `--command-port PORT` | accept commands on a TCP port |
| `--realtime` | pace the loop to wall-clock time |

The physics and the core loop stay native; Python is only a command and
telemetry client. The `VIEW|` channel is visualization/debug telemetry and is
never copied into `RobotState`. Controller input is limited to the noisy,
delayed MPU6500-shaped IMU data, six encoder channels and the optional GDS68
feedback model. The IPC command list and the `STATE|`/`VIEW|` channels are
described in `architecture.md` section 5.

The native dashboard limits operator drive commands (default `0.30 m/s`) and
shapes forward/yaw references with a finite slew rate. Wheel speed and torque
limits come from the motor data in `docs/MOTOR_SELECTION.md`; the actuator
model's friction, lag and saturation values are exposed simulation assumptions,
not identified hardware parameters.

This is simulation evidence only. Host timing does not certify ESP32 timing,
and the simulator's results do not apply to the firmware's controller. The
firmware is exercised on the host by `src_fw/tests` (`v3_tier_tests` and the
other `v3_*` tests) and by robot logs.
