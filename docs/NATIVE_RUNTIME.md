# Native TAP runtime

The canonical runtime is a detachable native stack:

```text
Tk/Python GUI  -- line IPC -->  tap_mujoco_native (C++/MuJoCo)
                                      |
                                      +-- tap_control_core (C++17)
                                      +-- MPU6500/encoder sensor model
                                      +-- bounded actuator model
```

Build and run it from the repository root:

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON
cmake --build build/native -j2
ctest --test-dir build/native --output-on-failure
python3 tools/tap_native_gui.py --rate 3000
```

Use `--rate 4000` for the stress configuration. The physics and core loop
remain native; Python is only a command/telemetry client. The GUI `VIEW`
channel is visualization/debug telemetry and is never copied into
`RobotState`. Controller input is limited to noisy, delayed MPU6500-shaped
IMU data, six encoder channels and the explicitly enabled GDS68 feedback
model.

The sensor-only landing setup is exercised by the native contract test:

```bash
build/native/bin/tap_mujoco_native --model src/tap_sim/model/tap_robot.xml \
  --rate 3000 --landing-30cm --run-for 0.8
```

The test starts from the supported natural ENABLE reference, waits for the MPU6500
attitude seed, then applies the 30 cm ledge/drop event as simulator setup. The
drop event is never copied into `RobotState`; this avoids an unobservable
free-fall startup that would leave the pitch estimate at its zero initial
value. It detects `BallisticFlight -> TouchdownCandidate` from the sensor
boundary. The native telemetry also records estimator seeding, support
confidence, landing phase, wheel-balance suspension and MPC output so a short
phase cannot be hidden by the 20 ms display period.

The multi-rate contract runs the same sensor-only trial at 1, 3 and 4 kHz and
checks that phase timing follows the 500 Hz sensor stream rather than the
control-loop tick. The controller refuses normal re-enable while pitch remains
past the fall limit; recovery is a separate explicit `STAND_UP` request.

The native dashboard limits operator drive commands to `0.30 m/s` and shapes
forward/yaw references with a finite slew rate. The MPC retains a wider
internal catch envelope so it can briefly overspeed a wheel to arrest pitch;
this is not an increase to the operator command or a hardware capability claim.

The 1.5 m/s approach is inside the wheel speed envelope: the BL4820M12 is
rated 400 rpm under load, which on the 0.040 m wheel is 1.6755 m/s (see
`docs/MOTOR_SELECTION.md`). An earlier revision of this file quoted 1.2566 m/s
(300 rpm) and used it to explain the landing failure; that figure was wrong
and no code ever used it - every executable path clamps at 41.887902 rad/s.
The binding hardware limit is wheel TORQUE, not wheel speed: 0.13 Nm across
two wheels caps ground acceleration at 1.41 m/s^2 for the 4.621 kg model.
Post-touchdown recovery is evidence-pending against that acceleration
ceiling, not against a speed ceiling.

Latest controlled observation: 10 seeds all detected the sensor-only flight and
touchdown sequence, but all entered `Fallen` after the first/bounce landing;
the observed peak estimated pitch was about 2.31--2.34 rad. This is a failure
of the current physical catch envelope, not a CMF detection pass. Neutral
`knee=0` also leaves little pre-impact compression stroke. No torque cap was
increased to hide that limitation.

This is simulation evidence only. The host timing result does not certify the
ESP32 WCET, and GDS68 bias/friction parameters still require hardware
identification before CMF is promoted on the robot.
