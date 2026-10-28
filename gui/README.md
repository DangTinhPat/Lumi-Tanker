# TAP controller interfaces

The Python/Tk interfaces operate the ESP32 firmware or the native simulator.
Camera processing and person following have been removed. The firmware owns
balance, sensor feedback and motor commands.

## Robot GUI

From the repository root:

```bash
python3 -m pip install pyserial
python3 gui/robot_gui.py --port tcp://10.10.60.68:8023
```

Use the IP reported by the robot at startup. For USB, pass `/dev/ttyUSB0`.
`--no-auto-connect` opens the interface without connecting to hardware.
The GUI provides manual drive, height control, stand-up, finite move/turn,
STOP, DISABLE and ESTOP. Logs are written to `~/tap_logs/`.

`MotionAPI` uses firmware MOVE/TURN and MOT telemetry when available, with
encoder odometry as the fallback for finite actions. Continuous movement
continues until a stop or replacement command. The GUI does not require
OpenCV, ONNX or camera calibration.

## Command gateway

Manual input and JSON commands share `RobotCommandRouter`. The schema is
[`config/robot_command_tree.json`](config/robot_command_tree.json). Supported
actions are `move`, `turn`, `set_height` and `emergency_stop`; supported
sources are `manual_ui`, `luvox` and `system`.

`TapSerialControlApp.execute_robot_command(payload)` queues commands from
worker threads for execution on the Tk thread. ESTOP cancels active actions.
Basic actions replace one another without a long queue. Movement requires a
connected robot in Balancing, and finite actions require fresh telemetry.

## Native simulator GUI

The simulator and its model are retained:

```bash
cmake -S . -B build/simulation -DBUILD_TESTING=ON -DTAP_BUILD_SIMULATOR=ON
cmake --build build/simulation -j4
python3 gui/simulator_gui.py --rate 3000
```

`Start SIM` opens the native process and MuJoCo viewer. Explicitly enable the
robot after confirming its fixture pose. The native dashboard provides manual
drive, posture controls, stand-up and stop controls. Its 3-D view is telemetry;
it is not a sensor input to the firmware controller.
