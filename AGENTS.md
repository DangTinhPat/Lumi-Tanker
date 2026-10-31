# Project guidance — Luckfox gateway (ESP32-S3 lower layer)

Read `README.md` for the layout. Before describing input/output, read
`docs/audit-data-flow.md`, `docs/overview.md`, `docs/hcr1-protocol.md` and the
actual producer/codec/consumer in `src/` and `include/gateway/`.

## Interface contract

LF sends **four absolute hip/crank angle targets (rad) and two wheel angular
velocity targets (rad/s)**, plus profile/enables and four recovery rate FF.
There is no torque command in HCR TARGET. Torque feedback is optional upstream;
simulation torque and motor Torque Enable must not be confused with downlink.
Feedback knee hinge is phi-hip; target phi is crank angle. Never send the
hinge angle to a physical crank motor by assuming they are interchangeable.

The firmware builds `EspBlMotor` / `BlMotorBackend`; read
`docs/motor-integration.md`. LEFT is UART2/GPIO15, RIGHT UART1/GPIO17
(user correction). IDs are hip2/crank3/wheel1; GUI invert flags were all false.
`apply()` is asynchronous Pending/Applied/Fault, and ACK needs BOTH buses for
one batch. An enqueue or one bus reply must never be acknowledged as six motors.
Manual MOTOR_HOME/MOTOR_ZERO is required; zero is volatile. Build/mock evidence
is not physical validation or proof of servo equivalence to the simulator.

## Checks

Use `tests/check_hybrid_gateway.py` and the LF runtime `test_flow_contract.py` for
contract checks, and distinguish software tests/HIL from actual motor measurements.
Other host checks: `tests/check_bl_motor.py`, `tests/check_inherited_bus.py`,
`python3 -m unittest discover -s tests -p 'test_*.py'`.

## Hardware

Hardware operation — flashing, OTA, motor actuation, ARM — happens only when the
user asks for it in the current task. The USB serial port has one owner: close any
other monitor before flashing, and expect the ESP32 to reset (losing the IMU bias
and the motor zero) whenever it reboots.
