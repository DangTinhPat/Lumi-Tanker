# Battery replacement comparison — 2026-10-31

User reported a new battery, then explicitly confirmed HOME, stationary restraint,
and authorized IMU calibration, HOME capture and one test bounded to 3.25 seconds.
No firmware/config changes or flashing in this comparison.

## Before motion
LF had rebooted; FAULT2, missing IMU calibration/HOME. Both motor buses configured,
verified OFF, fresh encoder samples, zero bus errors. HCR linked, ESP CRC0.
CAL_IMU accepted: 500 samples, gyro bias [0.00771,-0.23810,0.00846] rad/s.
MOTOR_HOME accepted; both homed=1. LF RESET accepted; DISARMED, flags7.
Firmware ELF SHA256: ca251b0c681daea076646c8c158500835e35187ba50faabb9500e288c9748072.

## Single authorized motion test
Used unchanged bounded_arm_probe.py, executed locally on LF; guard at3.25s.
ARM accepted. HOME held3s. STANDUP/Folding observed at3.0195s.
FAULT3 driver_fault observed at3.1894s; DISARM accepted at3.1975s.
LF max handler1.5587ms, policy_steps0, no deadline misses.
RIGHT UART1 GPIO17 wheelID1 WRITE0x20: EchoTimeout, bus_err1.
TX:   ff ff 01 05 03 20 00 00 d6
Echo:    ff 01 05 03 20 00 00 d6
Echo8/9; FIFO8 at release, ring0. tx_us323, echo_us8247,total_us8262.
TX timing includes pre-transmission preparation, flush and scheduling; it is not
an oscilloscope measurement of wire transmission length. Echo failure returns
before the reply-read stage; reply_bytes0 alone does NOT prove no driver reply.
LEFT bus_err0. Both sides verified OFF afterward, enable_readback=[0,0,0].
IMU remains valid; HOME cleared by motor fault. No automatic retry/reset/rehome.

## Interpretation and limits
New battery did not prevent the motor-bus failure. Missing firstFF recurs as in
previous explicit-OEN test, now on a WRITE packet rather than READ. Proven
failure boundary is ESP motor-UART echo capture; actual wire bytes vs RX loss
remain unresolved. No battery/driver supply measurement was taken; a new battery
alone does not rule out connector/DC supply drops or ground disturbance under load.
LF parser_errors increased0->1 by the first HOME snapshot, stayed1; ESP CRC0.
This separate HCR parser event did not immediately trip the runtime; the recorded
stop reason is motor driver fault. Do not report this run as wholly error-free HCR.

Next discriminator: synchronized waveform on GPIO17/bus and driver supply during
transition, inspect first start bit, TX enable/idle setup, bus handover and local
RX FIFO/reset timing. Do not skip echo/CRC or relax deadlines to hide the failure.
Further motion requires a fresh HOME/setup confirmation because joints moved.
