# TAP Dabble gamepad bridge

`DabbleGamepadMapper` is the controller-facing bridge. It does not contain
balance logic and it does not send motor frames; the Dabble input task maps
button state into `GamepadState`, then the 500 Hz firmware loop passes the
resulting command to `FirmwareControlLoop`.

## Dabble setup

Dabble uses the `DabbleESP32` library's own BLE protocol. Do not configure the
custom UUIDs below in Dabble; use them only if a future generic BLE app writes
directly to `BleCommandEndpoint`.

The application-side integration is:

```cpp
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include "tap_fw/DabbleGamepadMapper.hpp"

tap_fw::ble::CommandEndpoint endpoint;
tap_fw::ble::DabbleGamepadMapper mapper(endpoint);

void setup() {
  Dabble.begin("TAP-RB");
}

void loop() {
  Dabble.processInput();
  tap_fw::ble::GamepadState input{
      GamePad.isUpPressed(), GamePad.isDownPressed(),
      GamePad.isLeftPressed(), GamePad.isRightPressed(),
      GamePad.isStartPressed(), GamePad.isSelectPressed(),
      GamePad.isTrianglePressed(), GamePad.isCirclePressed(),
      GamePad.isCrossPressed(), GamePad.isSquarePressed()};
  mapper.update(input, monotonic_us());
  const auto command = endpoint.command(monotonic_us());
  firmware_loop.step(sensor_state, command, monotonic_us());
}
```

The mapper uses this gamepad mapping:

| Dabble input | Action |
|---|---|
| Up | Forward `+0.25 m/s` |
| Down | Reverse `-0.25 m/s` |
| Left | Turn left |
| Right | Turn right |
| Start | `ENABLE` plus automatic `FIXTURE_RELEASE` after sensor qualification |
| Select | `DISABLE` |
| Triangle | `MAX_STAND` (`HEIGHT 0.245`) |
| Circle | `DEEP_SQUAT` (`HEIGHT 0.140`) |
| Cross | `NATURAL` (`HEIGHT 0.148`) |
| Square | `ESTOP` |

There are no pose or height sliders. Pose buttons act on the press edge and
also send zero drive demand. When no direction is pressed, the mapper sends
zero drive demand. The endpoint watchdog disables the controller after 500 ms
without a valid update. `RESET` and `CLEAR_ESTOP` remain available through the
generic command endpoint. One Start press requests enable and fixture release;
the state machine still waits for valid sensor qualification before it leaves
the startup hold and ramps into balance. No second button press or PCB button
is needed for this flow.

## Generic BLE alternative

For a generic BLE app that can write text, the optional service is:

```text
Device: TAP-RB
Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
Write characteristic: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
```

It accepts `ENABLE`, `DISABLE`, `ESTOP`, `HEARTBEAT`, `DRIVE`, `STOP`,
`HEIGHT`, `POSE`, `STAND_UP`, and `FIXTURE_RELEASE`.

## ESP-IDF callback shape

The BLE write callback should do only this work:

```cpp
const auto result = ble_endpoint.onWrite(data, length, monotonic_us());
if (result == tap_fw::ble::WriteResult::ResetRequested) {
  controller.reset(sensor_state);
}
```

The control task uses:

```cpp
const auto command = ble_endpoint.command(monotonic_us());
firmware_loop.step(sensor_state, command, monotonic_us());
```

On a generic GATT disconnect, call `ble_endpoint.onDisconnect()`. The endpoint
then forces a safe disable until a new `ENABLE` command is received.
