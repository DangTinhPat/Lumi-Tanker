# Deep diagnosis of repeated ARM failures — 2026-10-31

## What is established

The initial ARM was ACCEPTED every time. LF held HOME for3s, entered STANDUP,
then faulted during Folding about0.12–0.15s later. Subsequent ARM requests were
rejected because FAULT is latched. NPU/model inference count remained0.
After the earlier LQR preparation fix, LF maximum handler stayed~1.35ms, not15ms.

The current failing boundary is ESP32-to-motor one-wire UART, not LF–ESP HCR.
A missing or incomplete echo means ESP did not receive back the exact bytes it
intended to transmit. It alone cannot distinguish waveform corruption from a
local UART RX capture problem. No claim that cables, driver or supply are proven
faulty, and no claim that the current motor-motion issue is fixed.

## Reproduction and control cases

- Original user run: RIGHT/UART1/GPIO17, wheelID1 WRITE0x20, EchoTimeout;
  TX113us, echo0, timeout8297us. Both buses confirmed OFF afterward.
- Instrumented controlled probe1: accepted ARM, stop at3.164s after fault;
  RIGHT wheel READ0x24, TX105us, echo0, RX FIFO at release0, ring0,
  no UART data/error events; DATA sampled high before/after TX.
- Probe2 (measuring GPIO levels): accepted ARM, stop at3.161s;
  LEFT wheel READ0x24 had full echo8/8 but no reply15bytes;
  RIGHT OFF WRITE0x18 also failed with echo0, low_samples0/high69,
  GPIO route UART1=0x0f, output-enable bit17 set, peripheral OEN selected.
  Right-side OFF failure can be secondary to the left-side initial fault;
  first-failure records are per bus, not a globally ordered event trace.
- Probe3 with explicit GPIO OEN ownership: accepted ARM, stop at3.183s;
  RIGHT READ0x24 TX=ffff0104022406ce, echo=ff0104022406ce (7/8bytes),
  low39/high31, RX FIFO7, no reply. Explicit OE changed the observed trace
  but did NOT resolve the motion failure. It is not a proven complete fix.
- Two OFF-only control runs: each2000cycles per bus, total8000cycles across
  the two builds, zero bus errors. Each cycle writes wheel0, reads it back,
  and polls encoders; no MIT/enable writes. This does not reproduce motor load.

## Difference from the BL4012e bench firmware

BL4012e/3in1/src/leg.c uses CONTROL_PERIOD_MS=4 and MISS_LIMIT=25 consecutive
failed cycles before declaring a motor lost. Enabled wheels alternate speed
write and feedback read; the speed-write function does not return its failure
as a wheel_cycle liveness failure. Thus a working bench GUI does not establish
error-free transaction-by-transaction communication. HCR requires verified
both-bus application and20ms motor lease. These protections were not relaxed.
Do not infer that the bench actually lost packets without its recorded counters.

## Changes retained

- UART hardware diagnostics: transmitted/echo bytes, pad levels during TX,
  GPIO routing/OE/mux, FIFO, ring, raw/enabled interrupts and UART events.
- MOTOR_BUS_TEST: bounded2000 OFF-only cycles per bus; rejects active/unconfigured/
  faulty states; invalidates HOME and blocks home capture until complete.
- Explicit GPIO output enable during TX, released at hardware TX idle; still
  checked echo, reply, checksum, ID/status and original transaction budgets.
- LF command rejection details distinguish wrong state, stale telemetry and HOME.
  GUI displays blockers and disables ARM outside valid DISARMED/sensor conditions.
- Motor mock/core interop2700cycles PASS, OFF-test safety assertions PASS;
  firmware builds/OTA PASS;24runtime Python tests PASS; JS syntax check PASS.
  No additional physical ARM after probe3. No current/torque payload changes.

## Final state and unresolved cause

All six drivers confirmed OFF. MCU driver fault remains latched, homed=0;
LF remains FAULT3. GUI accurately shows the blocked state, rather than claiming
readiness. Calibration/home cannot be assumed after faults/OTA or movement.

To distinguish remaining mechanisms, capture DATA at ESP GPIO17 and at the
motor bus during the first failing transaction, ideally with supply voltage at
the driver simultaneously. Verify the first FF start bit, bus release/drive
handover and any supply dip under joint motion. Firmware samples do not prove
signal integrity or voltage margin; do not blame wiring or change thresholds
without this evidence. Further motion requires a new controlled-test setup.

Logs: lf_before.txt, esp_before.txt, off_test.txt, motion_lf.log,
motion_console.txt, pad_motion_lf.log, pad_motion_console.txt,
oe_off_test.txt, oe_motion_lf.log, oe_motion_console.txt, lf_final.json,
esp_final.txt. Probe code bounded_arm_probe.py never retries ARM and sends
DISARM on fault or at3.25s. Three probes were separately authorized by user.
