# Missing UART header byte — BL4012e comparison and independent pad capture

## Source findings
Read original BL4012e/3in1/src/joint_bus.c and leg.c, the supplier joint HTML,
and original one-wire POC. Each leg has one bus, addressed ID1/2/3. HCR's request
header/length/checksum and wheel/joint reply offsets agree with joint_bus.c.
Original codec tests and actual HCR motor backend mock tests PASS.

Differences requiring physical evidence:
- BL4012e uses ESP-IDF5.5 uart_write_bytes/uart_wait_tx_done. HCR uses Arduino
  ESP-IDF4.4.7; direct FIFO filling and immediate hardware-idle pad release.
- BL4012e task order wheel/crank/hip, 4ms cycle; enabled wheel alternates write/read.
  HCR active order hip/crank/wheel-write/wheel-read, with both-bus ACK and20ms lease.
- BL4012e permits25 consecutive missed cycles; HCR fails a bad verified command.
  These differences do not establish the cause; tolerances were not imported.
- Checked uart_flush_input and ISR source for matching IDF4.4.7. Flush occurs
  before TX, not after. ISR enabled on the same Arduino core1 as workers; critical
  section suppresses normal RX ISR during FIFO TX. No proof of a flush race.
- Checked a possible destructive FIFO read-modify-write bug. Installed S3 header
  declares rxfifo_rd_byte as full uint32_t. Actual deployed firmware disassembly
  at0x420055f0..0x420055f8 loads RAM byte then performs s32i to FIFO, with NO FIFO
  read. This suspected mechanism is ruled out for that binary.
- Disassembled S3 ROM gpio_matrix_out at0x40050044: routes signal before output
  enable. A known enable-before-route glitch on other ESP targets cannot simply
  be attributed to this S3. Explicit OEN candidate remains unproven.

## Diagnostic addition
Passive RMT RX channel4, memory blocks4/5, GPIO17,10MHz counter, no glitch filter,
no RMT transmitter/interrupt/task/ring buffer. Initialized before UART pad config.
Records prefix without wrap, first known-good wheel READ and first failure.
MOTOR_WAVE reads evidence only; adds no motor commands. UART TX/RX algorithms,
checksums, echo verification, timeouts, targets and gains unchanged.
Monitor arms immediately before TX pad routing. Captures are finite; stopped
capture may omit the trailing high duration and final reply byte. Prefix and
first start bits are the focus, not a claim that every reply byte was captured.
RMT sees the digital input at ESP pad, not analog voltage at the far motor.

Build PASS; original BL codec PASS; HCR motor mock PASS; passive decoder4tests
PASS (all256byte values, missing header, short pulse, truncation). OTA PASS with
same MCU identity and verified OFF; no motor motion performed after this upload.

## Physical OFF control
MOTOR_WAVE ready=1. First good ID1 READ0x24:
TX and UART echo: ffff0104022406ce.
Independent RMT decode starts with exactly ffff0104022406ce, first two start
pulses each1us. Driver reply follows. Trailing reply is truncated by capture stop,
reported explicitly by decoder. Both motor buses configured/OFF, bus_err0.

## Authorized RMT motion capture
User explicitly confirmed test after HOME/stationary restraint request.
CAL_IMU accepted500samples; MOTOR_HOME accepted on both buses; LF RESET reached
DISARMED/flags7. One bounded_arm_probe.py run only, maximum3.25s.
ARM accepted, HOME3s, STANDUP/Folding at3.0337s. FAULT3 at3.1702s; DISARM accepted
at3.1761s. Both drivers OFF verified afterward by six0 enable readbacks.
Firmware ELF SHA256:4a5d66136fe9bbd8a288ed0ee2d620331ac9ab4c016759416212f9f649cd4646.

First failing RIGHT transaction: ID1 READ0x24, expected TXffff0104022406ce.
Echo0/8, RX FIFO0 at release, no UART events. GPIO sampler low0/high69.
RMT FAIL serial203454: all96words zero, status0x000600c0, raw IRQ0. RX memory
write pointer stayed192 (start of block4), no memory ownership/full/read errors.
Independent RMT and GPIO polling both observed no low pulses; UART echo absent.
Good control was exactly the same request and decoded correctly.
TX prepare+wait time130us, echo timeout8284us, total8293us. Hardware TX reported
idle rather than timeout. Routing/OE snapshot: OE0x21c00, UART1 route0x40f,
RX route0x91, IOMUX0x1b00. LEFT bus errors0; RIGHT errors1.
LF policy_steps0; max_handler1.5587ms (cumulative); parser_errors remained1,
no new parser errors during this trial. ESP CRC0/queue_err0.

## Narrowed conclusion and limits
This run reproduced absent whole-packet echo, not the prior one-byte-only loss.
At the ESP digital pad observation, no UART pulse train was captured. A bug solely
in the software echo reader does not explain both independent pad observations.
The remaining boundary is UART transmitter-to-pad output/direction vs a shared
bus held high/contended (or electrical disturbance at pad thresholds). Correct
configuration snapshots alone do not prove actual output drive throughout TX.
This does NOT yet distinguish a GPIO/driver timing fault from motor-bus contention,
and it does not establish that every historical missing-FF case has the same cause.
RMT is digital capture at ESP, not a scope at the motor or supply measurement.

No additional movement/reset/rehome/OTA after this test. All six confirmed OFF,
IMU valid, HOME invalidated by fault, LF FAULT3 intentionally retained.
Next discriminator is UART internal TX mirrored to an independent, confirmed-free
pad versus GPIO17 plus direction timing, or external scope measuring bus/supply.
Do not change pins without checking actual PCB wiring; GPIO18 is tied to DATA and
is not a free diagnostic output. No retry/checksum bypass/relaxed protection.
