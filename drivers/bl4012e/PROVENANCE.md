# BL4012e 3in1 bus component — miss-recovery revision

Source: the user's `BL4012e/3in1/src/libs/joint_bus/joint_bus.c` and
`include/joint_bus.h`, captured during the 2026-10-31 miss-recovery integration.
`upstream/` is byte-identical to that snapshot; SHA256 is recorded alongside it.
The earlier snapshot/implementation is retained in Git commit `c53d6de`.
No license is inferred or added to the user's source.

The firmware compiles **src/joint_bus.c**. This new upstream revision itself
uses TX FIFO, checks idle/free space, releases the pad at TX idle, adds a 100 us
turnaround gap, drains late replies, and can restore exactly one missing leading
FF before validating the reply. This is a deliberate update from the previous
upstream's uart_write_bytes/uart_wait_tx_done implementation; it is not the old
custom EspMotorBus OEN/RMT code reintroduced under another name.

Local changes on top of this snapshot:

- Per-request absolute deadline and cancellation callback. The HCR adapter limits
  requests to 20 ms or the remaining batch/lease budget, whichever is shorter.
  Check before TX, after drain/gap, after reads, and before accepting a reply.
  Cap read waits and TX spin to remaining budget. Require enough remaining wire
  time before starting TX. No callback/owner lock inside the TX critical section.
- Keep draining latched and report Quiet if all ten drain reads still contain
  data. Do not clear the latch and start TX without observing silence.
- Start the turnaround gap after the last drained bytes as well as normal replies.
- Count sync recovery only after ID/length/checksum/opcode/address validation.
  Retain received bytes before inserting the missing FF in the decode buffer.
- Last-error classification, raw TX/echo/reply trace and elapsed timing, C++ linkage.

No automatic retry is added inside the C module. The HCR worker permits one retry
for ping/read/identical absolute MIT only, with the same immutable deadline and
generation. Wheel speed writes keep the ACK-or-matching-target-readback rule.

The bench's `leg.c`, trajectory generation, 10 ms wheel slot and MISS_LIMIT25
remain outside HCR. HCR skips only unchanged, previously confirmed wheel RPM
within the same enabled session (100 ms keepalive), while continuing feedback
reads. Changed RPM is sent immediately; it is not queued behind a wheel slot.

Review local changes with `diff -u upstream/joint_bus.c src/joint_bus.c` and the
corresponding header diff. From the repository root run:

```sh
python3 tests/check_inherited_bus.py
python3 tests/check_bl_motor.py
```

Host fixtures derive from BL4012e's original SDK mock. They compare SDK call
traces on the covered common unguarded paths, test the actual C++ adapter + C
module with deadline/cancellation, late/noisy replies and corrupted frames, and
use negative controls for omitted TX drive and falsely declaring a busy bus quiet.
They do not prove electrical timing or mechanical performance.
