# ARM rejection and first stand-up deadline — 2026-10-31

## Observed physical run (operator pressed ARM)

LF accepted ARM at monotonic453.608s and entered HOME. At456.629s it entered
FAULT11/control_deadline in Folding; telemetry handler15.941ms exceeded15ms.
No policy inference occurred. ESP local_fault1/TargetTimeout, all six OFF,
both bus_err0, IMU and home retained. Later ARM requests were refused by the
latched FAULT. This is distinct from the earlier missing-calibration FAULT2.

## Cause and fix

V3Controller::update builds one point of a 12-point LQR gain table each tick.
The hybrid adapter had not run Idle ticks before its first stand-up tick.
Independent helper probe on LF reproduced 12 slow ticks after every reset,
15.1–15.4ms each, followed by ~0.8ms ticks.

Added a V3Controller boot preparation method forwarding to the existing gain
builder, without advancing controller or estimator state. Helper prepares a
pristine controller once at startup and copies it on reset. Same table points,
gains, configuration and update logic; no dummy sensor history. Startup exchange
has2s timeout before UART service starts; control/reset helper timeout remains20ms.
LF handler deadline15ms and MCU target TTL20ms unchanged.
GUI now displays the returned fault reason on rejected commands. Deadline faults
also include measured handler duration. No ARM was sent by the assistant.

## Evidence

Host/ARM builds passed. Old/new host helper replies were byte-identical for6,600
samples in3 reset scenarios; output_equivalence.json. Existing23 Python tests PASS.
LF staging probe: boot189.304ms,600ticks/6resets, maximum1.672ms.
Deployed helper probe: boot184.184ms, maximum1.983ms; all6reset output sequences
identical. Final physical motion/stand-up/handoff has NOT been rerun.

The second probe ran with FIFO20 while the DISARMED runtime was active. Its
184ms startup starved the runtime, causing queued-telemetry FAULT4 and output
backpressure exit. This is recorded in lf_after.json; not a clean runtime PASS.
Probe completed, service restored. Timing tool now refuses to run while the
runtime lock is held; use only with runtime stopped and motors OFF. No protection
threshold was relaxed to hide this diagnostic-induced failure.

Final state is recorded separately in lf_final.json and esp_final.txt.
Deployment files were hash-verified in deployed_sha256.json, backups under
before/ locally and *.before_lqr_prepare_20261031 on LF. Original vendor header
V3Controller.hpp intentionally gained the additive preparation API; original
package hashes for that file are consequently no longer unchanged.

Runtime remains DISARMED. ARM still means HOME3s then stand-up automatically;
no dedicated HOME-only physical test mode was added. Current source/build and
isolated timing evidence do not establish physical balance or servo equivalence.
