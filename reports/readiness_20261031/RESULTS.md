# Readiness check — 2026-10-31

Read-only check on the running LF and ESP, no ARM, reset, restart, flash or motor enable.

- 15-second console observation; LF counter samples span 13.015 seconds.
- Valid telemetry increased 93627 → 96229; measured rate 199.921 Hz.
- All sampled LF statuses: DISARMED, fault=0, flags=7, parser_errors=0, rejected=0.
- ESP: IMU valid, calibration retained; both motor buses configured/homed/OFF, bus_err=0.
- ACK of disabled targets advances; no enabled target was sent.
- UART stale_tx and queue-full counters remain 0. Service missed wakeups 2586 → 2647;
  tick_max_us=650. This is not proof of hard real-time scheduling under motion load.
- Web GET /api/status authenticated: HTTP200, runtime_online=true; 99,631 valid frames.
- ONNX, RKNN and normalizer hashes match the deployed manifest; backend=rknn.
- Source verified: LF requires fresh valid telemetry and home tolerance .15rad for ARM;
  MCU also gates fresh IMU/encoder/home, no local fault, verified OFF, valid epoch/source seq.
- Model is loaded; current session has 0 inference because it remains DISARMED.

Expected sequence after an operator ARM: HOME for3s → CPU logic STANDUP →
model warm-up at12.5s with logic owning targets → MODEL no earlier than13s,
only with Support and >=50warmup steps. MCU maps four hip/crank targets and two
wheel rates to drivers, ACKs completion on both buses, sends feedback upstream.

Conclusion: communication/sensing/configuration prerequisites are present now.
Actual motor-enable startup, six-motor command ACK latency, stand-up, policy handoff
and balance remain physically unverified. Driver servo gains/response are not yet
proven equivalent to the MuJoCo fixture. Prior LF NPU tests include a >15ms jitter
failure; no inference or full-load test was performed in this read-only check.
Next meaningful physical validation is a controlled HOME hold with motor enable,
followed by stand-up/handoff only after its response and timing are checked.
Current ARM automatically continues into stand-up after3s; it is not HOME-only.
IMU bias/HOME are volatile. LF persistent bind-mount hook is installed but reboot
behavior remains untested; underlying read-only rootfs needs offline maintenance.
