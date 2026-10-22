# TCP interruptions and uncommanded restart, 22 October 2026

Recording: `/home/dvt/tap_logs/tap_20261022_195321.log` (deployed standprep build).
The user confirms no intentional reset: the robot was moving on the floor.

Observed evidence:

- TCP reconnects at log seconds 59.490, 60.305, 61.028, 61.812, 67.695,
  and 117.245. At 117.482 the robot is Idle/unhomed and heap/counters are
  reinitialized, consistent with a reboot around 19:55:18. No reset reason
  was exposed by this deployed build, so the reboot trigger is unconfirmed.
- During the first reconnect cluster, sampled RSSI is approximately -33 to
  -45 dBm. At 61.294 free heap is 5916 bytes; lifetime low-water mark is
  2848 bytes. Network-task report gaps remain approximately 7 ms. Strong RSSI
  does not exclude radio interference, AP stalls or power problems.
- GUI console (read via Tk IPC, without opening a second robot client)
  reports `no data from the robot for 3 s`, `Connection timed out`,
  `robot closed the Wi-Fi connection`, and `Broken pipe`.
- One GUI process is present (PID 145198); no competing TCP controller found.
  The log has zero RC rows, so automatic recording replay does not explain
  this active-run interruption. No runtime tuning was changed for diagnosis.
- A later eight-packet idle ping test received all replies. An idle ping does
  not establish stability during motor motion.

Source findings and prepared changes:

1. GUI send/reconnect socket ownership had a race. While an old socket's write
   fails, the reader may install a replacement; the old exception handler then
   closes `self.port`, potentially closing that replacement. Keep the actual
   written port locally and close only it; serialize replacement with the
   existing write lock. Check stop_event under that lock before installing a
   newly connected socket. This repairs a reproducible race; the available
   log cannot prove how many recorded reconnects it caused.
2. Persist read/write exception text in the session log, rather than only the
   GUI text panel, for future causal analysis.
3. Firmware GET now reports esp_reset_reason(), uptime and heap, plus last
   fatal send errno and counts of fatal-send and stall/low-heap socket closes.
   These are diagnostics only; no controller, lease, motor, Wi-Fi power or
   reconnect timeout settings changed. The deployed firmware closed sockets
   on any send errno except EAGAIN/EWOULDBLOCK, but did not record which errno.

Validation: two fake-port ownership/shutdown regressions and four existing
telemetry decoder tests pass. The roll firmware candidate builds successfully.
Deployment follow-up, at the user's request: the diagnostic firmware was
OTA-flashed and its new GET diagnostics verified. The patched GUI was launched
from `/home/dvt/Lumi-Tanker-1840-addwifi`, then reopened at 20:21; connection
confirmed in `tap_20261022_202147.log`. Deployed binary SHA256:
`0e11ef0431fa74139cab807f60dc0b44197cc6f7cfc9d83342b2cdd09dfcdaa8`.
The immediate post-OTA reset_reason=3 is the expected software restart from
upload, not evidence of the earlier uncommanded reboot. No robot motion was
commanded by the agent.
Root cause of the ESP reboot remains pending captured reset/error diagnostics;
heap pressure is a leading concern, not proof of brownout, OOM or watchdog.
