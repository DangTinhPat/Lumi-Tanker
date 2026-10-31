# Runtime fault recovery — 2026-10-31

Observed LF FAULT2: IMU/encoder/calibration validity missing, flags=2.
LF uptime ~356s confirmed a reboot since prior verification. ESP IMU bias=0,
calibration samples=0, imu_valid=0, homed=0, cmd=0; firmware unchanged.
Both motor buses remained configured/OFF with zero bus errors. GUI status API
was HTTP200 and online. This was a sensor-calibration gate, not a dead GUI.

After user reconfirmed exact HOME and stationary robot:
1. CAL_IMU completed, bias=[.00778,-.23508,.00909]rad/s.
2. MOTOR_HOME accepted, both sides homed=1, flags=7.
3. RESET via authenticated localhost8080 /api/command accepted.
4. Twelve status observations remained DISARMED/fault0/linktrue/flags7.
   Valid telemetry grew2930→5129 over11.005s (~199.8Hz).
   Parser errors remained4 and rejected remained26 (historical, no increment).
5. ESP local_fault0, armed0/enabled0; both buses OFF/homed/error0.

No ARM, movement, firmware flash, service restart or safety-gate change.
Calibration and home still volatile: repeat after ESP reboot with stationary
and correct-pose confirmation. Stored logs: esp_status.txt, lf_status.txt,
recalibration.txt, after_reset.json, esp_after.txt. Token is not stored here.
