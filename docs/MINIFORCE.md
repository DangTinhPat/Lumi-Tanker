# miniforce

Base: a787a73 (18:40t). The default PlatformIO environment is
`esp32-s3-v3-1840-roll`, matching the firmware selected for deployment.

Boot configuration: hip lean share 0.85, hip disturbance share 0.45,
roll enabled, wheel UART workers on CPU 1 with priority 22. MOVE/TURN,
finite motion, yaw, hop and portable controller sources remain from 18:40t.
Wi-Fi is station-only; AP fallback is removed. Local credentials belong in
ignored `firmware/esp32/include/tap_hw/WifiSecrets.hpp` (see the example header).

The three explicit environments retain the comparison sequence:

- `esp32-s3-v3-1840-addwifi`: hip 1.00/0.60, roll off, UART CPU 1 priority 22.
- `esp32-s3-v3-1840-hip`: hip 0.85/0.45, roll off, same UART settings.
- `esp32-s3-v3-1840-roll`: hip 0.85/0.45, roll on, same UART settings.

`SET addwifibal 0/1` requires Idle and released legs; it changes hip/roll only,
not UART CPU/priority. Regular `esp32-s3-v3` and single-loop environments
retain their original 18:40t balance configuration.

Validation: candidate, regular and single-loop firmware builds passed.
Six host checks passed: rolling handoff/disable with both hip profiles,
stand-up, motor feedback, hop, motion profile and S-curve tests.
The planar balance/height diagnostic had 0/24 falls for both compared profiles,
but the donor profile increased mean gyro and hip >5 Hz residuals by 27.7% and
22.5%; it does not establish improved hardware stability or exercise Wi-Fi,
UART scheduling or the floor stand-up sequence.

The user reported successful stand-up with the donor profile and again after
restoring the 18:40t hip values. Roll was then enabled and persisted at the
user request. The final roll-enabled firmware was USB-flashed, its flash hash
and post-boot settings verified, and GUI connected. This is not a claim of
validated driving/turning or repeated stand-up success for every profile.

Final deployed binary SHA256:
`aa3cb8a965d334d4dd238f35409dab13ec5f3536a094211ab0a249b4932b8ac0`.

USB flashing after OTA must select the written application partition. During
this session OTA metadata selected app1 while USB writes targeted app0.
After confirming the partition layout, metadata was backed up and app0 selected
with Arduino boot_app0.bin. Verify the boot settings rather than assuming a
successful write means the written image is running.

Build without flashing: `pio run`.
Host: configure with `cmake -S . -B build -DTAP_BUILD_SIMULATOR=OFF`, then build
and run the corresponding CTest targets. The optional diagnostic is
`TIER_ADDWIFI_COMPARE=1 TIER_ADDWIFI_PROFILE=0/1 build/bin/v3_tier_tests`.
