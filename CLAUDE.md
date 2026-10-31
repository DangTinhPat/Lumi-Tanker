# Workspace notes — Luckfox gateway

This repository is the ESP32-S3 lower layer of the RB R1 robot: it reads the IMU
and drives the BL4012e motors on two one-wire buses, and relays data to the
Luckfox (LF) upper layer over UART (HCR1). Control decisions live on the LF.
Layout and roles of every folder: `README.md`. Interface rules: `AGENTS.md`.

Do not operate hardware, flash firmware, run OTA or send motor commands unless the
user asks for it in the current task. Host tests and mocks are evidence for the
models they exercise only, not validation of the physical robot.

Build and test from the repository root:

```bash
~/.platformio/penv/bin/pio run -e esp32-s3-hybrid-mcu
python3 tests/check_hybrid_gateway.py
python3 tests/check_bl_motor.py
python3 tests/check_inherited_bus.py
python3 -m unittest discover -s tests -p 'test_*.py'
```

`reports/` holds dated experiment logs and snapshots of earlier code; treat it as
evidence, not as live source. Credentials in `include/gateway/*Secrets.hpp` and
`.local/` are git-ignored and must not be copied into documents.
