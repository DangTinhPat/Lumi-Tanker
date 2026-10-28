# TAP workspace notes

This repository is the RB R1 two-wheel-legged robot. Two things live here:

- the **v3.3 firmware that runs on the robot** (ESP32-S3): `firmware/esp32/`
  (adapter, console, logging) and `controller/v3/` (`V3Controller`, leg lock, tests);
- a **native MuJoCo simulation** with a Tk/3-D viewer GUI: `simulation`,
  `gui`. The canonical model is `simulation/model/tap_robot.xml`,
  generated from the supplied STEP assembly and detailed STL assets.

Both use the portable C++17 core in `controller/{core,hal}`. The
simulator runs the older `EmbeddedController`, not the firmware's
`V3Controller`; the firmware's control law is exercised by the host tests in
`controller/v3/tests` (planar plant, `v3_tier_tests`) and by robot logs. A simulator
result is not evidence about the firmware, and vice versa.

Do not operate hardware, flash firmware or deploy motor commands unless the user
asks for it in the current task. Simulation and host tests are evidence for the
models they exercise only, not certification of physical robot capability. The
deployable sensor contract remains one MPU6500 plus six motor encoders;
simulator pose, contact and constraint data must not enter controller decisions.

Build and test from the repository root (add `-DTAP_BUILD_SIMULATOR=OFF` when
MuJoCo is not installed):

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON
cmake --build build/native -j4
ctest --test-dir build/native --output-on-failure
~/.platformio/penv/bin/pio run -e esp32-s3-v3
```

Run the simulation, the dashboard, or the robot console:

```bash
build/native/bin/tap_mujoco_native \
  --model simulation/model/tap_robot.xml --run-for 5
python3 tools/tap_native_gui.py --rate 3000
python3 tools/tap_serial_control_gui.py --port /dev/ttyUSB0
```

Robot-frame contract: joint order is `[LH, LK, LW, RH, RK, RW]`, both leg sides
use native `+Y` axes, the root is free, and the CAD model is the source of
visual geometry, joint origins and provisional mass/inertia estimates. Keep
visual STL detail separate from primitive collision envelopes.

Relevant references: `README.md`, `architecture.md`, `docs/V3.3_HANDOFF.md`
(current firmware state, boot configuration, measured hardware facts, rejected
ideas), `controller/v3/FIRMWARE_V3.md` (console commands, `SET` keys, log fields),
`docs/NATIVE_RUNTIME.md`, `simulation/model/PHYSICS.md` and the checked-in CAD
manifest. Preserve the protected project contracts in `docs/RULES.md` and
`architecture.md`.
