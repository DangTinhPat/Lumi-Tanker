# Lumi-Tanker

Firmware v3.3 and host simulation tools for a two-wheel legged robot.

The control firmware is in `firmware_esp32/` and `src_fw/`; the portable
control code and motor interfaces are in `src/tap_control/`. Host planar
simulation harnesses are in `src_fw/tests`. The MuJoCo runner and GUI remain
available as software, but this workspace currently has no robot model
configured for them. Do not use their old generated geometry as a replacement
model. `ROBOT (GIM6010-8).STEP` is the supplied CAD input and has not yet been
converted into a validated simulation model.

Read [`architecture.md`](architecture.md), [`docs/RULES.md`](docs/RULES.md),
and [`docs/V3.3_HANDOFF.md`](docs/V3.3_HANDOFF.md) before control or model
changes. Simulation and host tests are evidence only for the models they
actually exercise; they do not certify physical performance.

## Host build

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON -DTAP_BUILD_SIMULATOR=OFF
cmake --build build/native -j4
ctest --test-dir build/native --output-on-failure
```

The planar firmware simulation harnesses are selected with environment
variables, for example:

```bash
TIER_IQ_COMP=1 build/native/bin/v3_tier_tests
```

## Firmware build

```bash
~/.platformio/penv/bin/pio run -e esp32-s3-v3
```

Hardware operation, flashing, and motor commands require a separate explicit
request.
