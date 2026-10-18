# TAP workspace notes

This repository contains firmware v3.3, portable controller code and host
simulation tools. The supplied CAD input is `ROBOT (GIM6010-8).STEP`; it has
not yet been converted into a validated simulation model. Do not launch the
native MuJoCo runner or viewer until a replacement model is derived and checked
from that CAD.

The firmware controller is `src_fw/src/v3/V3Controller.cpp`. Host planar
simulation harnesses are in `src_fw/tests`; the native runner in `src/tap_sim`
uses the separate `EmbeddedController` and is not evidence for firmware
behavior. Existing generated geometry has not been verified against the
supplied STEP, so report its simulation results as provisional.

Preserve the sensor contract and evidence distinctions in `AGENTS.md` and
`docs/RULES.md`. Do not operate hardware, flash firmware, or issue motor
commands without an explicit request.
