# TAP Project Guidance

`/home/dvt/Lumi-Tanker` is the main workspace. Preserve concurrent user/agent
edits. The user authorizes algorithm work for the robot firmware (v3.3) and its
simulations. Hardware operation — flashing, motor actuation, running the robot —
happens only when the user asks for it in the current task.

## Session Hygiene

Before reopening a simulator or GUI session, stop the existing TAP sessions
(native runner, dashboards, vision follower and their child processes,
including orphans). Identify ownership from launch ancestry, workspace paths
and session metadata (`tools/owned_process.py`); never kill unrelated
applications or all Python processes by name, and confirm no live leftovers
before launching the replacement. The USB serial port has one owner: close the
serial GUI before flashing, and expect the ESP32 to reset (losing runtime `SET`
overrides) whenever the port is reopened.

## Expert Routing

Read the relevant discoverable skill before domain work. For cross-domain work,
use each relevant expert and explicitly hand off frame conventions and evidence.
These are persistent expert instructions, not claims of running agent processes.

| Work | Skill |
| --- | --- |
| CAD, FK, COM, inertia, contact geometry | `.agents/skills/tap-robot-physics/SKILL.md` |
| Balance control, trajectories, control modes, recovery | `.agents/skills/tap-control-expert/SKILL.md` |
| Tests, regressions, simulation evidence | `.agents/skills/tap-qa-regression/SKILL.md` |
| Balance/fall-recovery theory (humanoid, quadruped, wheel-legged), before designing a new mechanism | `.agents/skills/legged-locomotion-references/SKILL.md` |

## Robot Model

The default MuJoCo model on `lab` is derived from `ROBOT (GIM6010-8).STEP`:
`src/tap_sim/model/gim_dynamic.xml`. Joint frames and geometry are CAD-derived;
mass, inertia and actuator response are documented estimates, not identified
hardware values. Keep their provenance in `src/tap_sim/model/PHYSICS.md` and
`docs/GIM_SIM_PHYSICS_SPEC.md`.

The deployable sensor inventory is one MPU6500 (3-axis accelerometer and
3-axis gyroscope) plus six motor encoders (four leg joints, two wheel joints).
Leg-driver current feedback is allowed only under Rule 1 in `docs/RULES.md`.
Do not add hidden control inputs from cameras, localization or MuJoCo truth.
Simulator contacts, efforts and pose are telemetry/rendering only.

## Architecture And Evidence

Firmware and the GIM MuJoCo runner both call `V3Controller`
(`src_fw/src/v3`). The runner is `src/tap_sim/native/gim_v3_runner.cpp`; the
GUI loads the GIM model above. The legacy `EmbeddedController` remains in the
host core for its tests and examples, but is not a MuJoCo runtime controller.
Keep the control core free of desktop middleware and robot-specific parameters
explicit. Use portable resource lookup, not machine-specific paths in runtime
code.

This branch focuses on AI training in MuJoCo. The recovery Gymnasium environment
and PPO entrypoints are in `src/tap_sim/training`; no policy is trained or
firmware-integrated yet. Preserve seeded resets and record each model, actuator
setting, seed and fall scenario. Firmware state and hardware evidence remain in
`docs/V3.3_HANDOFF.md`.

Do not modify `docs/RULES.md` or `architecture.md` unless the user explicitly
authorizes that document change. Reading and proposing changes is allowed;
writing either protected contract requires clear user permission.

Provenance of ported code is in `docs/UPSTREAM.md`. Report source inspection,
build, unit tests, planar firmware sims, MuJoCo runs and robot logs as different
evidence levels. A working GUI or 3-D view does not prove balance.
