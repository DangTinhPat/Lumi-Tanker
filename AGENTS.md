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

No CAD model is currently configured for simulation. Keep the supplied GIM
STEP as source input; derive joint frames, link inertias, collision geometry
and limits from it before enabling a simulator model. Do not reuse cached
geometry or physical constants from the removed model. The deployable sensor
inventory is one MPU6500 (3-axis accelerometer and
  3-axis gyroscope) plus six motor encoders (four leg joints, two wheel
  joints). Wheel-contact switches, force/torque sensors, motor current feedback
  (except the leg-driver exception in `docs/RULES.md` Rule 1), range sensors,
  cameras and external localization are excluded from the control design and
  must not become hidden dependencies. Simulator contacts, efforts and pose are
  telemetry only; derive support hypotheses from MPU6500 and encoder data.
- Record verified model parameters and their source when a replacement model
  is adopted.

## Architecture And Evidence

The layout and the firmware/simulator split are in `architecture.md`. The robot
runs `V3Controller` (`src_fw/src/v3`); the MuJoCo simulator runs the older
`EmbeddedController`, so a simulator result is not a result about the firmware.
Keep the control core free of desktop middleware and robot-specific parameters
explicit. Use portable resource lookup, not machine-specific paths in runtime
code. Current firmware state and rejected ideas: `docs/V3.3_HANDOFF.md`.

Do not modify `docs/RULES.md` or `architecture.md` unless the user explicitly
authorizes that document change. Reading and proposing changes is allowed;
writing either protected contract requires clear user permission.

Provenance of ported code is in `docs/UPSTREAM.md`. Report source inspection,
build, unit tests, planar firmware sims, MuJoCo runs and robot logs as different
evidence levels. A working GUI or 3-D view does not prove balance.
