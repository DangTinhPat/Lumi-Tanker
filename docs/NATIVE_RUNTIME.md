# MuJoCo runtime for the lab branch

## Active simulation path

The only MuJoCo controller runner is `tap_mujoco_gim_v3`. It loads
`src/tap_sim/model/gim_dynamic.xml`, uses firmware `V3Controller`, and models
the four leg actuators with current-difference compensation. The old
`EmbeddedController` runner has been removed. Its core implementation remains
for host tests and examples; it is not used by this simulation.

Build and launch:

```bash
cmake -S . -B build -DBUILD_TESTING=OFF -DTAP_BUILD_SIMULATOR=ON
cmake --build build --target tap_mujoco_gim_v3 -j4
python3 tools/tap_native_gui.py
```

The dashboard opens the separate MuJoCo renderer. Spawn and Reset use the
extended GIM pose defined by the `spawn_extended` keyframe. Model frames,
collision geometry, mass assumptions, friction and actuator assumptions are
recorded in [`GIM_SIM_PHYSICS_SPEC.md`](GIM_SIM_PHYSICS_SPEC.md) and
[`src/tap_sim/model/PHYSICS.md`](../src/tap_sim/model/PHYSICS.md).

## Sensor and action boundary

Controller state comes from the simulated MPU6500 and six joint encoders.
Simulated leg-driver current is used only by the current-compensation actuator
layer. MuJoCo world pose, contact, constraint forces and VIEW snapshots are
telemetry/rendering; they do not enter `V3Controller` feedback.

The runner accepts `ENABLE`, `GO`, `DISABLE`, `ESTOP`, `RESET`,
`DRIVE <forward_m_s> <yaw_rad_s>`, and `HEIGHT <body_height_m>` commands. It
emits `STATE` diagnostics at 50 Hz and separate `VIEW` snapshots. DRIVE is
bounded to ±0.5 m/s and ±0.8 rad/s; height is clamped to the supported model
range.

## AI-training status

The recovery Gymnasium environment and PPO entrypoints are under
`src/tap_sim/training`; see [`RECOVERY_TRAINING.md`](RECOVERY_TRAINING.md).
They step the same GIM MJCF directly through the Python MuJoCo bindings. The
dashboard and native runner remain realtime interactive tools, not training
interfaces. No recovery policy is trained or integrated with firmware yet.
Record the model hash, actuator settings, seed, fall scenario and observation
contract with each run.

Mass/inertia and actuator response remain estimates. A successful MuJoCo run
is evidence for this model only, not hardware validation.
