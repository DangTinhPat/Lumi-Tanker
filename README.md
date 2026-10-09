# Lumi-Tanker AI lab

The `lab` branch is for AI experiments in MuJoCo using the GIM6010-8 robot
model and firmware V3.3 as the reference controller. The default GUI runs
`tap_mujoco_gim_v3` with `src/tap_sim/model/gim_dynamic.xml`.

The model geometry comes from `ROBOT (GIM6010-8).STEP`. Mass, inertia and
actuator response are documented estimates; simulator results are not hardware
validation. V3 balance feedback uses simulated MPU6500 data and six joint
encoders; simulated leg-driver current is used only by the ΔI actuator layer.
MuJoCo pose and contact data are for rendering and diagnostics.

## Build and run

```bash
cmake -S . -B build -DBUILD_TESTING=OFF -DTAP_BUILD_SIMULATOR=ON
cmake --build build --target tap_mujoco_gim_v3 -j4
python3 tools/tap_native_gui.py
```

The GUI starts the GIM V3.3 runner and opens its MuJoCo viewer. For controller,
model and sensor details, see [Native runtime](docs/NATIVE_RUNTIME.md) and the
[GIM physics specification](docs/GIM_SIM_PHYSICS_SPEC.md).

The branch now includes a Gymnasium recovery environment and PPO train/evaluate
entrypoints. No learned policy has been trained or integrated into firmware yet;
see [recovery training](docs/RECOVERY_TRAINING.md). Do not operate or flash
hardware as part of AI experiments.
