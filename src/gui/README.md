# MuJoCo dashboard

The dashboard runs the GIM MuJoCo model with firmware `V3Controller` and the
simulated current-difference leg actuator. It does not run the legacy
`EmbeddedController`.

Build `tap_mujoco_gim_v3`, then start the dashboard from the repository root:

```bash
python3 tools/tap_native_gui.py
```

The 3-D viewer is for rendering and diagnostics. Controller feedback uses the
simulated MPU6500 and six encoders, not MuJoCo world pose or contacts. The
dashboard is an operator UI, not an AI training environment; the runner's
current commands and telemetry are documented in `docs/NATIVE_RUNTIME.md`.
