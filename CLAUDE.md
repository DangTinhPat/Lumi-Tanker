# Lab branch notes

This branch is the MuJoCo AI-training workspace. Use the GIM6010-8 model at
`src/tap_sim/model/gim_dynamic.xml` and `V3Controller` from `src_fw/` as the
simulation baseline. The dashboard launches only `tap_mujoco_gim_v3`.

Read `AGENTS.md`, `README.md`, `docs/NATIVE_RUNTIME.md`, and the GIM physics
spec before changing simulation behavior. Keep simulated controller inputs to
the MPU6500 and six encoders; MuJoCo world pose and contacts are not sensors.
Mass and actuator parameters remain estimates. Do not describe simulation as
hardware validation or operate/flash hardware without a current request.

Training focus: the branch does not yet define a Gymnasium API, policy action
space, or reward. Keep AI code separate from the portable V3 controller and
make reset, seed, scenario and observation provenance reproducible.
