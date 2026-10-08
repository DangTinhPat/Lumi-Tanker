# Native TAP dashboard

The active dashboard is a pure Python/Tk client for `tap_mujoco_native`.
Python handles process lifecycle, operator commands and low-rate telemetry; the
native C++ child owns the MuJoCo physics and control loop.

From the repository root:

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON
cmake --build build/native -j2
python3 tools/tap_native_gui.py --rate 3000
```

`--rate 3000` is the previous default and `--rate 4000` is the stress mode.
Press `Start SIM` in the dashboard to open the real MuJoCo 3-D
viewer from `src/tap_sim/model/tap_robot.xml`, including the floor, CAD
meshes and materials. The viewer is a render-only mirror of the runner's
debug `VIEW` stream; it is never fed into the controller. The Tk window is
the command and sensor-diagnostics panel. The default spawn is the belly-down
`power_off_collapse_current` rocking fixture. The dashboard still sends `ENABLE` when
starting, but V3.1 safely rejects it outside the straight-hang `v3_hang` zero
pose. Use the "Đứng dậy (bập bênh)" button (or send `STAND_UP`) to run the
prone-to-standing sequence: `tap_fw::v3::RockerLower` folds the hips down,
snaps to a low pivot, then wheelies (both wheels driving forward while the
hips turn the body up - a sim-only, above-datasheet wheel torque assumption,
see RockerLower's config comments), catches the lean with a V3.1-style wheel
balancer and folds the hips back to level. It ends in a standing, balanced
hold; STAND_UP_STAGE/STAND_UP_ABORTED lines report progress.
Use `--keyframe v3_hang` with the native runner for direct V3.1 balance tests.

The pose controls reuse the checked-in CAD/CoM-aware curves for squat,
left/right roll and the finite Show sequence. Requests still pass through the
native core planner and its joint limits.
