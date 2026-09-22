# Native TAP workspace notes

This repository is the native MuJoCo simulation for the RB R1 robot.
The canonical model is `src/tap_mujoco/model/tap_robot.xml`, generated from
the supplied STEP assembly and detailed STL assets. The controller core is
portable C++17 in `src/tap_control/core`; the native runner owns the MuJoCo
physics, sensor shaping and actuator loop. `src/gui` is an optional Tk/3-D
viewer client.

Do not operate hardware, flash firmware or deploy motor commands. Simulation
and host tests are evidence for the native model only, not certification of
physical robot capability. The deployable sensor contract remains one
MPU6500 plus six motor encoders; simulator pose, contact and constraint data
must not enter controller decisions.

Build and test from the repository root:

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON
cmake --build build/native -j2
ctest --test-dir build/native --output-on-failure
```

Run the native simulation or dashboard:

```bash
build/native/bin/tap_mujoco_native \
  --model src/tap_mujoco/model/tap_robot.xml --run-for 5
python3 tools/tap_native_gui.py --rate 3000
```

Robot-frame contract: joint order is `[LH, LK, LW, RH, RK, RW]`, both leg
sides use native `+Y` axes, the root is free, and the CAD model is the source
of visual geometry, joint origins and provisional mass/inertia estimates.
Keep visual STL detail separate from primitive collision envelopes.

Relevant native references are `README.md`, `docs/NATIVE_RUNTIME.md`,
`src/tap_mujoco/model/PHYSICS.md` and the checked-in CAD manifest. Preserve
the protected project contracts in `docs/RULES.md` and `architecture.md`.
