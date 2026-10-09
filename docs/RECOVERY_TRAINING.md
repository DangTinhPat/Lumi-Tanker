# GIM recovery training

The `lab` branch has an isolated Gymnasium environment and PPO entrypoint for
training a policy to stand from the single CAD-checked `home` pose. It uses
`src/tap_sim/model/gim_dynamic.xml`; it does not run in the dashboard or use
the realtime native runner.

## Python environment

The training environment is separate from `wheel-leg-ros`. Create it once on a
fresh machine, then install its packages:

```bash
micromamba create -n tap-recovery python=3.12 pip
env -u PYTHONPATH PYTHONNOUSERSITE=1 micromamba run -n tap-recovery \
  python -m pip install torch==2.13.0 --index-url https://download.pytorch.org/whl/cu132
env -u PYTHONPATH PYTHONNOUSERSITE=1 micromamba run -n tap-recovery \
  python -m pip install -r src/tap_sim/training/requirements-recovery.txt
```

Unset `PYTHONPATH` when running this environment: the development shell may set
it to the `wheel-leg-ros` site-packages. The PyTorch wheel includes its CUDA
runtime; verify that `torch.cuda.is_available()` is true before a long run.

## Train and evaluate

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 micromamba run -n tap-recovery \
  python src/tap_sim/training/train_recovery.py --timesteps 2000000 --envs 2 --seed 7
env -u PYTHONPATH PYTHONNOUSERSITE=1 micromamba run -n tap-recovery \
  python src/tap_sim/training/evaluate_recovery.py \
  training_runs/recovery/<run>/best/best_model.zip --episodes 20
env -u PYTHONPATH PYTHONNOUSERSITE=1 micromamba run -n tap-recovery \
  tensorboard --logdir training_runs/recovery/<run>/tensorboard
```

Runs, checkpoints, evaluation results and TensorBoard logs are written under
`training_runs/recovery/` and ignored by Git. `run.json` records the model hash,
Git commit, seed, configuration and runtime versions.

## Environment contract

- **Observations (29 values):** MPU6500 accelerometer/gyro, six encoder angles
  and rates, four measured leg-driver currents, previous three actions, and four
  rate-limited leg-position references. They contain no MuJoCo world pose or
  contact state.
- **Actions (3 values):** shared hip and knee offsets from the measured home
  pose for both legs, plus one shared wheel-speed target capped at 1.5 rad/s.
  Hip offset spans ±2.5 rad and knee offset spans 0–1.972 rad. A zero action
  holds the home leg references. Leg references move no faster than 0.6 rad/s
  and use the V3 CAD gravity feed-forward plus its
  12 N·m/rad and 0.8 N·m·s/rad impedance gains, current-difference compensation,
  current limit, and 20 ms current response. This prevents random exploration
  from commanding full motor torque directly. Wheel targets use the simulated
  delay, speed loop, torque cap and lag from the GIM runner.
- **Task reward/evaluation:** uses MuJoCo body orientation, height and velocity
  internally; these privileged values are not exposed to the policy. After one
  second upright and nearly still, control hands over to the firmware
  `V3Controller`. Success means V3 reaches `Balancing` and keeps the robot
  stable for two seconds. Reward rises when the CAD shell/mounts clear the floor,
  with the clearance reward gated by an upright body and both wheels in contact;
  it gives a smaller interim reward when the +X nose rises before full lift.
  Action changes, current and joint speed are penalized. The evaluator reports
  full shell clearance, standing, V3 activation and completed balance separately.
- **Initial state:** the body rests on its CAD contact surface. Both hips are at
  74.8° and both knees are straight at their 0 rad limit; the shell and wheels
  touch the floor. This is a starting pose, not a static equilibrium.
  Reset checks penetration and returns the first observation immediately
  without an unpowered settling interval. `spawn_extended` is not used as an
  easier curriculum start: it falls before the V3 handoff can engage.
- **Physics randomization:** off by default. The optional preset varies mass,
  contact friction and actuator response within provisional bounds; those
  bounds are training heuristics, not measured uncertainty distributions.
- **Sensor caveat:** MuJoCo accelerometer, gyro and encoders are currently
  noiseless and have no sampling/transport delay. Do not treat a learned result
  as robust to sensor noise until that model is added from documented bounds.

The learned policy currently supplies the get-up action only; a C ABI bridge
calls the same V3 controller core used by firmware for the balance phase. This
is training integration, not a deployable firmware recovery mode. The handoff
uses simulated IMU/encoder signals and must be evaluated on held-out falls and
physics settings. Current CAD mass/inertia and actuator parameters remain
provisional, so results apply to this simulation model only.

## Method references

- [Learning to Recover: Dynamic Reward Shaping with Wheel-Leg Coordination for
  Fallen Robots](https://arxiv.org/abs/2506.05516) studies wheel-assisted
  recovery and keeps fixed-duration episodes running through the stabilization
  phase. Its robot models and Isaac Lab setup are not reused here.
- [GetUp Gym: Force-Guided Fall Recovery for Bipedal-Wheeled and Humanoid
  Robots](https://github.com/2350575870/getup_gym) is the closest open-source
  morphology reference. Its staged recovery idea informs the task milestone;
  its Isaac Gym implementation, auxiliary-force teacher and robot assets are
  not copied into TAP.
