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
  python src/tap_sim/training/train_recovery.py --stage deployable --timesteps 1000000 \
  --envs 2 --seed 21 --resume training_runs/recovery/<discovery-run>/best/best_model.zip
env -u PYTHONPATH PYTHONNOUSERSITE=1 micromamba run -n tap-recovery \
  python src/tap_sim/training/evaluate_recovery.py \
  training_runs/recovery/<run>/best/best_model.zip --episodes 20
env -u PYTHONPATH PYTHONNOUSERSITE=1 micromamba run -n tap-recovery \
  python src/tap_sim/training/train_recovery.py --timesteps 100000 --envs 2 \
  --seed 21 --resume training_runs/recovery/<run>/best/best_model.zip --ent-coef 0.02
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
- **Task reward/evaluation:** uses MuJoCo body orientation, height, velocity
  and contact internally; these privileged values are not exposed to the policy
  and contacts do not trigger the controller. The bridge waits for upright
  gravity, a quiet encoder-measured recovery stance, then seeds tilt from 50
  samples; only low-rate gyro samples contribute bias. Encoder-derived IMU-point
  velocity compensates specific force after seeding. V3 is requested only after
  the recovery pose has held for 0.1 s: upright within 20°, shell clearance
  above 2 cm, both wheels as the only floor contacts, COM within 4.5 cm of the
  axle, and low body/wheel speed. It must also pass sensor readiness checks of
  pitch within 5°, leg speeds below 0.5 rad/s and wheel speeds below 2 rad/s.
  Contacts remain reward/evaluation telemetry; a request cannot occur while
  another robot part touches the floor. The
  bridge preserves the current leg pose while locking, then seeds at least
  0.8 rad knee flexion for support. Success
  means V3 reaches `Balancing` and keeps the robot
  stable for two seconds. The clearance shaping saturates at 10 cm. An extra
  per-step bonus of 30 applies at that height only when the body is within 20°
  of upright and both wheels contact the floor; entering V3 `Balancing` gives
  a one-time bonus of 30. The earlier clearance progress reward is retained;
  a pitch-only reward scales with clearance and wheel support, peaking at 60
  per step at 0° pitch and decreasing linearly to zero at ±15°;
  the `gim_body` origin height earns up to 30 per step at 12 cm only during
  wheel-only support, then loses 6 points per centimeter above 15 cm;
  a combined convergence bonus peaks at 80 per step across 8–12 cm body-origin
  height and pitch within ±5°, requires the shell clear of the floor and both
  wheels supported, then fades to zero by 15 cm or ±15° pitch;
  when both wheels are the only robot parts touching the floor, it receives 100
  points per step. Non-wheel contact costs 25 points early, rising linearly to
  160 as the body reaches the 10 cm lift target, so the loaded rocking pivot
  remains discoverable while a lifted body is strongly pushed toward wheel-only
  support. Lift-target, pitch, convergence, handoff-readiness, and stable-balance
  rewards also require wheel-only contact. Contact rewards use actual
  nonpositive-distance MuJoCo contacts;
  Monitor logs the episode fractions and whether the sensor-based V3 request
  coincided with wheel-only contact;
  before full lift, a larger staged reward points the +X nose upward and rewards
  further nose-up motion. That term fades as the entire body clears the floor,
  so upright balance becomes the goal again. A bounded knee-opening reward is
  gated by positive upright progress, encouraging the knees to contribute
  during the whole-body push. A penalty on measured MuJoCo
  body-to-floor normal load discourages pivoting the nose up while pressing the
  body harder into the floor; this contact force is reward telemetry only.
  A small per-step cost before recovery discourages slow dithering, while a
  capped bonus rewards upright progress above 0.5/s. The reward penalizes COM displacement from the wheel
  axle, measured from the full GIM subtree COM and both CAD wheel centers; this
  penalty fades in during shell lift so the policy can shift COM while loading
  and rocking, then emphasizes alignment within 2 cm during the catch. Angular
  speed is penalized more strongly in that catch phase. Before handoff, a dense
  readiness bonus combines shell clearance, uprightness, both wheel contacts,
  COM alignment, and low body and wheel speed. Stable wheel-supported upright
  steps also receive a positive bonus. This shaping follows TPA's
  load/fling/push/catch sequence: upright and knee-opening progress encourage
  the push, while all standing and catch rewards wait for wheel-only contact.
  The first V3 request receives a one-time +300 bonus after the wheel-only
  recovery-pose hold and sensor-readiness checks pass; it cannot occur while
  another robot part touches the floor.
  Simulator contacts and pose are reward telemetry only, never policy
  observations or controller inputs. TPA's model, joint zero and wheel torque
  differ from GIM, so its numeric targets are not copied. An initial fine-tune
  that directly rewarded per-step COM/load reductions increased shell
  clearance but reduced wheel-only contact and eliminated the handoff on held
  out seeds; that shaping was removed. Action-change cost increases during catch to
  discourage the observed hip/knee/wheel command reversals; current and joint
  speed remain penalized. The evaluator reports
  full shell clearance, standing, V3 activation and completed balance separately.
  `Stood up` requires the sensor/controller recovery pose to persist for 0.1 s
  with wheel-only contact; a request coinciding with wheel-only contact alone
  does not satisfy it.
  Discovery PPO's entropy coefficient defaults to `0.01`; `--ent-coef 0.02`
  increases exploration when continuing discovery.
- **Training stages:** `--stage discovery` (default) keeps action-rate, effort
  and velocity penalties light while the policy discovers a rocking get-up.
  `--stage deployable` requires `--resume` from that policy, retains the task
  rewards and handoff criteria, and increases smoothness, current, joint-speed
  and angular-speed costs gradually, especially as the body clears the floor.
  Its default learning rate is `5e-5` (discovery: `3e-4`) and entropy
  coefficient is `0.002` (discovery: `0.01`). This is a reward
  curriculum for the same task; it does not imitate a saved trajectory.
- **Initial state:** the body rests on its CAD contact surface. Both hips are at
  74.8° and both knees start folded at the 0 rad lower stop; the shell and wheels
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

## Latest held-out comparison

Using seeds 900000–900019, the prior policy
`training_runs/recovery/20261010T081845Z/best/best_model.zip` lifted the body
20/20 and made V3 requests during wheel-only contact 20/20; wheel-only contact
covered 28.4% of steps. The 500k-step COM/load-progress experiment
`20261010T094826Z` lifted 20/20 but requested V3 at wheel-only contact 0/20 and
reduced wheel-only contact to 15.8%, so its shaping was removed. The subsequent
250k-step wheel-only-gated run `20261010T101734Z` raised wheel-only contact to
28.1% but request coincidence was 0/20. The 200k-step request-event candidate
`20261010T103129Z` reached 29.0% wheel-only contact and still lifted 20/20,
but request coincidence remained 0/20. Neither candidate met the stricter 0.1 s
stood-up criterion or 2 s balance success. Keep the 081845Z checkpoint as the
current handoff candidate; do not treat these results as a successful recovery
policy.

The 250k-step milestone-reward fine-tune `20261010T123048Z` resumed from the
081845Z best checkpoint with 3 environments and seed 21. On evaluation seeds
100000–100019, it lifted 20/20 but stood up, requested V3, and completed balance
0/20; wheel-only contact was 15.2%, mean shell clearance 0.4 cm, body load
0.10x robot weight, and mean absolute COM/axle offset 2.0 cm. The original
081845Z checkpoint evaluated under the new V3 gate also stood up and requested
V3 0/20, with the same 15.2% wheel-only contact, while mean shell clearance was
4.3 cm. The fine-tune did not improve the contact milestone; keep the original
checkpoint for comparison and do not promote the new best-reward checkpoint.
The new gate prevented the old sensor-only trigger from requesting V3 without
a held wheel-only recovery pose.

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
