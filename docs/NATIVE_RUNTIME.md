# Native simulation runtime

The GIM native dashboard runs `tap_mujoco_gim_v3` with `V3Controller` and the
current-difference leg torque actuator model. It loads
`src/tap_sim/model/gim_dynamic.xml`; its separate viewer mirrors debug snapshots
from the same model. Start it with `python3 tools/tap_native_gui.py`, then
`Start SIM`, then `ENABLE`. The extended fixture holds the robot during IMU
seeding, the 1.2 s leg lock and wheel startup. It releases automatically when
V3 reports `Balancing` and the wheel enable/command-delay windows have elapsed
(about 1.36 s after ENABLE in the default profile). DRIVE is blocked until then.

After release, MuJoCo keeps integrating gravity and contacts through Fault,
DISABLE and ESTOP. Fault stops the wheels and retains the last leg hold target;
DISABLE and ESTOP remove actuator torque. Only a fresh spawn or Reset restores
the paused fixture. A fallen robot may rest on the body edge and wheels at an
angle because its legs remain held; the simulator does not force a flat pose.

Every native spawn and Reset uses `spawn_extended`: knees at 135 degrees
internal angle (+113 degrees from CAD zero), hips -56.5 degrees, body origin
0.21649 m above ground, and both wheels on the floor. The initial controller
lock target matches the spawn; the `cad_neutral` keyframe records the original
assembly pose separately.

The joint frames and mass assumptions are in `src/tap_sim/model/PHYSICS.md`.
The planar host harness in `src_fw/tests` remains available. COM/inertias and
actuator response remain provisional. Collisions use CAD convex
hulls for the body/legs and cylinders for the wheels. The physics/evidence
specification is [GIM_SIM_PHYSICS_SPEC.md](GIM_SIM_PHYSICS_SPEC.md).

## GIM V3.3 integration

The runner uses the firmware balancing estimator, a CAD-derived **virtual-pitch**
equilibrium reference, and the model's effective pendulum inertia. It retains
V3's wheel LQR and paired hip/knee control; the four leg actuators use ΔI torque
compensation. Simulator pose/contact forces never enter balance feedback.

Default wheel command dead time is 50 ms and speed-response lag is 75 ms;
`--wheel-delay` and `--wheel-lag` allow checking alternative response assumptions.
These are a provisional profile based on previous motor logs, not identification
of the new assembly. Wheel torque is capped at the provisional 0.13 N·m per wheel;
the corresponding LQR acceleration bound is 2.0833 m/s².

Spawn remains fully extended at body height 0.21649 m. During balance V3 reserves
12 mm of knee travel, so the maximum commanded body height is **0.20449 m**.
HEIGHT reports/clamps the effective target; the dashboard uses that same limit.

The automated nominal-model runs cover 60 s standing, 0.2 m/s drive/stop,
±2 N pushes lasting 0.2 s, height changes 0.17 → 0.13 → 0.20 m, and turning.
The current 0.1 N·m joint-friction profile also passes low-rest, low-push,
low-drive/stop, minimum-height and repeated-height regressions. Separate 300 s
runs at 0.08 m and 0.06988 m remain in Balancing with the reflex gate closed
at quiet rest. The 0.08 m run ends with peak speed 0.01322 m/s and hip travel
0.00275 rad over its final 20 s. The earlier profile oscillated continuously
at low height; those historical results remain in the physics specification.

The GIM profile enables `LqrActuationPredictor` on IMU/encoder state and the
history of sent wheel commands. It predicts through command dead time;
the fixed acceleration lead covers only speed-response lag. `--wheel-delay`
and `--wheel-lag` configure both the simulated response and its predictor.
`--no-lqr-prediction` is the regression countercase: with the same physics,
quiet speed reaches 0.302 m/s and hip travel 0.295 rad. The inertial reflex
gate opens on body angular motion or gravity-removed IMU acceleration;
static lean is handled by wheel balance and the slow body-level loop.
Native ideal encoders use the full COM Jacobian rate; acceleration-based
equilibrium adaptation is disabled in this profile.

The hip rate-lead path uses body gyro (`tier.lead_gyro_only=true`). Feeding
virtual-pitch rate, which includes the hip's own encoder motion, back into
the hip lead can amplify stronger speed changes at low height. The wheel LQR
still uses full virtual-pitch rate; acceleration preview and the slow body-level
loop remain active. `--virtual-hip-lead` restores the previous source for a
simulation countercase. A deterministic host test isolates encoder-rate feedback:
with zero body gyro, the previous source produces 0.02378 rad of hip request;
the gyro source produces zero. A physical countercase does not fall every run.

The GIM maneuver profile uses 0.6 m/s² planned acceleration and 10 m/s³ jerk.
Hip advance equals the 0.12 s wheel-reference preview delay, so hip preview
uses the current planned acceleration without the previous 40 ms history lag.
The core/firmware defaults remain 0.3 m/s², 1 m/s³ and 0.08 s hip advance.
These GIM-specific settings were accepted with the same physical model and
existing fall/current/torque limits. The maneuver regression now requires
signed hip preview and both physical hip responses within 80 ms of each
acceleration/braking command. First forward-step responses were 60 ms at
all three tested heights. Reaching 90% of 0.3 m/s still takes about 2.28 s
at 0.08 m, 1.44 s at 0.13 m and 0.90 s at 0.20 m.
Those speed/60 ms figures describe the earlier 3 m/s³ profile; the current
6 m/s³ profile prioritized early hip response, with unchanged wheel gain costs.
The current joint velocity gain is 0.8 N·m·s/rad (previously 0.5); Kp remains
12 N·m/rad, current limit 8 A, torque limit 3.76 N·m and modeled current lag
20 ms. Higher tested Kp values were excluded because the existing motor
calibration caps Kp at 12. The current maneuver regression additionally
requires a signed one-degree change of both hips within 60 ms (50 Hz telemetry),
gyro >5 Hz RMS below 0.08 rad/s and hip-target >5 Hz RMS below 0.005 rad during
the 35–85 s speed-change window. These spectral checks cover only frequencies
below the 25 Hz telemetry Nyquist limit.
The updated actual-GUI check at the same 0.15 m height measured 39–42 ms
from button event to a signed 0.008 rad encoder change (previously 64–65 ms),
and 80–83 ms to one degree (previously 105–106 ms). Renderer state submission
also measured 39–42 ms. Evidence is under
`build/gim_physics_evidence/gui_joint_fast/`; these are desktop/simulation
measurements with provisional actuator parameters.

Direction-button presses send DRIVE immediately after the existing readiness,
enable and estop gates. The 20 ms GUI tick continues to refresh the command;
release still sends STOP immediately. Backend telemetry is bounded to 256
recent lines and each drain forwards only the latest VIEW. The viewer pipe is
nonblocking with atomic snapshots up to 4096 bytes, and the viewer applies the
latest complete frame from each read instead of replaying queued frames.
A full/stalled viewer pipe cannot block the button handler.

An actual Tk button-event/UDP/renderer check at 0.15 m with 200 ms forward and
reverse presses measured 0.21–0.25 ms to send DRIVE and 63.9–64.7 ms to receive
a signed 0.008 rad hip-encoder change. The viewer applied those changed
snapshots at 64.1–64.9 ms. This measures renderer state submission, not the
display panel's scan-out, and is not a hard realtime guarantee under arbitrary
desktop load. Evidence: `build/gim_physics_evidence/gui_response_new/`.

`gim_maneuver` checks eight signed acceleration/braking events and six speed
windows per height (0.08, 0.13, 0.20 m), with forward/reverse speeds 0.1,
0.3 and 0.5 m/s, plus pushes and quiet station hold. It also checks 200 ms and
800 ms press/release sequences in both directions. The dashboard has separate
unit coverage for button release and for speed-slider updates during driving.
STATE diagnostics expose planned acceleration, hip preview/level/request,
command targets and the selected gyro source. Evidence is retained under
`build/gim_physics_evidence/maneuver_fix/`; repeated 300 s low/minimum-height
checks for this profile are under `maneuver_long/`.

For native simulation diagnostics only, `PUSH force_N duration_s` applies a
bounded world-X force at the body COM. Fault, Disable and Estop tests still
integrate gravity/contact dynamics after stopping the controller.

## Sensor boundary

Balance feedback uses simulated MPU6500 accelerometer/gyro and the six motor
encoders. The leg ΔI loop additionally uses its simulated driver-current
measurement. World pose, quaternion, root velocity, contact state and contact
forces are not controller inputs. VIEW remains renderer/diagnostic telemetry.

Reset initializes leg targets and equilibrium from the configured CAD pose and
mass model, not MuJoCo qpos. Model loading verifies each encoder is a joint
position/velocity sensor on its assigned joint and both IMU channels are actual
accelerometer/gyro sensors on the IMU site; a world-frame sensor with a renamed
IMU/encoder identifier is rejected. While the startup fixture is held stationary,
IMU seeding represents a fixed 1g/zero-gyro sample; after release the runner reads
the IMU sensor channels. Sensors currently have no noise or transport delay.

Joint Coulomb friction is now 0.1 N·m on all four hip/knee joints, per the
latest user setting. Current regressions use this value. Wheel-floor friction
remains 0.8. The predictor and inertial gate remain default-off in the firmware
core; firmware builds include reversible SET controls. These are simulation
results with provisional actuator parameters and ideal sensor channels.
