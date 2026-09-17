# Simulation contract

- `ros2 launch tap_sim sim.launch.py gui:=false auto_enable:=true` starts a
  headless run. Without auto-enable, activation waits paused and disabled;
  its live gate unpauses after a subsequent `/enable=true` Bool. Early enable
  requests are ignored by the gate. Once running, the controller owns disable
  and emergency-stop behavior; the startup helper exits.
  External `ROS_DOMAIN_ID` and `GZ_PARTITION` are inherited, never overridden.
- Model `tap_robot`, world `empty`, root and IMU frame `cad_body`. The visual
  body rotation of -90 degrees is not an IMU mounting rotation.
- No model-pose publisher or dynamic-pose bridge is installed. Gazebo still
  renders its internal scene, but model state is not exposed as a ROS topic,
  controller input, or acceptance signal.
- Global `/joint_states`, `/imu`, `/clock`, `/enable`, `/cmd_vel` retain the
  controller topic contract. Commands to `/leg_effort_controller/commands`
  are Nm ordered left hip, left knee, right hip, right knee; commands to
  `/wheel_effort_controller/commands` are rad/s ordered left, right.
- CAD joint zeros and all +Y axes are unchanged. Simulation initial hips are
  +0.11548309575 rad on both sides, knees/wheels zero. The matching wheel-ground
  height is 0.23715084313 m; spawn defaults to 0.2372 m. Preview stays unchanged.
- Physics 1000 Hz, ros2_control 1000 Hz (the physics step is a hard ceiling:
  polling/actuating faster than one physics step just re-reads unchanged
  state, so this is the practical maximum control rate this simulator can
  usefully exercise), IMU 500 Hz (matches the target hardware IMU range of
  400-500 Hz; sampled independently of the 1 kHz loop, which reuses the last
  IMU sample between updates like real embedded hardware would). The real target embedded control
  loop (ESP32-S3) is ~5 kHz —
  this sim cannot represent that rate meaningfully because of the physics
  ceiling above; 1 kHz here is a simulation-imposed approximation, not
  evidence the algorithm has been validated at 5 kHz.
- All seven inertials are reused unchanged, total 3.938 kg. Primitive collisions
  approximate the CAD. Body uses a rotated bounding box, legs slender boxes,
  wheels 30 mm-radius by 25 mm cylinders; wheel spacing is 309 mm.
- Provisional joint limits: legs 12 Nm / 15 rad/s, wheels 1 Nm / 80 rad/s.
  Damping: legs 1.0, wheels 0.001 Nms/rad. These are not measured motor data.
  Gazebo's direct wheel velocity interface is an idealized actuator; do not
  assume realistic electrical dynamics or guaranteed torque saturation.
- Selected wheel motor: CJC BL4820M12, four leg motors: GIM6010-8.
  `tap_robot_description/description/cad_robot/wheel_motor.yaml` records CJC
  reference specifications only. They do not replace the provisional limits
  above or validate the ideal wheel servo against a real BL4820M12.
- Hip/knee angle envelopes are inherited placeholders, not a validated mapping
  of the supplier's mechanical limits. The native negative knee folds the leg.
- Bringup leaves gravity at -9.81 and never resets pose or controller state.
  At most 50 single-ms steps are allowed for activation and IMU/joint-state
  readiness. There is no pose-readiness dependency.
  On failure it disables, pauses, and requests launch shutdown. No balancing
  claim follows from successful activation alone.

`tap_control` must provide `balance_controller_node` and install
`config/balance_controller.yaml`. It must start disabled, consume `/enable`
as a Bool, reject stale/invalid feedback, and use the native joint convention.

## Excluded simulator-state paths

- `jump_model:=true` remains an actuator/physics experiment selector, but it
  does not create or bridge contact sensors. `BalanceControllerNode` rejects
  `jump.enabled:=true` and `station_keeper.enabled:=true` because the hardware
  contract has no contact switch, force sensor, absolute pose, or localization.
- There is no `measure_contacts` launch argument, contact bridge, contact
  plugin, or wheel-contact ROS topic.
- Landing support/flight is inferred in the control core from MPU6500
  accelerometer/gyro signatures plus the six motor encoders. Gazebo collision
  physics still exists to produce the IMU response, but its contact state is
  not exported to or consumed by the controller.
- `/actuator_enable` is a continuously published `std_msgs/msg/Bool` reflecting
  controller `motors.enable_motors`, not wheel velocity. Initially torque is off.
  A false sample arms the gate; subsequent true samples permit active commands.
  False or a 100 ms monotonic wall-time timeout of an enabled stream removes ALL active torque,
  including wheel braking. Timeout requires false then true to rearm. A pause
  can therefore require explicit disable/enable after resuming. An explicit false
  while disabled arms indefinitely awaiting true, allowing paused startup; it
  does not permit any torque until true. Initial true without false is rejected.
- `/emergency_stop=true` disarms immediately at the next hardware write.
  Clearing it alone never enables torque: a post-clear false enable sample,
  then true is required. Active controller interfaces are also required.
- Four leg effort commands are bounded to +/-5 Nm. Default `jump_leg_speed`
  is 12.566370614359172 rad/s (120 rpm). Experimental cutoff may increase to
  28.274333882308138 rad/s (270 rpm), never the earlier 420 rpm box. Positive-work
  torque is additionally capped at 5 Nm through 190 rpm and linearly tapered to
  zero at 270 rpm. Braking remains bounded to 5 Nm; external overspeed is allowed.
  This is a provisional graph-informed approximation, not a calibrated curve.
  GIM rev2.2 page6 red curve was visually inspected: approximately 285 rpm at
  zero torque, 200 rpm at 5.1 Nm, 120 rpm at 9.7 Nm. Test voltage/temperature
  are not shown. These points do not establish burst duration or thermal safety.
- Jump URDF/SDF velocity limits are 1000 rad/s on ALL six joints: numerical
  solver guards, explicitly NOT motor limits. DART imports finite SDF velocity
  bounds and enforces them through constraint impulses outside motor effort.
  Decoupling avoids that artificial braking at the motor cutoff; reaching the
  1000 rad/s guard would invalidate a trial. Joint position bounds remain.
  Canonical leg motor cutoff is hardware parameter `leg_speed_limit`, populated
  by `jump_leg_speed`. Wheels retain the plugin's 41.887902 rad/s motor cutoff.
- Optional `jump_leg_velocity_damping` (default 0, finite nonnegative) populates
  hardware `leg_velocity_damping`, units Nm/(rad/s). Net active leg torque is
  `effort_cmd + D * (reference_velocity - actual_joint_velocity)`, then capped at +/-5 Nm and the
  positive-work envelope. This is local motor feedback, not mechanical friction;
  braking is bounded, gate-off disables it, and wheels are unaffected. Candidate
  D values 0.05/0.1/0.2 do not imply measured motor properties. Passive leg
  damping remains 0.02 Nms/rad, separate from this net active torque cap.
  Harness metadata must record hardware cutoff and D independently of URDF
  solver velocity. Node/harness owners must propagate launch overrides into
  their archived expanded model to avoid mismatched provenance.
- `/leg_velocity_reference`: reliable depth1 `sensor_msgs/msg/JointState`,
  published continuously at control rate (1 kHz). `name` must contain exactly once each of
  `cad_left_hip`, `cad_left_knee`, `cad_right_hip`, `cad_right_knee`; order is
  arbitrary. `velocity` must contain four finite rad/s values, interpreted as
  relative native joint rates (+Y), not world-frame angular velocities. Targets
  are clamped to +/-hardware `leg_speed_limit`. Position/effort/frame_id are not
  used. Publish zero velocity outside jump, with positive simulation timestamp.
  Before any reference message, zero targets support legacy baseline nodes.
  After the first message (even invalid), malformed or stale references suppress
  ALL active torque, including wheel braking. Source age must be <=100 ms and
  no more than 5 ms in the future; monotonic wall receipt age must be <=100 ms.
  After valid references have been used while enabled, a malformed/stale stream
  disarms the actuator gate: fresh references plus an explicit false/true enable
  sequence are required. Replacing a lost reference before the next write cannot
  bypass this latch. Cold-start invalid timestamps before any active valid
  reference suppress torque without disarming the disabled startup handshake.
  The independent e-stop gate must still permit torque. Duplicate stamps cannot
  renew source freshness. Physics velocity feedback is local to each write,
  while velocity references arrive at control rate. D remains motor feedback,
  with total net torque bounded; it is not added after saturation.
- Wheels retain velocity COMMAND interfaces, but apply only force:
  `clamp(0.02 * (clamp(qd_cmd, +/-41.887902) - qd), +/-0.13)` Nm, with no
  positive work above that speed. Zero command brakes only when enabled.
  The 0.13 Nm cap is provisional; CJC rating type is unresolved.
- Passive damping is explicit: legs 0.02, wheels 0.001 Nms/rad. It is not
  included in the active effort cap. Joint-state effort reports active motor
  torque, excluding passive damping and contact reaction. No velocity/state
  clipping occurs; ResetPosition is used only once during hardware initialization.
- Physics is configured at 1 kHz and control at 1 kHz. Actual hardware-write
  cadence and contact delivery must be measured during isolated integration;
  configuration alone is not evidence of 1 kHz torque enforcement or a jump.

### Gate/contact integration evidence

`log/jump_1789203080139232077` is a domain119 baseline-only trial, not a jump.
Paused startup had a 0.7555 s false-to-true wall gap; measured active efforts
subsequently reached 3 Nm on all four legs and 0.13 Nm on both wheels. E-stop
verified sustained zero applied efforts for 0.2 wall seconds. Contact snapshots
included 202 nonempty/69 empty left reports and 184 nonempty/84 empty right
reports, with source timestamps. Conversion test checks sensor collision
references against actual `gz sdf -p` output: this libsdformat appends a second
`_collision` suffix to the named URDF collisions.

The baseline FAILED pre-jump settling despite 42 coherent samples. Leg speeds
reached exactly the URDF 12.56637 rad/s bound, reinforcing the unresolved DART
solver velocity-limit enforcement concern. These fixes establish working gates
and contact transport, not stable low-damping balance or bounded landing physics.
