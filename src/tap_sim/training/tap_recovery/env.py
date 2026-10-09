from __future__ import annotations

from collections import deque
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from .v3_balance import V3BalanceBridge


MODEL_PATH = Path(__file__).resolve().parents[2] / "model" / "gim_dynamic.xml"
JOINT_NAMES = (
    "hip_left", "knee_left", "wheel_left",
    "hip_right", "knee_right", "wheel_right",
)
POSITION_SENSORS = (
    "enc_left_hip_pos", "enc_left_knee_pos", "enc_left_wheel_pos",
    "enc_right_hip_pos", "enc_right_knee_pos", "enc_right_wheel_pos",
)
VELOCITY_SENSORS = (
    "enc_left_hip_vel", "enc_left_knee_vel", "enc_left_wheel_vel",
    "enc_right_hip_vel", "enc_right_knee_vel", "enc_right_wheel_vel",
)
START_POSE = "home"
WHEEL_SPEED_ACTION_LIMIT_RAD_S = 1.5
LEG_TARGET_RATE_RAD_S = 0.6
HIP_ACTION_RANGE_RAD = 2.5


class GimRecoveryEnv(gym.Env):
    """Standalone recovery policy plant using the GIM CAD MuJoCo model.

    Policy observations contain simulated MPU6500 signals, six encoders,
    measured leg-driver currents and the previous action. MuJoCo pose/contact
    data are used only to initialize episodes and calculate training rewards.
    Actions request rate-limited leg positions through a V3-style impedance
    loop and one shared wheel speed target through the BL4820 response model.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        model_path: str | Path = MODEL_PATH,
        control_hz: float = 50.0,
        max_episode_s: float = 8.0,
        success_hold_s: float = 1.0,
        balance_hold_s: float = 2.0,
        randomize_physics: bool = False,
    ):
        super().__init__()
        self.model_path = Path(model_path).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self.sensor_period_s = 1.0 / 500.0
        self.sensor_divider = int(round(self.sensor_period_s / self.dt))
        if self.sensor_divider < 1 or abs(self.sensor_divider * self.dt - self.sensor_period_s) > 1e-6:
            raise ValueError("MuJoCo timestep must divide the 500 Hz V3 sensor period")
        self.frame_skip = int(round(1.0 / (control_hz * self.dt)))
        if self.frame_skip < 1 or abs(self.frame_skip * self.dt * control_hz - 1.0) > 1e-6:
            raise ValueError("control_hz must divide the MuJoCo timestep")
        self.control_hz = float(control_hz)
        self.max_episode_steps = int(round(max_episode_s * control_hz))
        self.success_hold_steps = int(round(success_hold_s * control_hz))
        self.balance_hold_steps = int(round(balance_hold_s * control_hz))
        if self.success_hold_steps < 1 or self.balance_hold_steps < 1:
            raise ValueError("success_hold_s and balance_hold_s must be positive")
        self.randomize_physics = bool(randomize_physics)
        if self.model.nq != 13 or self.model.nv != 12 or self.model.nu != 6:
            raise ValueError("GIM model must have nq=13, nv=12 and nu=6")

        self.joint_qvel = [self._id(mujoco.mjtObj.mjOBJ_JOINT, n, self.model.jnt_dofadr)
                           for n in JOINT_NAMES]
        self.position_sensor_ids = [self._id(mujoco.mjtObj.mjOBJ_SENSOR, n)
                                    for n in POSITION_SENSORS]
        self.velocity_sensor_ids = [self._id(mujoco.mjtObj.mjOBJ_SENSOR, n)
                                    for n in VELOCITY_SENSORS]
        self.accel_id = self._id(mujoco.mjtObj.mjOBJ_SENSOR, "mpu6500_accel")
        self.gyro_id = self._id(mujoco.mjtObj.mjOBJ_SENSOR, "mpu6500_gyro")
        self.body_id = self._id(mujoco.mjtObj.mjOBJ_BODY, "gim_body")
        self.start_key_id = self._id(mujoco.mjtObj.mjOBJ_KEY, START_POSE)
        self.ground_id = self._id(mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self.body_shell_geom_ids = [
            self._id(mujoco.mjtObj.mjOBJ_GEOM, name) for name in (
                "cad_50_0", "cad_55_0", "cad_07_0", "cad_08_0",
                "cad_31_0", "cad_32_0",
            )
        ]
        self.wheel_geom_ids = [self._id(mujoco.mjtObj.mjOBJ_GEOM, name) for name in (
            "left_wheel_collision", "right_wheel_collision",
        )]
        self.actuator_ids = [self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in (
            "left_hip_torque", "left_knee_torque", "right_hip_torque",
            "right_knee_torque", "left_wheel_torque", "right_wheel_torque",
        )]
        self.actuator_limits = np.array([3.76] * 4 + [0.15] * 2, dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(29,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)

        self.base_mass = self.model.body_mass.copy()
        self.base_inertia = self.model.body_inertia.copy()
        self.base_friction = self.model.geom_friction.copy()
        self.base_pair_friction = self.model.pair_friction.copy()
        self.current_target_a = np.zeros(4, dtype=np.float64)
        self.current_measured_a = np.zeros(4, dtype=np.float64)
        self.wheel_speed_command = np.zeros(2, dtype=np.float64)
        self.wheel_speed_reference = np.zeros(2, dtype=np.float64)
        self.wheel_lagged_torque = np.zeros(2, dtype=np.float64)
        self.leg_position_reference = np.zeros(4, dtype=np.float64)
        self.recovery_home_reference = np.zeros(4, dtype=np.float64)
        self.wheel_delay = deque()
        self.previous_action = np.zeros(3, dtype=np.float32)
        self.episode_step = 0
        self.stable_steps = 0
        self.recovered = False
        self.recovery_time_s = -1.0
        self.current_sum_a = 0.0
        self.episode_start_time = 0.0
        self.balance_bridge: V3BalanceBridge | None = None
        self.balance_requested = False
        self.balance_activated = False
        self.balance_activation_time_s = -1.0
        self.balance_stable_steps = 0
        self.v3_state = V3BalanceBridge.IDLE
        self.friction_scale = 1.0
        self.actuator_scale = 1.0
        self.wheel_delay_s = 0.050
        self.wheel_lag_s = 0.075
        self.previous_up_dot = 1.0
        self.previous_body_clearance = 0.0
        self.lifted = False

    def _id(self, kind, name: str, address=None) -> int:
        value = mujoco.mj_name2id(self.model, kind, name)
        if value < 0:
            raise ValueError(f"GIM model is missing {name}")
        return int(address[value] if address is not None else value)

    def _randomize_model(self):
        self.model.body_mass[:] = self.base_mass
        self.model.body_inertia[:] = self.base_inertia
        self.model.geom_friction[:] = self.base_friction
        self.model.pair_friction[:] = self.base_pair_friction
        self.actuator_limits[:] = [3.76] * 4 + [0.15] * 2
        if self.randomize_physics:
            scales = self.np_random.uniform(0.95, 1.05, size=self.model.nbody)
            scales[0] = 1.0
            self.model.body_mass[:] *= scales
            self.model.body_inertia[:] *= scales[:, None]
            self.friction_scale = float(self.np_random.uniform(0.8, 1.2))
            self.model.geom_friction[:] = self.base_friction
            self.model.geom_friction[:, :2] *= self.friction_scale
            self.model.pair_friction[:] = self.base_pair_friction
            self.model.pair_friction[:, :2] *= self.friction_scale
            self.actuator_scale = float(self.np_random.uniform(0.9, 1.0))
            self.actuator_limits *= self.actuator_scale
            self.wheel_delay_s = float(self.np_random.uniform(0.040, 0.060))
            self.wheel_lag_s = float(self.np_random.uniform(0.060, 0.090))
        else:
            self.friction_scale = self.actuator_scale = 1.0
            self.wheel_delay_s, self.wheel_lag_s = 0.050, 0.075
        self.model.actuator_ctrlrange[:] = np.column_stack((-self.actuator_limits,
                                                            self.actuator_limits))

    def _lift_out_of_ground(self):
        for _ in range(8):
            mujoco.mj_forward(self.model, self.data)
            penetration = 0.0
            for i in range(self.data.ncon):
                contact = self.data.contact[i]
                if self.ground_id in (contact.geom1, contact.geom2):
                    penetration = max(penetration, -float(contact.dist))
            if penetration <= 1e-5:
                return
            self.data.qpos[2] += penetration + 0.002
        mujoco.mj_forward(self.model, self.data)
        if any(self.ground_id in (self.data.contact[i].geom1, self.data.contact[i].geom2)
               and self.data.contact[i].dist < -1e-3 for i in range(self.data.ncon)):
            raise RuntimeError("could not create a non-penetrating recovery reset")

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if self.balance_bridge is not None:
            self.balance_bridge.close()
        self._randomize_model()
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_setConst(self.model, self.data)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.start_key_id)
        self.current_start_pose = START_POSE
        self._lift_out_of_ground()
        self.data.ctrl[:] = 0.0
        self.episode_start_time = float(self.data.time)

        self.current_target_a.fill(0.0)
        self.current_measured_a.fill(0.0)
        self.wheel_speed_command.fill(0.0)
        self.wheel_speed_reference.fill(0.0)
        self.wheel_lagged_torque.fill(0.0)
        delay_steps = max(0, round(self.wheel_delay_s / self.dt))
        self.wheel_delay = [deque([0.0] * delay_steps),
                            deque([0.0] * delay_steps)]
        self.previous_action.fill(0.0)
        self.episode_step = self.stable_steps = 0
        self.recovered = False
        self.lifted = False
        self.recovery_time_s = -1.0
        self.balance_requested = False
        self.balance_activated = False
        self.balance_activation_time_s = -1.0
        self.balance_stable_steps = 0
        self.v3_state = V3BalanceBridge.IDLE
        self.balance_bridge = V3BalanceBridge(self.sensor_period_s)
        self.current_sum_a = 0.0
        self.data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.leg_position_reference[:] = [
            self._sensor(self.position_sensor_ids[i])[0] for i in (0, 1, 3, 4)
        ]
        self.recovery_home_reference[:] = self.leg_position_reference
        rotation = self.data.xmat[self.body_id].reshape(3, 3)
        self.previous_up_dot = float(rotation[2, 2])
        self.previous_body_clearance = self._body_clearance()
        return self._observation(), {"start_pose": self.current_start_pose}

    def _sensor(self, sensor_id: int) -> np.ndarray:
        start = self.model.sensor_adr[sensor_id]
        end = start + self.model.sensor_dim[sensor_id]
        return self.data.sensordata[start:end]

    def _observation(self) -> np.ndarray:
        accel = np.clip(self._sensor(self.accel_id) / 9.81, -4.0, 4.0)
        gyro = np.clip(self._sensor(self.gyro_id) / 10.0, -4.0, 4.0)
        position = np.array([self._sensor(i)[0] for i in self.position_sensor_ids])
        velocity = np.array([self._sensor(i)[0] for i in self.velocity_sensor_ids])
        position = np.clip(position / np.array([np.pi, 2.0, 10.0] * 2), -4.0, 4.0)
        velocity = np.clip(velocity / np.array([10.0, 10.0, 30.0] * 2), -4.0, 4.0)
        current = np.clip(self.current_measured_a / 8.0, -1.0, 1.0)
        reference = self.leg_position_reference.copy()
        reference[[0, 2]] /= np.pi
        reference[[1, 3]] = 2.0 * reference[[1, 3]] / 1.9722222 - 1.0
        return np.concatenate((accel, gyro, position, velocity, current,
                               self.previous_action, reference)).astype(np.float32)

    def _physics_step(self, action: np.ndarray):
        desired_reference = np.array([
            self.recovery_home_reference[0] + action[0] * HIP_ACTION_RANGE_RAD,
            self.recovery_home_reference[1] + max(0.0, action[1]) * 1.9722222,
            self.recovery_home_reference[2] + action[0] * HIP_ACTION_RANGE_RAD,
            self.recovery_home_reference[3] + max(0.0, action[1]) * 1.9722222,
        ])
        if not self.balance_requested:
            max_reference_step = LEG_TARGET_RATE_RAD_S / self.control_hz
            self.leg_position_reference += np.clip(
                desired_reference - self.leg_position_reference,
                -max_reference_step, max_reference_step)
        self.wheel_speed_command[:] = action[2] * WHEEL_SPEED_ACTION_LIMIT_RAD_S
        control_divider = self.sensor_divider
        for tick in range(self.frame_skip):
            if tick % control_divider == 0:
                use_balance_output = False
                if self.balance_requested:
                    accel = self._sensor(self.accel_id)
                    gyro = self._sensor(self.gyro_id)
                    q = np.array([self._sensor(i)[0] for i in self.position_sensor_ids],
                                 dtype=np.float32)
                    qd = np.array([self._sensor(i)[0] for i in self.velocity_sensor_ids],
                                  dtype=np.float32)
                    currents, wheel_speed, state, _, _, imu_valid = self.balance_bridge.step(
                        accel, gyro, q, qd, self.current_measured_a,
                        float(self.data.time), True)
                    self.v3_state = state
                    if imu_valid:
                        self.current_target_a[:] = currents
                        self.wheel_speed_command[:] = wheel_speed
                        use_balance_output = True
                if not use_balance_output:
                    q = np.array([self._sensor(i)[0] for i in self.position_sensor_ids],
                                 dtype=np.float32)
                    qd = np.array([self._sensor(i)[0] for i in self.velocity_sensor_ids],
                                  dtype=np.float32)
                    leg_q = q[[0, 1, 3, 4]]
                    leg_qd = qd[[0, 1, 3, 4]]
                    gravity = np.asarray(self.balance_bridge.gravity_torque(leg_q))
                    requested_nm = gravity + 12.0 * (
                        self.leg_position_reference - leg_q) - 0.8 * leg_qd
                    target_current = np.clip(requested_nm / 0.47, -8.0, 8.0)
                    delta_current = target_current - self.current_measured_a
                    compensated_nm = np.clip(requested_nm + 0.2 * 0.47 * delta_current,
                                             -3.76, 3.76)
                    self.current_target_a[:] = compensated_nm / 0.47

            alpha = self.dt / (0.020 + self.dt)
            self.current_measured_a += alpha * (self.current_target_a - self.current_measured_a)
            for i, actuator in enumerate(self.actuator_ids[:4]):
                self.data.ctrl[actuator] = 0.47 * self.current_measured_a[i]

            delayed = []
            for side in range(2):
                if self.wheel_delay[side]:
                    delayed.append(self.wheel_delay[side].popleft())
                    self.wheel_delay[side].append(float(self.wheel_speed_command[side]))
                else:
                    delayed.append(float(self.wheel_speed_command[side]))
            lag_alpha = self.dt / (self.wheel_lag_s + self.dt)
            for side in range(2):
                wheel_index = 3 * side + 2
                measured_speed = float(self.data.qvel[self.joint_qvel[wheel_index]])
                self.wheel_speed_reference[side] += lag_alpha * (
                    delayed[side] - self.wheel_speed_reference[side])
                desired_torque = np.clip(3.0 * (self.wheel_speed_reference[side] - measured_speed),
                                         -0.15, 0.15)
                torque_alpha = self.dt / (0.001 + self.dt)
                self.wheel_lagged_torque[side] += torque_alpha * (
                    desired_torque - self.wheel_lagged_torque[side])
                friction = 0.001 * measured_speed
                friction += (np.copysign(0.006, measured_speed) if abs(measured_speed) > 0.02
                             else np.clip(self.wheel_lagged_torque[side], -0.006, 0.006))
                self.data.ctrl[self.actuator_ids[4 + side]] = np.clip(
                    self.wheel_lagged_torque[side] - friction, -0.15, 0.15)
            mujoco.mj_step(self.model, self.data)

    def _body_clearance(self) -> float:
        return min(mujoco.mj_geomDistance(self.model, self.data, geom_id,
                                          self.ground_id, 0.25, None)
                   for geom_id in self.body_shell_geom_ids)

    def _wheel_support(self) -> tuple[bool, bool]:
        supported = [False, False]
        for contact in self.data.contact[:self.data.ncon]:
            if contact.geom1 == self.ground_id:
                wheel = contact.geom2
            elif contact.geom2 == self.ground_id:
                wheel = contact.geom1
            else:
                continue
            for side, geom_id in enumerate(self.wheel_geom_ids):
                supported[side] |= wheel == geom_id
        return supported[0], supported[1]

    def _task_state(self):
        rotation = self.data.xmat[self.body_id].reshape(3, 3)
        up_dot = float(np.clip(rotation[2, 2], -1.0, 1.0))
        head_up = float(np.clip(rotation[0, 2], -1.0, 1.0))
        height = float(self.data.xpos[self.body_id, 2])
        body_clearance = self._body_clearance()
        wheel_support = self._wheel_support()
        angular_speed = float(np.linalg.norm(self.data.qvel[3:6]))
        wheel_speed = max(abs(float(self.data.qvel[a])) for a in self.joint_qvel[2::3])
        stable = (up_dot > np.cos(np.deg2rad(20.0)) and body_clearance > 0.08 and
                  all(wheel_support) and angular_speed < 0.25 and wheel_speed < 2.0)
        lifted = (up_dot > np.cos(np.deg2rad(50.0)) and body_clearance > 0.02 and
                  all(wheel_support))
        return (up_dot, head_up, height, body_clearance, wheel_support,
                angular_speed, wheel_speed, lifted, stable)

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (3,) or not np.all(np.isfinite(action)):
            raise ValueError("action must contain three finite values")
        action = np.clip(action, -1.0, 1.0)
        self._physics_step(action)
        self.episode_step += 1
        self.current_sum_a += float(np.mean(np.abs(self.current_measured_a)))
        (up_dot, head_up, height, body_clearance, wheel_support,
         angular_speed, wheel_speed, lifted, stable) = self._task_state()
        self.stable_steps = self.stable_steps + 1 if stable else 0
        just_lifted = not self.lifted and lifted
        self.lifted = self.lifted or lifted
        normalized_current = self.current_measured_a / 8.0
        leg_speed = np.array([self._sensor(i)[0] for i in self.velocity_sensor_ids])[[0, 1, 3, 4]]
        upright_progress = up_dot - self.previous_up_dot
        clearance_progress = body_clearance - self.previous_body_clearance
        upright_gate = float(np.clip(up_dot, 0.0, 1.0))
        lift_fraction = float(np.clip((body_clearance - 0.005) / 0.10, 0.0, 1.0))
        wheel_support_reward = 0.5 * sum(wheel_support)
        reward = (5.0 * up_dot +
                  12.0 * lift_fraction * upright_gate +
                  20.0 * clearance_progress * upright_gate +
                  2.0 * max(head_up, 0.0) * (1.0 - lift_fraction) +
                  wheel_support_reward +
                  2.0 * upright_progress -
                  0.1 * float(np.mean((action - self.previous_action) ** 2)) -
                  0.02 * float(np.mean(action ** 2)) -
                  0.05 * float(np.mean(normalized_current ** 2)) -
                  0.02 * float(np.mean((leg_speed / 2.0) ** 2)) -
                  0.05 * angular_speed ** 2 -
                  0.1 * float(stable))
        if just_lifted:
            reward += 10.0
        self.previous_up_dot = up_dot
        self.previous_body_clearance = body_clearance
        just_recovered = not self.recovered and self.stable_steps >= self.success_hold_steps
        if just_recovered:
            self.recovered = True
            self.balance_requested = True
            self.recovery_time_s = float(self.data.time - self.episode_start_time)
            reward += 10.0
        if self.balance_requested and self.v3_state == V3BalanceBridge.BALANCING:
            if not self.balance_activated:
                self.balance_activated = True
                self.balance_activation_time_s = float(self.data.time - self.episode_start_time)
                reward += 10.0
            self.balance_stable_steps += 1 if stable else 0
        self.previous_action[:] = action if not self.balance_requested else 0.0
        catastrophic = not np.all(np.isfinite(self.data.qpos)) or height < -0.05
        lost_balance = self.balance_activated and not stable
        controller_fault = self.balance_requested and self.v3_state == V3BalanceBridge.FAULT
        success = bool(self.balance_activated and
                       self.balance_stable_steps >= self.balance_hold_steps and stable)
        terminated = bool(catastrophic or lost_balance or controller_fault or success)
        truncated = self.episode_step >= self.max_episode_steps
        info = {
            "is_success": success, "is_recovered": self.recovered,
            "body_lifted": self.lifted,
            "stood_up": self.recovered, "balance_activated": self.balance_activated,
            "balance_stable_s": self.balance_stable_steps / self.control_hz,
            "balance_activation_time_s": self.balance_activation_time_s,
            "v3_state": self.v3_state,
            "recovery_time_s": self.recovery_time_s,
            "start_pose": self.current_start_pose,
            "upright_cos": up_dot, "head_up_sin": head_up,
            "body_height_m": height, "body_clearance_m": body_clearance,
            "wheel_support": wheel_support,
            "angular_speed_rad_s": angular_speed,
            "wheel_speed_rad_s": wheel_speed,
            "mean_abs_leg_current_a": self.current_sum_a / self.episode_step,
            "sim_time_s": float(self.data.time - self.episode_start_time),
        }
        return self._observation(), float(reward), terminated, truncated, info

    def close(self):
        if self.balance_bridge is not None:
            self.balance_bridge.close()
            self.balance_bridge = None
