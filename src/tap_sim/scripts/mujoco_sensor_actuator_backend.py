#!/usr/bin/env python3
"""Sensor/actuator boundary for the TAP MuJoCo backend.

The only values leaving this module for the controller are deliberately
hardware-shaped IMU, encoder and (optionally) GDS68 feedback samples.  MuJoCo
qpos, body pose, contacts and constraint forces are not part of the exported
interface.  They remain available to an offline recorder only when explicitly
requested by a test, never to the controller path.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Deque, Iterable, Optional, Sequence, Tuple

import mujoco
import numpy as np


JOINT_NAMES = (
    "cad_left_hip", "cad_left_knee", "cad_left_wheel",
    "cad_right_hip", "cad_right_knee", "cad_right_wheel",
)
LEG_INDICES = (0, 1, 3, 4)
WHEEL_INDICES = (2, 5)


@dataclass
class SensorConfig:
    # Rates are intentionally independent of the physics and controller rates.
    imu_rate_hz: float = 500.0
    encoder_rate_hz: float = 500.0
    sensor_latency_s: float = 0.002
    dropout_probability: float = 0.002
    seed: int = 6814

    # MPU6500-like imperfections.  These are conservative provisional values,
    # not calibration claims; every one is exposed in the launch parameters.
    accel_noise_stddev_m_s2: float = 0.045
    gyro_noise_stddev_rad_s: float = 0.0025
    accel_bias_random_walk_m_s2_sqrt_s: float = 0.012
    gyro_bias_random_walk_rad_s_sqrt_s: float = 0.0006
    accel_bias_stddev_m_s2: float = 0.08
    gyro_bias_stddev_rad_s: float = 0.008
    accel_quantization_m_s2: float = 0.0012
    gyro_quantization_rad_s: float = 0.00027

    # Encoder imperfections.  The exact sensor resolution remains a hardware
    # calibration item, so these defaults are explicitly tunable.
    encoder_position_noise_stddev_rad: float = 0.00035
    encoder_velocity_noise_stddev_rad_s: float = 0.012
    encoder_position_quantization_rad: float = 2.0 * math.pi / 16384.0
    encoder_wheel_position_quantization_rad: float = 2.0 * math.pi / 4096.0
    encoder_velocity_quantization_rad_s: float = 0.02
    encoder_dropout_probability: float = 0.001

    # GDS68 output-shaft feedback mock.  It is disabled by default because
    # the real hardware contract only permits this channel after provenance
    # is explicitly established.
    gds68_feedback: bool = False
    gds68_torque_time_constant_s: float = 0.003
    gds68_feedback_period_s: float = 0.002
    gds68_torque_noise_stddev_nm: float = 0.03
    gds68_torque_bias_stddev_nm: float = 0.08
    gds68_torque_bias_drift_nm_per_sqrt_s: float = 0.002
    gds68_output_torque_limit_nm: float = 11.0
    gds68_coulomb_friction_nm: float = 0.08
    gds68_viscous_friction_nm_s_rad: float = 0.006
    gds68_stiction_velocity_rad_s: float = 0.02


@dataclass
class ActuatorConfig:
    # Boot hold is a bounded motor-startup phase, not a kinematic constraint.
    # It keeps the explicitly supported natural ENABLE reference from rolling away while
    # the 6-axis IMU collects its startup seed.  After this window, a disabled
    # backend is genuinely torque-off.
    startup_hold_s: float = 1.0
    startup_leg_kp_nm_per_rad: float = 40.0
    startup_leg_kd_nm_s_per_rad: float = 2.5
    command_latency_s: float = 0.002
    leg_torque_limit_nm: float = 12.0
    wheel_torque_limit_nm: float = 0.13
    leg_time_constant_s: float = 0.003
    wheel_time_constant_s: float = 0.005
    wheel_velocity_kp: float = 2.0
    # CJC BL4012 rated speed: 400 rpm, direct drive.
    wheel_velocity_limit_rad_s: float = 41.88790204786391
    wheel_coulomb_friction_nm: float = 0.006
    wheel_viscous_friction_nm_s_rad: float = 0.001
    command_timeout_s: float = 0.10


@dataclass(frozen=True)
class ImuSample:
    stamp_s: float
    accel_m_s2: Tuple[float, float, float]
    gyro_rad_s: Tuple[float, float, float]


@dataclass(frozen=True)
class EncoderSample:
    stamp_s: float
    position_rad: Tuple[float, ...]
    velocity_rad_s: Tuple[float, ...]
    effort_nm: Optional[Tuple[float, ...]]


@dataclass
class _GdsChannel:
    motor_torque_nm: float = 0.0
    feedback_torque_nm: float = 0.0
    elapsed_s: float = 0.0
    bias_nm: float = 0.0


class TapMujocoBackend:
    """One deterministic fixed-step physics backend.

    ``step`` returns only samples that are ready at the simulated time.  An
    IPC adapter may serialize those samples, but it cannot access simulator
    truth through this API.
    """

    def __init__(self, model_path: Path, sensor: SensorConfig | None = None,
                 actuator: ActuatorConfig | None = None,
                 initial_root_z_m: float = 0.1484350375,
                 initial_hip_rad: float = 1.0104670525,
                 initial_knee_rad: float = -1.0682730675):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.sensor = sensor or SensorConfig()
        self.actuator = actuator or ActuatorConfig()
        # The initial pose prevents startup collision from becoming an
        # artificial landing test; it does not constrain the free body.
        self.initial_root_z_m = float(initial_root_z_m)
        self.initial_hip_rad = float(initial_hip_rad)
        self.initial_knee_rad = float(initial_knee_rad)
        self.rng = np.random.default_rng(self.sensor.seed)
        self.sim_time_s = 0.0
        self.enabled = False
        self.emergency_stop = False
        self.last_command_time_s = -math.inf
        self.leg_command = np.zeros(4, dtype=float)
        self.wheel_command = np.zeros(2, dtype=float)
        self._command_queue: Deque[Tuple[float, np.ndarray, np.ndarray]] = deque()
        self._leg_state = np.zeros(4, dtype=float)
        self._wheel_state = np.zeros(2, dtype=float)
        self._leg_feedback = np.zeros(4, dtype=float)
        self._gds = [_GdsChannel() for _ in range(4)]
        for channel in self._gds:
            channel.bias_nm = self._normal(self.sensor.gds68_torque_bias_stddev_nm)
        self._imu_accel_bias = np.array([
            self._normal(self.sensor.accel_bias_stddev_m_s2) for _ in range(3)])
        self._imu_gyro_bias = np.array([
            self._normal(self.sensor.gyro_bias_stddev_rad_s) for _ in range(3)])
        self._next_imu_s = 0.0
        self._next_encoder_s = 0.0
        self._imu_queue: Deque[Tuple[float, ImuSample]] = deque()
        self._encoder_queue: Deque[Tuple[float, EncoderSample]] = deque()
        self._accel_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "mpu6500_accel")
        self._gyro_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "mpu6500_gyro")
        if self._accel_id < 0 or self._gyro_id < 0:
            raise ValueError("MJCF is missing MPU6500 sensors")
        self._joint_pos_ids = [mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, f"enc_{name[4:]}_pos")
            for name in JOINT_NAMES]
        self._joint_vel_ids = [mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, f"enc_{name[4:]}_vel")
            for name in JOINT_NAMES]
        if any(i < 0 for i in self._joint_pos_ids + self._joint_vel_ids):
            raise ValueError("MJCF is missing encoder sensors")

        self._initialize_state()

    def _initialize_state(self) -> None:
        self.data.qpos[:] = 0.0
        self.data.qpos[2] = self.initial_root_z_m
        self.data.qpos[3] = 1.0
        self.data.qpos[7] = self.initial_hip_rad
        self.data.qpos[8] = self.initial_knee_rad
        self.data.qpos[10] = self.initial_hip_rad
        self.data.qpos[11] = self.initial_knee_rad
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _normal(self, sigma: float) -> float:
        return float(self.rng.normal(0.0, max(float(sigma), 0.0)))

    @staticmethod
    def _quantize(value: float, quantum: float) -> float:
        if quantum <= 0.0:
            return value
        return float(round(value / quantum) * quantum)

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self._initialize_state()
        self.sim_time_s = 0.0
        self.enabled = False
        self.emergency_stop = False
        self.last_command_time_s = -math.inf
        self.leg_command.fill(0.0)
        self.wheel_command.fill(0.0)
        self._command_queue.clear()
        self._leg_state.fill(0.0)
        self._wheel_state.fill(0.0)
        self._leg_feedback.fill(0.0)
        for channel in self._gds:
            channel.motor_torque_nm = 0.0
            channel.feedback_torque_nm = 0.0
            channel.elapsed_s = 0.0
            channel.bias_nm = self._normal(self.sensor.gds68_torque_bias_stddev_nm)
        self._imu_queue.clear()
        self._encoder_queue.clear()
        self._next_imu_s = 0.0
        self._next_encoder_s = 0.0

    def set_enable(self, enabled: bool) -> None:
        self.enabled = bool(enabled) and not self.emergency_stop

    def set_emergency_stop(self, stopped: bool) -> None:
        self.emergency_stop = bool(stopped)
        if self.emergency_stop:
            self.enabled = False

    def receive_commands(self, leg_torque_nm: Sequence[float],
                         wheel_velocity_rad_s: Sequence[float]) -> None:
        if len(leg_torque_nm) != 4 or len(wheel_velocity_rad_s) != 2:
            return
        leg = np.asarray(leg_torque_nm, dtype=float)
        wheel = np.asarray(wheel_velocity_rad_s, dtype=float)
        if not (np.all(np.isfinite(leg)) and np.all(np.isfinite(wheel))):
            return
        leg = np.clip(leg, -self.actuator.leg_torque_limit_nm,
                      self.actuator.leg_torque_limit_nm)
        wheel = np.clip(wheel, -self.actuator.wheel_velocity_limit_rad_s,
                        self.actuator.wheel_velocity_limit_rad_s)
        self._command_queue.append((self.sim_time_s + self.actuator.command_latency_s,
                                     leg, wheel))
        self.last_command_time_s = self.sim_time_s

    def _apply_due_command(self) -> None:
        while self._command_queue and self._command_queue[0][0] <= self.sim_time_s:
            _, leg, wheel = self._command_queue.popleft()
            self.leg_command = leg.copy()
            self.wheel_command = wheel.copy()

    def _first_order(self, current: float, target: float, tau: float,
                     dt: float) -> float:
        alpha = dt / (max(tau, 1.0e-6) + dt)
        return current + alpha * (target - current)

    def _friction(self, velocity: float, coulomb: float, viscous: float,
                  stiction: float = 0.02, drive: float = 0.0) -> float:
        result = viscous * velocity
        if abs(velocity) > stiction:
            result += math.copysign(coulomb, velocity)
        else:
            result += float(np.clip(drive, -coulomb, coulomb))
        return result

    def _update_actuators(self, dt: float) -> None:
        boot_hold = (not self.enabled and not self.emergency_stop and
                     self.sim_time_s < self.actuator.startup_hold_s)
        safe = (self.enabled and not self.emergency_stop and
                self.sim_time_s - self.last_command_time_s <= self.actuator.command_timeout_s)
        if safe:
            target_leg = self.leg_command
        elif boot_hold:
            target_q = np.array([self.initial_hip_rad, self.initial_knee_rad,
                                 self.initial_hip_rad, self.initial_knee_rad])
            q = np.array([self.data.qpos[7], self.data.qpos[8],
                          self.data.qpos[10], self.data.qpos[11]])
            qd = np.array([self.data.qvel[6], self.data.qvel[7],
                           self.data.qvel[9], self.data.qvel[10]])
            target_leg = np.clip(
                self.actuator.startup_leg_kp_nm_per_rad * (target_q - q) -
                self.actuator.startup_leg_kd_nm_s_per_rad * qd,
                -self.actuator.leg_torque_limit_nm,
                self.actuator.leg_torque_limit_nm)
        else:
            target_leg = np.zeros(4)
        target_wheel = self.wheel_command if safe else np.zeros(2)
        self.data.ctrl[:] = 0.0

        # Leg output torque path: command delay -> motor/gearbox lag -> output
        # friction.  The feedback channel reports the lagged output estimate,
        # not contact force and not MuJoCo constraint force.
        for out, joint_index, channel in zip(range(4), LEG_INDICES, self._gds):
            velocity = float(self.data.qvel[6 + joint_index])
            channel.motor_torque_nm = self._first_order(
                channel.motor_torque_nm, float(target_leg[out]),
                self.actuator.leg_time_constant_s, dt)
            friction = self._friction(
                velocity, self.sensor.gds68_coulomb_friction_nm,
                self.sensor.gds68_viscous_friction_nm_s_rad,
                self.sensor.gds68_stiction_velocity_rad_s,
                channel.motor_torque_nm)
            applied = channel.motor_torque_nm - friction if (safe or boot_hold) else 0.0
            self.data.ctrl[out] = float(np.clip(
                applied, -self.actuator.leg_torque_limit_nm,
                self.actuator.leg_torque_limit_nm))
            channel.elapsed_s += dt
            if channel.elapsed_s >= self.sensor.gds68_feedback_period_s:
                channel.elapsed_s %= max(self.sensor.gds68_feedback_period_s, 1.0e-6)
                channel.bias_nm += self._normal(
                    self.sensor.gds68_torque_bias_drift_nm_per_sqrt_s * math.sqrt(dt))
                measured = channel.motor_torque_nm + channel.bias_nm + \
                    self._normal(self.sensor.gds68_torque_noise_stddev_nm)
                measured = float(np.clip(
                    measured, -self.sensor.gds68_output_torque_limit_nm,
                    self.sensor.gds68_output_torque_limit_nm))
                channel.feedback_torque_nm = measured
            self._leg_feedback[out] = channel.feedback_torque_nm

        # Wheels receive a bounded velocity reference, but are driven by a
        # torque-limited proportional loop.  This avoids an ideal velocity
        # servo masking landing impulse and wheel-ground slip.
        for out, joint_index in enumerate(WHEEL_INDICES):
            velocity = float(self.data.qvel[6 + joint_index])
            target = float(target_wheel[out])
            requested = self.actuator.wheel_velocity_kp * (target - velocity)
            requested = float(np.clip(requested,
                                      -self.actuator.wheel_torque_limit_nm,
                                      self.actuator.wheel_torque_limit_nm))
            self._wheel_state[out] = self._first_order(
                self._wheel_state[out], requested,
                self.actuator.wheel_time_constant_s, dt)
            friction = self._friction(
                velocity, self.actuator.wheel_coulomb_friction_nm,
                self.actuator.wheel_viscous_friction_nm_s_rad,
                0.02, self._wheel_state[out])
            self.data.ctrl[4 + out] = float(np.clip(
                self._wheel_state[out] - friction,
                -self.actuator.wheel_torque_limit_nm,
                self.actuator.wheel_torque_limit_nm)) if (safe or boot_hold) else 0.0

    def _sensor_value(self, sensor_id: int) -> np.ndarray:
        adr = int(self.model.sensor_adr[sensor_id])
        dim = int(self.model.sensor_dim[sensor_id])
        return np.array(self.data.sensordata[adr:adr + dim], dtype=float)

    def _sample_imu(self) -> ImuSample:
        accel = self._sensor_value(self._accel_id)
        gyro = self._sensor_value(self._gyro_id)
        dt = 1.0 / self.sensor.imu_rate_hz
        self._imu_accel_bias += self.rng.normal(
            0.0, self.sensor.accel_bias_random_walk_m_s2_sqrt_s * math.sqrt(dt), 3)
        self._imu_gyro_bias += self.rng.normal(
            0.0, self.sensor.gyro_bias_random_walk_rad_s_sqrt_s * math.sqrt(dt), 3)
        accel = accel + self._imu_accel_bias + self.rng.normal(
            0.0, self.sensor.accel_noise_stddev_m_s2, 3)
        gyro = gyro + self._imu_gyro_bias + self.rng.normal(
            0.0, self.sensor.gyro_noise_stddev_rad_s, 3)
        accel = np.array([self._quantize(v, self.sensor.accel_quantization_m_s2)
                          for v in accel])
        gyro = np.array([self._quantize(v, self.sensor.gyro_quantization_rad_s)
                         for v in gyro])
        return ImuSample(self.sim_time_s,
                         tuple(float(v) for v in accel),
                         tuple(float(v) for v in gyro))

    def _sample_encoders(self) -> EncoderSample:
        position = []
        velocity = []
        for i, (pos_id, vel_id) in enumerate(zip(self._joint_pos_ids, self._joint_vel_ids)):
            p = float(self._sensor_value(pos_id)[0]) + self._normal(
                self.sensor.encoder_position_noise_stddev_rad)
            v = float(self._sensor_value(vel_id)[0]) + self._normal(
                self.sensor.encoder_velocity_noise_stddev_rad_s)
            p_quantum = (self.sensor.encoder_wheel_position_quantization_rad
                         if i in WHEEL_INDICES else self.sensor.encoder_position_quantization_rad)
            position.append(self._quantize(p, p_quantum))
            velocity.append(self._quantize(v, self.sensor.encoder_velocity_quantization_rad_s))
        effort = tuple(float(v) for v in self._leg_feedback) + (0.0, 0.0) \
            if self.sensor.gds68_feedback else None
        return EncoderSample(self.sim_time_s, tuple(position), tuple(velocity), effort)

    def _drop(self, probability: float) -> bool:
        return probability > 0.0 and bool(self.rng.random() < probability)

    @staticmethod
    def _latest(queue: Deque[Tuple[float, object]], now: float):
        latest = None
        while queue and queue[0][0] <= now:
            _, latest = queue.popleft()
        return latest

    def step(self) -> Tuple[Optional[ImuSample], Optional[EncoderSample]]:
        dt = float(self.model.opt.timestep)
        self._apply_due_command()
        self._update_actuators(dt)
        mujoco.mj_step(self.model, self.data)
        self.sim_time_s += dt

        if self.sim_time_s + 1.0e-12 >= self._next_imu_s:
            self._next_imu_s += 1.0 / self.sensor.imu_rate_hz
            if not self._drop(self.sensor.dropout_probability):
                sample = self._sample_imu()
                self._imu_queue.append((sample.stamp_s + self.sensor.sensor_latency_s, sample))
        if self.sim_time_s + 1.0e-12 >= self._next_encoder_s:
            self._next_encoder_s += 1.0 / self.sensor.encoder_rate_hz
            if not self._drop(self.sensor.encoder_dropout_probability):
                sample = self._sample_encoders()
                self._encoder_queue.append((sample.stamp_s + self.sensor.sensor_latency_s, sample))
        return (self._latest(self._imu_queue, self.sim_time_s),
                self._latest(self._encoder_queue, self.sim_time_s))
