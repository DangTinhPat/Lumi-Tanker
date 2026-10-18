#!/usr/bin/env python3
"""Small black-box gates for the MuJoCo sensor/actuator boundary."""

import argparse
from pathlib import Path

import numpy as np

from mujoco_sensor_actuator_backend import ActuatorConfig, SensorConfig, TapMujocoBackend


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()

    # Default settings intentionally contain noise, latency and packet loss.
    backend = TapMujocoBackend(args.model)
    # The natural ENABLE reference is tangent to the CAD-derived 40 mm wheel radius.
    # MuJoCo may report zero contacts at exact tangency; either zero contacts
    # or non-penetrating wheel contacts is valid before the first step.
    assert abs(float(backend.data.qpos[2]) - 0.1484350375) < 1.0e-9
    assert all(float(backend.data.contact[i].dist) >= -1.0e-6
               for i in range(backend.data.ncon)), \
        "natural ENABLE reference penetrates the ground"
    samples = []
    encoders = []
    for _ in range(30):
        imu, enc = backend.step()
        if imu:
            samples.append((backend.sim_time_s, imu))
        if enc:
            encoders.append(enc)
    assert samples and encoders, "sensor streams did not emerge"
    assert all(np.all(np.isfinite(s.accel_m_s2 + s.gyro_rad_s)) for _, s in samples)
    assert all(np.all(np.isfinite(e.position_rad + e.velocity_rad_s)) for e in encoders)
    assert all(e.effort_nm is None for e in encoders), "GDS68 disabled path leaked effort"
    # At the native 3 kHz step the first 2 ms-latent packet is observable on
    # the next integration boundary (approximately 2.67 ms).
    assert samples[0][0] >= 0.002, "sensor latency was not applied"

    # Enable the explicit GDS68 provenance path.  The actuator must have a
    # finite lag and bounded output; it must not become an ideal torque source.
    sensor = SensorConfig(
        dropout_probability=0.0, encoder_dropout_probability=0.0,
        sensor_latency_s=0.002, gds68_feedback=True, seed=7,
        accel_noise_stddev_m_s2=0.0, gyro_noise_stddev_rad_s=0.0,
        accel_bias_stddev_m_s2=0.0, gyro_bias_stddev_rad_s=0.0,
    )
    actuator = ActuatorConfig(command_latency_s=0.002, command_timeout_s=0.10)
    feedback_backend = TapMujocoBackend(args.model, sensor, actuator)
    feedback_backend.set_enable(True)
    feedback_backend.receive_commands((5.0, 0.0, 0.0, 0.0), (80.0, 80.0))
    assert np.all(np.abs(feedback_backend._command_queue[0][2]) <=
                  41.88790204786391 + 1.0e-9), \
        "wheel command exceeded CJC 400 rpm rated-speed limit"
    feedback_backend.step()
    assert abs(float(feedback_backend.data.ctrl[0])) < 5.0, "command latency/lag missing"
    feedback_backend.step()
    feedback_backend.step()
    assert 0.0 < abs(float(feedback_backend.data.ctrl[0])) < 5.0
    feedback_samples = []
    for _ in range(8):
        _, enc = feedback_backend.step()
        if enc:
            feedback_samples.append(enc)
    assert feedback_samples and all(e.effort_nm is not None for e in feedback_samples)
    assert all(np.all(np.isfinite(e.effort_nm)) for e in feedback_samples)
    assert abs(float(feedback_backend.data.ctrl[4])) <= actuator.wheel_torque_limit_nm + 1.0e-9

    # Reset is a simulator lifecycle operation; it must clear actuator state
    # and return time to zero without leaving a stale command active.
    feedback_backend.reset()
    assert feedback_backend.sim_time_s == 0.0
    assert not feedback_backend.enabled
    assert np.allclose(feedback_backend.data.ctrl, 0.0)
    print("PASS MuJoCo sensor/actuator boundary: noisy streams, latency, bounds, provenance, reset")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
