#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <new>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/v3/CurrentTorque.hpp"
#include "tap_fw/v3/LegHeight.hpp"
#include "tap_fw/v3/V3Controller.hpp"

namespace {
using tap_fw::v3::V3Controller;

struct Bridge {
  explicit Bridge(float timestep_s) : dt_s(timestep_s) {
    robot_control::ImuStateEstimator::Config config{};
    config.accel_correction_gain_hz = 5.0F;
    config.commanded_accel_blend = 0.0F;
    config.seed_window_samples = 50;
    estimator.setConfig(tap_fw::v3::balancingEstimatorConfig(config));
  }

  float dt_s;
  robot_control::ImuStateEstimator estimator;
  V3Controller controller;
  V3Controller::Output output{};
  std::array<float, 4> previous_target{};
  bool imu_seeded = false;
  bool start_sent = false;
};

void configureBalance(Bridge& bridge, const float* q) {
  namespace geometry = robot_control::geometry;
  auto config = tap_fw::v3::defaultV3Config();
  config.lqr.sampling_period_s = bridge.dt_s;
  config.engage.immediate = true;
  config.engage.automatic = true;
  config.engage.wb_armed_only = false;
  config.roll.enabled = false;
  config.initial_knee_rad = std::clamp(0.5F * (q[1] + q[4]),
      tap_fw::v3::LegHeight::kKneeMinRad, tap_fw::v3::LegHeight::kKneeMaxRad);
  config.com_height_m = tap_fw::v3::LegHeight::forKnee(
      config.initial_knee_rad).com_above_axle_m;
  config.lqr_inertia_length = true;
  config.lqr.max_pendulum_length_m = 0.24F;
  config.lqr.max_ground_accel_m_s2 = std::min(config.lqr.wheel_ground_mu * 9.81F,
      2.0F * 0.15F / (geometry::wheel_radius_m * geometry::total_mass_kg));
  config.lqr_prediction = {true, 0.050F, 0.075F};
  config.lqr_lead_adaptive = false;
  config.lqr_lead_s = 0.075F;
  config.tier.reflex_gate = true;
  config.tier.reflex_inertial_gate = true;
  config.tier.lead_gyro_only = true;
  config.tier.plan_accel_m_s2 = 0.6F;
  config.tier.plan_jerk_m_s3 = 10.0F;
  config.tier.hip_advance_s = config.tier.preview_s;
  config.tier.height_exp = 0.15F;
  config.tier.body_damping_s = 0.08F;
  config.tier.body_damp_lpf_hz = 4.5F;
  config.tier.knee_follow_hz = 1.0F;
  config.tier.knee_follow_rate_rad_s = 0.6F;
  config.tier.sus_adaptive = true;
  config.trim_acc = false;
  config.height.rate_m_s = 0.015F;
  config.height.accel_m_s2 = 0.05F;
  config.height.jerk_m_s3 = 0.20F;
  config.tier.joint_rate_gain_low = 1.0F;
  bridge.controller = V3Controller(config);

  const auto com = robot_control::WholeBodyModel{}.compute(q[0], q[1], q[3], q[4]);
  const float hang_hip = tap_fw::v3::LegHeight::hangHipCad();
  const auto hang = robot_control::WholeBodyModel{}.compute(hang_hip, 0.0F,
                                                             hang_hip, 0.0F);
  const float virtual_offset = config.tier.virtual_pitch
      ? com.com_line_angle_rad - hang.com_line_angle_rad : 0.0F;
  bridge.controller.setBalanceAngle(true,
      robot_control::WholeBodyModel::equilibriumPitch(com) + virtual_offset);
  bridge.previous_target = {q[0], q[1], q[3], q[4]};
}
}  // namespace

extern "C" {
void* tap_v3_balance_create(float timestep_s) {
  if (!std::isfinite(timestep_s) || timestep_s <= 0.0F) return nullptr;
  return new (std::nothrow) Bridge(timestep_s);
}

void tap_v3_balance_destroy(void* handle) {
  delete static_cast<Bridge*>(handle);
}

// q and qd order is [LH, LK, LW, RH, RK, RW]. Sensor vectors are body-frame.
int tap_v3_balance_step(void* handle, const float* accel, const float* gyro,
                       const float* q, const float* qd, const float* measured_current,
                       double now_s, int request_balance, float* target_current,
                       float* wheel_speed, int* state, int* wheels_enabled,
                       int* legs_locked, int* imu_valid) {
  if (!handle || !accel || !gyro || !q || !qd || !measured_current ||
      !target_current || !wheel_speed || !state || !wheels_enabled ||
      !legs_locked || !imu_valid || !std::isfinite(now_s)) return -1;
  try {
    auto& bridge = *static_cast<Bridge*>(handle);
    const std::array<float, 3> a{accel[0], accel[1], accel[2]};
    const std::array<float, 3> g{gyro[0], gyro[1], gyro[2]};
    robot_control::ImuStateEstimator::Output estimate{};
    if (request_balance) {
      estimate = bridge.estimator.update(a, g, bridge.dt_s,
          static_cast<float>(now_s), 0.0F, bridge.output.legs_moving, false);
      bridge.imu_seeded = bridge.imu_seeded || estimate.seeded;
    }

    if (request_balance && !bridge.start_sent && bridge.imu_seeded && estimate.valid) {
      configureBalance(bridge, q);
      bridge.start_sent = true;
    }

    V3Controller::Input in{};
    in.now_us = static_cast<std::uint64_t>(now_s * 1.0e6);
    in.dt_s = bridge.dt_s;
    in.pitch_rad = estimate.pitch_rad;
    in.pitch_rate_rad_s = estimate.pitch_rate_rad_s;
    in.yaw_rate_rad_s = estimate.yaw_rate_rad_s;
    in.roll_rate_rad_s = estimate.roll_rate_rad_s;
    in.accel_x_m_s2 = accel[0];
    in.accel_y_m_s2 = accel[1];
    in.accel_z_m_s2 = accel[2];
    in.sensors_ok = bridge.imu_seeded && estimate.valid;
    in.legs_measured = true;
    in.legs_ready = true;
    for (int side = 0; side < 2; ++side) {
      const int joint = 3 * side;
      in.hip_rad[side] = q[joint];
      in.knee_rad[side] = q[joint + 1];
      in.hip_rate_rad_s[side] = qd[joint];
      in.knee_rate_rad_s[side] = qd[joint + 1];
      in.wheel_rad_s[side] = qd[joint + 2];
      in.wheel_pos_rad[side] = q[joint + 2];
      in.leg_chain_rad += 0.5F * (q[joint] + q[joint + 1]);
      in.legs_ready = in.legs_ready &&
          std::abs(bridge.previous_target[2 * side] - q[joint]) < 0.08F &&
          std::abs(bridge.previous_target[2 * side + 1] - q[joint + 1]) < 0.08F;
    }

    V3Controller::Command command{};
    command.start = bridge.start_sent && bridge.controller.state() == V3Controller::State::Idle;
    command.height_m = tap_fw::v3::LegHeight::forKnee(
        bridge.controller.config().initial_knee_rad).axle_below_hip_m;
    bridge.output = bridge.controller.update(in, command);

    const std::array<float, 4> target{
        bridge.output.leg_hip_offset_rad + bridge.output.leg_roll_hip_rad[0],
        bridge.output.leg_knee_offset_rad + bridge.output.leg_roll_knee_rad[0],
        bridge.output.leg_hip_offset_rad + bridge.output.leg_roll_hip_rad[1],
        bridge.output.leg_knee_offset_rad + bridge.output.leg_roll_knee_rad[1]};
    for (int i = 0; i < 4; ++i) {
      const float rate = (target[i] - bridge.previous_target[i]) / bridge.dt_s;
      bridge.previous_target[i] = target[i];
      const int joint = (i / 2) * 3 + (i % 2);
      const auto distal = robot_control::WholeBodyModel::legGravityTorque(q[joint], q[joint + 1]);
      const auto body = robot_control::WholeBodyModel::bodySupportTorque(q[joint], q[joint + 1]);
      const float gravity = i % 2 == 0 ? distal.hip_nm + body.hip_nm
                                       : distal.knee_nm + body.knee_nm;
      const float requested = bridge.output.legs_locked
          ? gravity + 12.0F * (target[i] - q[joint]) + 0.8F * (rate - qd[joint])
          : 0.0F;
      target_current[i] = tap_fw::v3::currentTorqueCommand(
          requested, measured_current[i], bridge.output.legs_locked,
          0.47F, 8.0F, 0.2F).torque_nm / 0.47F;
    }
    for (int side = 0; side < 2; ++side)
      wheel_speed[side] = bridge.output.wheels_enabled
          ? bridge.output.wheel_cmd_rad_s[side] : 0.0F;
    *state = static_cast<int>(bridge.output.state);
    *wheels_enabled = bridge.output.wheels_enabled;
    *legs_locked = bridge.output.legs_locked;
    *imu_valid = in.sensors_ok;
    return 0;
  } catch (...) {
    return -2;
  }
}

int tap_v3_leg_gravity(const float* q, float* torque_nm) {
  if (!q || !torque_nm) return -1;
  try {
    const robot_control::WholeBodyModel model{};
    for (int side = 0; side < 2; ++side) {
      const int joint = 2 * side;
      const auto distal = model.legGravityTorque(q[joint], q[joint + 1]);
      const auto body = model.bodySupportTorque(q[joint], q[joint + 1]);
      torque_nm[joint] = distal.hip_nm + body.hip_nm;
      torque_nm[joint + 1] = distal.knee_nm + body.knee_nm;
    }
    return 0;
  } catch (...) {
    return -2;
  }
}
}  // extern "C"
