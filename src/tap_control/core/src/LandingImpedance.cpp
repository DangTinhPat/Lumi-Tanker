#include "robot_control/LandingImpedance.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {

LandingImpedance::LandingImpedance() : LandingImpedance(Config{}) {}

void LandingImpedance::reset() {
  phase_ = Phase::Grounded;
  prev_floor_contact_ = true;
  airborne_elapsed_s_ = 0.0F;
  phase_elapsed_s_ = 0.0F;
}

LandingImpedance::Output LandingImpedance::update(
    bool floor_contact, float left_hip, float left_knee, float right_hip,
    float right_knee, float left_hip_vel, float left_knee_vel,
    float right_hip_vel, float right_knee_vel, float dt_s) {
  Output out{};
  if (!std::isfinite(dt_s) || dt_s <= 0.0F || !std::isfinite(left_hip) ||
      !std::isfinite(left_knee) || !std::isfinite(right_hip) ||
      !std::isfinite(right_knee) || !std::isfinite(left_hip_vel) ||
      !std::isfinite(left_knee_vel) || !std::isfinite(right_hip_vel) ||
      !std::isfinite(right_knee_vel)) {
    return out;
  }

  const bool liftoff_edge = !floor_contact && prev_floor_contact_;
  const bool touchdown_reading = floor_contact;
  prev_floor_contact_ = floor_contact;

  // Real (measured, not commanded) leg compression velocity - the physical
  // signal this whole component reacts to, never drives.
  const auto left = WholeBodyModel::legHeight(left_hip, left_knee);
  const auto right = WholeBodyModel::legHeight(right_hip, right_knee);
  const float left_h_dot =
      left.d_height_d_hip * left_hip_vel + left.d_height_d_knee * left_knee_vel;
  const float right_h_dot = right.d_height_d_hip * right_hip_vel +
                            right.d_height_d_knee * right_knee_vel;
  const float h_dot_avg = 0.5F * (left_h_dot + right_h_dot);

  if (liftoff_edge && phase_ == Phase::Grounded) {
    phase_ = Phase::Airborne;
    airborne_elapsed_s_ = 0.0F;
  } else if (!floor_contact &&
             (phase_ == Phase::Impact || phase_ == Phase::Rebound)) {
    // Bounced back into the air: abandon this touchdown and start a fresh
    // debounce window for the next real one.
    phase_ = Phase::Airborne;
    airborne_elapsed_s_ = 0.0F;
  }

  if (phase_ == Phase::Airborne) {
    airborne_elapsed_s_ += dt_s;
    if (touchdown_reading &&
        airborne_elapsed_s_ >= config_.min_airborne_before_touchdown_s) {
      // Real-impact gate: only engage the damper for a genuine dynamic
      // touchdown, not a gentle settle.
      if (std::abs(h_dot_avg) >= config_.touchdown_velocity_threshold_m_s) {
        phase_ = Phase::Impact;
      } else {
        phase_ = Phase::Grounded;
      }
      phase_elapsed_s_ = 0.0F;
    }
  }

  if (phase_ == Phase::Grounded || phase_ == Phase::Airborne) {
    return out;
  }

  phase_elapsed_s_ += dt_s;

  if (phase_ == Phase::Impact) {
    // Bottomed out: the leg's real compression velocity has stopped
    // increasing (crouch, i.e. height_m rising, has peaked) - a physical
    // trigger from real joint velocities, never a commanded motion, since
    // this component never moves the joint itself.
    const bool bottomed_out =
        phase_elapsed_s_ >= config_.min_impact_before_rebound_s &&
        h_dot_avg <= 0.0F;
    if (bottomed_out || phase_elapsed_s_ >= config_.max_impact_duration_s) {
      phase_ = Phase::Rebound;
      phase_elapsed_s_ = 0.0F;
    }
  }

  if (phase_ == Phase::Impact) {
    out.leg_kp_scale = config_.impact_kp_scale;
    out.leg_kd_scale = config_.impact_kd_scale;
  } else {  // Rebound
    const float blend = std::max(config_.rebound_blend_s, 1.0e-3F);
    const float t = std::clamp(phase_elapsed_s_ / blend, 0.0F, 1.0F);
    out.leg_kp_scale = config_.impact_kp_scale + t * (1.0F - config_.impact_kp_scale);
    out.leg_kd_scale = config_.impact_kd_scale + t * (1.0F - config_.impact_kd_scale);
    if (t >= 1.0F) {
      phase_ = Phase::Grounded;
      phase_elapsed_s_ = 0.0F;
    }
  }
  return out;
}

}  // namespace robot_control
