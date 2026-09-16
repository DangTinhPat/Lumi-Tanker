#include "robot_control/LandingImpedance.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {

LandingImpedance::LandingImpedance() : LandingImpedance(Config{}) {}

void LandingImpedance::reset() {
  left_side_ = Side{};
  right_side_ = Side{};
}

void LandingImpedance::updateSide(Side& side, bool floor_contact, float h_dot,
                                  float dt_s, float& kp_scale,
                                  float& kd_scale) const {
  const bool liftoff_edge = !floor_contact && side.prev_floor_contact;
  const bool touchdown_reading = floor_contact;
  side.prev_floor_contact = floor_contact;

  if (liftoff_edge && side.phase == Phase::Grounded) {
    side.phase = Phase::Airborne;
    side.airborne_elapsed_s = 0.0F;
  } else if (!floor_contact &&
             (side.phase == Phase::Impact || side.phase == Phase::Rebound)) {
    // Bounced back into the air: abandon this touchdown and start a fresh
    // debounce window for the next real one.
    side.phase = Phase::Airborne;
    side.airborne_elapsed_s = 0.0F;
  }

  if (side.phase == Phase::Airborne) {
    side.airborne_elapsed_s += dt_s;
    if (touchdown_reading &&
        side.airborne_elapsed_s >= config_.min_airborne_before_touchdown_s) {
      // Real-impact gate: only engage the damper for a genuine dynamic
      // touchdown, not a gentle settle.
      if (std::abs(h_dot) >= config_.touchdown_velocity_threshold_m_s) {
        side.phase = Phase::Impact;
      } else {
        side.phase = Phase::Grounded;
      }
      side.phase_elapsed_s = 0.0F;
    }
  }

  if (side.phase == Phase::Grounded || side.phase == Phase::Airborne) {
    kp_scale = 1.0F;
    kd_scale = 1.0F;
    return;
  }

  side.phase_elapsed_s += dt_s;

  if (side.phase == Phase::Impact) {
    // Bottomed out: this side's real compression velocity has stopped
    // increasing (crouch, i.e. height_m rising, has peaked) - a physical
    // trigger from real joint velocities, never a commanded motion, since
    // this component never moves the joint itself.
    const bool bottomed_out =
        side.phase_elapsed_s >= config_.min_impact_before_rebound_s &&
        h_dot <= 0.0F;
    if (bottomed_out || side.phase_elapsed_s >= config_.max_impact_duration_s) {
      side.phase = Phase::Rebound;
      side.phase_elapsed_s = 0.0F;
    }
  }

  if (side.phase == Phase::Impact) {
    kp_scale = config_.impact_kp_scale;
    kd_scale = config_.impact_kd_scale;
  } else {  // Rebound
    const float blend = std::max(config_.rebound_blend_s, 1.0e-3F);
    const float t = std::clamp(side.phase_elapsed_s / blend, 0.0F, 1.0F);
    kp_scale = config_.impact_kp_scale + t * (1.0F - config_.impact_kp_scale);
    kd_scale = config_.impact_kd_scale + t * (1.0F - config_.impact_kd_scale);
    if (t >= 1.0F) {
      side.phase = Phase::Grounded;
      side.phase_elapsed_s = 0.0F;
    }
  }
}

LandingImpedance::Output LandingImpedance::update(
    bool left_floor_contact, bool right_floor_contact, float left_hip,
    float left_knee, float right_hip, float right_knee, float left_hip_vel,
    float left_knee_vel, float right_hip_vel, float right_knee_vel,
    float dt_s) {
  Output out{};
  if (!std::isfinite(dt_s) || dt_s <= 0.0F || !std::isfinite(left_hip) ||
      !std::isfinite(left_knee) || !std::isfinite(right_hip) ||
      !std::isfinite(right_knee) || !std::isfinite(left_hip_vel) ||
      !std::isfinite(left_knee_vel) || !std::isfinite(right_hip_vel) ||
      !std::isfinite(right_knee_vel)) {
    return out;
  }

  // Real (measured, not commanded) leg compression velocity - the physical
  // signal this whole component reacts to, never drives. Independent per
  // side (ROLL axis): each side's own contact/velocity drives its own
  // phase machine, so an asymmetric touchdown (one wheel down, one still
  // reaching) lets that side alone yield while the other keeps its normal
  // tracking stiffness.
  const auto left = WholeBodyModel::legHeight(left_hip, left_knee);
  const auto right = WholeBodyModel::legHeight(right_hip, right_knee);
  const float left_h_dot =
      left.d_height_d_hip * left_hip_vel + left.d_height_d_knee * left_knee_vel;
  const float right_h_dot = right.d_height_d_hip * right_hip_vel +
                            right.d_height_d_knee * right_knee_vel;

  updateSide(left_side_, left_floor_contact, left_h_dot, dt_s,
            out.left_leg_kp_scale, out.left_leg_kd_scale);
  updateSide(right_side_, right_floor_contact, right_h_dot, dt_s,
            out.right_leg_kp_scale, out.right_leg_kd_scale);
  return out;
}

}  // namespace robot_control
