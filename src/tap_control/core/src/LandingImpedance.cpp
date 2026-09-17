#include "robot_control/LandingImpedance.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {

LandingImpedance::LandingImpedance() : LandingImpedance(Config{}) {}

void LandingImpedance::reset() {
  reset(true, true);
}

void LandingImpedance::reset(bool left_floor_contact, bool right_floor_contact) {
  left_side_ = Side{};
  right_side_ = Side{};
  left_side_.phase = left_floor_contact ? Phase::Grounded : Phase::Airborne;
  right_side_.phase = right_floor_contact ? Phase::Grounded : Phase::Airborne;
  left_side_.prev_floor_contact = left_floor_contact;
  right_side_.prev_floor_contact = right_floor_contact;
}

bool LandingImpedance::energySizingConfigured() const noexcept {
  return config_.leg_knee_inertia_kgm2 > 0.0F &&
         config_.leg_kp_nominal_nm_rad > 0.0F &&
         config_.leg_kd_nominal_nm_s_rad > 0.0F &&
         std::isfinite(config_.knee_hard_limit_rad);
}

void LandingImpedance::sizeFromEnergy(Side& side, float knee_vel_rad_s) const {
  side.impact_kp_scale = config_.impact_kp_scale;
  side.impact_kd_scale = config_.impact_kd_scale;
  if (!energySizingConfigured()) return;
  const float stroke = (side.knee_rad_at_impact - config_.knee_hard_limit_rad) *
      std::clamp(config_.energy_stroke_margin, 0.05F, 1.0F);
  if (stroke <= 1.0e-4F) return;
  const float v = std::abs(knee_vel_rad_s);
  const float k_required =
      config_.leg_knee_inertia_kgm2 * v * v / (stroke * stroke);
  side.impact_kp_scale = std::clamp(
      k_required / config_.leg_kp_nominal_nm_rad,
      config_.energy_kp_scale_min, config_.energy_kp_scale_max);
  const float kp_absolute = side.impact_kp_scale * config_.leg_kp_nominal_nm_rad;
  const float kd_critical =
      2.0F * std::sqrt(std::max(kp_absolute, 0.0F) * config_.leg_knee_inertia_kgm2);
  side.impact_kd_scale = std::clamp(
      kd_critical / config_.leg_kd_nominal_nm_s_rad, 1.0F,
      config_.energy_kd_scale_max);
}

void LandingImpedance::updateSide(Side& side, bool floor_contact, float h_dot,
                                  float knee_rad, float knee_vel_rad_s,
                                  float accel_mag_mps2, float dt_s,
                                  bool hard_impact_event,
                                  float& hip_kp_scale, float& hip_kd_scale,
                                  float& knee_kp_scale,
                                  float& knee_kd_scale) const {
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
    // The IMU event is only a supplementary impact cue after this side has a
    // validated support contact. A global shock must never put the opposite,
    // still-airborne side into Impact.
    const bool event_touchdown = hard_impact_event && floor_contact;
    if ((touchdown_reading || event_touchdown) &&
        side.airborne_elapsed_s >= config_.min_airborne_before_touchdown_s) {
      // Real-impact gate: only engage the damper for a genuine dynamic
      // touchdown, not a gentle settle. OR'd with the accelerometer spike
      // gate (see Config::hard_impact_accel_mps2's own comment): h_dot
      // alone, checked at this exact tick, was shown 2026-09-16 to miss a
      // real hard impact on a 50cm drop - the joint had not yet responded
      // to the real contact force at the instant contact first reads true.
      const bool dynamic_touchdown =
          std::abs(h_dot) >= config_.touchdown_velocity_threshold_m_s ||
          accel_mag_mps2 >= config_.hard_impact_accel_mps2 || event_touchdown;
      if (dynamic_touchdown) {
        side.phase = Phase::Impact;
        side.knee_rad_at_impact = knee_rad;
        if (config_.energy_capture_window_s > 0.0F && energySizingConfigured()) {
          // Defer final sizing until the leg has actually had time to
          // respond to the real impact force - see
          // Config::energy_capture_window_s's own comment. Safest interim:
          // the softest configured scale, never full/uncapped stiffness,
          // while the true peak velocity is still being tracked.
          side.capturing_peak = true;
          side.peak_knee_vel_rad_s = std::abs(knee_vel_rad_s);
          side.impact_kp_scale = config_.energy_kp_scale_min;
          side.impact_kd_scale = config_.energy_kd_scale_max;
        } else {
          side.capturing_peak = false;
          sizeFromEnergy(side, knee_vel_rad_s);
        }
      } else {
        side.phase = Phase::Grounded;
      }
      side.phase_elapsed_s = 0.0F;
    }
  }

  if (side.phase == Phase::Grounded) {
    hip_kp_scale = 1.0F;
    hip_kd_scale = 1.0F;
    knee_kp_scale = 1.0F;
    knee_kd_scale = 1.0F;
    return;
  }

  if (side.phase == Phase::Airborne) {
    hip_kp_scale = config_.airborne_hip_kp_scale;
    hip_kd_scale = config_.airborne_hip_kd_scale;
    knee_kp_scale = config_.airborne_knee_kp_scale;
    knee_kd_scale = config_.airborne_knee_kd_scale;
    return;
  }

  side.phase_elapsed_s += dt_s;

  if (side.phase == Phase::Impact && side.capturing_peak) {
    side.peak_knee_vel_rad_s =
        std::max(side.peak_knee_vel_rad_s, std::abs(knee_vel_rad_s));
    if (side.phase_elapsed_s >= config_.energy_capture_window_s) {
      sizeFromEnergy(side, side.peak_knee_vel_rad_s);
      side.capturing_peak = false;
    }
  }

  if (side.phase == Phase::Impact) {
    // Bottomed out: the leg was compressing in the negative body-z height
    // convention (h_dot < 0) and has now stopped/started rebounding. The old
    // <= test exited Impact on the first compression tick, before any energy
    // could be dissipated.
    const bool bottomed_out =
        side.phase_elapsed_s >= config_.min_impact_before_rebound_s &&
        h_dot >= 0.0F;
    if (bottomed_out || side.phase_elapsed_s >= config_.max_impact_duration_s) {
      side.phase = Phase::Rebound;
      side.phase_elapsed_s = 0.0F;
    }
  }

  if (side.phase == Phase::Impact) {
    hip_kp_scale = 1.0F;
    hip_kd_scale = 1.0F;
    knee_kp_scale = side.impact_kp_scale;
    knee_kd_scale = side.impact_kd_scale;
  } else {  // Rebound
    const float blend = std::max(config_.rebound_blend_s, 1.0e-3F);
    const float t = std::clamp(side.phase_elapsed_s / blend, 0.0F, 1.0F);
    hip_kp_scale = 1.0F;
    hip_kd_scale = 1.0F;
    knee_kp_scale = side.impact_kp_scale + t * (1.0F - side.impact_kp_scale);
    knee_kd_scale = side.impact_kd_scale + t * (1.0F - side.impact_kd_scale);
    if (t >= 1.0F &&
        floor_contact) {
      side.phase = Phase::Grounded;
      side.phase_elapsed_s = 0.0F;
    }
  }
}

LandingImpedance::Output LandingImpedance::update(
    bool left_floor_contact, bool right_floor_contact, float left_hip,
    float left_knee, float right_hip, float right_knee, float left_hip_vel,
    float left_knee_vel, float right_hip_vel, float right_knee_vel,
    float accel_mag_mps2, float dt_s, bool hard_impact_event) {
  Output out{};
  if (!std::isfinite(dt_s) || dt_s <= 0.0F || !std::isfinite(left_hip) ||
      !std::isfinite(left_knee) || !std::isfinite(right_hip) ||
      !std::isfinite(right_knee) || !std::isfinite(left_hip_vel) ||
      !std::isfinite(left_knee_vel) || !std::isfinite(right_hip_vel) ||
      !std::isfinite(right_knee_vel) || !std::isfinite(accel_mag_mps2)) {
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

  updateSide(left_side_, left_floor_contact, left_h_dot, left_knee,
            left_knee_vel, accel_mag_mps2, dt_s, hard_impact_event,
            out.left_hip_kp_scale,
            out.left_hip_kd_scale, out.left_leg_kp_scale,
            out.left_leg_kd_scale);
  updateSide(right_side_, right_floor_contact, right_h_dot, right_knee,
            right_knee_vel, accel_mag_mps2, dt_s, hard_impact_event,
            out.right_hip_kp_scale,
            out.right_hip_kd_scale, out.right_leg_kp_scale,
            out.right_leg_kd_scale);
  return out;
}

}  // namespace robot_control
