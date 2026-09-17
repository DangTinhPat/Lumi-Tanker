#include "robot_control/LandingController.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {
namespace {
constexpr auto kLh = static_cast<std::size_t>(JointId::LeftHip);
constexpr auto kLk = static_cast<std::size_t>(JointId::LeftKnee);
constexpr auto kRh = static_cast<std::size_t>(JointId::RightHip);
constexpr auto kRk = static_cast<std::size_t>(JointId::RightKnee);

float slewToward(float current, float target, float rate, float dt) {
  if (rate <= 0.0F) return target;
  const float step = std::max(rate, 0.0F) * dt;
  return current + std::clamp(target - current, -step, step);
}

bool finiteArray(const JointArray& a) {
  for (float v : a) {
    if (!std::isfinite(v)) return false;
  }
  return true;
}

bool confirmedSupport(const FlightObserver::Output* flight) {
  if (flight == nullptr) {
    // A missing observer output cannot prove support.  In particular, never
    // fall back to RobotState::floor_contact: a ROS/simulator boundary may
    // populate that legacy field from ground truth.
    return false;
  }

  // `airborne == false` is not equivalent to support.  In particular,
  // NoSupportTimeout deliberately reports airborne=false so that the higher
  // level fall/recovery path can take over after an unrecoverable flight.
  // TouchdownCandidate is also deliberately excluded until its debounce has
  // completed.  Only the observer's stable support phases may arm Impact.
  switch (flight->phase) {
    case FlightObserver::Phase::Grounded:
    case FlightObserver::Phase::PartialSupport:
      return true;
    case FlightObserver::Phase::TransientUnloading:
    case FlightObserver::Phase::BallisticFlight:
    case FlightObserver::Phase::CollisionCandidate:
    case FlightObserver::Phase::TouchdownCandidate:
    case FlightObserver::Phase::NoSupportTimeout:
      return false;
  }
  return false;
}

void addSideTorque(JointArray& tau, std::size_t hip, std::size_t knee,
                   float qh, float qk, float qdh, float qdk,
                   const LandingController::Config& cfg,
                   const LandingController::Output& out,
                   float ref_h, float ref_x) {
  (void)out;
  const auto leg = WholeBodyModel::legHeight(qh, qk);
  const float h_dot = leg.d_height_d_hip * qdh + leg.d_height_d_knee * qdk;
  const float x_dot = leg.d_x_d_hip * qdh + leg.d_x_d_knee * qdk;
  float f_h = cfg.height_kp_n_m * (ref_h - leg.height_m) -
              cfg.height_kd_n_s_m * h_dot;
  float f_x = cfg.x_kp_n_m * (ref_x - leg.x_m) -
              cfg.x_kd_n_s_m * x_dot;
  f_h = std::clamp(f_h, -cfg.max_virtual_force_n, cfg.max_virtual_force_n);
  f_x = std::clamp(f_x, -cfg.max_virtual_force_n, cfg.max_virtual_force_n);
  const float tau_h = leg.d_height_d_hip * f_h + leg.d_x_d_hip * f_x;
  const float tau_k = leg.d_height_d_knee * f_h + leg.d_x_d_knee * f_x;
  tau[hip] += std::clamp(tau_h, -cfg.max_feedforward_torque_nm,
                         cfg.max_feedforward_torque_nm);
  tau[knee] += std::clamp(tau_k, -cfg.max_feedforward_torque_nm,
                          cfg.max_feedforward_torque_nm);
}
}  // namespace

LandingController::LandingController() : LandingController(Config{}) {}

void LandingController::reset() {
  phase_ = Phase::Grounded;
  prev_floor_contact_ = true;
  prev_left_contact_ = true;
  prev_right_contact_ = true;
  phase_elapsed_s_ = 0.0F;
  airborne_elapsed_s_ = 0.0F;
  recovery_elapsed_s_ = 0.0F;
  drive_handoff_elapsed_s_ = 0.0F;
  drive_handoff_active_ = false;
  flight_hip_offset_rad_ = 0.0F;
  flight_knee_offset_rad_ = 0.0F;
  left_ref_ = SideRef{};
  right_ref_ = SideRef{};
}

void LandingController::updatePhase(
    const RobotState& state, const FlightObserver::Output* flight) {
  // A debounced observer prevents a few-millisecond stair-step unloading
  // event from becoming a full landing cycle. During re-contact validation,
  // keep the robot in Flight until support persists.
  const bool observer_airborne = flight != nullptr && flight->airborne;
  const bool observer_no_support_timeout =
      flight != nullptr &&
      flight->phase == FlightObserver::Phase::NoSupportTimeout;
  const bool contact = confirmedSupport(flight);
  const bool liftoff = flight != nullptr
      ? (observer_airborne || observer_no_support_timeout) &&
            phase_ == Phase::Grounded
      : !contact && prev_floor_contact_;
  // Per-side support is only taken from the sensor-derived observer output.
  // RobotState contact fields are intentionally ignored at this boundary.
  const bool left_contact = flight != nullptr && flight->left_support_contact;
  const bool right_contact = flight != nullptr && flight->right_support_contact;
  const bool left_touchdown = left_contact && !prev_left_contact_;
  const bool right_touchdown = right_contact && !prev_right_contact_;
  prev_floor_contact_ = contact;
  prev_left_contact_ = left_contact;
  prev_right_contact_ = right_contact;

  phase_elapsed_s_ += state.dt_s;
  if (!contact) {
    airborne_elapsed_s_ += state.dt_s;
  }

  if (phase_ == Phase::Grounded && liftoff) {
    phase_ = Phase::Flight;
    phase_elapsed_s_ = 0.0F;
    airborne_elapsed_s_ = state.dt_s;
    recovery_elapsed_s_ = 0.0F;
    drive_handoff_elapsed_s_ = 0.0F;
    drive_handoff_active_ = config_.recovery_drive_ramp_s > 0.0F;
    flight_hip_offset_rad_ = 0.0F;
    flight_knee_offset_rad_ = 0.0F;
    const auto left = WholeBodyModel::legHeight(
        state.joints.position_rad[kLh], state.joints.position_rad[kLk]);
    const auto right = WholeBodyModel::legHeight(
        state.joints.position_rad[kRh], state.joints.position_rad[kRk]);
    left_ref_ = {left.height_m, left.x_m};
    right_ref_ = {right.height_m, right.x_m};
    return;
  }

  if (phase_ == Phase::Flight) {
    // A shock is evidence that something struck the robot, not evidence that
    // a wheel has acquired a support wrench. In particular, the ledge fillet
    // and body/edge collisions produce the same IMU signature while both
    // wheels are still airborne. Only the validated IMU-derived support path may
    // create the touchdown phase.
    const bool impact =
        airborne_elapsed_s_ >= config_.min_flight_before_impact_s &&
        contact;
    if (impact) {
      phase_ = Phase::Impact;
      phase_elapsed_s_ = 0.0F;
      if (left_contact) {
        const auto left = WholeBodyModel::legHeight(
            state.joints.position_rad[kLh], state.joints.position_rad[kLk]);
        left_ref_ = {left.height_m, left.x_m, true};
      }
      if (right_contact) {
        const auto right = WholeBodyModel::legHeight(
            state.joints.position_rad[kRh], state.joints.position_rad[kRk]);
        right_ref_ = {right.height_m, right.x_m, true};
      }
    }
    return;
  }

  if (phase_ == Phase::Impact) {
    if (!contact) {
      // The support was lost before the impact schedule settled. Abandon the
      // ground-referenced phase immediately; the next valid contact can create
      // a new Impact event with fresh leg references.
      phase_ = Phase::Flight;
      phase_elapsed_s_ = 0.0F;
      airborne_elapsed_s_ = state.dt_s;
      left_ref_.in_impact = false;
      right_ref_.in_impact = false;
      return;
    }
    if (left_touchdown && !left_ref_.in_impact) {
      const auto left = WholeBodyModel::legHeight(
          state.joints.position_rad[kLh], state.joints.position_rad[kLk]);
      left_ref_ = {left.height_m, left.x_m, true};
    }
    if (right_touchdown && !right_ref_.in_impact) {
      const auto right = WholeBodyModel::legHeight(
          state.joints.position_rad[kRh], state.joints.position_rad[kRk]);
      right_ref_ = {right.height_m, right.x_m, true};
    }
    const float joint_speed =
        std::abs(state.joints.velocity_rad_s[kLh]) +
        std::abs(state.joints.velocity_rad_s[kLk]) +
        std::abs(state.joints.velocity_rad_s[kRh]) +
        std::abs(state.joints.velocity_rad_s[kRk]);
    const bool quiet_enough =
        phase_elapsed_s_ >= config_.min_impact_s &&
        std::abs(state.imu.pitch_rate_rad_s) <= config_.recovery_pitch_rate_rad_s &&
        joint_speed <= config_.recovery_joint_speed_rad_s;
    if (quiet_enough || phase_elapsed_s_ >= config_.max_impact_s) {
      phase_ = Phase::Recovery;
      phase_elapsed_s_ = 0.0F;
      recovery_elapsed_s_ = 0.0F;
    }
    return;
  }

  if (phase_ == Phase::Recovery) {
    recovery_elapsed_s_ += state.dt_s;
    if (!contact) {
      phase_ = Phase::Flight;
      phase_elapsed_s_ = 0.0F;
      airborne_elapsed_s_ = state.dt_s;
      left_ref_.in_impact = false;
      right_ref_.in_impact = false;
      return;
    }
    if (recovery_elapsed_s_ >= config_.recovery_blend_s) {
      phase_ = Phase::Grounded;
      phase_elapsed_s_ = 0.0F;
      airborne_elapsed_s_ = 0.0F;
    }
  }
}

JointArray LandingController::flightOffset(
    const RobotState& state, const JointArray& nominal_target_rad) {
  JointArray offset{};
  const float t = std::max(
      0.0F, airborne_elapsed_s_ - std::max(config_.flight_extension_delay_s, 0.0F));
  const float desired_extension = std::clamp(
      0.5F * std::max(config_.flight_gravity_m_s2, 0.0F) * t * t,
      0.0F, std::max(config_.flight_extension_max_m, 0.0F));
  const auto leg = WholeBodyModel::legHeight(
      nominal_target_rad[kLh], nominal_target_rad[kLk]);
  float hip_target = 0.0F;
  float knee_target = 0.0F;
  const float desired_dh = -desired_extension;
  constexpr float desired_dx = 0.0F;
  const float det =
      leg.d_height_d_hip * leg.d_x_d_knee -
      leg.d_height_d_knee * leg.d_x_d_hip;
  if (std::abs(det) > 1.0e-5F) {
    hip_target = (leg.d_x_d_knee * desired_dh -
                  leg.d_height_d_knee * desired_dx) / det;
    hip_target = std::clamp(hip_target, -config_.flight_hip_offset_max_rad,
                            config_.flight_hip_offset_max_rad);
    if (std::abs(leg.d_height_d_knee) > 1.0e-5F) {
      knee_target = (desired_dh - leg.d_height_d_hip * hip_target) /
                    leg.d_height_d_knee;
    }
  } else if (std::abs(leg.d_height_d_knee) > 1.0e-5F) {
    knee_target = desired_dh / leg.d_height_d_knee;
  }
  knee_target = std::clamp(knee_target, 0.0F,
                           std::max(config_.flight_knee_offset_max_rad, 0.0F));
  const float pitch_rate_hip = std::clamp(
      -config_.flight_pitch_rate_hip_gain_rad_per_rad_s *
          state.imu.pitch_rate_rad_s,
      -std::max(config_.flight_pitch_rate_hip_offset_max_rad, 0.0F),
      std::max(config_.flight_pitch_rate_hip_offset_max_rad, 0.0F));
  const float hip_total_limit =
      std::max(config_.flight_hip_offset_max_rad, 0.0F) +
      std::max(config_.flight_pitch_rate_hip_offset_max_rad, 0.0F);
  hip_target = std::clamp(hip_target + pitch_rate_hip, -hip_total_limit,
                          hip_total_limit);
  flight_hip_offset_rad_ = slewToward(
      flight_hip_offset_rad_, hip_target, config_.flight_target_slew_rate_rad_s,
      state.dt_s);
  flight_knee_offset_rad_ = slewToward(
      flight_knee_offset_rad_, knee_target, config_.flight_target_slew_rate_rad_s,
      state.dt_s);
  offset[kLh] = flight_hip_offset_rad_;
  offset[kRh] = flight_hip_offset_rad_;
  offset[kLk] = flight_knee_offset_rad_;
  offset[kRk] = flight_knee_offset_rad_;
  return offset;
}

void LandingController::addImpactTorques(const RobotState& state,
                                         Output& out) const {
  if (left_ref_.in_impact) {
    addSideTorque(out.feedforward_nm, kLh, kLk,
                  state.joints.position_rad[kLh], state.joints.position_rad[kLk],
                  state.joints.velocity_rad_s[kLh], state.joints.velocity_rad_s[kLk],
                  config_, out, left_ref_.height_m, left_ref_.x_m);
  }
  if (right_ref_.in_impact) {
    addSideTorque(out.feedforward_nm, kRh, kRk,
                  state.joints.position_rad[kRh], state.joints.position_rad[kRk],
                  state.joints.velocity_rad_s[kRh], state.joints.velocity_rad_s[kRk],
                  config_, out, right_ref_.height_m, right_ref_.x_m);
  }
}

LandingController::Output LandingController::update(
    const RobotState& state, const JointArray& nominal_target_rad,
    const FlightObserver::Output* flight) {
  Output out{};
  out.kp_scale.fill(1.0F);
  out.kd_scale.fill(1.0F);
  if (!config_.enabled || !std::isfinite(state.dt_s) || state.dt_s <= 0.0F ||
      !finiteArray(state.joints.position_rad) ||
      !finiteArray(state.joints.velocity_rad_s) ||
      !finiteArray(nominal_target_rad) ||
      !std::isfinite(state.imu.accel_magnitude_m_s2) ||
      !std::isfinite(state.imu.pitch_rate_rad_s)) {
    return out;
  }

  updatePhase(state, flight);
  out.phase = phase_;
  out.active = phase_ != Phase::Grounded;
  out.suspend_wheel_balance = phase_ == Phase::Flight;

  if (drive_handoff_active_) {
    if (phase_ != Phase::Grounded) {
      drive_handoff_elapsed_s_ = 0.0F;
      out.drive_velocity_scale = 0.0F;
    } else {
      drive_handoff_elapsed_s_ += state.dt_s;
      const float ramp = std::max(config_.recovery_drive_ramp_s, 1.0e-3F);
      out.drive_velocity_scale =
          std::clamp(drive_handoff_elapsed_s_ / ramp, 0.0F, 1.0F);
      if (out.drive_velocity_scale >= 1.0F) {
        drive_handoff_active_ = false;
      }
    }
  }

  if (phase_ == Phase::Flight) {
    out.position_offset_rad = flightOffset(state, nominal_target_rad);
    out.kp_scale[kLh] = out.kp_scale[kRh] = config_.flight_kp_scale;
    out.kp_scale[kLk] = out.kp_scale[kRk] = config_.flight_kp_scale;
    out.kd_scale[kLh] = out.kd_scale[kRh] = config_.flight_kd_scale;
    out.kd_scale[kLk] = out.kd_scale[kRk] = config_.flight_kd_scale;
  } else if (phase_ == Phase::Impact) {
    out.kp_scale[kLh] = out.kp_scale[kRh] = config_.impact_kp_scale;
    out.kp_scale[kLk] = out.kp_scale[kRk] = config_.impact_kp_scale;
    out.kd_scale[kLh] = out.kd_scale[kRh] = config_.impact_kd_scale;
    out.kd_scale[kLk] = out.kd_scale[kRk] = config_.impact_kd_scale;
    addImpactTorques(state, out);
  } else if (phase_ == Phase::Recovery) {
    const float blend = std::max(config_.recovery_blend_s, 1.0e-3F);
    const float s = std::clamp(recovery_elapsed_s_ / blend, 0.0F, 1.0F);
    const float kp = config_.impact_kp_scale +
                     s * (1.0F - config_.impact_kp_scale);
    const float kd = config_.impact_kd_scale +
                     s * (1.0F - config_.impact_kd_scale);
    out.kp_scale[kLh] = out.kp_scale[kRh] = kp;
    out.kp_scale[kLk] = out.kp_scale[kRk] = kp;
    out.kd_scale[kLh] = out.kd_scale[kRh] = kd;
    out.kd_scale[kLk] = out.kd_scale[kRk] = kd;
  }
  return out;
}

}  // namespace robot_control
