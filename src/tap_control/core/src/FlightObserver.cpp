#include "robot_control/FlightObserver.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {
namespace {

float clamp01(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}

}  // namespace

FlightObserver::FlightObserver() : FlightObserver(Config{}) {}

void FlightObserver::reset() {
  phase_ = Phase::Grounded;
  previous_contact_ = true;
  support_loss_elapsed_s_ = 0.0F;
  flight_time_s_ = 0.0F;
  free_fall_elapsed_s_ = 0.0F;
  collision_elapsed_s_ = 0.0F;
  touchdown_elapsed_s_ = 0.0F;
  support_settle_elapsed_s_ = 0.0F;
  support_settle_active_ = false;
  downward_velocity_m_s_ = 0.0F;
}

void FlightObserver::reset(const RobotState& state) {
  reset();
  previous_contact_ = state.floor_contact;
  if (!state.floor_contact) {
    phase_ = Phase::TransientUnloading;
  } else if (state.left_floor_contact != state.right_floor_contact) {
    phase_ = Phase::PartialSupport;
    support_settle_active_ = true;
  }
}

bool FlightObserver::finiteInput(const RobotState& state) const noexcept {
  return std::isfinite(state.dt_s) && state.dt_s > 0.0F &&
         std::isfinite(state.imu.accel_magnitude_m_s2) &&
         std::isfinite(state.imu.pitch_rate_rad_s);
}

FlightObserver::Output FlightObserver::makeOutput(bool partial_support) const {
  Output output;
  output.phase = phase_;
  output.partial_support = partial_support;
  output.support_settling = support_settle_active_ &&
      support_settle_elapsed_s_ < std::max(config_.support_settle_s, 0.0F);
  output.support_lost = support_loss_elapsed_s_ >=
                        std::max(config_.support_loss_debounce_s, 0.0F);
  // TransientUnloading is deliberately not exposed as airborne until the
  // support-loss debounce has elapsed. This is what lets short staircase
  // contact gaps pass through without arming the landing controller.
  output.airborne =
      (output.support_lost &&
       (phase_ == Phase::TransientUnloading ||
        phase_ == Phase::BallisticFlight ||
        phase_ == Phase::CollisionCandidate)) ||
      phase_ == Phase::TouchdownCandidate;
  output.ballistic = phase_ == Phase::BallisticFlight;
  output.collision_candidate = phase_ == Phase::CollisionCandidate;
  output.touchdown_candidate = phase_ == Phase::TouchdownCandidate;
  output.support_loss_elapsed_s = support_loss_elapsed_s_;
  output.flight_time_s = flight_time_s_;
  output.downward_velocity_m_s = downward_velocity_m_s_;
  output.free_fall_confidence = clamp01(
      free_fall_elapsed_s_ / std::max(config_.free_fall_debounce_s, 1.0e-4F));
  output.collision_confidence = clamp01(
      collision_elapsed_s_ / std::max(config_.collision_hold_s, 1.0e-4F));
  return output;
}

FlightObserver::Output FlightObserver::update(const RobotState& state) {
  if (!finiteInput(state)) {
    return makeOutput(false);
  }

  const float dt = std::clamp(state.dt_s, 1.0e-5F, 0.05F);
  const bool contact = state.floor_contact;
  const bool partial_support = contact &&
      (state.left_floor_contact != state.right_floor_contact);
  const float free_fall_threshold = std::max(
      config_.free_fall_accel_threshold_m_s2, 0.0F);
  const float collision_threshold = std::max(
      config_.collision_accel_threshold_m_s2, free_fall_threshold);

  if (contact) {
    const bool confirmed_loss = support_loss_elapsed_s_ >=
        std::max(config_.support_loss_debounce_s, 0.0F);
    support_loss_elapsed_s_ = 0.0F;
    free_fall_elapsed_s_ = 0.0F;
    collision_elapsed_s_ = 0.0F;
    flight_time_s_ = 0.0F;
    downward_velocity_m_s_ = 0.0F;

    if (confirmed_loss || phase_ == Phase::TouchdownCandidate) {
      // Re-contact is a candidate until it persists. This prevents a single
      // short-step contact sample from consuming the touchdown event.
      support_settle_active_ = true;
      touchdown_elapsed_s_ += dt;
      phase_ = Phase::TouchdownCandidate;
      if (touchdown_elapsed_s_ >=
          std::max(config_.touchdown_debounce_s, 0.0F)) {
        phase_ = partial_support ? Phase::PartialSupport : Phase::Grounded;
        touchdown_elapsed_s_ = 0.0F;
      }
    } else {
      touchdown_elapsed_s_ = 0.0F;
      phase_ = partial_support ? Phase::PartialSupport : Phase::Grounded;
      if (partial_support) support_settle_active_ = true;
    }
    if (support_settle_active_) {
      support_settle_elapsed_s_ += dt;
      if (support_settle_elapsed_s_ >=
          std::max(config_.support_settle_s, 0.0F)) {
        support_settle_active_ = false;
      }
    }
    previous_contact_ = true;
    return makeOutput(partial_support);
  }

  touchdown_elapsed_s_ = 0.0F;
  if (support_settle_active_) {
    support_settle_elapsed_s_ += dt;
    if (support_settle_elapsed_s_ >=
        std::max(config_.support_settle_s, 0.0F)) {
      support_settle_active_ = false;
    }
  }
  if (previous_contact_) {
    // The timer starts at the first missing support sample, but the observer
    // does not expose a confirmed flight state until the debounce expires.
    support_loss_elapsed_s_ = dt;
    flight_time_s_ = 0.0F;
    downward_velocity_m_s_ = 0.0F;
  } else {
    support_loss_elapsed_s_ += dt;
  }
  previous_contact_ = false;

  const float accel_mag = state.imu.accel_magnitude_m_s2;
  if (accel_mag <= free_fall_threshold) {
    free_fall_elapsed_s_ += dt;
  } else {
    free_fall_elapsed_s_ = std::max(0.0F, free_fall_elapsed_s_ - 2.0F * dt);
  }
  if (accel_mag >= collision_threshold || state.imu.hard_impact_event) {
    collision_elapsed_s_ += dt;
  } else {
    collision_elapsed_s_ = std::max(0.0F, collision_elapsed_s_ - 2.0F * dt);
  }

  const bool support_lost = support_loss_elapsed_s_ >=
      std::max(config_.support_loss_debounce_s, 0.0F);
  // Propagate the relative ballistic clock from the first missing-contact
  // sample. If the gap turns out to be a short step, the contact branch above
  // resets this state before it is exposed as airborne.
  flight_time_s_ += dt;
  downward_velocity_m_s_ += std::max(config_.gravity_m_s2, 0.0F) * dt;
  if (support_lost) {
    if (flight_time_s_ >= std::max(config_.max_airborne_s, 0.0F)) {
      phase_ = Phase::NoSupportTimeout;
    } else if (collision_elapsed_s_ >=
               std::max(config_.collision_hold_s, 0.0F)) {
      phase_ = Phase::CollisionCandidate;
    } else if (free_fall_elapsed_s_ >=
               std::max(config_.free_fall_debounce_s, 0.0F)) {
      phase_ = Phase::BallisticFlight;
    } else {
      phase_ = Phase::TransientUnloading;
    }
  } else {
    phase_ = Phase::TransientUnloading;
  }
  return makeOutput(false);
}

const char* toString(FlightObserver::Phase phase) noexcept {
  switch (phase) {
    case FlightObserver::Phase::Grounded:
      return "Grounded";
    case FlightObserver::Phase::PartialSupport:
      return "PartialSupport";
    case FlightObserver::Phase::TransientUnloading:
      return "TransientUnloading";
    case FlightObserver::Phase::BallisticFlight:
      return "BallisticFlight";
    case FlightObserver::Phase::CollisionCandidate:
      return "CollisionCandidate";
    case FlightObserver::Phase::TouchdownCandidate:
      return "TouchdownCandidate";
    case FlightObserver::Phase::NoSupportTimeout:
      return "NoSupportTimeout";
  }
  return "Unknown";
}

}  // namespace robot_control
