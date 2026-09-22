#include "robot_control/FlightObserver.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {
namespace {

float clamp01(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}

float minLegHeightRate(const RobotState& state) {
  const auto left = WholeBodyModel::legHeight(
      state.joints.position_rad[static_cast<std::size_t>(JointId::LeftHip)],
      state.joints.position_rad[static_cast<std::size_t>(JointId::LeftKnee)]);
  const auto right = WholeBodyModel::legHeight(
      state.joints.position_rad[static_cast<std::size_t>(JointId::RightHip)],
      state.joints.position_rad[static_cast<std::size_t>(JointId::RightKnee)]);
  const float left_rate =
      left.d_height_d_hip *
          state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::LeftHip)] +
      left.d_height_d_knee *
          state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::LeftKnee)];
  const float right_rate =
      right.d_height_d_hip *
          state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::RightHip)] +
      right.d_height_d_knee *
          state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::RightKnee)];
  return std::min(left_rate, right_rate);
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
  support_contact_ = true;
  sensor_touchdown_candidate_ = false;
  sensor_free_fall_elapsed_s_ = 0.0F;
  sensor_collision_elapsed_s_ = 0.0F;
  sensor_touchdown_elapsed_s_ = 0.0F;
  support_recovery_elapsed_s_ = 0.0F;
  support_recovery_armed_ = false;
  previous_min_leg_height_rate_m_s_ = 0.0F;
  encoder_touchdown_elapsed_s_ = 0.0F;
  encoder_touchdown_evidence_ = false;
  torque_corroboration_used_ = false;
  last_imu_sample_sequence_ = 0;
  last_encoder_sample_sequence_ = 0;
}

void FlightObserver::reset(const RobotState& state) {
  reset();
  if (!finiteInput(state)) return;
  previous_min_leg_height_rate_m_s_ = minLegHeightRate(state);
}

bool FlightObserver::finiteInput(const RobotState& state) const noexcept {
  if (!(std::isfinite(state.dt_s) && state.dt_s > 0.0F &&
         std::isfinite(state.imu.accel_magnitude_m_s2) &&
         std::isfinite(state.imu.pitch_rate_rad_s) &&
         std::isfinite(state.wheel.left_velocity_m_s) &&
         std::isfinite(state.wheel.right_velocity_m_s))) {
    return false;
  }
  // Encoder-derived compression/reversal is part of the sensor contract.
  // Reject the whole observation if any leg channel is invalid rather than
  // allowing a NaN to pass through WholeBodyModel and fabricate touchdown.
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(state.joints.position_rad[i]) ||
        !std::isfinite(state.joints.velocity_rad_s[i])) {
      return false;
    }
  }
  return true;
}

bool FlightObserver::encoderTouchdownEvidence(
    const RobotState& state) const noexcept {
  if (!config_.encoder_touchdown_enabled ||
      sensor_free_fall_elapsed_s_ <
          std::max(config_.free_fall_debounce_s, 0.0F) ||
      state.imu.accel_magnitude_m_s2 <=
          std::max(config_.free_fall_accel_threshold_m_s2, 0.0F)) {
    return false;
  }

  const float current_min_rate = minLegHeightRate(state);
  const float compression_threshold = std::max(
      config_.encoder_compression_rate_threshold_m_s, 0.0F);
  const float reversal_threshold = std::max(
      config_.encoder_compression_reversal_threshold_m_s, 0.0F);
  // At least one leg must have been compressing in the previous sample and
  // must now have measurably slowed/reversed.  This rejects a steady
  // commanded airborne extension, while accepting an impact before the IMU
  // spike has satisfied the older acceleration-only gate.
  return previous_min_leg_height_rate_m_s_ <= -compression_threshold &&
      current_min_rate - previous_min_leg_height_rate_m_s_ >= reversal_threshold;
}

void FlightObserver::updateSupportHypothesis(
    const RobotState& state, float dt,
    const TorqueResidualEstimator::Output* torque) noexcept {
  torque_corroboration_used_ = false;
  // Rule 1 condition #1 ("never the sole gate"): agreement is only ever
  // consulted below to shorten a debounce that IMU evidence has already
  // begun (guarded by "elapsed_s_ > 0.0F" at each use site) -- it is never
  // consulted to decide free_fall/impact/collision themselves.
  const bool torque_agrees_unloading =
      torque != nullptr && config_.torque_corroboration_enabled &&
      torque->valid &&
      torque->unloading_confidence >=
          clamp01(config_.torque_agreement_confidence_threshold) &&
      torque->direction_consistent_with_unloading;
  const bool torque_agrees_impact =
      torque != nullptr && config_.torque_corroboration_enabled &&
      torque->valid &&
      torque->impact_confidence >=
          clamp01(config_.torque_agreement_confidence_threshold) &&
      torque->direction_consistent_with_impact;
  const float speedup_multiplier =
      std::clamp(config_.torque_corroboration_speedup_multiplier,
                 1.0F, 4.0F);

  const float free_fall_threshold = std::max(
      config_.free_fall_accel_threshold_m_s2, 0.0F);
  const float collision_threshold = std::max(
      config_.collision_accel_threshold_m_s2, free_fall_threshold);
  const float accel = state.imu.accel_magnitude_m_s2;
  const bool free_fall = accel <= free_fall_threshold;
  const bool impact = accel >= collision_threshold || state.imu.hard_impact_event;
  const bool supported_specific_force =
      !free_fall &&
      accel >= std::max(config_.support_recovery_accel_min_m_s2, 0.0F) &&
      accel <= std::max(config_.support_recovery_accel_max_m_s2,
                        config_.support_recovery_accel_min_m_s2);
  encoder_touchdown_evidence_ = encoderTouchdownEvidence(state);
  if (encoder_touchdown_evidence_) {
    // This is an event latch, not a requirement that the velocity reversal
    // remain visible for several samples.  The impact can stop the joint in a
    // single encoder sample; keep the corroboration alive long enough for the
    // normal collision debounce to consume it.
    encoder_touchdown_elapsed_s_ = std::max(
        encoder_touchdown_elapsed_s_,
        std::max(config_.encoder_touchdown_hold_s, dt));
  } else {
    encoder_touchdown_elapsed_s_ = std::max(
        0.0F, encoder_touchdown_elapsed_s_ - dt);
  }
  const bool encoder_collision = encoder_touchdown_elapsed_s_ > 0.0F;

  if (free_fall) {
    float effective_dt = dt;
    if (torque_agrees_unloading && sensor_free_fall_elapsed_s_ > 0.0F) {
      // Already progressing from IMU evidence alone -- torque may only
      // shorten the remaining time, never manufacture this from zero.
      effective_dt = dt * speedup_multiplier;
      torque_corroboration_used_ = true;
    }
    sensor_free_fall_elapsed_s_ += effective_dt;
  } else {
    sensor_free_fall_elapsed_s_ = std::max(
        0.0F, sensor_free_fall_elapsed_s_ - 2.0F * dt);
  }
  if ((impact || encoder_collision) && !support_contact_) {
    float effective_dt = dt;
    if (torque_agrees_impact && sensor_collision_elapsed_s_ > 0.0F) {
      effective_dt = dt * speedup_multiplier;
      torque_corroboration_used_ = true;
    }
    sensor_collision_elapsed_s_ += effective_dt;
  } else if (support_contact_) {
    sensor_collision_elapsed_s_ = 0.0F;
  } else {
    sensor_collision_elapsed_s_ = std::max(
        0.0F, sensor_collision_elapsed_s_ - 2.0F * dt);
  }

  if (support_contact_) {
    support_recovery_elapsed_s_ = 0.0F;
  } else if (support_recovery_armed_ && supported_specific_force) {
    support_recovery_elapsed_s_ += dt;
  } else {
    support_recovery_elapsed_s_ = 0.0F;
  }

  if (support_contact_) {
    sensor_touchdown_candidate_ = false;
    sensor_touchdown_elapsed_s_ = 0.0F;
    if (sensor_free_fall_elapsed_s_ >=
        std::max(config_.free_fall_debounce_s, 0.0F)) {
      // Sustained near-zero specific force is the available support-loss
      // signal under the MPU6500 + encoder hardware contract.
      support_contact_ = false;
      support_recovery_armed_ = true;
      sensor_collision_elapsed_s_ = 0.0F;
    }
    previous_min_leg_height_rate_m_s_ = minLegHeightRate(state);
    return;
  }

  if (support_recovery_armed_ &&
      support_recovery_elapsed_s_ >=
          std::max(config_.support_recovery_hold_s, 0.0F) &&
      supported_specific_force && !impact) {
    // Recover support from the sustained specific-force/load hypothesis. This
    // is sensor-only and deliberately bounded by the recovery band; it does
    // not use MuJoCo contacts or absolute pose. The normal touchdown debounce
    // below still protects the state machine from a single recovered sample.
    support_contact_ = true;
    support_recovery_armed_ = false;
    support_recovery_elapsed_s_ = 0.0F;
    sensor_touchdown_candidate_ = false;
    sensor_free_fall_elapsed_s_ = 0.0F;
    sensor_collision_elapsed_s_ = 0.0F;
    sensor_touchdown_elapsed_s_ = 0.0F;
    previous_min_leg_height_rate_m_s_ = minLegHeightRate(state);
    return;
  }

  if (!sensor_touchdown_candidate_ &&
      sensor_collision_elapsed_s_ >=
          std::max(config_.collision_hold_s, 0.0F)) {
    sensor_touchdown_candidate_ = true;
    sensor_touchdown_elapsed_s_ = 0.0F;
  }
  if (sensor_touchdown_candidate_) {
    if (!free_fall && accel > free_fall_threshold) {
      sensor_touchdown_elapsed_s_ += dt;
    } else {
      sensor_touchdown_elapsed_s_ = 0.0F;
    }
    if (sensor_touchdown_elapsed_s_ >=
        std::max(config_.touchdown_debounce_s, 0.0F)) {
      // Re-contact hypothesis from IMU impact followed by non-ballistic
      // specific force; this is not a measured wheel contact.
      support_contact_ = true;
      sensor_touchdown_candidate_ = false;
      sensor_free_fall_elapsed_s_ = 0.0F;
      sensor_collision_elapsed_s_ = 0.0F;
      sensor_touchdown_elapsed_s_ = 0.0F;
    }
  }

  // Keep the encoder derivative memory live through flight so the next
  // sample can detect a compression-rate reversal.  The observer never
  // integrates this into height or world pose; it is only an event feature.
  previous_min_leg_height_rate_m_s_ = minLegHeightRate(state);
}

FlightObserver::Output FlightObserver::makeOutput(bool partial_support) const {
  Output output;
  output.phase = phase_;
  output.partial_support = partial_support;
  output.support_settling = support_settle_active_ &&
      support_settle_elapsed_s_ < std::max(config_.support_settle_s, 0.0F);
  output.support_lost = support_loss_elapsed_s_ >=
                        std::max(config_.support_loss_debounce_s, 0.0F);
  const float free_fall_debounce =
      std::max(config_.free_fall_debounce_s, 0.0F);
  const float unloading_threshold = free_fall_debounce * std::clamp(
      config_.support_unloading_confidence_threshold, 0.0F, 1.0F);
  output.support_unloading = support_contact_ &&
      sensor_free_fall_elapsed_s_ >= unloading_threshold &&
      (unloading_threshold > 0.0F || sensor_free_fall_elapsed_s_ > 0.0F);
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
  output.encoder_touchdown_evidence = encoder_touchdown_evidence_;
  output.support_contact = support_contact_;
  // Per-wheel support is not observable without excluded contact switches.
  output.left_support_contact = support_contact_;
  output.right_support_contact = support_contact_;
  output.support_loss_elapsed_s = support_loss_elapsed_s_;
  output.flight_time_s = flight_time_s_;
  output.downward_velocity_m_s = downward_velocity_m_s_;
  output.free_fall_confidence = clamp01(
      free_fall_elapsed_s_ / std::max(config_.free_fall_debounce_s, 1.0e-4F));
  output.collision_confidence = clamp01(
      collision_elapsed_s_ / std::max(config_.collision_hold_s, 1.0e-4F));
  output.torque_corroboration_used = torque_corroboration_used_;
  return output;
}

FlightObserver::Output FlightObserver::update(
    const RobotState& state, const TorqueResidualEstimator::Output* torque) {
  if (!finiteInput(state)) {
    return makeOutput(false);
  }

  const float dt = std::clamp(state.dt_s, 1.0e-5F, 0.05F);
  const bool imu_fresh = state.imu_sample_sequence == 0 ||
      state.imu_sample_sequence != last_imu_sample_sequence_;
  const bool encoder_fresh = state.encoder_sample_sequence == 0 ||
      state.encoder_sample_sequence != last_encoder_sample_sequence_;
  if (state.imu_sample_sequence != 0) {
    last_imu_sample_sequence_ = state.imu_sample_sequence;
  }
  if (state.encoder_sample_sequence != 0) {
    last_encoder_sample_sequence_ = state.encoder_sample_sequence;
  }
  // CMF evidence arrives at the sensor rate, while MPC may run at 3-4 kHz.
  // Run the observer's elapsed-time gates only on a fresh relevant packet;
  // repeated control ticks still publish the last hypothesis but cannot
  // manufacture a faster free-fall/touchdown transition.
  const float sensor_dt = state.imu.sample_dt_s > 0.0F &&
          std::isfinite(state.imu.sample_dt_s)
      ? std::clamp(state.imu.sample_dt_s, 1.0e-5F, 0.05F)
      : dt;
  const float observation_dt = (imu_fresh || encoder_fresh) ? sensor_dt : 0.0F;
  updateSupportHypothesis(state, observation_dt, torque);
  const bool contact = support_contact_;
  const bool partial_support = false;

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
      touchdown_elapsed_s_ += observation_dt;
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
      support_settle_elapsed_s_ += observation_dt;
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
    support_settle_elapsed_s_ += observation_dt;
    if (support_settle_elapsed_s_ >=
        std::max(config_.support_settle_s, 0.0F)) {
      support_settle_active_ = false;
    }
  }
  if (previous_contact_) {
    // The timer starts at the first missing support sample, but the observer
    // does not expose a confirmed flight state until the debounce expires.
    support_loss_elapsed_s_ = observation_dt;
    flight_time_s_ = 0.0F;
    downward_velocity_m_s_ = 0.0F;
  } else {
    support_loss_elapsed_s_ += observation_dt;
  }
  previous_contact_ = false;

  free_fall_elapsed_s_ = sensor_free_fall_elapsed_s_;
  collision_elapsed_s_ = sensor_collision_elapsed_s_;

  const bool support_lost = support_loss_elapsed_s_ >=
      std::max(config_.support_loss_debounce_s, 0.0F);
  // Propagate the relative ballistic clock from the first missing-contact
  // sample. If the gap turns out to be a short step, the contact branch above
  // resets this state before it is exposed as airborne.
  flight_time_s_ += observation_dt;
  downward_velocity_m_s_ += std::max(config_.gravity_m_s2, 0.0F) * observation_dt;
  if (support_lost) {
    if (flight_time_s_ >= std::max(config_.max_airborne_s, 0.0F)) {
      phase_ = Phase::NoSupportTimeout;
    } else if (collision_elapsed_s_ >=
               std::max(config_.collision_hold_s, 0.0F)) {
      phase_ = Phase::CollisionCandidate;
    } else if (sensor_touchdown_candidate_ &&
               state.imu.accel_magnitude_m_s2 >
                   std::max(config_.free_fall_accel_threshold_m_s2, 0.0F)) {
      // The collision hold is an edge detector.  Once it has latched a
      // touchdown candidate, a normal 1g sample may legitimately make
      // sensor_collision_elapsed_s_ decay below collision_hold_s before the
      // touchdown debounce completes.  Do not relabel that latched candidate
      // as BallisticFlight in the meantime.
      //
      // But a latched candidate is only a hypothesis, and touchdown and free
      // fall are mutually exclusive: a robot reading near-zero specific force
      // is plainly still falling, whatever edge latched earlier. Compliant
      // legs make this reachable - they snap outward as the load leaves at
      // lift-off, and that jolt clears the collision threshold while the
      // robot is unambiguously airborne. Holding the stale label there hid
      // BallisticFlight for the whole descent. Defer to the live specific
      // force; the latch survives and still resolves through its own
      // debounce once real support returns.
      phase_ = Phase::TouchdownCandidate;
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
