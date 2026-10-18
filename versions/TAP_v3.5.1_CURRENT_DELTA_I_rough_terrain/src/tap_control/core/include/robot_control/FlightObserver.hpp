#pragma once

#include "robot_control/TorqueResidualEstimator.hpp"
#include "robot_control/Types.hpp"

namespace robot_control {

// Sensor-backed support/flight observer for wheel-legged motion. It does not
// consume RobotState::floor_contact as an input: that field is simulator
// telemetry only. Support is hypothesized from MPU6500
// specific-force/gyro events; absolute terrain height is unobservable with
// MPU6500 + encoders.
class FlightObserver {
 public:
  enum class Phase {
    Grounded,
    PartialSupport,
    TransientUnloading,
    BallisticFlight,
    CollisionCandidate,
    TouchdownCandidate,
    NoSupportTimeout,
  };

  struct Config {
    float gravity_m_s2 = 9.81F;
    // A short contact gap below this duration is treated as transient
    // unloading, which prevents a staircase from becoming repeated landings.
    float support_loss_debounce_s = 0.012F;
    float free_fall_accel_threshold_m_s2 = 3.5F;
    float free_fall_debounce_s = 0.012F;
    // Early unloading is useful to pre-arm the landing reflex, but a single
    // low-specific-force sample is not enough evidence.  Require this
    // fraction of the free-fall debounce before exposing support_unloading.
    // Clamp to [0, 1]; 0 preserves immediate exposure and 1 waits for the
    // same duration as confirmed sensor free-fall.
    float support_unloading_confidence_threshold = 0.5F;
    float collision_accel_threshold_m_s2 = 25.0F;
    float collision_hold_s = 0.02F;
    // A height transition can briefly unload the IMU without making the
    // robot airborne.  Once a debounced sensor loss has occurred, sustained
    // non-ballistic specific force in this band is sufficient to reacquire
    // support; waiting for an impact spike would deadlock a supported robot.
    float support_recovery_accel_min_m_s2 = 7.5F;
    float support_recovery_accel_max_m_s2 = 18.0F;
    float support_recovery_hold_s = 0.020F;
    // Encoder-only touchdown corroboration.  A real touchdown usually makes
    // at least one leg compress rapidly and then decelerate/rebound.  This is
    // deliberately a corroborating hypothesis, never a contact measurement:
    // it is gated by a completed IMU free-fall interval and by non-ballistic
    // specific force so commanded airborne leg motion cannot create contact
    // on its own.
    bool encoder_touchdown_enabled = true;
    float encoder_compression_rate_threshold_m_s = 0.10F;
    float encoder_compression_reversal_threshold_m_s = 0.04F;
    float encoder_touchdown_hold_s = 0.004F;
    float touchdown_debounce_s = 0.015F;
    // A one-wheel touchdown is usable support for the landing controller,
    // but the state machine must not declare a fall during the short
    // bilateral-support settling interval required by the landing contract.
    float support_settle_s = 0.10F;
    // Bound the time for which a no-contact robot is considered recoverable
    // flight. A robot resting on its body with no wheel contact must
    // eventually be allowed to enter the normal fall/recovery path.
    float max_airborne_s = 0.75F;

    // Leg motor current-based torque corroboration (docs/RULES.md Rule 1's
    // 2026-09-18 exception). Disabled by default -- an exact no-op even if a
    // non-null TorqueResidualEstimator::Output is passed to update(). When
    // enabled, torque agreement may only SHORTEN the time-to-threshold of an
    // already-independently-progressing IMU debounce (sensor_free_fall_
    // elapsed_s_/sensor_collision_elapsed_s_ in FlightObserver.cpp); it can
    // never start, sustain alone, lengthen, or block a transition -- see
    // FlightObserver.cpp's updateSupportHypothesis() for the exact gate.
    bool torque_corroboration_enabled = false;
    // torque_confidence must reach this before it counts as agreement.
    // Defaults to 1.0 (the maximum possible clamp01 value), an intentionally
    // maximal bar that keeps this feature inert until deliberately tuned.
    float torque_agreement_confidence_threshold = 1.0F;
    // Multiplies the debounce accumulator's dt increment when torque agrees.
    // Must stay >= 1.0 (never lengthens); 1.0 is an exact no-op even if
    // torque_corroboration_enabled is mistakenly left true.
    float torque_corroboration_speedup_multiplier = 1.0F;
  };

  struct Output {
    Phase phase = Phase::Grounded;
    bool support_lost = false;
    // Early MPU6500 unloading evidence. This is exposed before the full
    // free-fall debounce so a landing trajectory can pre-arm without treating
    // a short unloading gap as confirmed touchdown/support loss.
    bool support_unloading = false;
    bool partial_support = false;
    bool support_settling = false;
    bool airborne = false;
    bool ballistic = false;
    bool collision_candidate = false;
    bool touchdown_candidate = false;
    bool encoder_touchdown_evidence = false;
    // Sensor-only support hypothesis consumed by EmbeddedController. This is
    // not a measured contact flag or simulator ground truth.
    bool support_contact = true;
    bool left_support_contact = true;
    bool right_support_contact = true;
    float support_loss_elapsed_s = 0.0F;
    float flight_time_s = 0.0F;
    // Positive means downward. This is a ballistic estimate initialized from
    // zero vertical speed at support loss; it is not an absolute-height
    // estimate and must not be treated as one.
    float downward_velocity_m_s = 0.0F;
    float free_fall_confidence = 0.0F;
    float collision_confidence = 0.0F;
    // Evidence-labeling per docs/RULES.md Rule 1 condition #5: true only when
    // torque corroboration actually shortened a debounce this tick, so a
    // report can state plainly whether this signal contributed.
    bool torque_corroboration_used = false;
  };

  FlightObserver();
  explicit FlightObserver(Config config) : config_(config) {}

  void reset();
  void reset(const RobotState& state);

  [[nodiscard]] Output update(
      const RobotState& state,
      const TorqueResidualEstimator::Output* torque = nullptr);
  [[nodiscard]] Phase phase() const noexcept { return phase_; }

 private:
  [[nodiscard]] Output makeOutput(bool partial_support) const;
  [[nodiscard]] bool finiteInput(const RobotState& state) const noexcept;
  void updateSupportHypothesis(
      const RobotState& state, float dt,
      const TorqueResidualEstimator::Output* torque,
      float current_min_leg_height_rate_m_s) noexcept;
  [[nodiscard]] bool encoderTouchdownEvidence(
      const RobotState& state, float current_min_leg_height_rate_m_s) const
      noexcept;

  Config config_;
  Phase phase_ = Phase::Grounded;
  bool previous_contact_ = true;
  float support_loss_elapsed_s_ = 0.0F;
  float flight_time_s_ = 0.0F;
  float free_fall_elapsed_s_ = 0.0F;
  float collision_elapsed_s_ = 0.0F;
  float touchdown_elapsed_s_ = 0.0F;
  float support_settle_elapsed_s_ = 0.0F;
  bool support_settle_active_ = false;
  float downward_velocity_m_s_ = 0.0F;
  bool support_contact_ = true;
  bool sensor_touchdown_candidate_ = false;
  float sensor_free_fall_elapsed_s_ = 0.0F;
  float sensor_collision_elapsed_s_ = 0.0F;
  float sensor_touchdown_elapsed_s_ = 0.0F;
  float support_recovery_elapsed_s_ = 0.0F;
  bool support_recovery_armed_ = false;
  float previous_min_leg_height_rate_m_s_ = 0.0F;
  float encoder_touchdown_elapsed_s_ = 0.0F;
  bool encoder_touchdown_evidence_ = false;
  bool torque_corroboration_used_ = false;
  std::uint64_t last_imu_sample_sequence_ = 0;
  std::uint64_t last_encoder_sample_sequence_ = 0;
};

[[nodiscard]] const char* toString(FlightObserver::Phase phase) noexcept;

}  // namespace robot_control
