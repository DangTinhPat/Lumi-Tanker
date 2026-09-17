#pragma once

#include "robot_control/Types.hpp"

namespace robot_control {

// Sensor-backed support/flight observer for wheel-legged motion. It does not
// estimate absolute terrain height: with IMU + encoders that quantity is not
// observable during flight. Its job is to separate brief unloading over a
// short step, true loss of support, edge collision, and re-contact.
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
    float collision_accel_threshold_m_s2 = 25.0F;
    float collision_hold_s = 0.02F;
    float touchdown_debounce_s = 0.015F;
    // A one-wheel touchdown is usable support for the landing controller,
    // but the state machine must not declare a fall during the short
    // bilateral-support settling interval required by the landing contract.
    float support_settle_s = 0.10F;
    // Bound the time for which a no-contact robot is considered recoverable
    // flight. A robot resting on its body with no wheel contact must
    // eventually be allowed to enter the normal fall/recovery path.
    float max_airborne_s = 0.75F;
  };

  struct Output {
    Phase phase = Phase::Grounded;
    bool support_lost = false;
    bool partial_support = false;
    bool support_settling = false;
    bool airborne = false;
    bool ballistic = false;
    bool collision_candidate = false;
    bool touchdown_candidate = false;
    float support_loss_elapsed_s = 0.0F;
    float flight_time_s = 0.0F;
    // Positive means downward. This is a ballistic estimate initialized from
    // zero vertical speed at support loss; it is not an absolute-height
    // estimate and must not be treated as one.
    float downward_velocity_m_s = 0.0F;
    float free_fall_confidence = 0.0F;
    float collision_confidence = 0.0F;
  };

  FlightObserver();
  explicit FlightObserver(Config config) : config_(config) {}

  void reset();
  void reset(const RobotState& state);

  [[nodiscard]] Output update(const RobotState& state);
  [[nodiscard]] Phase phase() const noexcept { return phase_; }

 private:
  [[nodiscard]] Output makeOutput(bool partial_support) const;
  [[nodiscard]] bool finiteInput(const RobotState& state) const noexcept;

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
};

[[nodiscard]] const char* toString(FlightObserver::Phase phase) noexcept;

}  // namespace robot_control
