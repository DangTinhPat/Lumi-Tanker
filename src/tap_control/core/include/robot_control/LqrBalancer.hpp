#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "robot_control/Types.hpp"

namespace robot_control {

// Reduced-order wheel balance controller. The model is the linearized wheeled
// inverted pendulum with ground acceleration as its bounded input. The output
// is the integrated wheel speed target consumed by the motor speed servo.
class LqrBalancer {
 public:
  static constexpr int kStateDim = 4;
  using StateVec = std::array<float, kStateDim>;
  using StateMatrix = std::array<StateVec, kStateDim>;

  struct Output {
    float ground_velocity_m_s = 0.0F;
    float left_wheel_velocity_rad_s = 0.0F;
    float right_wheel_velocity_rad_s = 0.0F;
    float leg_kp_scale = 1.0F;
    float leg_kd_scale = 1.0F;
    // Outstanding speed error, target minus measured. This is the signal a
    // leg task must be driven from. Acceleration is the wrong one in both
    // available forms: the commanded value is exogenous but asks for a lean
    // the wheels have not earned, and the measured value closes a POSITIVE
    // loop (offset -> moment -> pitch -> wheels accelerate -> measured
    // acceleration rises -> offset grows). Speed error closes a NEGATIVE
    // loop instead - shifting the body forward raises speed, which shrinks
    // the very error that shifted it - so the leg task becomes the outer
    // speed loop of a cascade rather than a disturbance injected into one.
    float velocity_error_m_s = 0.0F;
    bool fallen = false;
  };

  struct Config {
    float wheel_radius_m = 0.04F;
    float half_wheel_base_m = 0.1545F;
    float max_ground_velocity_m_s = 0.35F;
    float max_yaw_rate_rad_s = 0.8F;
    bool position_anchor_hold = false;
    float max_target_distance_m = 0.25F;
    float fall_pitch_rad = 0.65F;
    float leg_kp_scale = 1.0F;
    float leg_kd_scale = 1.0F;
    float turning_stiffness_boost = 0.25F;
    float stiff_yaw_rate_rad_s = 0.1F;
    // The reduced model's ground velocity is mapped to the native +Y wheel
    // command by the active robot profile. TAP's current contact convention
    // uses the same sign on both sides.
    float left_wheel_sign = 1.0F;

    float leg_length_m = 0.24F;
    float pendulum_length_per_com_height = 1.10F;
    float min_pendulum_length_m = 0.15F;
    float max_pendulum_length_m = 0.27F;
    float gravity_m_s2 = 9.81F;
    float max_ground_accel_m_s2 = 2.0F;
    float wheel_ground_mu = 1.0F;
    bool wheel_velocity_reference_from_measured = false;
    float target_velocity_slew_rate_m_s2 = 0.0F;
    // Stop-specific reference profile. Zero falls back to the normal target
    // slew/jerk values; a nonzero value lets an operator release brake faster
    // without stepping the velocity target or the LQR acceleration demand.
    float stop_velocity_slew_rate_m_s2 = 0.0F;
    float stop_velocity_jerk_m_s3 = 0.0F;
    // Jerk limit for the velocity command. A pure rate limit bounds how fast
    // the SPEED may change but lets the ACCELERATION step - so an operator
    // pressing a button steps the demanded acceleration from 0 to the slew
    // rate in one sample. Both the wheels and the leg feed-forward feel that
    // step as a jolt. Bounding the acceleration's own rate turns the profile
    // into an S-curve, which is what makes a start or stop read as smooth
    // rather than as a lurch. 0 disables it and restores the pure rate limit.
    float target_velocity_jerk_m_s3 = 0.0F;
    float target_yaw_slew_rate_rad_s2 = 0.0F;
    float yaw_rate_feedback_kp = 0.0F;
    float wheel_pitch_feedback_scale = 1.0F;
    bool use_bias_corrected_pitch_rate = false;

    // Nominal design period.  The native multi-rate boundary rebuilds no
    // matrices in the fast actuator loop; it advances the state on fresh IMU
    // packets and uses this period for the scheduled discrete model.
    float sampling_period_s = 0.002F;
    float position_cost_scale = 4.0F;
    float pitch_cost_scale = 32.0F;
    float velocity_cost_scale = 3000.0F;
    float pitch_rate_cost_scale = 4.0F;
    float input_cost_scale = 100.0F;
    unsigned riccati_iterations = 80U;

    // Station hold is a scheduled operating point, not a second controller:
    // use the same inverted-pendulum model with stronger position/velocity
    // penalties only after pitch energy is quiet.
    float station_position_cost_scale = 4.0F;
    float station_pitch_cost_scale = 32.0F;
    float station_velocity_cost_scale = 3000.0F;
    float station_pitch_rate_cost_scale = 4.0F;
    float station_input_cost_scale = 100.0F;
    float station_gain_pitch_start_rad = 0.08F;
    float station_gain_pitch_quiet_rad = 0.02F;
    float station_gain_rate_start_rad_s = 0.60F;
    float station_gain_rate_quiet_rad_s = 0.15F;
    float station_gain_entry_dwell_s = 0.25F;
    float station_gain_transition_s = 0.15F;
    float station_pitch_return_gain_rad_per_m_s = 0.0F;
    float station_pitch_return_max_rad = 0.0F;

    float station_hold_gain_hz = 0.0F;
    bool station_hold_yaw_return = false;
    float station_hold_max_v = 0.10F;
    float station_posture_hold_max_v = 0.0F;
    float station_hold_command_max_v = 0.0F;
    float station_hold_dynamic_max_v = 0.0F;
    float station_hold_command_slew_rate_m_s2 = 0.0F;
    float station_hold_velocity_damping_gain = 0.0F;
    float station_adaptive_full_capture_distance_m = 0.0F;
    float station_quiet_gain_scale = 1.0F;
    float station_capture_gain = 0.0F;
    float station_hold_velocity_servo_gain_hz = 0.0F;
    bool hold_station_during_pose = false;
    float station_velocity_decay_gain_hz = 0.0F;

    // Adaptive balance angle (hardware bring-up). The measured equilibrium of
    // a real robot differs from the model's pitch reference (IMU mounting
    // tilt, CAD mass error, joint zero error). With that offset the pitch +
    // velocity loop's only steady state is rolling at constant speed while
    // leaning at the true equilibrium. An integral of the wheel speed error
    // moves the reference to the true equilibrium instead - the same job as
    // the velocity loop's position integral in a cascaded PID balancer.
    // Units: rad of reference per metre of accumulated speed error. 0 = off.
    float balance_trim_gain_rad_per_m = 0.0F;
    float balance_trim_max_rad = 0.15F;
    // Only adapt while the body is near the (trimmed) reference, not while a
    // fall or a large transient is in progress.
    float balance_trim_pitch_gate_rad = 0.25F;
    // Anti-windup: keep the integrated wheel speed command within this band
    // of the measured ground speed. A wheel that cannot realize the command
    // (torque limit, stall, a held body) otherwise lets the command ramp to
    // max_ground_velocity_m_s. 0 = off.
    float command_velocity_tracking_margin_m_s = 0.0F;
  };

  struct DebugState {
    float commanded_ground_velocity_m_s = 0.0F;
    float lqr_velocity_reference_m_s = 0.0F;
    float station_velocity_target_m_s = 0.0F;
    float target_ground_position_m = 0.0F;
    float optimal_command_m_s = 0.0F;
    float balance_pitch_ref_rad = 0.0F;
    float lqr_pitch_ref_rad = 0.0F;
    float lqr_pitch_state_rad = 0.0F;
    float effective_pendulum_length_m = 0.0F;
    float adaptive_gain_scale = 1.0F;
    float station_gain_blend = 0.0F;
    float capture_distance_m = 0.0F;
    bool station_hold_active = false;
    unsigned riccati_iterations = 0U;
    float leg_velocity_error_m_s = 0.0F;
    float balance_trim_rad = 0.0F;
  };

  LqrBalancer();
  explicit LqrBalancer(Config config);

  void reset(float ground_position_m = 0.0F, bool floor_contact = true);
  void reanchorPosition(float ground_position_m) noexcept;
  void movePositionAnchorToward(float ground_position_m, float alpha) noexcept;
  void clearForwardVelocityReference() noexcept;
  void setStationVelocityReference(float velocity_m_s) noexcept;

  [[nodiscard]] Output update(const RobotState& state,
                              const ControlCommand& command,
                              bool integrate = true,
                              float balance_pitch_ref_rad = 0.0F,
                              bool hold_station = false);
  [[nodiscard]] const DebugState& debugState() const noexcept {
    return last_debug_;
  }

  void setComHeight(float com_height_m);
  void setStationPositionMode(bool high_gain) noexcept {
    station_position_high_gain_ = high_gain;
  }
  void setPositionAnchorHold(bool enabled) noexcept {
    config_.position_anchor_hold = enabled;
  }
  void setStationVelocityDecayEnabled(bool enabled) noexcept {
    station_velocity_decay_enabled_ = enabled;
  }
  void setPostureMotionActive(bool active) noexcept {
    posture_motion_active_ = active;
  }
  void setWheelPitchFeedbackScale(float scale) noexcept {
    config_.wheel_pitch_feedback_scale = std::clamp(scale, 0.0F, 1.0F);
  }
  void setFallPitchRad(float fall_pitch_rad) noexcept {
    config_.fall_pitch_rad = fall_pitch_rad;
  }
  [[nodiscard]] float fallPitchRad() const noexcept {
    return config_.fall_pitch_rad;
  }
  [[nodiscard]] float activePendulumLengthM() const noexcept {
    return active_length_m_;
  }

  // Adaptive balance angle state (see Config::balance_trim_gain_rad_per_m).
  // The learned value is a property of the robot and survives reset().
  void setBalanceTrim(float trim_rad) noexcept {
    balance_trim_rad_ = std::isfinite(trim_rad)
        ? std::clamp(trim_rad, -std::abs(config_.balance_trim_max_rad),
                     std::abs(config_.balance_trim_max_rad))
        : 0.0F;
  }
  [[nodiscard]] float balanceTrim() const noexcept { return balance_trim_rad_; }
  // Runtime tuning hooks for hardware bring-up.
  void setBalanceTrimGain(float gain) noexcept {
    config_.balance_trim_gain_rad_per_m = std::isfinite(gain) ? std::max(gain, 0.0F) : 0.0F;
  }
  void setBalanceTrimMax(float max_rad) noexcept {
    config_.balance_trim_max_rad = std::isfinite(max_rad) ? std::abs(max_rad) : 0.0F;
    setBalanceTrim(balance_trim_rad_);
  }
  void setCommandTrackingMargin(float margin_m_s) noexcept {
    config_.command_velocity_tracking_margin_m_s =
        std::isfinite(margin_m_s) ? std::max(margin_m_s, 0.0F) : 0.0F;
  }
  void setMaxGroundAccel(float accel_m_s2) noexcept {
    if (std::isfinite(accel_m_s2) && accel_m_s2 > 0.0F) {
      config_.max_ground_accel_m_s2 = accel_m_s2;
    }
  }
  [[nodiscard]] const Config& config() const noexcept { return config_; }

  // Embedded gain schedule. The DARE solve runs in double precision, which an
  // ESP32-S3 (single-precision FPU only) takes ~50 ms to finish - far over the
  // 2 ms control period, and it reruns whenever the COM height moves 2 mm.
  // A target can precompute the gains over [min, max] pendulum length before
  // it enables, one point per call so the caller can yield between points;
  // afterwards every rebuild is a bounded linear interpolation. Callers that
  // never build the table (the native simulator) keep the exact online solve.
  static constexpr unsigned kMaxGainTablePoints = 64U;
  bool buildGainTableStep(unsigned points);
  [[nodiscard]] bool gainTableReady() const noexcept { return table_ready_; }
  [[nodiscard]] unsigned onlineSolveCount() const noexcept { return online_solves_; }
  [[nodiscard]] const StateVec& activeGain() const noexcept { return gain_; }
  [[nodiscard]] const StateVec& activeStationGain() const noexcept {
    return station_gain_;
  }

 private:
  using GainVec = StateVec;

  void rebuildGain();
  void solveGainsAt(float length, GainVec& gain, GainVec& station_gain,
                    GainVec& posture_station_gain) const;
  [[nodiscard]] float solveGroundAcceleration(
      const StateVec& state, float station_gain_blend) const noexcept;
  [[nodiscard]] float stationGainBlend(float pitch_error, float pitch_rate,
                                       bool hold_station, float dt_s) noexcept;
  [[nodiscard]] float adaptiveStationGain(float capture_distance) const noexcept;
  [[nodiscard]] float stationReturnVelocity(const RobotState& state,
                                             float pitch_error,
                                             float pitch_rate,
                                             bool hold_station,
                                             float forward_target,
                                             float& capture_distance) const noexcept;
  [[nodiscard]] static float clampFinite(float value, float lo, float hi) noexcept;

  Config config_{};
  float active_length_m_ = 0.24F;
  float balance_trim_rad_ = 0.0F;
  GainVec gain_{};
  GainVec station_gain_{};
  GainVec posture_station_gain_{};
  std::array<GainVec, kMaxGainTablePoints> table_gain_{};
  std::array<GainVec, kMaxGainTablePoints> table_station_gain_{};
  std::array<GainVec, kMaxGainTablePoints> table_posture_station_gain_{};
  unsigned table_points_ = 0U;
  unsigned table_built_ = 0U;
  bool table_ready_ = false;
  unsigned online_solves_ = 0U;
  float commanded_ground_velocity_m_s_ = 0.0F;
  float station_gain_quiet_elapsed_s_ = 0.0F;
  float station_gain_blend_ = 0.0F;
  bool station_gain_active_ = false;
  float target_forward_velocity_m_s_ = 0.0F;
  float station_velocity_reference_m_s_ = 0.0F;
  float target_yaw_rate_rad_s_ = 0.0F;
  float command_accel_m_s2_ = 0.0F;
  bool station_velocity_decay_enabled_ = true;
  bool station_position_high_gain_ = false;
  bool posture_motion_active_ = false;
  float ground_position_target_m_ = 0.0F;
  float station_origin_position_m_ = 0.0F;
  DebugState last_debug_{};
};

}  // namespace robot_control
