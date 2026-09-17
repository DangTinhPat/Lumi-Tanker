#pragma once

#include "robot_control/Types.hpp"
#include "robot_control/FlightObserver.hpp"

namespace robot_control {

// Phase-structured landing controller for the ledge/fall interval.
//
// This is intentionally sensor-backed only: phase transitions use wheel
// sensor-derived support hypothesis, IMU acceleration magnitude, IMU pitch
// rate, joint encoders and wheel odometry. No simulator pose/velocity or raw
// simulator contact is accepted here.
class LandingController {
 public:
  enum class Phase { Grounded, Flight, Impact, Recovery };

  struct Config {
    bool enabled = false;

    // Flight: MPC wheel balance has no ground authority. Shape the legs toward
    // a touchdown posture by extending in task space after a short edge-clear
    // delay. Limits are in joint space to keep the command bounded.
    float flight_gravity_m_s2 = 9.81F;
    float flight_extension_delay_s = 0.08F;
    float flight_extension_max_m = 0.035F;
    float flight_hip_offset_max_rad = 0.04F;
    float flight_knee_offset_max_rad = 0.25F;
    float flight_target_slew_rate_rad_s = 2.0F;
    float flight_kp_scale = 0.5F;
    float flight_kd_scale = 1.0F;
    // Airborne attitude shaping: a bounded symmetric hip placement from live
    // pitch rate. Positive pitch-rate in TAP ledge traces maps to a negative
    // hip offset, moving the future wheel contact forward under the falling
    // body. 0 gain is inert.
    float flight_pitch_rate_hip_gain_rad_per_rad_s = 0.0F;
    float flight_pitch_rate_hip_offset_max_rad = 0.0F;

    // Impact detection is gated by the FlightObserver's IMU-derived support
    // hypothesis. A shock by itself is not enough: a body/edge strike must
    // not make the controller apply a ground reference while unsupported.
    float min_flight_before_impact_s = 0.03F;
    float impact_accel_mps2 = 35.0F;
    float impact_pitch_rate_rad_s = 4.0F;
    float imu_impact_debounce_s = 0.02F;
    // Cartesian impedance in sagittal leg coordinates during impact:
    // F = K(x_ref - x) + D(v_ref - v), tau = J^T F.
    // Height is the dominant shock axis; x is optional and should stay small
    // for TAP because hip authority can inject pitch disturbances.
    float height_kp_n_m = 0.0F;
    float height_kd_n_s_m = 0.0F;
    float x_kp_n_m = 0.0F;
    float x_kd_n_s_m = 0.0F;
    float max_virtual_force_n = 40.0F;
    float max_feedforward_torque_nm = 3.0F;
    float impact_kp_scale = 0.05F;
    float impact_kd_scale = 1.0F;

    // Recovery starts when impact motion has decayed or this cap is reached,
    // then blends authority back to the normal balance stack.
    float min_impact_s = 0.04F;
    float max_impact_s = 0.35F;
    float recovery_blend_s = 0.25F;
    // After validated touchdown, ramp the requested drive velocity back in
    // instead of handing the pre-edge command to the wheels in one tick.
    // Zero preserves the historical behavior. This is a command handoff
    // guard, not an extra sensor or a simulator-state shortcut.
    float recovery_drive_ramp_s = 0.0F;
    float recovery_pitch_rate_rad_s = 1.2F;
    float recovery_joint_speed_rad_s = 1.5F;
  };

  struct Output {
    Output() {
      kp_scale.fill(1.0F);
      kd_scale.fill(1.0F);
    }
    Phase phase = Phase::Grounded;
    bool active = false;
    // True only while unsupported in Flight. Once the IMU-derived support
    // exists, wheel MPC is allowed to catch the body concurrently with the
    // leg's impact damping; leaving it disabled for the whole Impact window
    // would remove the primary pitch actuator exactly at touchdown.
    bool suspend_wheel_balance = false;
    // [0, 1] scale for forward/yaw commands during the post-touchdown drive
    // handoff. It is 1.0 when this feature is disabled or no landing is in
    // progress.
    float drive_velocity_scale = 1.0F;
    JointArray position_offset_rad{};
    JointArray kp_scale{};
    JointArray kd_scale{};
    JointArray feedforward_nm{};
  };

  LandingController();
  explicit LandingController(Config config) : config_(config) {}

  void reset();

  [[nodiscard]] Output update(const RobotState& state,
                              const JointArray& nominal_target_rad,
                              const FlightObserver::Output* flight = nullptr);

  [[nodiscard]] Phase phase() const noexcept { return phase_; }

 private:
  struct SideRef {
    float height_m = 0.0F;
    float x_m = 0.0F;
    bool in_impact = false;
  };

  [[nodiscard]] JointArray flightOffset(const RobotState& state,
                                        const JointArray& nominal_target_rad);
  void addImpactTorques(const RobotState& state, Output& out) const;
  void updatePhase(const RobotState& state,
                   const FlightObserver::Output* flight);

  Config config_;
  Phase phase_ = Phase::Grounded;
  bool prev_floor_contact_ = true;
  bool prev_left_contact_ = true;
  bool prev_right_contact_ = true;
  float phase_elapsed_s_ = 0.0F;
  float airborne_elapsed_s_ = 0.0F;
  float recovery_elapsed_s_ = 0.0F;
  float drive_handoff_elapsed_s_ = 0.0F;
  bool drive_handoff_active_ = false;
  float flight_hip_offset_rad_ = 0.0F;
  float flight_knee_offset_rad_ = 0.0F;
  SideRef left_ref_{};
  SideRef right_ref_{};
};

}  // namespace robot_control
