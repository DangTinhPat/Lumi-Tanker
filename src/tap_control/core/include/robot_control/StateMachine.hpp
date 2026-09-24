#pragma once

#include "robot_control/Types.hpp"

namespace robot_control {

enum class ControlMode {
  Disabled,
  SensorQualify,
  FixtureHold,
  BalanceRamp,
  Enabling,
  StandingUp,
  Balance,
  Drive,
  PoseChange,
  Fallen,
  EmergencyStop,
};

class StateMachine {
 public:
  struct Config {
    unsigned enable_cycles = 20;
    float fall_pitch_rad = 0.65F;
    bool fixture_startup_enabled = false;
    unsigned sensor_qualify_cycles = 50;
    unsigned balance_ramp_cycles = 100;
    float startup_pitch_rad = 0.15F;
    float startup_pitch_rate_rad_s = 0.35F;
    float startup_joint_tolerance_rad = 0.20F;
  };

  StateMachine();
  explicit StateMachine(Config config);

  void reset();
  void forceFallen() { enter(ControlMode::Fallen); }
  void recoverToBalance() { enter(ControlMode::Balance); }
  void abortStandUp() { forceFallen(); }
  [[nodiscard]] ControlMode update(const RobotState& state,
                                   const ControlCommand& command,
                                   bool pose_motion_active,
                                   bool stand_up_active,
                                   bool airborne = false);
  [[nodiscard]] ControlMode mode() const noexcept;
  [[nodiscard]] float fallPitchRad() const noexcept { return config_.fall_pitch_rad; }
  [[nodiscard]] float balanceRampProgress() const noexcept;

 private:
  void enter(ControlMode next_mode);

  Config config_;
  ControlMode mode_ = ControlMode::Disabled;
  unsigned cycles_in_mode_ = 0;
  bool stand_up_requested_ = false;
  bool enable_requested_ = false;
};

[[nodiscard]] const char* toString(ControlMode mode) noexcept;

}  // namespace robot_control
