#pragma once

#include "robot_control/Types.hpp"

namespace robot_control {

enum class ControlMode {
  Disabled,
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
