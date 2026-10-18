#pragma once

#include "robot_control/Types.hpp"

namespace robot_control {

class JointMotionPlanner {
 public:
  struct Config {
    float default_duration_s = 1.0F;
  };

  JointMotionPlanner();
  explicit JointMotionPlanner(Config config);

  void reset(const JointArray& current_position);
  void startMove(const JointArray& current_position, const JointArray& target,
                 float duration_s);
  [[nodiscard]] JointArray update(float dt_s);
  [[nodiscard]] bool active() const noexcept;
  [[nodiscard]] const JointArray& target() const noexcept;

 private:
  [[nodiscard]] static float smoothStep(float x);

  Config config_;
  JointArray start_{};
  JointArray target_{};
  float elapsed_s_ = 0.0F;
  float duration_s_ = 1.0F;
  bool active_ = false;
};

}  // namespace robot_control
