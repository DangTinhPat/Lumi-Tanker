#include "robot_control/JointMotionPlanner.hpp"

#include <algorithm>

namespace robot_control {

JointMotionPlanner::JointMotionPlanner() : JointMotionPlanner(Config{}) {}

JointMotionPlanner::JointMotionPlanner(Config config) : config_(config) {}

void JointMotionPlanner::reset(const JointArray& current_position) {
  start_ = current_position;
  target_ = current_position;
  elapsed_s_ = 0.0F;
  duration_s_ = config_.default_duration_s;
  active_ = false;
}

void JointMotionPlanner::startMove(const JointArray& current_position,
                                   const JointArray& target,
                                   float duration_s) {
  start_ = current_position;
  target_ = target;
  elapsed_s_ = 0.0F;
  duration_s_ = std::max(duration_s, 1.0e-3F);
  active_ = true;
}

JointArray JointMotionPlanner::update(float dt_s) {
  if (!active_) {
    return target_;
  }

  elapsed_s_ += std::max(dt_s, 0.0F);
  const float ratio = std::clamp(elapsed_s_ / duration_s_, 0.0F, 1.0F);
  const float blend = smoothStep(ratio);

  JointArray output{};
  for (std::size_t i = 0; i < output.size(); ++i) {
    output[i] = start_[i] + blend * (target_[i] - start_[i]);
  }

  if (ratio >= 1.0F) {
    active_ = false;
    output = target_;
  }
  return output;
}

bool JointMotionPlanner::active() const noexcept { return active_; }

const JointArray& JointMotionPlanner::target() const noexcept {
  return target_;
}

float JointMotionPlanner::smoothStep(float x) {
  x = std::clamp(x, 0.0F, 1.0F);
  return x * x * (3.0F - 2.0F * x);
}

}  // namespace robot_control
