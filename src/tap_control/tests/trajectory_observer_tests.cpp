#include <cmath>
#include <iostream>

#include "robot_control/TrajectoryObserver.hpp"

using namespace robot_control;

namespace {
int failures = 0;

void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}

RobotState state() {
  RobotState value;
  value.dt_s = 0.01F;
  value.imu.accel_magnitude_m_s2 = 9.81F;
  return value;
}

void observesForwardAndBackwardMotion() {
  TrajectoryObserver observer;
  auto s = state();
  observer.reset(s);
  for (int i = 0; i < 100; ++i) {
    s.wheel.left_position_m += 0.01F;
    s.wheel.right_position_m += 0.01F;
    s.wheel.left_velocity_m_s = 1.0F;
    s.wheel.right_velocity_m_s = 1.0F;
    const auto out = observer.update(s);
    if (i > 0) check(out.valid && out.forward_velocity_m_s > 0.9F,
                     "forward encoder motion has positive observed velocity");
  }
  const auto forward = observer.output();
  check(forward.forward_position_m > 0.9F,
        "forward motion accumulates positive relative displacement");

  for (int i = 0; i < 100; ++i) {
    s.wheel.left_position_m -= 0.01F;
    s.wheel.right_position_m -= 0.01F;
    s.wheel.left_velocity_m_s = -1.0F;
    s.wheel.right_velocity_m_s = -1.0F;
    (void)observer.update(s);
  }
  const auto backward = observer.output();
  check(backward.forward_velocity_m_s < -0.9F,
        "backward encoder motion has negative observed velocity");
  check(std::abs(backward.forward_position_m) < 0.03F,
        "equal forward and backward travel returns near the origin");
}

void rejectsEncoderDiscontinuity() {
  TrajectoryObserver observer;
  auto s = state();
  observer.reset(s);
  s.wheel.left_position_m += 0.01F;
  s.wheel.right_position_m += 0.01F;
  (void)observer.update(s);
  s.wheel.left_position_m += 1.0F;
  s.wheel.right_position_m += 1.0F;
  const auto out = observer.update(s);
  check(!out.valid && out.confidence == 0.0F,
        "encoder reset-sized discontinuity is rejected");
}

void rejectsReportedVelocityOutsidePhysicalEnvelope() {
  TrajectoryObserver observer;
  auto s = state();
  observer.reset(s);
  s.wheel.left_position_m += 0.01F;
  s.wheel.right_position_m += 0.01F;
  s.wheel.left_velocity_m_s = 1.0F;
  s.wheel.right_velocity_m_s = 1.0F;
  (void)observer.update(s);

  // Position increments remain plausible, but the independently reported
  // velocity exceeds max_wheel_speed_m_s. It must fail closed rather than
  // reach landing capture with an impossible speed and confidence.
  s.wheel.left_position_m += 0.01F;
  s.wheel.right_position_m += 0.01F;
  s.wheel.left_velocity_m_s = 2.5F;
  s.wheel.right_velocity_m_s = 2.5F;
  const auto out = observer.update(s);
  check(!out.valid && out.confidence == 0.0F &&
            out.forward_velocity_m_s == 0.0F,
        "reported wheel velocity outside the physical envelope is rejected");
}

void observesDifferentialYaw() {
  TrajectoryObserver observer;
  auto s = state();
  observer.reset(s);
  for (int i = 0; i < 20; ++i) {
    s.wheel.left_position_m += 0.005F;
    s.wheel.right_position_m += 0.015F;
    s.wheel.left_velocity_m_s = 0.5F;
    s.wheel.right_velocity_m_s = 1.5F;
    s.imu.yaw_rate_rad_s = (1.0F / (2.0F * 0.1545F));
    (void)observer.update(s);
  }
  check(observer.output().yaw_rad > 0.5F,
        "differential wheel motion produces positive relative yaw");
}
}  // namespace

int main() {
  observesForwardAndBackwardMotion();
  rejectsEncoderDiscontinuity();
  rejectsReportedVelocityOutsidePhysicalEnvelope();
  observesDifferentialYaw();
  return failures == 0 ? 0 : 1;
}
