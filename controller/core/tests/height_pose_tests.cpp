#include "robot_control/WholeBodyModel.hpp"
#include "robot_control/EmbeddedController.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace robot_control;

namespace {
int failures = 0;

void check(bool condition, const char* name) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL " << name << '\n';
  }
}
}  // namespace

int main() {
  constexpr float hmin = geometry::native_hip_min_rad;
  constexpr float hmax = geometry::native_hip_max_rad;
  constexpr float kmin = geometry::native_knee_min_rad;
  constexpr float kmax = geometry::native_knee_max_rad;

  const auto natural = WholeBodyModel::solveSymmetricHeight(
      geometry::neutral_spawn_z_m);
  check(natural.feasible, "natural height is reachable");
  check(std::abs(natural.hip_rad - geometry::neutral_hip_rad) < 1.0e-4F &&
            std::abs(natural.knee_rad - geometry::neutral_knee_rad) < 1.0e-4F,
        "natural height returns the requested lowered natural pose");

  const auto deep = WholeBodyModel::solveSymmetricKnee(kmin);
  const auto extension = WholeBodyModel::solveSymmetricKnee(kmax);
  const auto seeded = WholeBodyModel::solveSymmetricHeight(
      0.160F, 0.428F, -0.028F, hmin, hmax, kmin, kmax);
  check(seeded.feasible, "height target remains solvable from measured sagged seed");
  const auto startup_seeded = WholeBodyModel::solveSymmetricHeight(
      0.160F, 0.425F, -0.006F, hmin, hmax, kmin, kmax);
  check(startup_seeded.feasible, "height target remains solvable from startup seed");
  check(deep.feasible && extension.feasible, "both knee envelope endpoints are FK-feasible");
  check(deep.knee_rad == kmin && extension.knee_rad == kmax,
        "endpoint knee values are not silently clipped");
  check(std::abs(deep.axle_x_m - natural.axle_x_m) < 2.0e-5F &&
            std::abs(extension.axle_x_m - natural.axle_x_m) < 2.0e-5F,
        "endpoint solver holds the natural axle fore/aft coordinate");

  const float low = std::min(deep.body_height_m, extension.body_height_m);
  const float high = std::max(deep.body_height_m, extension.body_height_m);
  float seed_hip = natural.hip_rad;
  float seed_knee = natural.knee_rad;
  for (int i = 0; i <= 40; ++i) {
    const float target = low + (high - low) * static_cast<float>(i) / 40.0F;
    const auto pose = WholeBodyModel::solveSymmetricHeight(
        target, seed_hip, seed_knee, hmin, hmax, kmin, kmax);
    check(pose.feasible, "intermediate height has a valid inverse FK solution");
    check(pose.hip_rad >= hmin && pose.hip_rad <= hmax &&
              pose.knee_rad >= kmin && pose.knee_rad <= kmax,
          "intermediate height respects joint envelope");
    check(std::abs(pose.axle_x_m - natural.axle_x_m) < 2.0e-4F,
          "intermediate height keeps axle fore/aft coordinate");
    seed_hip = pose.hip_rad;
    seed_knee = pose.knee_rad;
  }

  EmbeddedController::Config controller_config;
  // The production state machine qualifies over multiple samples; this unit
  // test advances that lifecycle in two calls so it can isolate HEIGHT.
  controller_config.state_machine.enable_cycles = 1;
  controller_config.hip_min_rad = hmin;
  controller_config.hip_max_rad = hmax;
  controller_config.knee_min_rad = kmin;
  controller_config.knee_max_rad = kmax;
  RobotState state;
  state.joints.position_rad = {geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F,
                               geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F};
  state.imu.sample_dt_s = 0.002F;
  state.imu.accel_magnitude_m_s2 = 9.81F;
  ControlCommand command;
  command.target_joint_position_rad = state.joints.position_rad;
  command.request_enable = true;
  EmbeddedController controller(controller_config);
  controller.reset(state);
  (void)controller.update(state, command);
  command.request_enable = false;
  // The production state machine changes Enabling -> Balance on this sample;
  // submit HEIGHT on the following sample so the test exercises the same
  // public transaction boundary as the native runner.
  (void)controller.update(state, command);
  command.request_height_change = true;
  command.target_body_height_m = 0.160F;
  (void)controller.update(state, command);
  check(controller.heightMotionActive(), "controller owns a height transaction");

  std::cout << (failures == 0 ? "PASS" : "FAIL")
            << " height pose FK sweep (" << (41 - failures) << "/41 samples)\n";
  return failures == 0 ? 0 : 1;
}
