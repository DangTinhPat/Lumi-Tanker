#include <cassert>

#include "tap_fw/v3/V3Controller.hpp"

int main() {
  assert(tap_fw::v3::LegHeight::kKneeMinRad ==
         robot_control::geometry::native_knee_min_rad);
  assert(tap_fw::v3::LegHeight::forKnee(
             tap_fw::v3::LegHeight::kKneeMinRad).ok);
  tap_fw::v3::V3Controller controller(tap_fw::v3::defaultV3Config());
  tap_fw::v3::V3Controller::Input input{};
  tap_fw::v3::V3Controller::Command command{};
  command.start = true;
  auto output = controller.update(input, command);
  assert(output.state == tap_fw::v3::V3Controller::State::Locking);
  assert(output.engage_legs && output.legs_locked && !output.wheels_enabled);

  command.start = false;
  input.legs_ready = true;
  input.now_us = 1'300'000U;
  output = controller.update(input, command);
  input.now_us += 2'000U;
  output = controller.update(input, command);
  assert(output.state == tap_fw::v3::V3Controller::State::Balancing);
  assert(output.legs_locked && output.wheels_enabled);

  input.pitch_rad = 0.7F;
  input.now_us += 2'000U;
  output = controller.update(input, command);
  assert(output.state == tap_fw::v3::V3Controller::State::Fault);
  assert(output.legs_locked && !output.wheels_enabled);

  command.disable = true;
  output = controller.update(input, command);
  assert(output.state == tap_fw::v3::V3Controller::State::Idle);
  assert(!output.legs_locked && !output.wheels_enabled);
}
