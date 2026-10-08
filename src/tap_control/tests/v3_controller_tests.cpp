#include <cassert>
#include <cmath>

#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/v3/RockerLower.hpp"
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

  // Rocking stage 1: N-P from encoders + IMU pitch matches MuJoCo FK
  // (homing 74.1 mm, COM over the axle at hip 1.10 / pitch 0.03).
  using tap_fw::v3::RockerLower;
  constexpr float kHomeHip = 2.27392F;
  constexpr float kHomePitch = 0.0F;
  assert(std::abs(RockerLower::nToP(kHomeHip, -1.2F, kHomePitch) - 0.0741F) < 0.001F);
  assert(std::abs(RockerLower::nToP(1.10F, -1.2F, 0.03F)) < 0.001F);
  RockerLower lower;
  lower.reset(kHomeHip, -1.2F);
  RockerLower::Input in{};  // named field assignment: robust to struct reordering
  in.dt_s = 0.002F;
  in.pitch_rad = kHomePitch;
  in.hip_rad = kHomeHip;
  in.knee_rad = -1.2F;
  in.sensors_ok = true;
  auto low = lower.update(in);
  assert(low.phase == RockerLower::Phase::Lowering);
  assert(low.hip_target_rad < kHomeHip && low.knee_target_rad == -1.2F);
  assert(low.wheel_rad_s > 0.0F);
  in.hip_rad = 1.56F;   // servo lags the command; N-P still shrinking
  in.pitch_rad = 0.35F;
  in.dt_s = 2.0F;       // command reaches the lowering floor: fling, wheels held
  low = lower.update(in);
  assert(low.phase == RockerLower::Phase::Flinging && low.wheel_rad_s == 0.0F);
  in.dt_s = 0.002F;
  low = lower.update(in);
  assert(low.hip_target_rad < 1.5F - 0.01F);  // fast hip fold
  in.dt_s = 0.1F;       // command reaches the lock: pivot, hip held there
  low = lower.update(in);
  assert(low.phase == RockerLower::Phase::Pivot && low.hip_target_rad == 1.2F);
  // From Pivot, loop-drive the rest of the stand-up: track the hip encoder to
  // the just-commanded target (approximating the leg servo) and set pitch to
  // the balance loop's own equilibrium for that hip (approximating a robot
  // that is otherwise perfectly balanced) - not a physics engine, just enough
  // to exercise every phase transition and confirm the state machine reaches
  // Balanced with the hip at the level-zero target.
  in.dt_s = 0.002F;
  bool saw_wheelie = false, saw_catch = false, saw_level = false;
  for (int i = 0; i < 5000; ++i) {
    low = lower.update(in);
    saw_wheelie |= low.phase == RockerLower::Phase::Wheelie;
    saw_catch |= low.phase == RockerLower::Phase::Catch;
    saw_level |= low.phase == RockerLower::Phase::Level;
    if (low.phase == RockerLower::Phase::Balanced ||
        low.phase == RockerLower::Phase::Aborted) {
      break;
    }
    in.hip_rad = low.hip_target_rad;
    const auto body = robot_control::WholeBodyModel{}.compute(
        in.hip_rad, in.knee_rad, in.hip_rad, in.knee_rad);
    in.pitch_rad = -body.com_line_angle_rad;
  }
  assert(saw_wheelie && saw_catch && saw_level);
  assert(low.phase == RockerLower::Phase::Balanced);
  assert(std::abs(low.hip_target_rad - 1.1248F) < 1.0e-3F);
  in.pitch_rad = 3.0F;  // fallen: Balanced/Catch/Level abort past the lean limit
  assert(lower.update(in).phase == RockerLower::Phase::Aborted);
  lower.reset(kHomeHip, -1.2F);
  in = RockerLower::Input{};
  in.dt_s = 0.002F;
  in.pitch_rad = kHomePitch;
  in.hip_rad = kHomeHip;
  in.knee_rad = -1.2F;
  in.sensors_ok = false;
  assert(lower.update(in).phase == RockerLower::Phase::Aborted);
  lower.reset(kHomeHip, -1.2F);
  in.sensors_ok = true;
  in.hip_rad = 0.60F;   // pitch + hip < 0.70: thigh/shin would reach the floor
  assert(lower.update(in).phase == RockerLower::Phase::Aborted);
}
