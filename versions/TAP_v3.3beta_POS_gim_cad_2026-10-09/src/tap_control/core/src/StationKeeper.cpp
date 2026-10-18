#include "robot_control/StationKeeper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_control {
namespace {
constexpr double kTwoPi = 6.28318530717958647692;
double wrap(double angle) { return std::remainder(angle, kTwoPi); }
bool nonnegativeFinite(double value) {
  return std::isfinite(value) && value >= 0.0;
}
}

StationKeeper::StationKeeper() : StationKeeper(Config{}) {}

StationKeeper::StationKeeper(Config config) : config_(config) {
  if (!nonnegativeFinite(config.position_gain_hz) ||
      !nonnegativeFinite(config.yaw_gain_hz) ||
      !nonnegativeFinite(config.max_forward) ||
      !nonnegativeFinite(config.max_yaw) || !nonnegativeFinite(config.acquire_dwell_s)) {
    throw std::invalid_argument("StationKeeper gains and bounds must be finite and nonnegative");
  }
}

void StationKeeper::reset() noexcept {
  have_anchor_ = false;
  acquire_elapsed_s_ = 0.0;
  anchor_x_ = anchor_y_ = anchor_yaw_ = 0.0;
}

StationKeeper::Output StationKeeper::update(const Input& input) {
  if (!input.active || !std::isfinite(input.x) || !std::isfinite(input.y) ||
      !std::isfinite(input.yaw) || !std::isfinite(input.dt) || input.dt <= 0.0) {
    reset();
    return {};
  }

  const double yaw = wrap(input.yaw);
  if (!have_anchor_ && !input.operator_driving && config_.acquire_dwell_s>0) {
    acquire_elapsed_s_=input.ready_to_acquire?acquire_elapsed_s_+input.dt:0.;
    if(acquire_elapsed_s_<config_.acquire_dwell_s) return {};
  }
  if (!have_anchor_ || input.operator_driving) {
    anchor_x_ = input.x;
    anchor_y_ = input.y;
    anchor_yaw_ = yaw;
    have_anchor_ = true;
  }
  if (input.operator_driving) return {};

  Output out;
  out.ex = anchor_x_ - input.x;
  out.ey = anchor_y_ - input.y;
  out.yawerror = wrap(anchor_yaw_ - yaw);
  const double forward_error = out.ex*std::cos(yaw) + out.ey*std::sin(yaw);
  const double forward = config_.position_gain_hz*forward_error;
  const double turning = config_.yaw_gain_hz*out.yawerror;
  // Finite inputs can still overflow when subtracting extreme coordinates.
  if (!std::isfinite(out.ex) || !std::isfinite(out.ey) ||
      !std::isfinite(forward) || !std::isfinite(turning)) {
    reset();
    return {};
  }
  out.forward_correction_m_s = std::clamp(forward, -config_.max_forward, config_.max_forward);
  out.yaw_correction_rad_s = std::clamp(turning, -config_.max_yaw, config_.max_yaw);
  out.holding = true;
  return out;
}

}  // namespace robot_control
