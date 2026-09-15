#pragma once
#include <algorithm>
#include <cmath>

namespace tap_sim
{
inline double boundedTorque(bool wheel, double command, double velocity, double leg_speed,
  double leg_velocity_damping = 0.0, double leg_velocity_reference = 0.0)
{
  if (!std::isfinite(command) || !std::isfinite(velocity)) {return 0;}
  if (!std::isfinite(leg_velocity_damping) || leg_velocity_damping < 0) {return 0;}
  if (!std::isfinite(leg_velocity_reference)) {return 0;}
  const double limit = wheel ? 0.13 : 5.0;
  const double max_speed = wheel ? 41.88790204786391 : leg_speed;
  double torque = wheel ? 0.02 * (std::clamp(command, -max_speed, max_speed) - velocity) :
    command + leg_velocity_damping *
    (std::clamp(leg_velocity_reference, -leg_speed, leg_speed) - velocity);
  torque = std::clamp(torque, -limit, limit);
  // Conservative graph-informed positive-work envelope: 5 Nm at 190 rpm,
  // zero at 270 rpm. A lower configured cutoff further restricts this curve.
  if (!wheel && torque * velocity > 0) {
    const double full_speed = 19.896753472735355;
    const double zero_speed = 28.274333882308138;
    const double available = 5.0 * std::clamp(
      (zero_speed - std::abs(velocity)) / (zero_speed - full_speed), 0.0, 1.0);
    torque = std::clamp(torque, -available, available);
  }
  if (std::abs(velocity) >= max_speed && torque * velocity > 0) {return 0;}
  return torque;
}
}  // namespace tap_sim
