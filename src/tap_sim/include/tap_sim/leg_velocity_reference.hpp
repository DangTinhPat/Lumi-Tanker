#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include "tap_sim/actuator_gate.hpp"

namespace tap_sim
{
class LegVelocityReference
{
  bool seen_ = false, valid_ = false;
  bool active_reference_ = false, pending_loss_ = false;
  double stamp_ = 0, received_ = 0;
  std::array<double, 4> values_{};
public:
  inline static const std::array<std::string, 4> names = {
    "cad_left_hip", "cad_left_knee", "cad_right_hip", "cad_right_knee"};
  void receive(const std::vector<std::string> & joints, const std::vector<double> & velocities,
    double stamp, double wall, double speed_limit)
  {
    const bool expired = active_reference_ &&
      (wall - received_ > 0.1 || stamp - stamp_ > 0.1);
    const bool previous_loss = pending_loss_;
    // Preserve loss even if malformed/stale data is replaced before write().
    pending_loss_ = pending_loss_ || active_reference_;
    seen_ = true; valid_ = false;
    if (joints.size() != 4 || velocities.size() != 4 || !std::isfinite(stamp) || stamp <= 0) {return;}
    std::array<bool, 4> found{};
    for (std::size_t i = 0; i < 4; ++i) {
      const auto it = std::find(names.begin(), names.end(), joints[i]);
      if (it == names.end() || !std::isfinite(velocities[i])) {return;}
      const auto index = static_cast<std::size_t>(it - names.begin());
      if (found[index]) {return;}
      found[index] = true;
      values_[index] = std::clamp(velocities[i], -speed_limit, speed_limit);
    }
    stamp_ = stamp; received_ = wall; valid_ = true;
    pending_loss_ = previous_loss || expired;
  }
  bool sample(double sim, double wall, std::array<double, 4> & values) const
  {
    values.fill(0);
    if (!seen_) {return true;}
    if (!valid_ || sim - stamp_ < -0.005 || sim - stamp_ > 0.1 ||
      wall - received_ < 0 || wall - received_ > 0.1) {return false;}
    values = values_;
    return true;
  }
  bool allowTorque(ActuatorGate & gate, double sim, double wall,
    std::array<double, 4> & values)
  {
    const bool enabled = gate.enabled(wall);
    const bool fresh = sample(sim, wall, values);
    if (pending_loss_ || (active_reference_ && !fresh)) {
      gate.disarm();
      pending_loss_ = false; active_reference_ = false;
      return false;
    }
    active_reference_ = enabled && fresh && seen_;
    return enabled && fresh;
  }
};
}  // namespace tap_sim
