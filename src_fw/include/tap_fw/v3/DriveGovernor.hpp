#pragma once

#include <algorithm>
#include <cmath>

namespace tap_fw::v3 {

// Constraint governor upstream of the shared wheel/hip preview plan.
// Garone, Di Cairano & Kolmanovsky, Automatica 75 (2017), 306-328,
// doi:10.1016/j.automatica.2016.08.013. This is a conservative torque budget,
// not an invariant-set/capturability guarantee. No motor-current/contact input.
struct DriveGovernor {
  struct Config {
    bool enabled = false;  // SET tgov: opt in after actuator calibration
    float wheel_torque_nm = 0.13F;  // continuous, never the stand-up peak
    float balance_reserve = 0.45F;  // fraction of available torque kept for balance
  };

  static float accelerationLimit(const Config& c, float velocity_m_s,
                                 float yaw_rad_s, float direction,
                                 float radius_m, float track_m, float mass_kg,
                                 float rated_rad_s, float noload_rad_s) noexcept {
    const float radius = std::max(radius_m, 0.001F);
    const float half_turn = 0.5F * std::max(track_m, 0.0F) * yaw_rad_s;
    auto fraction = [&](float wheel_v) {
      // The adopted wheel model keeps braking torque. When acceleration has
      // the same sign as rotation, motoring torque falls above rated speed.
      if (direction * wheel_v <= 0.0F) return 1.0F;
      const float speed = std::abs(wheel_v) / radius;
      return std::clamp((noload_rad_s - speed) /
          std::max(noload_rad_s - rated_rad_s, 0.001F), 0.0F, 1.0F);
    };
    // Both wheels must retain balance authority during a turn. Budget from
    // the weaker wheel, rather than counting all of the inner wheel's torque.
    const float available = std::min(fraction(velocity_m_s - half_turn),
                                     fraction(velocity_m_s + half_turn));
    return 2.0F * std::max(c.wheel_torque_nm, 0.0F) * available *
        (1.0F - std::clamp(c.balance_reserve, 0.0F, 1.0F)) /
        (radius * std::max(mass_kg, 0.01F));
  }
};

}  // namespace tap_fw::v3
