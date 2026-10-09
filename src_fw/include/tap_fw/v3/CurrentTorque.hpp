#pragma once

#include <algorithm>
#include <cmath>

namespace tap_fw::v3 {

struct CurrentTorqueCommand {
  float target_current_a = 0.0F;
  float delta_current_a = 0.0F;
  float torque_nm = 0.0F;
};

inline CurrentTorqueCommand currentTorqueCommand(float requested_nm,
                                                  float measured_current_a,
                                                  bool feedback_valid,
                                                  float kt_nm_per_a,
                                                  float current_limit_a,
                                                  float compensation_gain) noexcept {
  CurrentTorqueCommand out{};
  if (!std::isfinite(requested_nm) || !std::isfinite(measured_current_a) ||
      !std::isfinite(kt_nm_per_a) || kt_nm_per_a <= 0.0F ||
      !std::isfinite(current_limit_a) || current_limit_a <= 0.0F ||
      !std::isfinite(compensation_gain)) return out;
  out.target_current_a = std::clamp(requested_nm / kt_nm_per_a,
                                    -current_limit_a, current_limit_a);
  if (!feedback_valid) return out;
  const float gain = std::clamp(compensation_gain, 0.0F, 1.0F);
  out.delta_current_a = out.target_current_a - measured_current_a;
  out.torque_nm = std::clamp(requested_nm + gain * kt_nm_per_a * out.delta_current_a,
                             -current_limit_a * kt_nm_per_a,
                             current_limit_a * kt_nm_per_a);
  return out;
}

}  // namespace tap_fw::v3
