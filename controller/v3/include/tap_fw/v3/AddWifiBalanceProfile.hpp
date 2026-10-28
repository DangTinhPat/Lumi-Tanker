#pragma once
#include "tap_fw/v3/V3Controller.hpp"

namespace tap_fw::v3 {
// Boot/Idle configuration only. Values from 0e2c95a; motion stays at a787a73.
// This is a diagnostic transplant, not a proven cure or an exact old firmware.
inline void applyAddWifiBalanceProfile(V3Controller::Config& cfg, bool enabled) noexcept {
  cfg.tier.lean_share = enabled ? 1.0F : 0.85F;
  cfg.tier.disturbance_share = enabled ? 0.60F : 0.45F;
  cfg.roll.enabled = !enabled;
}
inline bool isAddWifiBalanceProfile(const V3Controller::Config& cfg) noexcept {
  return cfg.tier.lean_share == 1.0F && cfg.tier.disturbance_share == 0.60F && !cfg.roll.enabled;
}
}  // namespace tap_fw::v3
