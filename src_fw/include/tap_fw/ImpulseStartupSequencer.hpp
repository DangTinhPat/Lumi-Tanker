#pragma once

#include <cstdint>

namespace tap_fw {

// Non-blocking TAP adaptation of Impulse's coldStart transaction. It gates
// fixture release until the leg posture has arrived and a wheel-balance
// warmup has elapsed. It does not home or infer joint zero.
class ImpulseStartupSequencer {
 public:
  enum class Stage { Idle, Preposition, Release, BalanceRamp, BalanceWarmup, Run, Failed };

  struct Config {
    std::uint64_t preposition_timeout_us = 15'000'000U;
    std::uint64_t balance_ramp_timeout_us = 2'000'000U;
    std::uint64_t balance_warmup_us = 1'000'000U;
  };

  struct Input {
    std::uint64_t now_us = 0U;
    bool enable_requested = false;
    bool disable_requested = false;
    bool release_requested = false;
    bool fixture_hold = false;
    bool fixture_arrived = false;
    bool balance_ramp = false;
    bool balance = false;
  };

  struct Output {
    Stage stage = Stage::Idle;
    bool allow_fixture_release = false;
    bool allow_drive = false;
    bool failed = false;
  };

  ImpulseStartupSequencer() noexcept = default;
  explicit ImpulseStartupSequencer(Config config) noexcept : config_(config) {}

  [[nodiscard]] Output update(const Input& input) noexcept;
  void reset() noexcept;
  [[nodiscard]] Stage stage() const noexcept { return stage_; }

 private:
  void enter(Stage stage, std::uint64_t now_us) noexcept;

  Config config_{};
  Stage stage_ = Stage::Idle;
  std::uint64_t stage_since_us_ = 0U;
  bool arrived_once_ = false;
};

[[nodiscard]] const char* toString(ImpulseStartupSequencer::Stage stage) noexcept;

}  // namespace tap_fw
