#include "tap_fw/ImpulseStartupSequencer.hpp"

namespace tap_fw {

void ImpulseStartupSequencer::enter(Stage stage, std::uint64_t now_us) noexcept {
  stage_ = stage;
  stage_since_us_ = now_us;
}

void ImpulseStartupSequencer::reset() noexcept {
  stage_ = Stage::Idle;
  stage_since_us_ = 0U;
  arrived_once_ = false;
}

ImpulseStartupSequencer::Output ImpulseStartupSequencer::update(
    const Input& input) noexcept {
  if (input.disable_requested || !input.enable_requested) {
    reset();
    return {};
  }

  if (stage_ == Stage::Idle) enter(Stage::Preposition, input.now_us);

  switch (stage_) {
    case Stage::Idle:
      break;
    case Stage::Preposition:
      if (input.fixture_hold && input.fixture_arrived) {
        // Legs held at the target: wait (without a deadline) for the operator
        // to set the wheels down and press Start again.
        arrived_once_ = true;
        if (input.release_requested) enter(Stage::Release, input.now_us);
      } else if (!arrived_once_ &&
                 (input.now_us < stage_since_us_ ||
                  input.now_us - stage_since_us_ > config_.preposition_timeout_us)) {
        // The deadline covers reaching the pose, not the wait for Start #2.
        enter(Stage::Failed, input.now_us);
      }
      break;
    case Stage::Release:
      if (input.balance_ramp) {
        enter(Stage::BalanceRamp, input.now_us);
      } else if (!input.fixture_hold || input.now_us < stage_since_us_ ||
                 input.now_us - stage_since_us_ > config_.balance_ramp_timeout_us) {
        enter(Stage::Failed, input.now_us);
      }
      break;
    case Stage::BalanceRamp:
      if (input.balance) {
        enter(Stage::BalanceWarmup, input.now_us);
      } else if (!input.balance_ramp) {
        enter(Stage::Failed, input.now_us);
      } else if (input.now_us < stage_since_us_ ||
                 input.now_us - stage_since_us_ > config_.balance_ramp_timeout_us) {
        enter(Stage::Failed, input.now_us);
      }
      break;
    case Stage::BalanceWarmup:
      if (!input.balance) {
        enter(Stage::Failed, input.now_us);
      } else if (input.now_us >= stage_since_us_ &&
                 input.now_us - stage_since_us_ >= config_.balance_warmup_us) {
        enter(Stage::Run, input.now_us);
      }
      break;
    case Stage::Run:
      if (input.fixture_hold || (!input.balance && !input.balance_ramp)) {
        enter(Stage::Failed, input.now_us);
      }
      break;
    case Stage::Failed:
      break;
  }

  Output output{};
  output.stage = stage_;
  output.allow_fixture_release = stage_ == Stage::Release ||
      stage_ == Stage::BalanceRamp || stage_ == Stage::BalanceWarmup ||
      stage_ == Stage::Run;
  output.allow_drive = stage_ == Stage::Run;
  output.failed = stage_ == Stage::Failed;
  return output;
}

const char* toString(ImpulseStartupSequencer::Stage stage) noexcept {
  switch (stage) {
    case ImpulseStartupSequencer::Stage::Idle: return "Idle";
    case ImpulseStartupSequencer::Stage::Preposition: return "Preposition";
    case ImpulseStartupSequencer::Stage::Release: return "Release";
    case ImpulseStartupSequencer::Stage::BalanceRamp: return "BalanceRamp";
    case ImpulseStartupSequencer::Stage::BalanceWarmup: return "BalanceWarmup";
    case ImpulseStartupSequencer::Stage::Run: return "Run";
    case ImpulseStartupSequencer::Stage::Failed: return "Failed";
  }
  return "Unknown";
}

}  // namespace tap_fw
