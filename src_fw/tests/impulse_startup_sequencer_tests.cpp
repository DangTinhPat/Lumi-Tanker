// Staged v2 startup: preposition deadline, two-press release, ramp, warmup.

#include <cassert>
#include <cstdio>

#include "tap_fw/ImpulseStartupSequencer.hpp"

using tap_fw::ImpulseStartupSequencer;
using Stage = ImpulseStartupSequencer::Stage;

int main() {
  ImpulseStartupSequencer::Config config{};
  config.preposition_timeout_us = 15'000'000U;
  config.balance_ramp_timeout_us = 2'000'000U;
  config.balance_warmup_us = 1'000'000U;

  // Nothing requested: idle, nothing allowed.
  {
    ImpulseStartupSequencer seq(config);
    const auto out = seq.update({});
    assert(out.stage == Stage::Idle && !out.allow_fixture_release && !out.allow_drive);
  }

  // Full path: Start #1 -> held at the pose; a long wait before Start #2 is
  // not a failure; Start #2 -> release -> ramp -> balance -> warmup -> run.
  {
    ImpulseStartupSequencer seq(config);
    ImpulseStartupSequencer::Input in{};
    in.enable_requested = true;
    in.fixture_hold = true;
    in.now_us = 0U;
    auto out = seq.update(in);
    assert(out.stage == Stage::Preposition && !out.allow_fixture_release);
    in.fixture_arrived = true;
    for (std::uint64_t t = 1'000'000U; t <= 60'000'000U; t += 1'000'000U) {
      in.now_us = t;
      out = seq.update(in);
      assert(out.stage == Stage::Preposition && !out.failed);  // waits for Start #2
      assert(!out.allow_fixture_release && !out.allow_drive);
    }
    in.release_requested = true;
    in.now_us = 61'000'000U;
    out = seq.update(in);
    assert(out.stage == Stage::Release && out.allow_fixture_release && !out.allow_drive);
    in.fixture_hold = false;
    in.balance_ramp = true;
    in.now_us = 61'002'000U;
    out = seq.update(in);
    assert(out.stage == Stage::BalanceRamp);
    in.balance_ramp = false;
    in.balance = true;
    in.now_us = 61'200'000U;
    out = seq.update(in);
    assert(out.stage == Stage::BalanceWarmup && !out.allow_drive);
    in.now_us = 62'300'000U;
    out = seq.update(in);
    assert(out.stage == Stage::Run && out.allow_drive);
    // Falling out of balance in Run fails the sequence.
    in.balance = false;
    in.now_us = 63'000'000U;
    out = seq.update(in);
    assert(out.failed);
    // Disable resets.
    in.disable_requested = true;
    out = seq.update(in);
    assert(out.stage == Stage::Idle && !out.failed);
  }

  // The deadline still applies while the legs have not reached the pose.
  {
    ImpulseStartupSequencer seq(config);
    ImpulseStartupSequencer::Input in{};
    in.enable_requested = true;
    in.fixture_hold = true;
    in.fixture_arrived = false;
    in.now_us = 0U;
    (void)seq.update(in);
    in.now_us = 14'000'000U;
    assert(!seq.update(in).failed);
    in.now_us = 15'100'000U;
    assert(seq.update(in).failed);
  }

  // Release that never reaches the balance ramp fails after its timeout.
  {
    ImpulseStartupSequencer seq(config);
    ImpulseStartupSequencer::Input in{};
    in.enable_requested = true;
    in.fixture_hold = true;
    in.fixture_arrived = true;
    in.release_requested = true;
    in.now_us = 0U;
    (void)seq.update(in);  // Preposition -> Release in one step
    assert(seq.stage() == Stage::Release);
    in.now_us = 2'100'000U;
    assert(seq.update(in).failed);
  }

  std::puts("impulse_startup_sequencer_tests: all checks passed");
  return 0;
}
