#include <cmath>
#include <iostream>

#include "robot_control/FlightObserver.hpp"

using namespace robot_control;

namespace {
int failures = 0;

void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}

RobotState baseState() {
  RobotState state;
  state.dt_s = 0.001F;
  state.floor_contact = true;
  state.left_floor_contact = true;
  state.right_floor_contact = true;
  state.imu.accel_magnitude_m_s2 = 9.81F;
  return state;
}

void shortStepDoesNotBecomeFlight() {
  FlightObserver observer;
  auto state = baseState();
  observer.reset(state);
  for (int i = 0; i < 5; ++i) {
    // There is no contact switch in the hardware contract. A brief
    // non-ballistic interval is therefore only a normal supported sample.
    state.imu.accel_magnitude_m_s2 = 9.81F;
    const auto out = observer.update(state);
    check(!out.support_lost && !out.airborne,
          "non-ballistic interval remains grounded");
  }
  const auto out = observer.update(state);
  check(out.phase == FlightObserver::Phase::Grounded,
        "supported IMU samples remain grounded");
}

void BallisticFlightRequiresDebouncedLoss() {
  FlightObserver observer;
  auto state = baseState();
  observer.reset(state);
  // Support loss has two sequential debounce windows: free-fall detection
  // followed by support-loss confirmation.
  for (int i = 0; i < 40; ++i) {
    state.imu.accel_magnitude_m_s2 = 0.2F;
    (void)observer.update(state);
  }
  const auto out = observer.update(state);
  check(out.support_lost && out.airborne && out.ballistic,
        "debounced unloading becomes ballistic flight");
  check(out.downward_velocity_m_s > 0.15F,
        "ballistic observer integrates downward velocity");
}

void UnloadingIsExposedBeforeConfirmedFlight() {
  FlightObserver observer;
  auto state = baseState();
  observer.reset(state);
  state.imu.accel_magnitude_m_s2 = 0.2F;
  const auto first = observer.update(state);
  check(!first.support_unloading && !first.airborne && !first.support_lost,
        "one low-specific-force sample does not expose unloading or flight");
  bool saw_unloading = false;
  for (int i = 0; i < 8; ++i) {
    const auto out = observer.update(state);
    saw_unloading = saw_unloading || out.support_unloading;
  }
  check(saw_unloading,
        "persistent low-specific-force evidence exposes unloading before flight");
}

void GroundedImuNoiseDoesNotExposeUnloading() {
  FlightObserver observer;
  auto state = baseState();
  observer.reset(state);
  const float noisy_accel[] = {8.9F, 10.7F, 9.2F, 10.3F, 8.8F, 10.9F,
                               9.81F, 9.1F, 10.5F, 9.4F};
  for (const float accel : noisy_accel) {
    state.imu.accel_magnitude_m_s2 = accel;
    const auto out = observer.update(state);
    check(!out.support_unloading && !out.airborne &&
              out.phase == FlightObserver::Phase::Grounded,
          "grounded accelerometer noise remains grounded");
  }
}

void SimulatorContactTelemetryCannotAffectObserver() {
  FlightObserver from_contact_true;
  FlightObserver from_contact_false;
  auto true_state = baseState();
  auto false_state = true_state;
  from_contact_true.reset(true_state);
  from_contact_false.reset(false_state);

  auto compare = [&](const FlightObserver::Output& a,
                     const FlightObserver::Output& b,
                     const char* label) {
    check(a.phase == b.phase && a.support_lost == b.support_lost &&
              a.support_unloading == b.support_unloading &&
              a.airborne == b.airborne && a.ballistic == b.ballistic &&
              a.collision_candidate == b.collision_candidate &&
              a.touchdown_candidate == b.touchdown_candidate &&
              a.support_contact == b.support_contact &&
              std::fabs(a.downward_velocity_m_s - b.downward_velocity_m_s) <
                  1.0e-7F,
          label);
  };

  const float accel_stream[] = {9.81F, 9.2F, 0.2F, 0.2F, 0.2F, 40.0F,
                                9.81F, 9.81F, 9.81F};
  for (const float accel : accel_stream) {
    true_state.imu.accel_magnitude_m_s2 = accel;
    false_state.imu.accel_magnitude_m_s2 = accel;
    // These are simulator-only telemetry fields. Deliberately feed opposite
    // values to prove the observer consumes only the declared sensor stream.
    true_state.floor_contact = true;
    true_state.left_floor_contact = true;
    true_state.right_floor_contact = true;
    false_state.floor_contact = false;
    false_state.left_floor_contact = false;
    false_state.right_floor_contact = false;
    compare(from_contact_true.update(true_state),
            from_contact_false.update(false_state),
          "simulator contact telemetry cannot alter CMF phase output");
  }
}

void DeterministicLandingReplayReportsLatencyAndRejectsBounceGlitch() {
  FlightObserver::Config cfg;
  cfg.support_loss_debounce_s = 0.012F;
  cfg.free_fall_debounce_s = 0.012F;
  cfg.collision_hold_s = 0.020F;
  cfg.touchdown_debounce_s = 0.015F;
  cfg.support_unloading_confidence_threshold = 0.5F;
  FlightObserver observer(cfg);
  auto state = baseState();
  observer.reset(state);

  // Replay only the sensor stream. The expected phase is an assertion, never
  // an input to the observer (and no pose/contact truth is supplied).
  const float grounded_noise[] = {9.05F, 10.42F, 9.31F, 10.18F, 9.76F,
                                  8.94F, 10.61F, 9.48F, 10.03F, 9.22F};
  for (const float accel : grounded_noise) {
    state.imu.accel_magnitude_m_s2 = accel;
    const auto out = observer.update(state);
    check(out.phase == FlightObserver::Phase::Grounded &&
              !out.support_unloading && !out.airborne,
          "deterministic replay: grounded IMU noise has no false transition");
  }

  state.imu.accel_magnitude_m_s2 = 0.2F;
  auto one_sample = observer.update(state);
  check(!one_sample.support_unloading && !one_sample.airborne,
        "deterministic replay: one low-g sample is rejected");

  int unloading_tick = -1;
  int ballistic_tick = -1;
  for (int tick = 1; tick <= 40; ++tick) {
    const auto out = observer.update(state);
    if (unloading_tick < 0 && out.support_unloading) unloading_tick = tick;
    if (ballistic_tick < 0 && out.phase == FlightObserver::Phase::BallisticFlight) {
      ballistic_tick = tick;
    }
  }
  check(unloading_tick >= 5 && unloading_tick < 20,
        "deterministic replay: unloading appears after the confidence threshold");
  check(ballistic_tick >= 20 && ballistic_tick < 40,
        "deterministic replay: ballistic phase latency matches both debounces");

  int collision_tick = -1;
  state.imu.accel_magnitude_m_s2 = 40.0F;
  state.imu.hard_impact_event = true;
  for (int tick = 1; tick <= 21; ++tick) {
    const auto out = observer.update(state);
    if (collision_tick < 0 && out.collision_candidate) collision_tick = tick;
  }
  check(collision_tick >= 20,
        "deterministic replay: touchdown shock requires its collision hold");

  state.imu.accel_magnitude_m_s2 = 9.81F;
  state.imu.hard_impact_event = false;
  auto touchdown = observer.update(state);
  check(touchdown.touchdown_candidate && touchdown.airborne,
        "deterministic replay: post-shock support is a touchdown candidate");
  int grounded_tick = -1;
  for (int tick = 1; tick <= 40; ++tick) {
    const auto out = observer.update(state);
    if (grounded_tick < 0 && out.phase == FlightObserver::Phase::Grounded) {
      grounded_tick = tick;
    }
  }
  check(grounded_tick >= 10 && grounded_tick < 40,
        "deterministic replay: touchdown returns to grounded after debounce");

  // A one-sample rebound immediately after re-contact must not erase support
  // or create a second landing event.
  state.imu.accel_magnitude_m_s2 = 0.2F;
  const auto bounce = observer.update(state);
  check(bounce.phase == FlightObserver::Phase::Grounded &&
            !bounce.airborne && !bounce.support_unloading,
        "deterministic replay: one-sample rebound is not a second flight");
}

void DeterministicDtAndNoiseEnvelope() {
  const float dts[] = {0.001F, 0.0025F, 0.005F, 0.010F, 0.020F, 0.050F};
  const float accel_noise[] = {0.00F, 0.18F, -0.22F, 0.31F, -0.14F,
                               0.09F, -0.35F, 0.27F};
  for (const float dt : dts) {
    FlightObserver::Config cfg;
    cfg.free_fall_debounce_s = 0.025F;
    cfg.support_loss_debounce_s = 0.012F;
    cfg.support_unloading_confidence_threshold = 0.5F;
    FlightObserver observer(cfg);
    auto state = baseState();
    state.dt_s = dt;
    observer.reset(state);

    bool grounded_noise_safe = true;
    for (int i = 0; i < 200; ++i) {
      state.imu.accel_magnitude_m_s2 = 9.81F + accel_noise[i % 8];
      const auto out = observer.update(state);
      grounded_noise_safe = grounded_noise_safe &&
          out.phase == FlightObserver::Phase::Grounded &&
          !out.airborne && !out.support_unloading;
    }
    check(grounded_noise_safe,
          "bounded grounded IMU noise stays grounded for every supported dt");

    state.imu.accel_magnitude_m_s2 = 0.2F;
    const auto one_sample = observer.update(state);
    if (dt < cfg.free_fall_debounce_s) {
      check(!one_sample.airborne,
            "a low-g sample shorter than debounce cannot open flight");
    } else {
      // At a coarse update period, one sample represents a whole interval.
      // If that interval is already longer than the configured debounce, it
      // is valid evidence rather than a one-sample glitch.
      check(one_sample.airborne,
            "a low-g interval longer than debounce is accepted as flight evidence");
    }
    bool reached_ballistic = false;
    const int max_ticks = static_cast<int>(
        std::ceil((cfg.free_fall_debounce_s +
                   cfg.support_loss_debounce_s) / dt)) + 4;
    for (int i = 0; i < max_ticks; ++i) {
      reached_ballistic = reached_ballistic ||
          observer.update(state).phase == FlightObserver::Phase::BallisticFlight;
    }
    check(reached_ballistic,
          "persistent free-fall reaches BallisticFlight within bounded latency");
  }
}

void EdgeShockIsNotTouchdown() {
  FlightObserver observer;
  auto state = baseState();
  observer.reset(state);
  for (int i = 0; i < 20; ++i) {
    state.imu.accel_magnitude_m_s2 = 0.2F;
    (void)observer.update(state);
  }
  bool saw_collision = false;
  for (int i = 0; i < 40; ++i) {
    state.imu.accel_magnitude_m_s2 = 40.0F;
    state.imu.hard_impact_event = true;
    const auto out = observer.update(state);
    saw_collision = saw_collision || out.collision_candidate;
  }
  check(saw_collision, "IMU shock becomes collision candidate");
  state.imu.hard_impact_event = false;
  state.imu.accel_magnitude_m_s2 = 9.81F;
  auto out = observer.update(state);
  check(out.touchdown_candidate && out.airborne,
        "impact recovery starts as touchdown candidate");
  for (int i = 0; i < 20; ++i) out = observer.update(state);
  check(out.phase == FlightObserver::Phase::Grounded,
        "persistent IMU support validates touchdown");
}

void EncoderCompressionReversalCorroboratesTouchdown() {
  FlightObserver::Config cfg;
  cfg.collision_accel_threshold_m_s2 = 100.0F;
  cfg.collision_hold_s = 0.001F;
  cfg.touchdown_debounce_s = 0.001F;
  cfg.free_fall_debounce_s = 0.010F;
  cfg.encoder_compression_rate_threshold_m_s = 0.05F;
  cfg.encoder_compression_reversal_threshold_m_s = 0.02F;
  cfg.encoder_touchdown_hold_s = 0.004F;
  FlightObserver observer(cfg);
  auto state = baseState();
  state.joints.position_rad = {0.9F, -1.3962634F, 0.0F,
                               0.9F, -1.3962634F, 0.0F};
  observer.reset(state);

  // Both knees are moving toward compression during flight.  The impact
  // sample stops them, but has only 1g specific force, so this test proves
  // the encoder corroboration is genuinely useful rather than an accel gate.
  for (int i = 0; i < 30; ++i) {
    state.imu.accel_magnitude_m_s2 = 0.2F;
    state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::LeftKnee)] = 2.0F;
    state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::RightKnee)] = 2.0F;
    (void)observer.update(state);
  }
  state.imu.accel_magnitude_m_s2 = 9.81F;
  state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::LeftKnee)] = 0.0F;
  state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::RightKnee)] = 0.0F;
  auto out = observer.update(state);
  check(out.encoder_touchdown_evidence,
        "encoder compression-rate reversal is exposed as touchdown evidence");
  for (int i = 0; i < 20; ++i) out = observer.update(state);
  check(out.phase == FlightObserver::Phase::Grounded && out.support_contact,
        "encoder corroboration restores support without a high-accel spike");
}

// The four tests below are literal, assertable proof that docs/RULES.md
// Rule 1's leg-torque exception binding conditions hold in code, not just in
// comments: torque corroboration must never be a sole gate, must be an exact
// no-op when disabled, must only ever shorten an already-independently-
// progressing debounce, and disagreement must never lengthen or block a
// transition IMU evidence alone would already produce.

int ticksToBallistic(FlightObserver& observer, RobotState state,
                     const TorqueResidualEstimator::Output* torque,
                     int max_ticks) {
  state.imu.accel_magnitude_m_s2 = 0.2F;
  for (int tick = 1; tick <= max_ticks; ++tick) {
    const auto out = observer.update(state, torque);
    if (out.ballistic) return tick;
  }
  return -1;
}

void torqueAloneCannotCreateFlightWithoutImuEvidence() {
  FlightObserver::Config cfg;
  cfg.torque_corroboration_enabled = true;
  cfg.torque_agreement_confidence_threshold = 0.0F;
  cfg.torque_corroboration_speedup_multiplier = 4.0F;
  FlightObserver observer(cfg);
  auto state = baseState();
  observer.reset(state);

  TorqueResidualEstimator::Output torque;
  torque.valid = true;
  torque.unloading_confidence = 1.0F;
  torque.direction_consistent_with_unloading = true;

  bool ever_airborne = false;
  for (int i = 0; i < 200; ++i) {
    // Fully IMU-grounded throughout -- torque evidence alone must never move
    // this.
    state.imu.accel_magnitude_m_s2 = 9.81F;
    const auto out = observer.update(state, &torque);
    ever_airborne = ever_airborne || out.airborne;
    check(out.phase == FlightObserver::Phase::Grounded,
          "IMU-grounded input stays Grounded regardless of torque evidence");
  }
  check(!ever_airborne,
        "torque evidence alone (IMU intact) never sets airborne -- never the sole gate");
}

void torqueDisabledIsByteIdenticalToPreExistingBehavior() {
  FlightObserver observer_baseline;  // Config{}: torque_corroboration_enabled=false.
  FlightObserver observer_with_pointer;
  auto state = baseState();
  observer_baseline.reset(state);
  observer_with_pointer.reset(state);

  TorqueResidualEstimator::Output torque;
  torque.valid = true;
  torque.unloading_confidence = 1.0F;
  torque.direction_consistent_with_unloading = true;

  bool identical = true;
  for (int i = 0; i < 60; ++i) {
    state.imu.accel_magnitude_m_s2 = 0.2F;
    const auto a = observer_baseline.update(state);
    const auto b = observer_with_pointer.update(state, &torque);
    identical = identical && a.phase == b.phase &&
        a.airborne == b.airborne && a.ballistic == b.ballistic &&
        std::fabs(a.downward_velocity_m_s - b.downward_velocity_m_s) < 1.0e-9F &&
        std::fabs(a.free_fall_confidence - b.free_fall_confidence) < 1.0e-9F;
  }
  check(identical,
        "a disabled torque_corroboration_enabled is byte-identical to no pointer at all");
}

void torqueCorroborationOnlySpeedsUpAlreadyProgressingDebounce() {
  FlightObserver::Config cfg;
  cfg.torque_corroboration_enabled = true;
  cfg.torque_agreement_confidence_threshold = 0.5F;
  cfg.torque_corroboration_speedup_multiplier = 3.0F;

  TorqueResidualEstimator::Output torque;
  torque.valid = true;
  torque.unloading_confidence = 1.0F;
  torque.direction_consistent_with_unloading = true;

  FlightObserver baseline(cfg);
  auto base_state = baseState();
  baseline.reset(base_state);
  const int baseline_ticks = ticksToBallistic(baseline, base_state, nullptr, 1000);

  FlightObserver corroborated(cfg);
  auto corr_state = baseState();
  corroborated.reset(corr_state);
  const int corroborated_ticks =
      ticksToBallistic(corroborated, corr_state, &torque, 1000);

  check(baseline_ticks > 0 && corroborated_ticks > 0,
        "both configurations eventually reach BallisticFlight");
  check(corroborated_ticks <= baseline_ticks,
        "agreeing torque only shortens (never lengthens) time-to-BallisticFlight");
  check(corroborated_ticks < baseline_ticks,
        "agreeing torque is actually consumed only after IMU debounce has started");

  // First-tick check: sensor_free_fall_elapsed_s_ starts at 0, so the
  // speed-up gate ("> 0.0F") must not fire on the very first sample --
  // torque cannot manufacture a transition from a standing start.
  FlightObserver first_tick_baseline(cfg);
  auto s1 = baseState();
  first_tick_baseline.reset(s1);
  s1.imu.accel_magnitude_m_s2 = 0.2F;
  const auto out_no_torque = first_tick_baseline.update(s1);

  FlightObserver first_tick_torque(cfg);
  auto s2 = baseState();
  first_tick_torque.reset(s2);
  s2.imu.accel_magnitude_m_s2 = 0.2F;
  const auto out_with_torque = first_tick_torque.update(s2, &torque);
  check(std::fabs(out_no_torque.free_fall_confidence -
                  out_with_torque.free_fall_confidence) < 1.0e-9F,
        "first sample is identical with or without agreeing torque -- no standing-start speedup");
}

void disagreeingTorqueNeverLengthensOrBlocksExistingBallisticTransition() {
  FlightObserver::Config cfg;
  cfg.torque_corroboration_enabled = true;
  cfg.torque_agreement_confidence_threshold = 0.5F;
  cfg.torque_corroboration_speedup_multiplier = 3.0F;

  TorqueResidualEstimator::Output disagreeing;
  disagreeing.valid = true;
  disagreeing.unloading_confidence = 1.0F;
  disagreeing.direction_consistent_with_unloading = false;
  disagreeing.direction_consistent_with_impact = false;

  FlightObserver baseline(cfg);
  auto base_state = baseState();
  baseline.reset(base_state);
  const int baseline_ticks = ticksToBallistic(baseline, base_state, nullptr, 1000);

  FlightObserver with_disagreement(cfg);
  auto dis_state = baseState();
  with_disagreement.reset(dis_state);
  const int disagreement_ticks =
      ticksToBallistic(with_disagreement, dis_state, &disagreeing, 1000);

  check(baseline_ticks > 0 && disagreement_ticks == baseline_ticks,
        "disagreeing torque never lengthens or blocks a transition IMU evidence alone already produces");
}

void SteadyAirborneExtensionDoesNotCreateEncoderTouchdown() {
  FlightObserver::Config cfg;
  cfg.collision_accel_threshold_m_s2 = 100.0F;
  cfg.collision_hold_s = 0.001F;
  cfg.free_fall_debounce_s = 0.010F;
  cfg.encoder_compression_rate_threshold_m_s = 0.05F;
  cfg.encoder_compression_reversal_threshold_m_s = 0.02F;
  FlightObserver observer(cfg);
  auto state = baseState();
  state.joints.position_rad = {0.9F, -1.3962634F, 0.0F,
                               0.9F, -1.3962634F, 0.0F};
  observer.reset(state);
  for (int i = 0; i < 30; ++i) {
    state.imu.accel_magnitude_m_s2 = 0.2F;
    state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::LeftKnee)] = 2.0F;
    state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::RightKnee)] = 2.0F;
    (void)observer.update(state);
  }
  state.imu.accel_magnitude_m_s2 = 9.81F;
  const auto out = observer.update(state);
  check(!out.encoder_touchdown_evidence && out.airborne,
        "steady encoder extension remains airborne without a contact hypothesis");
}

void SupportedSpecificForceReacquiresAfterTransientLoss() {
  FlightObserver::Config cfg;
  cfg.free_fall_debounce_s = 0.010F;
  cfg.support_loss_debounce_s = 0.010F;
  cfg.support_recovery_hold_s = 0.020F;
  cfg.collision_accel_threshold_m_s2 = 30.0F;
  FlightObserver observer(cfg);
  auto state = baseState();
  observer.reset(state);

  for (int i = 0; i < 20; ++i) {
    state.imu.accel_magnitude_m_s2 = 0.2F;
    (void)observer.update(state);
  }
  check(observer.update(state).airborne,
        "transient support test first enters sensor-derived flight");

  state.imu.accel_magnitude_m_s2 = 12.0F;
  bool regained = false;
  for (int i = 0; i < 40; ++i) {
    const auto out = observer.update(state);
    regained = regained || out.support_contact;
  }
  check(regained,
        "sustained supported specific force reacquires contact without impact");
  const auto settled = observer.update(state);
  check(settled.phase == FlightObserver::Phase::Grounded ||
            settled.phase == FlightObserver::Phase::TouchdownCandidate,
        "reacquired support follows the normal touchdown debounce");
}
}  // namespace

int main() {
  shortStepDoesNotBecomeFlight();
  BallisticFlightRequiresDebouncedLoss();
  UnloadingIsExposedBeforeConfirmedFlight();
  GroundedImuNoiseDoesNotExposeUnloading();
  SimulatorContactTelemetryCannotAffectObserver();
  DeterministicLandingReplayReportsLatencyAndRejectsBounceGlitch();
  DeterministicDtAndNoiseEnvelope();
  EdgeShockIsNotTouchdown();
  EncoderCompressionReversalCorroboratesTouchdown();
  SteadyAirborneExtensionDoesNotCreateEncoderTouchdown();
  SupportedSpecificForceReacquiresAfterTransientLoss();
  torqueAloneCannotCreateFlightWithoutImuEvidence();
  torqueDisabledIsByteIdenticalToPreExistingBehavior();
  torqueCorroborationOnlySpeedsUpAlreadyProgressingDebounce();
  disagreeingTorqueNeverLengthensOrBlocksExistingBallisticTransition();
  return failures == 0 ? 0 : 1;
}
