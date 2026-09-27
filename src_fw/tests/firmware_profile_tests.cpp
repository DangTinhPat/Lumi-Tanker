// The robot firmware profile: tuned simulator profile plus the adaptive
// balance angle, walked through the two-press fixture startup.

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <array>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/TunedConfig.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/BoardConfig.hpp"
#include "tap_fw/FirmwareProfile.hpp"
#include "tap_fw/LegHoming.hpp"
#include "tap_fw/SerialTuning.hpp"
#include <cstring>

using namespace robot_control;

namespace {

struct Landing {
  bool balanced = false;
  float trim_rad = 0.0F;
  float speed_m_s = 0.0F;
};

// Wheeled inverted pendulum with the wheel as a joint on a rigid leg: the
// drive servos the wheel joint speed (first order, tau), and the axle moves at
// R * (wheel joint rate + body pitch rate), as MotorFeedback measures it.
// The robot sits on the fixture at pitch 0, the operator stands it at
// `release_rad` and presses Start again; its true balance point is `eq_rad`.
Landing standAndRelease(float eq_rad, float release_rad, float tau_s,
                        const JointArray* legs = nullptr, float length = 0.22F) {
  EmbeddedController controller(tap_fw::firmwareControllerConfig());
  const float g = 9.80665F, dt = 0.002F, accel_limit = 1.5F;
  const float radius = geometry::wheel_radius_m;
  std::mt19937 rng(7);
  std::normal_distribution<float> pitch_noise(0.0F, 0.004F);
  std::normal_distribution<float> rate_noise(0.0F, 0.01F);
  std::normal_distribution<float> speed_noise(0.0F, 0.02F);
  RobotState s;
  s.dt_s = dt;
  s.imu.sample_dt_s = dt;
  s.floor_contact = true;
  s.joints.position_rad = {geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F,
                           geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F};
  if (legs != nullptr) s.joints.position_rad = *legs;
  ControlCommand command;
  command.target_joint_position_rad = s.joints.position_rad;
  command.request_enable = true;
  float x = 0.0F, v = 0.0F, th = 0.0F, w = 0.0F, wheel = 0.0F;
  const auto sense = [&] {
    ++s.imu_sample_sequence;
    ++s.encoder_sample_sequence;
    s.imu.pitch_rad = th + pitch_noise(rng);
    s.imu.pitch_rate_rad_s = w + rate_noise(rng);
    s.wheel.velocity_m_s = v + speed_noise(rng);
    s.wheel.position_m = x;
    s.joints.velocity_rad_s[2] = s.joints.velocity_rad_s[5] = wheel;
    s.joints.position_rad[2] = s.joints.position_rad[5] = x / radius - th;
  };
  for (int i = 0; i < 300 && controller.mode() != ControlMode::FixtureHold; ++i) {
    sense();
    (void)controller.update(s, command);
  }
  th = release_rad;
  command.request_fixture_release = true;
  Landing result;
  for (int i = 0; i < 10000; ++i) {  // 20 s
    sense();
    const auto out = controller.update(s, command);
    const float wheel_cmd = out.enable_motors
        ? 0.5F * (out.velocity_rad_s[2] + out.velocity_rad_s[5]) : 0.0F;
    float wheel_dot = (wheel_cmd - wheel) / tau_s;
    const float sn = std::sin(th - eq_rad), cs = std::cos(th);
    float w_dot = (g * sn - radius * cs * wheel_dot) / (length + radius * cs);
    float a = radius * (wheel_dot + w_dot);
    if (std::abs(a) > accel_limit) {
      a = std::copysign(accel_limit, a);
      w_dot = (g * sn - a * cs) / length;
      wheel_dot = a / radius - w_dot;
    }
    wheel += wheel_dot * dt;
    w += w_dot * dt;
    th += w * dt;
    v = radius * (wheel + w);
    x += v * dt;
    if (std::abs(th) > 0.7F) return result;
  }
  result.balanced = controller.mode() == ControlMode::Balance;
  result.trim_rad = controller.balanceTrim();
  result.speed_m_s = v;
  return result;
}

}  // namespace

int main() {
  const auto cfg = tap_fw::firmwareControllerConfig();
  const auto tuned = tunedControllerConfig();
  assert(cfg.state_machine.fixture_startup_enabled);
  assert(cfg.seed_balance_trim_from_pitch);
  assert(cfg.lqr.balance_trim_gain_rad_per_m > 0.0F);
  assert(cfg.lqr.command_velocity_tracking_margin_m_s > 0.0F);
  assert(!cfg.whole_body_stance.enabled);
  // Stance task off -> the wheel LQR keeps its full pitch feedback.
  assert(cfg.lqr.wheel_pitch_feedback_scale == 1.0F);
  // Everything else stays the simulator-tuned profile.
  // Two-wheel lock raises the leg envelope to the GIM6010-8 rated torque.
  assert(cfg.leg_max_torque_nm == (tap_fw::board::kLegsLockedAtZeroPose
                                       ? tap_fw::board::kLegLockTorqueNm
                                       : tuned.leg_max_torque_nm));
  assert(cfg.lqr.max_ground_accel_m_s2 == tuned.lqr.max_ground_accel_m_s2);
  assert(cfg.lqr.fall_pitch_rad == tuned.lqr.fall_pitch_rad);
  assert(tuned.lqr.balance_trim_gain_rad_per_m == 0.0F);

  EmbeddedController controller(cfg);
  RobotState state;
  state.dt_s = 0.002F;
  state.imu.sample_dt_s = 0.002F;
  state.floor_contact = true;
  state.joints.position_rad = {geometry::neutral_hip_rad, geometry::neutral_knee_rad,
                               0.0F, geometry::neutral_hip_rad,
                               geometry::neutral_knee_rad, 0.0F};
  ControlCommand command;
  command.target_joint_position_rad = state.joints.position_rad;
  command.request_enable = true;
  const auto step = [&] {
    ++state.imu_sample_sequence;
    ++state.encoder_sample_sequence;
    return controller.update(state, command);
  };
  for (unsigned i = 0; i < 120U && controller.mode() != ControlMode::FixtureHold; ++i) {
    (void)step();
  }
  assert(controller.mode() == ControlMode::FixtureHold);

  // Operator stands the robot at its balance point (here 4 deg nose-up)
  // and presses Start again.
  state.imu.pitch_rad = 0.07F;
  command.request_fixture_release = true;
  bool balanced = false;
  bool finite = true;
  for (unsigned i = 0; i < 400U; ++i) {
    const auto out = step();
    for (std::size_t j = 0; j < kJointCount; ++j) {
      finite = finite && std::isfinite(out.position_rad[j]) &&
          std::isfinite(out.velocity_rad_s[j]) && std::isfinite(out.feedforward_nm[j]);
    }
    balanced = balanced || controller.mode() == ControlMode::Balance;
  }
  assert(balanced);
  assert(finite);
  std::printf("seeded trim %.4f rad\n", controller.balanceTrim());
  assert(std::abs(controller.balanceTrim() - 0.07F) < 0.03F);
  // Legs hold the commanded natural pose: no stance COM correction.
  const auto out = step();
  assert(std::abs(out.position_rad[0] - geometry::neutral_hip_rad) < 1.0e-4F);
  assert(std::abs(out.position_rad[1] - geometry::neutral_knee_rad) < 1.0e-4F);

  // Closed loop: unknown balance point, hand error at release, 40 ms wheel lag.
  for (float eq_deg : {0.0F, 5.0F, -8.0F}) {
    for (float hand_deg : {0.0F, 2.0F, -2.0F}) {
      const float deg = 3.14159265F / 180.0F;
      const auto r = standAndRelease(eq_deg * deg, (eq_deg + hand_deg) * deg, 0.04F);
      std::printf("eq %+.0f release %+.0f: balanced=%d trim=%+.2f deg v=%+.3f\n", eq_deg,
                  hand_deg, r.balanced, r.trim_rad / deg, r.speed_m_s);
      assert(r.balanced);
      assert(std::abs(r.trim_rad - eq_deg * deg) < 1.0F * deg);
      assert(std::abs(r.speed_m_s) < 0.05F);
    }
  }

  // Two-wheel mode: legs locked at the extended (zero) pose, longer pendulum
  // (LQR length from FK ~0.23 m).
  {
    assert(tap_fw::board::kLegsLockedAtZeroPose);
    assert(!cfg.fixture_hold_via_enabled);
    const auto extended = tap_fw::hangingLegPose(
        tap_fw::hangPose(tap_fw::board::kKneeExtensionStopRad), 0.0F);
    for (float eq_deg : {0.0F, 5.0F, -6.0F}) {
      for (float hand_deg : {0.0F, 2.0F, -2.0F}) {
        const float deg = 3.14159265F / 180.0F;
        const auto r = standAndRelease(eq_deg * deg, (eq_deg + hand_deg) * deg, 0.04F,
                                       &extended, 0.23F);
        std::printf("extended legs eq %+.0f release %+.0f: balanced=%d trim=%+.2f deg "
                    "v=%+.3f\n", eq_deg, hand_deg, r.balanced, r.trim_rad / deg,
                    r.speed_m_s);
        assert(r.balanced);
        assert(std::abs(r.trim_rad - eq_deg * deg) < 1.0F * deg);
        assert(std::abs(r.speed_m_s) < 0.05F);
      }
    }
  }

  // Gravity-hang homing pose: hip where the straight leg's weight has no
  // moment about the hip, shifted by the body pitch.
  {
    const auto hang = tap_fw::hangPose(tap_fw::board::kKneeExtensionStopRad);
    std::printf("hang pose hip %+.4f knee %+.4f\n", hang.hip_rad, hang.knee_rad);
    assert(hang.knee_rad == geometry::native_knee_max_rad);
    assert(hang.hip_rad > -0.3F && hang.hip_rad < 0.4F);
    assert(std::abs(WholeBodyModel::legGravityTorque(hang.hip_rad, hang.knee_rad).hip_nm) < 1.0e-4F);
    const auto leg = WholeBodyModel::legHeight(hang.hip_rad, hang.knee_rad);
    assert(leg.height_m < -0.15F);  // hanging down, not the inverted solution
    const auto pitched = tap_fw::hangingLegPose(hang, 0.1F);
    assert(std::abs(pitched[0] - (hang.hip_rad - 0.1F)) < 1.0e-6F && pitched[0] == pitched[3]);
    assert(pitched[1] == hang.knee_rad && pitched[4] == hang.knee_rad);
  }

  // Start #1 with the legs hanging: FixtureHold folds them to the stance at
  // the slew rate and holds the release back until they arrive.
  {
    auto via_cfg = tap_fw::firmwareControllerConfig();
    via_cfg.fixture_hold_via_enabled = true;  // feature under test (off in 2-wheel mode)
    EmbeddedController hold(via_cfg);
    const auto hang = tap_fw::hangingLegPose(
        tap_fw::hangPose(tap_fw::board::kKneeExtensionStopRad), 0.0F);
    RobotState air;
    air.dt_s = 0.002F;
    air.imu.sample_dt_s = 0.002F;
    air.floor_contact = true;
    air.joints.position_rad = hang;
    ControlCommand start;
    start.request_enable = true;
    start.target_joint_position_rad = {geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F,
                                       geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F};
    const auto tick = [&] {
      ++air.imu_sample_sequence;
      ++air.encoder_sample_sequence;
      const auto out = hold.update(air, start);
      // Legs follow their position command (servo in the air).
      for (std::size_t j : {0U, 1U, 3U, 4U}) air.joints.position_rad[j] = out.position_rad[j];
      return out;
    };
    for (int i = 0; i < 300 && hold.mode() != ControlMode::FixtureHold; ++i) (void)tick();
    assert(hold.mode() == ControlMode::FixtureHold);  // hanging pose accepted
    start.request_fixture_release = true;  // operator presses Start #2 too early
    // Knee command sampled every 20 ms (below that, float rounding of the
    // 1e-7 rad position dominates a finite difference).
    std::vector<float> knee;
    float first_rate = -1.0F;
    float first_kp = -1.0F;
    float previous_knee = air.joints.position_rad[1];
    int ticks = 0;
    while (hold.mode() == ControlMode::FixtureHold && ticks < 8000) {
      const auto out = tick();
      assert(out.enable_motors);
      if (first_rate < 0.0F) {
        first_rate = std::abs(out.position_rad[1] - previous_knee) / 0.002F;
        first_kp = out.kp_scale[1];
      }
      if (ticks % 10 == 0) knee.push_back(out.position_rad[1]);
      ++ticks;
    }
    float max_rate = 0.0F;
    float max_accel = 0.0F;
    for (std::size_t i = 1; i < knee.size(); ++i) {
      max_rate = std::max(max_rate, std::abs(knee[i] - knee[i - 1]) / 0.02F);
      if (i >= 2) {
        max_accel = std::max(max_accel,
            std::abs(knee[i] - 2.0F * knee[i - 1] + knee[i - 2]) / (0.02F * 0.02F));
      }
    }
    const float seconds = ticks * 0.002F;
    std::printf("legs folded from hanging in %.2f s, max knee rate %.3f rad/s, "
                "max accel %.3f rad/s^2, first kp %.3f\n", seconds, max_rate, max_accel,
                first_kp);
    assert(first_rate < 1.0e-3F && first_kp < 0.01F);  // no snap at the enable edge
    assert(max_rate <= 0.30F + 1.0e-3F);
    assert(max_accel < 0.15F);
    // Knee travel 1.53 rad, min-jerk at 0.3 rad/s peak: ~9.5 s.
    assert(seconds > 9.0F && seconds < 10.0F);
    assert(hold.mode() == ControlMode::BalanceRamp);
    assert(std::abs(air.joints.position_rad[1] - geometry::neutral_knee_rad) < 1.0e-4F);
  }

  // From an arbitrary pose: back to the straight-leg reference first, then to
  // the stance, smoothly.
  {
    auto via_cfg = tap_fw::firmwareControllerConfig();
    via_cfg.fixture_hold_via_enabled = true;  // feature under test (off in 2-wheel mode)
    EmbeddedController hold(via_cfg);
    const auto hang = tap_fw::hangingLegPose(
        tap_fw::hangPose(tap_fw::board::kKneeExtensionStopRad), 0.0F);
    RobotState air;
    air.dt_s = 0.002F;
    air.imu.sample_dt_s = 0.002F;
    air.floor_contact = true;
    air.joints.position_rad = {0.6F, -0.3F, 0.0F, 0.5F, -0.2F, 0.0F};
    ControlCommand start;
    start.request_enable = true;
    start.request_fixture_release = true;
    start.target_joint_position_rad = {geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F,
                                       geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F};
    float max_knee = -10.0F;
    std::vector<float> hip;
    int ticks = 0;
    while (hold.mode() != ControlMode::BalanceRamp && ticks < 15000) {
      ++air.imu_sample_sequence;
      ++air.encoder_sample_sequence;
      const auto out = hold.update(air, start);
      for (std::size_t j : {0U, 1U, 3U, 4U}) air.joints.position_rad[j] = out.position_rad[j];
      max_knee = std::max(max_knee, out.position_rad[1]);
      if (ticks % 10 == 0) hip.push_back(out.position_rad[0]);
      ++ticks;
    }
    float max_accel = 0.0F;
    for (std::size_t i = 2; i < hip.size(); ++i) {
      max_accel = std::max(max_accel,
          std::abs(hip[i] - 2.0F * hip[i - 1] + hip[i - 2]) / (0.02F * 0.02F));
    }
    std::printf("random pose -> straight -> stance in %.2f s, knee reached %+.3f, "
                "max hip accel %.3f rad/s^2\n", ticks * 0.002F, max_knee, max_accel);
    assert(hold.mode() == ControlMode::BalanceRamp);
    assert(std::abs(max_knee - hang[1]) < 1.0e-3F);  // passed through the reference
    assert(max_accel < 0.15F);
    assert(std::abs(air.joints.position_rad[0] - geometry::neutral_hip_rad) < 1.0e-4F);
  }

  // Each leg runs its own path: the left leg starts near the stance and goes
  // straight there, the right leg starts elsewhere and passes through the
  // straight-leg reference first, on its own timing.
  {
    auto via_cfg = tap_fw::firmwareControllerConfig();
    via_cfg.fixture_hold_via_enabled = true;  // feature under test (off in 2-wheel mode)
    EmbeddedController hold(via_cfg);
    const auto hang = tap_fw::hangingLegPose(
        tap_fw::hangPose(tap_fw::board::kKneeExtensionStopRad), 0.0F);
    RobotState air;
    air.dt_s = 0.002F;
    air.imu.sample_dt_s = 0.002F;
    air.floor_contact = true;
    air.joints.position_rad = {geometry::neutral_hip_rad - 0.05F, geometry::neutral_knee_rad + 0.05F,
                               0.0F, 0.6F, -0.3F, 0.0F};
    ControlCommand start;
    start.request_enable = true;
    start.target_joint_position_rad = {geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F,
                                       geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F};
    float right_max_knee = -10.0F;
    float left_max_knee = -10.0F;
    float left_done_s = -1.0F;
    int ticks = 0;
    for (; ticks < 15000 && !hold.fixtureHoldArrived(); ++ticks) {
      ++air.imu_sample_sequence;
      ++air.encoder_sample_sequence;
      const auto out = hold.update(air, start);
      for (std::size_t j : {0U, 1U, 3U, 4U}) air.joints.position_rad[j] = out.position_rad[j];
      left_max_knee = std::max(left_max_knee, out.position_rad[1]);
      right_max_knee = std::max(right_max_knee, out.position_rad[4]);
      if (left_done_s < 0.0F &&
          std::abs(out.position_rad[1] - geometry::neutral_knee_rad) < 1.0e-4F &&
          std::abs(out.position_rad[0] - geometry::neutral_hip_rad) < 1.0e-4F) {
        left_done_s = ticks * 0.002F;
      }
    }
    std::printf("independent legs: left at stance after %.2f s (knee max %+.3f), "
                "right via straight (knee max %+.3f), both after %.2f s\n",
                left_done_s, left_max_knee, right_max_knee, ticks * 0.002F);
    assert(hold.fixtureHoldArrived());
    assert(left_max_knee < geometry::neutral_knee_rad + 0.06F);  // left never went straight
    assert(std::abs(right_max_knee - hang[4]) < 1.0e-3F);        // right passed the reference
    assert(left_done_s > 0.0F && left_done_s < 1.0F);            // left did not wait for right
    assert(ticks * 0.002F > 8.0F);
  }

  // Soft joints under a load (the robot stopped 0.25 rad short): the leg
  // position integral still brings every joint onto the commanded angle.
  {
    auto integral_cfg = tap_fw::firmwareControllerConfig();
    integral_cfg.leg_position_integral_gain_nm_per_rad_s = 8.0F;  // core feature
    EmbeddedController hold(integral_cfg);
    RobotState air;
    air.dt_s = 0.002F;
    air.imu.sample_dt_s = 0.002F;
    air.floor_contact = true;
    air.joints.position_rad = tap_fw::hangingLegPose(
        tap_fw::hangPose(tap_fw::board::kKneeExtensionStopRad), 0.0F);
    ControlCommand start;
    start.request_enable = true;
    start.request_fixture_release = true;
    start.target_joint_position_rad = {geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F,
                                       geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F};
    // Overdamped joint: c * dq = Kp_eff * (cmd - q) + feed-forward - load.
    const float kp_eff = 12.0F, c = 2.0F;
    const std::array<float, 6> load{2.2F, -2.2F, 0.0F, 2.2F, -2.2F, 0.0F};
    float worst_at_release = 0.0F;
    int ticks = 0;
    MotorCommand out{};
    while (hold.mode() != ControlMode::BalanceRamp && ticks < 15000) {
      ++air.imu_sample_sequence;
      ++air.encoder_sample_sequence;
      out = hold.update(air, start);
      for (std::size_t j : {0U, 1U, 3U, 4U}) {
        const float torque = kp_eff * out.kp_scale[j] * (out.position_rad[j] - air.joints.position_rad[j]) +
            out.feedforward_nm[j] - load[j];
        air.joints.position_rad[j] += torque / c * 0.002F;
      }
      ++ticks;
    }
    for (std::size_t j : {0U, 1U, 3U, 4U}) {
      worst_at_release = std::max(worst_at_release,
          std::abs(air.joints.position_rad[j] - start.target_joint_position_rad[j]));
    }
    std::printf("loaded legs: released after %.2f s, worst joint error %.4f rad, "
                "integral hip %+.2f knee %+.2f N.m\n", ticks * 0.002F, worst_at_release,
                hold.legPositionIntegralNm()[0], hold.legPositionIntegralNm()[1]);
    assert(hold.mode() == ControlMode::BalanceRamp);
    assert(worst_at_release < 0.03F);
  }

  // Serial tuning console.
  char reply[160]{};
  assert(!tap_fw::handleTuningLine("DRIVE 0 0", controller, reply, sizeof(reply)));
  assert(tap_fw::handleTuningLine("SET trim_gain 0.5", controller, reply, sizeof(reply)));
  assert(controller.lqrConfig().balance_trim_gain_rad_per_m == 0.5F);
  assert(tap_fw::handleTuningLine("SET trim_gain 99", controller, reply, sizeof(reply)));
  assert(controller.lqrConfig().balance_trim_gain_rad_per_m == 2.0F);
  assert(tap_fw::handleTuningLine("SET trim -0.9", controller, reply, sizeof(reply)));
  assert(controller.balanceTrim() == -cfg.lqr.balance_trim_max_rad);
  assert(tap_fw::handleTuningLine("SET margin 0.4", controller, reply, sizeof(reply)));
  assert(controller.lqrConfig().command_velocity_tracking_margin_m_s == 0.4F);
  assert(tap_fw::handleTuningLine("SET accel 1.0", controller, reply, sizeof(reply)));
  assert(controller.lqrConfig().max_ground_accel_m_s2 == 1.0F);
  assert(tap_fw::handleTuningLine("SET trim_gain abc", controller, reply, sizeof(reply)));
  assert(std::strstr(reply, "error") != nullptr);
  assert(controller.lqrConfig().balance_trim_gain_rad_per_m == 2.0F);
  assert(tap_fw::handleTuningLine("SET bogus 1", controller, reply, sizeof(reply)));
  assert(std::strstr(reply, "unknown key") != nullptr);
  assert(tap_fw::handleTuningLine("TRIM RESET", controller, reply, sizeof(reply)));
  assert(controller.balanceTrim() == 0.0F);
  assert(tap_fw::handleTuningLine("GET", controller, reply, sizeof(reply)));
  std::printf("%s\n", reply);
  assert(std::strncmp(reply, "TUNE trim_gain=2.000", 20) == 0);
  assert(tap_fw::handleTuningLine("SET pitch_scale 0.8", controller, reply, sizeof(reply)));
  assert(controller.lqrConfig().wheel_pitch_feedback_scale == 0.8F);
  std::puts("firmware_profile_tests: all checks passed");
  return 0;
}
