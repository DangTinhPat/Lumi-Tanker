// Adaptive balance angle (LqrBalancer balance trim) on a reduced wheeled
// inverted pendulum whose true equilibrium differs from the model reference
// by an unknown offset (IMU mounting tilt, CAD mass error, joint zero error).

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

#include "robot_control/EmbeddedController.hpp"
#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/LqrBalancer.hpp"
#include "robot_control/TunedConfig.hpp"

using namespace robot_control;

namespace {

int failures = 0;

void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}

constexpr float kDeg = 3.14159265F / 180.0F;

struct PlantResult {
  bool fell = false;
  float final_velocity_m_s = 0.0F;
  float final_pitch_error_rad = 0.0F;  // pitch - true equilibrium
  float trim_rad = 0.0F;
  float max_command_lead_m_s = 0.0F;   // |cmd - measured| after the release
  float max_speed_m_s = 0.0F;
};

struct PlantSpec {
  float equilibrium_rad = 0.0F;
  float release_pitch_rad = 0.0F;
  bool seed_trim = false;
  float trim_gain = 0.0F;
  float margin_m_s = 0.0F;
  float wheel_tracking = 1.0F;  // fraction of the commanded speed realized
  float seconds = 15.0F;
};

// Pendulum with a first-order wheel-speed servo (tau 40 ms) and a realizable
// acceleration limit, the shape seen on the robot's UART wheel drives.
PlantResult runPlant(const PlantSpec& spec) {
  auto cfg = tunedControllerConfig().lqr;
  cfg.balance_trim_gain_rad_per_m = spec.trim_gain;
  cfg.command_velocity_tracking_margin_m_s = spec.margin_m_s;
  LqrBalancer lqr(cfg);
  if (spec.seed_trim) lqr.setBalanceTrim(spec.release_pitch_rad);

  const float g = 9.80665F;
  const float dt = 0.002F;
  const float length = 0.22F;
  const float tau = 0.04F;
  const float accel_limit = 1.5F;
  std::mt19937 rng(3);
  std::normal_distribution<float> pitch_noise(0.0F, 0.004F);
  std::normal_distribution<float> speed_noise(0.0F, 0.02F);

  float x = 0.0F, v = 0.0F, th = spec.release_pitch_rad, w = 0.0F;
  RobotState s;
  s.dt_s = dt;
  s.imu.sample_dt_s = dt;
  s.floor_contact = true;
  const ControlCommand cmd{};
  PlantResult r;
  const int steps = static_cast<int>(spec.seconds / dt);
  for (int i = 0; i < steps; ++i) {
    s.imu.pitch_rad = th + pitch_noise(rng);
    s.imu.pitch_rate_rad_s = w;
    s.wheel.position_m = x;
    s.wheel.velocity_m_s = v + speed_noise(rng);
    const auto out = lqr.update(s, cmd, true, 0.0F, false);
    r.max_command_lead_m_s = std::max(
        r.max_command_lead_m_s,
        std::abs(lqr.debugState().commanded_ground_velocity_m_s - s.wheel.velocity_m_s));
    const float realized = spec.wheel_tracking * out.ground_velocity_m_s;
    const float a = std::clamp((realized - v) / tau, -accel_limit, accel_limit);
    v += a * dt;
    x += v * dt;
    w += (g * std::sin(th - spec.equilibrium_rad) - a * std::cos(th)) / length * dt;
    th += w * dt;
    r.max_speed_m_s = std::max(r.max_speed_m_s, std::abs(v));
    if (std::abs(th) > 0.7F) {
      r.fell = true;
      break;
    }
  }
  r.final_velocity_m_s = v;
  r.final_pitch_error_rad = th - spec.equilibrium_rad;
  r.trim_rad = lqr.balanceTrim();
  return r;
}

bool settled(const PlantResult& r) {
  return !r.fell && std::abs(r.final_velocity_m_s) < 0.1F &&
         std::abs(r.final_pitch_error_rad) < 0.02F;
}

}  // namespace

int main() {
  // Trim off, equilibrium 5 deg ahead of the model: no stationary solution,
  // the robot rolls away (the robot-log failure of 2026-09-26).
  {
    PlantSpec spec;
    spec.equilibrium_rad = 5.0F * kDeg;
    spec.release_pitch_rad = spec.equilibrium_rad;
    const auto r = runPlant(spec);
    check(!settled(r), "trim off: offset equilibrium does not settle");
  }

  // Seeded from the release pitch plus the slow integral: converges for
  // +-5 / +-8 deg equilibria with a +-2 deg hand error at release.
  bool all_settled = true;
  bool trim_tracks = true;
  for (float eq_deg : {5.0F, -5.0F, 8.0F, -8.0F}) {
    for (float hand_deg : {0.0F, 2.0F, -2.0F}) {
      PlantSpec spec;
      spec.equilibrium_rad = eq_deg * kDeg;
      spec.release_pitch_rad = (eq_deg + hand_deg) * kDeg;
      spec.seed_trim = true;
      spec.trim_gain = 0.3F;
      spec.margin_m_s = 0.25F;
      const auto r = runPlant(spec);
      if (!settled(r)) {
        all_settled = false;
        std::cout << "  eq " << eq_deg << " release " << hand_deg
                  << ": fell " << r.fell << " v " << r.final_velocity_m_s
                  << " pitch err " << r.final_pitch_error_rad << '\n';
      }
      if (std::abs(r.trim_rad - spec.equilibrium_rad) > 0.1F * std::abs(spec.equilibrium_rad)) {
        trim_tracks = false;
        std::cout << "  eq " << eq_deg << " release " << hand_deg
                  << ": trim " << r.trim_rad << '\n';
      }
    }
  }
  check(all_settled, "seeded trim settles for +-5/+-8 deg with +-2 deg release error");
  check(trim_tracks, "learned trim within 10% of the true equilibrium");

  // Stalling wheel (realizes 30 % of the command): anti-windup keeps the
  // command within the margin of the measured speed.
  {
    PlantSpec spec;
    spec.equilibrium_rad = 3.0F * kDeg;
    spec.release_pitch_rad = spec.equilibrium_rad;
    spec.seed_trim = true;
    spec.trim_gain = 0.3F;
    spec.margin_m_s = 0.25F;
    spec.wheel_tracking = 0.3F;
    spec.seconds = 3.0F;
    const auto r = runPlant(spec);
    // Measurement noise (sigma 0.02) sits on top of the clamp.
    check(r.max_command_lead_m_s <= 0.25F + 0.1F,
          "anti-windup bounds command lead over measured speed");
  }

  // Defaults off: identical to today's balancer.
  {
    const auto cfg = tunedControllerConfig().lqr;
    check(cfg.balance_trim_gain_rad_per_m == 0.0F &&
              cfg.command_velocity_tracking_margin_m_s == 0.0F,
          "trim and anti-windup default off in the tuned profile");
    LqrBalancer a(cfg);
    LqrBalancer b(cfg);
    b.setBalanceTrimGain(0.0F);
    RobotState s;
    s.dt_s = 0.002F;
    s.imu.sample_dt_s = 0.002F;
    s.floor_contact = true;
    s.imu.pitch_rad = 0.05F;
    s.wheel.velocity_m_s = 0.3F;
    bool identical = true;
    for (int i = 0; i < 200; ++i) {
      const auto oa = a.update(s, ControlCommand{}, true, 0.0F, false);
      const auto ob = b.update(s, ControlCommand{}, true, 0.0F, false);
      identical = identical && oa.ground_velocity_m_s == ob.ground_velocity_m_s;
    }
    check(identical && a.balanceTrim() == 0.0F, "trim gain 0 leaves the trim at 0");
  }

  // Controller-level seeding: releasing from FixtureHold at a tilted pose
  // seeds the trim with the measured pitch.
  {
    auto cfg = tunedControllerConfig();
    cfg.state_machine.fixture_startup_enabled = true;
    cfg.seed_balance_trim_from_pitch = true;
    cfg.lqr.balance_trim_max_rad = 0.2F;
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
    for (unsigned i = 0; i < 120U && controller.mode() != ControlMode::FixtureHold; ++i) {
      ++state.imu_sample_sequence;
      ++state.encoder_sample_sequence;
      (void)controller.update(state, command);
    }
    check(controller.mode() == ControlMode::FixtureHold, "seed test reaches FixtureHold");
    check(controller.balanceTrim() == 0.0F, "no trim before release");
    const float pitch_ref = controller.lqrDebugState().lqr_pitch_ref_rad;
    state.imu.pitch_rad = 0.06F;
    command.request_fixture_release = true;
    for (unsigned i = 0; i < 400U && !(controller.mode() == ControlMode::Balance ||
                                         controller.mode() == ControlMode::BalanceRamp);
         ++i) {
      ++state.imu_sample_sequence;
      ++state.encoder_sample_sequence;
      (void)controller.update(state, command);
    }
    const float trim = controller.balanceTrim();
    std::cout << "seeded trim " << trim << " (model ref " << pitch_ref << ")\n";
    check(std::abs(trim - 0.06F) < 0.03F, "release seeds the trim from the measured pitch");
  }

  if (failures != 0) {
    std::cout << failures << " balance-trim check(s) failed\n";
    return 1;
  }
  std::cout << "balance_trim_tests: all checks passed\n";
  return 0;
}
