// Pins the tuned controller profile the ESP32 firmware runs, and walks it
// through the fixture startup path used on the robot.

#include <cmath>
#include <iostream>

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

bool near(float lhs, float rhs, float epsilon = 1.0e-6F) {
  return std::abs(lhs - rhs) <= epsilon;
}

bool finiteCommand(const MotorCommand& output) {
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(output.position_rad[i]) ||
        !std::isfinite(output.velocity_rad_s[i]) ||
        !std::isfinite(output.torque_nm[i]) ||
        !std::isfinite(output.kp_scale[i]) ||
        !std::isfinite(output.kd_scale[i]) ||
        !std::isfinite(output.feedforward_nm[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

int main() {
  const auto cfg = tunedControllerConfig();
  const EmbeddedController::Config defaults{};

  // Values the robot bring-up depends on; a silent drift back to the
  // defaults is exactly the failure this profile exists to prevent.
  check(near(cfg.leg_max_torque_nm, 3.0F), "leg torque envelope 3 N.m");
  check(!near(defaults.leg_max_torque_nm, cfg.leg_max_torque_nm),
        "profile differs from Config{} defaults");
  check(near(cfg.wheel_max_torque_nm, 0.13F), "wheel torque 0.13 N.m");
  check(near(cfg.lqr.sampling_period_s, 0.002F), "LQR solved for 500 Hz");
  check(near(cfg.lqr.wheel_radius_m, geometry::wheel_radius_m),
        "LQR wheel radius from geometry");
  check(near(cfg.lqr.fall_pitch_rad, 0.55F), "LQR fall pitch 0.55 rad");
  check(near(cfg.lqr.max_ground_velocity_m_s, 1.50F), "ground speed cap 1.5 m/s");
  check(near(cfg.hip_min_rad, geometry::native_hip_min_rad) &&
            near(cfg.hip_max_rad, geometry::native_hip_max_rad),
        "hip limits from native envelope");
  check(near(cfg.knee_min_rad, geometry::native_knee_min_rad) &&
            near(cfg.knee_max_rad, geometry::native_knee_max_rad),
        "knee limits from native envelope");
  check(near(cfg.whole_body_stance.vmc_max_torque_nm, 0.8F * 3.0F),
        "VMC capped at 80% of the leg envelope");
  check(!cfg.keep_legs_active_when_fallen, "fall stops every actuator");
  check(!cfg.flight_observer.torque_corroboration_enabled,
        "no motor-current input in the balance path");
  // Neutral pose must sit inside the tuned joint envelope.
  check(geometry::neutral_hip_rad > cfg.hip_min_rad &&
            geometry::neutral_hip_rad < cfg.hip_max_rad &&
            geometry::neutral_knee_rad > cfg.knee_min_rad &&
            geometry::neutral_knee_rad < cfg.knee_max_rad,
        "neutral pose inside joint limits");

  // Fixture startup with the tuned profile, robot still on the fixture.
  auto startup = cfg;
  startup.state_machine.fixture_startup_enabled = true;
  EmbeddedController controller(startup);
  RobotState state;
  state.dt_s = 0.002F;
  state.floor_contact = true;
  state.joints.position_rad = {geometry::neutral_hip_rad, geometry::neutral_knee_rad,
                               0.0F, geometry::neutral_hip_rad,
                               geometry::neutral_knee_rad, 0.0F};
  state.imu.sample_dt_s = 0.002F;
  ControlCommand command;
  command.target_joint_position_rad = state.joints.position_rad;
  command.request_enable = true;

  MotorCommand out{};
  bool qualified = false;
  for (unsigned i = 0; i < 120U && !qualified; ++i) {
    ++state.imu_sample_sequence;
    ++state.encoder_sample_sequence;
    out = controller.update(state, command);
    qualified = controller.mode() == ControlMode::FixtureHold;
  }
  check(qualified, "neutral pose qualifies into FixtureHold");
  check(out.enable_motors && finiteCommand(out), "FixtureHold command enabled and finite");
  check(near(out.torque_nm[0], 3.0F) && near(out.torque_nm[4], 3.0F),
        "FixtureHold uses the 3 N.m leg envelope");
  check(near(out.position_rad[0], geometry::neutral_hip_rad, 1.0e-5F) &&
            near(out.position_rad[1], geometry::neutral_knee_rad, 1.0e-5F),
        "FixtureHold holds the measured pose");

  command.request_fixture_release = true;
  bool balanced = false;
  bool all_finite = true;
  for (unsigned i = 0; i < 400U; ++i) {
    ++state.imu_sample_sequence;
    ++state.encoder_sample_sequence;
    out = controller.update(state, command);
    all_finite = all_finite && finiteCommand(out);
    if (controller.mode() == ControlMode::Balance) balanced = true;
  }
  check(balanced, "fixture release reaches Balance");
  check(all_finite, "balance commands stay finite");
  check(out.enable_motors && near(out.torque_nm[1], 3.0F),
        "Balance keeps the 3 N.m leg envelope");

  // Embedded LQR gain schedule: interpolated gains track the exact DARE
  // solve across the whole pendulum-length range.
  const auto maxRelativeError = [&](unsigned points) {
    LqrBalancer table(cfg.lqr);
    while (!table.buildGainTableStep(points)) {}
    LqrBalancer exact(cfg.lqr);
    float worst = 0.0F;
    const float lo = cfg.lqr.min_pendulum_length_m / cfg.lqr.pendulum_length_per_com_height;
    const float hi = cfg.lqr.max_pendulum_length_m / cfg.lqr.pendulum_length_per_com_height;
    for (int i = 0; i <= 400; ++i) {
      const float height = lo + (hi - lo) * static_cast<float>(i) / 400.0F;
      table.setComHeight(height);
      exact.setComHeight(height);
      const auto compare = [&](const LqrBalancer::StateVec& a,
                               const LqrBalancer::StateVec& b) {
        float scale = 1.0e-6F;
        for (float v : b) scale = std::max(scale, std::abs(v));
        for (std::size_t k = 0; k < a.size(); ++k) {
          worst = std::max(worst, std::abs(a[k] - b[k]) / scale);
        }
      };
      compare(table.activeGain(), exact.activeGain());
      compare(table.activeStationGain(), exact.activeStationGain());
    }
    return worst;
  };
  const float error_18 = maxRelativeError(18U);
  const float error_35 = maxRelativeError(35U);
  std::cout << "gain schedule max relative error: 18 pts " << error_18
            << ", 35 pts " << error_35 << '\n';
  check(error_35 < 0.01F, "35-point gain schedule within 1% of the exact solve");

  // With the schedule built, balancing with a moving COM height never falls
  // back to the online double-precision solve.
  {
    EmbeddedController scheduled(startup);
    while (!scheduled.buildLqrGainTableStep(35U)) {}
    check(scheduled.lqrGainTableReady(), "gain schedule reports ready");
    const unsigned solves_before = scheduled.lqrOnlineSolveCount();
    RobotState moving = state;
    ControlCommand release = command;
    for (unsigned i = 0; i < 600U; ++i) {
      ++moving.imu_sample_sequence;
      ++moving.encoder_sample_sequence;
      const float wobble = 0.25F * std::sin(static_cast<float>(i) * 0.02F);
      moving.joints.position_rad[1] = geometry::neutral_knee_rad + wobble;
      moving.joints.position_rad[4] = geometry::neutral_knee_rad + wobble;
      (void)scheduled.update(moving, release);
    }
    check(scheduled.mode() == ControlMode::Balance ||
              scheduled.mode() == ControlMode::Drive,
          "scheduled controller reaches Balance");
    check(scheduled.lqrOnlineSolveCount() == solves_before,
          "no online DARE solve while balancing on the schedule");
  }

  if (failures != 0) {
    std::cout << failures << " tuned-config check(s) failed\n";
    return 1;
  }
  std::cout << "tuned_config_tests: all checks passed\n";
  return 0;
}
