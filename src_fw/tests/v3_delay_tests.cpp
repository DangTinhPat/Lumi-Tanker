#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "robot_control/LqrActuationPredictor.hpp"
#include "tap_fw/v3/V3Controller.hpp"

int main() {
  using Predictor = robot_control::LqrActuationPredictor;
  const auto defaults = tap_fw::v3::defaultV3Config();
  assert(!defaults.lqr_prediction.enabled);
  assert(!defaults.tier.reflex_inertial_gate);
  Predictor a, b;
  Predictor::Config cfg{true, .05F, .075F};
  Predictor::State quiet{};
  assert(a.predict(quiet, cfg, .002F, .1F, 0, 2) == quiet);
  for (int i = 0; i < 25; ++i) {
    (void)a.predict(quiet, cfg, .002F, .1F, 0, 2);
    a.record(.2F);
  }
  Predictor recovery;
  for (int i = 0; i < 25; ++i) recovery.record(.2F, cfg, .002F);
  // A new owner must see commands issued during recovery, not an empty queue.
  assert(recovery.predict(quiet, cfg, .002F, .1F, 0, 2) ==
         a.predict(quiet, cfg, .002F, .1F, 0, 2));
  const auto accelerating = a.predict(quiet, cfg, .002F, .1F, 0, 2);
  assert(accelerating[0] > 0 && accelerating[2] > 0);
  assert(accelerating[1] < 0 && accelerating[3] < 0);
  assert(accelerating[2] <= .1F + 1e-6F);  // bounded physical acceleration
  assert(b.predict(quiet, cfg, .002F, .1F, 0, 2) == quiet);  // independent instances
  a.reset();
  assert(a.predict(quiet, cfg, .002F, .1F, 0, 2) == quiet);
  const Predictor::State leaning{0, .01F, 0, .1F};
  const auto future = a.predict(leaning, cfg, .002F, .1F, 0, 2);
  assert(future[1] > leaning[1] && future[3] > leaning[3]);
  cfg.enabled = false;
  assert(a.predict(leaning, cfg, .002F, .1F, 0, 2) == leaning);
  // Oldest queued command acts first; the newest has less time to tip the body.
  cfg.enabled = true;
  a.reset(); b.reset();
  for (int i = 0; i < 25; ++i) {
    (void)a.predict(quiet, cfg, .002F, .1F, 0, 2);
    (void)b.predict(quiet, cfg, .002F, .1F, 0, 2);
    a.record(i == 0 ? .1F : 0);
    b.record(i == 24 ? .1F : 0);
  }
  assert(std::abs(a.predict(quiet, cfg, .002F, .1F, 0, 2)[1]) >
         std::abs(b.predict(quiet, cfg, .002F, .1F, 0, 2)[1]));
  cfg = {true, .1F, .075F};
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 10000; ++i) {
    const auto maximum_queue = a.predict(leaning, cfg, .00025F, .1F, 0, 2);
    assert(std::isfinite(maximum_queue[3]));
    a.record(.01F);
  }
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now()-start).count();
  std::printf("delay predictor: queue order, reset, independent instances, bounded acceleration, flag-off passed; host 400-step mean %.1f us\n", ns/10000.0/1000.0);

  using tap_fw::v3::V3Controller;
  auto control = defaults;
  control.engage.immediate = true;
  control.engage.wb_armed_only = false;
  control.tier.reflex_gate = control.tier.reflex_inertial_gate = true;
  control.tier.settle_s = 0;
  control.initial_knee_rad = tap_fw::v3::LegHeight::kKneeMaxRad;
  V3Controller c(control);
  V3Controller::Input in{};
  V3Controller::Command cmd{};
  in.legs_ready = in.legs_measured = true;
  in.pitch_rad = .02F;
  in.accel_x_m_s2 = -9.80665F*std::sin(in.pitch_rad);
  in.accel_z_m_s2 = 9.80665F*std::cos(in.pitch_rad);
  cmd.start = true;
  V3Controller::Output output{};
  auto step = [&]() {
    in.hip_rad = {c.hipCommand(), c.hipCommand()};
    in.knee_rad = {c.kneeCommand(), c.kneeCommand()};
    in.leg_chain_rad = c.hipCommand()+c.kneeCommand();
    in.now_us += 2000;
    output = c.update(in, cmd);
    cmd.start = false;
  };
  for (int i = 0; i < 2500; ++i) step();
  assert(output.state == V3Controller::State::Balancing);
  assert(c.reflexGate() == 0);  // static lean/gravity alone cannot fire the reflex
  in.pitch_rate_rad_s = .25F;
  in.accel_x_m_s2 += 1.0F;
  for (int i = 0; i < 50; ++i) step();
  assert(c.reflexGate() > .1F);
  in.sensors_ok = false;
  step();
  assert(output.state == V3Controller::State::Fault && !output.wheels_enabled);
  cmd.disable = true;
  step();
  assert(output.state == V3Controller::State::Idle && !output.wheels_enabled);
  std::puts("inertial reflex: quiet static lean blocked, motion triggers, sensor fault and disable stop wheels");

  auto hip_rate_countercase = [&](bool gyro_only) {
    auto test_cfg = defaults;
    test_cfg.initial_knee_rad = 1.0F;
    test_cfg.engage.immediate = true;
    test_cfg.engage.wb_armed_only = false;
    test_cfg.height.startup_delay_s = 1000;
    test_cfg.tier.preview = test_cfg.tier.level = false;
    test_cfg.tier.body_damping_s = 0;
    test_cfg.tier.settle_s = 0;
    test_cfg.tier.lead_gyro_only = gyro_only;
    test_cfg.tier.reflex_gate = test_cfg.tier.reflex_inertial_gate = true;
    V3Controller controller(test_cfg);
    V3Controller::Input sample{};
    sample.legs_ready = sample.legs_measured = true;
    sample.pitch_rad = .02F;
    sample.accel_x_m_s2 = -9.80665F*std::sin(sample.pitch_rad);
    sample.accel_z_m_s2 = 9.80665F*std::cos(sample.pitch_rad);
    V3Controller::Command command{};
    command.start = true;
    for (int i = 0; i < 2250; ++i) {
      const auto pose = controller.legPose();
      sample.hip_rad = {pose.hip_offset_rad, pose.hip_offset_rad};
      sample.knee_rad = {pose.knee_offset_rad, pose.knee_offset_rad};
      sample.leg_chain_rad = pose.hip_offset_rad+pose.knee_offset_rad;
      if (i == 2200) {
        sample.accel_x_m_s2 += .8F;  // opens the event gate
        sample.hip_rate_rad_s = {.5F, .5F};  // encoder-rate disturbance; body gyro stays zero
      }
      sample.now_us += 2000;
      const auto result = controller.update(sample, command);
      command.start = false;
      assert(result.state != V3Controller::State::Fault);
    }
    assert(controller.reflexGate() > .1F);
    return controller.trunkSwingRequest();
  };
  const float virtual_lead = hip_rate_countercase(false);
  const float gyro_lead = hip_rate_countercase(true);
  assert(std::abs(virtual_lead) > .01F);
  assert(std::abs(gyro_lead) < 1e-4F);
  std::printf("hip lead countercase: encoder-rate disturbance feeds %.5f rad into virtual lead, %.5f rad into gyro lead\n",
              virtual_lead, gyro_lead);

  // Independent planar acceleration plant: transport delay + speed lag, with
  // the same integrated LQR/lead seam. This is model evidence, not hardware.
  for (float length : {.08F, .12F, .20F}) {
    for (float delay : {.04F, .06F}) for (float lag : {.06F, .09F}) {
      auto lc = defaults.lqr;
      lc.min_pendulum_length_m = .035F;
      lc.max_pendulum_length_m = .24F;
      lc.max_ground_accel_m_s2 = 2.083333F;
      robot_control::LqrBalancer balance(lc);
      balance.setComHeight(length/lc.pendulum_length_per_com_height);
      balance.reset(0, true);
      Predictor predictor;
      Predictor::Config pc{true, delay, lag};
      Predictor::State plant{0, .02F, 0, 0};
      std::array<float, 50> commands{};
      const auto ticks = static_cast<unsigned>(std::lround(delay/.002F));
      unsigned head = 0;
      float accel_filter = 0;
      for (int i = 0; i < 15000; ++i) {
        if (i == 5000) plant[3] += .1F;
        const auto predicted = predictor.predict(plant, pc, .002F, length, 0, lc.max_ground_accel_m_s2);
        robot_control::RobotState sensor{};
        sensor.dt_s = .002F;
        sensor.imu.pitch_rad = predicted[1];
        sensor.imu.pitch_rate_rad_s = predicted[3];
        sensor.wheel.position_m = predicted[0];
        sensor.wheel.velocity_m_s = predicted[2];
        const auto result = balance.update(sensor, {}, true, 0, true);
        accel_filter += (.002F*2*3.14159265F*20) *
            (balance.debugState().optimal_command_m_s-accel_filter);
        const float sent = std::clamp(result.ground_velocity_m_s+lag*accel_filter,
                                     -lc.max_ground_velocity_m_s, lc.max_ground_velocity_m_s);
        predictor.record(sent);
        const float delayed = commands[head];
        commands[head] = sent;
        head = (head+1)%ticks;
        const float accel = std::clamp((delayed-plant[2])/lag,
                                      -lc.max_ground_accel_m_s2, lc.max_ground_accel_m_s2);
        const float dd = (9.81F*plant[1]-accel)/length;
        plant[0] += .002F*plant[2]+.000002F*accel;
        plant[1] += .002F*plant[3]+.000002F*dd;
        plant[2] += .002F*accel;
        plant[3] += .002F*dd;
        assert(std::abs(plant[1]) < .15F);
        if (i > 14000) assert(std::abs(plant[2]) < .02F);
      }
    }
  }
  std::puts("planar delay feedback: 12 height/delay/lag cases recovered initial lean and angular impulse");
}
