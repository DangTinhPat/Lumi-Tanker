#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/v3/V3Controller.hpp"

using tap_fw::v3::V3Controller;
using tap_fw::v3::LegHeight;
using robot_control::WholeBodyModel;

static void readyWindowContract() {
  auto cfg = tap_fw::v3::defaultV3Config();
  cfg.engage.immediate = false;
  V3Controller ctrl(cfg);
  V3Controller::Input in{};
  in.legs_measured = in.legs_ready = true;
  in.accel_z_m_s2 = 9.80665F;
  V3Controller::Command cmd{};
  cmd.start = true;
  (void)ctrl.update(in, cmd);
  cmd.start = false;
  in.now_us = 1'300'000U;
  (void)ctrl.update(in, cmd);
  auto tick = [&](float pitch, float rate) {
    in.now_us += 2000U;
    in.pitch_rad = pitch;
    in.pitch_rate_rad_s = rate;
    return ctrl.update(in, cmd);
  };
  // Small vibrations on the original stand must not bypass the carry gate.
  for (int k = 0; k < 600; ++k) {
    assert(tick(0.001F*std::sin(k*0.2F), k%80 < 5 ? 0.16F : 0.01F).state == V3Controller::State::Ready);
  }
  assert(ctrl.readyPhase() == 0);
  (void)tick(0.10F, 0.6F);
  assert(ctrl.readyPhase() == 1);
  // Sustained movement, including motion below the rate threshold, cannot
  // arm: the body-angle span rejects a slowly rotating hand-held robot.
  for (int k = 0; k < 1000; ++k) (void)tick(0.10F+k*0.0001F, 0.05F);
  assert(ctrl.readyPhase() == 1);
  (void)tick(0.08F, 0.35F);  // gross movement restarts the whole window
  for (int k = 0; k < 300; ++k) (void)tick(0.08F, 0.01F);
  assert(ctrl.readyPhase() == 1);
  in.now_us += 100'000U;  // no accumulation across a control pause
  (void)tick(0.08F, 0.01F);
  for (int k = 0; k < 200; ++k) (void)tick(0.08F, 0.01F);
  assert(ctrl.readyPhase() == 1);
  // >70 ms disturbed in one run cancels the earlier quiet time.
  for (int k = 0; k < 40; ++k) (void)tick(0.08F, 0.16F);
  for (int k = 0; k < 200; ++k) (void)tick(0.08F, 0.01F);
  assert(ctrl.readyPhase() == 1);
  (void)tick(0.08F, 0.35F);
  const auto start_us = in.now_us;
  int armed_ms = -1;
  for (int k = 0; k < 450; ++k) {
    // 10 ms spike each 160 ms; near-threshold jitter otherwise. Hysteresis
    // and glitch budget admit these without losing the complete window.
    const float rate = k%80 < 5 ? 0.16F : (k%2 ? 0.07F : -0.10F);
    const auto out = tick(0.08F+0.0005F*std::sin(k*0.2F), rate);
    assert(out.state == V3Controller::State::Ready);
    if (ctrl.readyPhase() == 2) { armed_ms = (in.now_us-start_us)/1000; break; }
  }
  assert(armed_ms >= 700 && armed_ms <= 850);
  for (int k = 0; k < 5; ++k) assert(tick(0.0805F, 0.16F).state == V3Controller::State::Ready);
  (void)tick(0.0805F, 0.01F);
  for (int k = 0; k < 5; ++k) assert(tick(0.0805F, 0.16F).state == V3Controller::State::Ready);
  (void)tick(0.0805F, 0.01F);
  V3Controller::Output out{};
  for (int k = 0; k < 11; ++k) out = tick(0.09F, 0.2F);
  assert(out.state == V3Controller::State::Balancing);
  assert(std::abs(ctrl.balanceRef()-0.08F) < 0.001F);
  ctrl.returnToReady();
  for (int k = 0; k < 600; ++k) assert(tick(0.09F, 0.01F).state == V3Controller::State::Ready);
  assert(ctrl.readyPhase() == 0);  // new Ready entry has a fresh carry baseline
  cmd.disable = true;
  assert(tick(0.09F, 0).state == V3Controller::State::Idle);
  cmd.disable = false;
  cmd.start = true;
  (void)tick(0, 0);
  cmd.start = false;
  in.now_us += 1'300'000U;
  (void)tick(0, 0);
  assert(ctrl.readyPhase() == 0);
  std::printf("Ready window: repeated short vibrations arm in %d ms; stand, drift, pause, "
              "long disturbance and false release rejected\n", armed_ms);
}

static void adaptiveHeightContract() {
  auto cfg = tap_fw::v3::defaultV3Config();
  cfg.engage.immediate = true;
  cfg.tier.model_check_rad = cfg.fall_pitch_rad = 2;
  cfg.pid.held_dv_m_s = 1.0e6F;
  cfg.runaway_us = 1'000'000'000U;
  V3Controller ctrl(cfg);
  V3Controller::Input in{};
  in.legs_measured = in.legs_ready = true;
  in.accel_z_m_s2 = 9.80665F;
  V3Controller::Command cmd{};
  cmd.start = true;
  (void)ctrl.update(in, cmd);
  cmd.start = false;
  in.now_us = 1'300'000U;
  (void)ctrl.update(in, cmd);
  cmd.height_m = 0.084F;  // deliberately differs from the measured height
  float previous_lead = 1, previous_rate = 10;
  for (float height : {LegHeight::maxHeight()-0.012F, 0.110F, 0.084F}) {
    const auto actual = LegHeight::forHeight(height);
    in.hip_rad = {actual.hip_offset_rad, actual.hip_offset_rad};
    in.knee_rad = {actual.knee_offset_rad, actual.knee_offset_rad};
    in.leg_chain_rad = actual.hip_offset_rad + actual.knee_offset_rad;
    for (int k = 0; k < 1000; ++k) {
      in.now_us += 2000U;
      assert(ctrl.update(in, cmd).state == V3Controller::State::Balancing);
    }
    const auto s = ctrl.trunkSchedule();
    const float ratio = actual.com_above_axle_m / LegHeight::forKnee(0).com_above_axle_m;
    assert(std::abs(s.com_ratio-ratio) < 1.0e-4F);
    assert(std::abs(s.lead_s-cfg.tier.lead_s*std::sqrt(ratio)) < 1.0e-4F);
    assert(std::abs(s.rate_rad_s-cfg.tier.rate_rad_s*std::sqrt(ratio)) < 1.0e-4F);
    assert(std::abs(s.level_tau_s-std::max(cfg.tier.level_tau_s*std::sqrt(ratio), 0.5F)) < 1.0e-4F);
    assert(s.lead_s < previous_lead && s.rate_rad_s < previous_rate);
    previous_lead = s.lead_s;
    previous_rate = s.rate_rad_s;
    std::printf("adaptive actual height %.1f mm: ratio %.3f lead %.1f ms rate %.3f rad/s gain %.3f level %.3f s\n",
                height*1000, s.com_ratio, s.lead_s*1000, s.rate_rad_s, s.hip_to_lean_gain, s.level_tau_s);
  }
  // With measured legs absent, the trunk stays inert and must not reserve
  // height as though a deployable joint-based balance estimate existed.
  in.legs_measured = false;
  in.now_us += 2000U;
  (void)ctrl.update(in, cmd);
  assert(std::abs(ctrl.supportedMaxHeight()-LegHeight::maxHeight()) < 1.0e-6F);
}

static void workspaceAntiWindupContract() {
  auto cfg = tap_fw::v3::defaultV3Config();
  cfg.engage.immediate = true;
  cfg.height.startup_delay_s = 0;
  cfg.roll.reserve_m = 0;  // deliberately request the no-travel edge
  cfg.tier.settle_s = cfg.tier.ramp_s = 0;
  cfg.tier.model_check_rad = cfg.fall_pitch_rad = 2;
  cfg.pid.held_dv_m_s = 1.0e6F;
  cfg.runaway_us = 1'000'000'000U;
  cfg.lqr.max_ground_velocity_m_s = 100;  // isolate joint, not wheel saturation
  V3Controller ctrl(cfg);
  V3Controller::Input in{};
  in.legs_measured = in.legs_ready = true;
  in.accel_z_m_s2 = 9.80665F;
  V3Controller::Command cmd{};
  cmd.start = true;
  (void)ctrl.update(in, cmd);
  cmd.start = false;
  in.now_us = 1'300'000U;
  (void)ctrl.update(in, cmd);
  auto hold = [&](float pitch) {
    for (int k = 0; k < 2000; ++k) {
      in.now_us += 2000U;
      in.pitch_rad = pitch;
      const auto out = ctrl.update(in, cmd);
      assert(out.state == V3Controller::State::Balancing);
    }
  };
  hold(0.06F);
  const float positive = ctrl.levelHip();
  assert(ctrl.trunkWorkspaceLimited() && positive > 0 && positive < 0.01F);
  hold(-0.06F);
  const float negative = ctrl.levelHip();
  assert(ctrl.trunkWorkspaceLimited() && negative < 0 && negative > -0.01F);
  std::printf("workspace anti-windup and reversal: level %.6f -> %.6f rad\n", positive, negative);
}

static void wheelCarrierContract() {
  auto cfg = tap_fw::v3::defaultV3Config();
  cfg.engage.immediate = true;
  cfg.lock_settle_us = 0;
  cfg.roll.reserve_m = 0;
  cfg.lqr_deadband_comp_rad_s = 0;  // isolate the frame transform
  cfg.tier.wheel_carrier_gain = 1.0F;  // prove the exact kinematic inverse separately
  cfg.pid.held_dv_m_s = 1.0e6F;
  cfg.tier.model_check_rad = cfg.fall_pitch_rad = 2;
  auto legacy_cfg = cfg;
  legacy_cfg.tier.wheel_carrier_compensation = false;
  V3Controller corrected(cfg), legacy(legacy_cfg);
  V3Controller::Input in{};
  in.legs_measured = in.legs_ready = true;
  in.accel_z_m_s2 = 9.80665F;
  V3Controller::Command cmd{};
  cmd.start = true;
  (void)corrected.update(in, cmd);
  (void)legacy.update(in, cmd);
  cmd.start = false;
  in.now_us = 1'300'000U;
  (void)corrected.update(in, cmd);
  (void)legacy.update(in, cmd);
  // Finish startup and the automatic posture/coordination handover first.
  for (int k = 0; k < 2000; ++k) {
    in.now_us += 2000U;
    (void)corrected.update(in, cmd);
    (void)legacy.update(in, cmd);
  }
  in.pitch_rate_rad_s = 0.2F;
  in.hip_rate_rad_s = {0.6F, 1.2F};
  in.knee_rate_rad_s = {-0.1F, 0.3F};
  in.wheel_rad_s = {-0.7F, -1.7F};  // equal zero ground speed, different carriers
  for (int k = 0; k < 8; ++k) {
    in.now_us += 2000U;
    const auto a = corrected.update(in, cmd), b = legacy.update(in, cmd);
    assert(a.state == V3Controller::State::Balancing && b.state == a.state);
    for (std::size_t side = 0; side < 2; ++side) {
      const float carrier = in.pitch_rate_rad_s + in.hip_rate_rad_s[side] + in.knee_rate_rad_s[side];
      assert(std::abs((a.wheel_cmd_rad_s[side]+carrier)-b.wheel_cmd_rad_s[side]) < 1.0e-5F);
      assert(std::abs(corrected.wheelCarrierCorrection()[side]-carrier) < 1.0e-6F);
    }
  }
  // Planned height motion uses the established balance/IK path. The new
  // disturbance compensation must fade out without changing the wheel anchor.
  cmd.height_m = 0.110F;
  for (int k = 0; k < 100; ++k) {
    in.now_us += 2000U;
    const auto a = corrected.update(in, cmd);
    assert(a.state == V3Controller::State::Balancing && a.legs_moving);
  }
  assert(corrected.wheelCarrierCorrection()[0] == 0 &&
         corrected.wheelCarrierCorrection()[1] == 0);
  cmd.disable = true;
  const auto stopped = corrected.update(in, cmd);
  assert(stopped.state == V3Controller::State::Idle && !stopped.wheels_enabled);
  assert(stopped.wheel_cmd_rad_s[0] == 0 && stopped.wheel_cmd_rad_s[1] == 0);
  assert(corrected.wheelCarrierCorrection()[0] == 0 && corrected.wheelCarrierCorrection()[1] == 0);
  std::puts("wheel carrier: per-side native +Y forward/inverse consistent; disable clears correction");
}

static void analyticHeightContract() {
  float worst_height = 0, worst_hip = 0;
  for (int i = 0; i <= 300; ++i) {
    const float knee = LegHeight::kKneeMinRad * i/300.0F;
    float old_hip = LegHeight::hangHipCad()-0.52F*knee;
    for (int j = 0; j < 30; ++j) {
      const auto fk = WholeBodyModel::legHeight(old_hip, knee);
      if (std::abs(fk.x_m) < 1.0e-7F) break;
      old_hip -= fk.x_m / fk.d_x_d_hip;
    }
    const auto pose = LegHeight::forKnee(knee);
    const auto inverse = LegHeight::forHeight(pose.axle_below_hip_m);
    assert(pose.ok && inverse.ok);
    worst_hip = std::max(worst_hip,
        std::abs(LegHeight::hangHipCad()+pose.hip_offset_rad-old_hip));
    worst_height = std::max(worst_height,
        std::abs(inverse.axle_below_hip_m-pose.axle_below_hip_m));
    assert(std::abs(inverse.knee_offset_rad-knee) < 2.0e-5F);
  }
  assert(worst_height < 2.0e-7F && worst_hip < 2.0e-6F);
  std::printf("analytic height 301 poses: height residual %.9f m, legacy hip difference %.9f rad\n",
              worst_height, worst_hip);
}

static void axlePlaneContract() {
  float worst_z = 0, worst_x = 0, worst_nominal = 0;
  int limited = 0, coordinated = 0;
  for (float knee : {0.0F, -0.12F, -0.5F, -1.0F, LegHeight::kKneeMinRad}) {
    const auto nominal = LegHeight::forKnee(knee);
    const float h = nominal.axle_below_hip_m;
    for (float pitch : {-0.18F, -0.07F, 0.0F, 0.07F, 0.18F}) {
      for (float swing : {-0.25F, -0.1F, 0.0F, 0.1F, 0.25F}) {
        const float x = -h * std::sin(pitch + swing), depth = h * std::cos(pitch);
        const auto q = LegHeight::forAxleOnPlane(x, depth, pitch);
        assert(q.ok);
        const auto fk = WholeBodyModel::legHeight(LegHeight::hangHipCad() + q.hip_offset_rad,
                                                  q.knee_offset_rad);
        const float wx = std::cos(pitch)*fk.x_m + std::sin(pitch)*fk.height_m;
        const float wz = -std::sin(pitch)*fk.x_m + std::cos(pitch)*fk.height_m;
        worst_z = std::max(worst_z, std::abs(wz + depth));
        assert(q.knee_offset_rad >= LegHeight::kKneeMinRad && q.knee_offset_rad <= 0);
        if (q.limited) ++limited;
        else worst_x = std::max(worst_x, std::abs(wx - x));
        if (swing == 0) {
          worst_nominal = std::max({worst_nominal,
              std::abs(q.hip_offset_rad - nominal.hip_offset_rad), std::abs(q.knee_offset_rad - knee)});
        } else if (!q.limited && std::abs(q.knee_offset_rad - knee) > 0.001F &&
                   std::abs(q.hip_offset_rad - nominal.hip_offset_rad) > 0.001F) ++coordinated;
      }
    }
  }
  assert(worst_z < 2.0e-6F && worst_x < 2.0e-6F && worst_nominal < 1.0e-5F);
  assert(limited > 0 && coordinated > 20);
  // The old fixed-hip/clamped-knee path loses height at full extension.
  const auto old = WholeBodyModel::legHeight(LegHeight::hangHipCad() + 0.25F, 0);
  const float sag = LegHeight::maxHeight() + old.height_m;
  const auto q = LegHeight::forAxleOnPlane(-LegHeight::maxHeight()*std::sin(0.25F),
                                        LegHeight::maxHeight(), 0);
  assert(sag > 0.005F && q.ok && q.limited && std::abs(q.hip_offset_rad) < 1.0e-5F);
  assert(!LegHeight::forAxleOnPlane(0, 0, 0).ok);
  assert(!LegHeight::forAxleOnPlane(std::numeric_limits<float>::quiet_NaN(), 0.1F, 0).ok);
  const auto too_tall = LegHeight::forAxleOnPlane(0, 0.3F, 0);
  assert(too_tall.ok && too_tall.limited);
  std::printf("Cartesian IK: height residual %.9f m x residual %.9f m nominal %.9f rad; "
              "%d limited / %d coordinated; old straight-leg sag %.3f mm\n",
              worst_z, worst_x, worst_nominal, limited, coordinated, sag*1000);
}

static void trunkPlaneContract() {
  auto cfg = tap_fw::v3::defaultV3Config();
  cfg.engage.immediate = true;
  cfg.height.startup_delay_s = 0;
  cfg.height.startup_rate_m_s = cfg.height.rate_m_s = 0.06F;
  cfg.height.startup_accel_m_s2 = cfg.height.accel_m_s2 = 0.2F;
  cfg.tier.settle_s = cfg.tier.ramp_s = 0;
  cfg.tier.model_check_rad = cfg.fall_pitch_rad = 2;
  cfg.height.max_pitch_error_rad = cfg.height.max_pitch_rate_rad_s = 1000;
  cfg.pid.held_dv_m_s = 1.0e6F;
  cfg.runaway_us = 1'000'000'000U;
  cfg.roll.target_rad = 0;
  cfg.roll.enabled = true;
  V3Controller ctrl(cfg);
  V3Controller::Input in{};
  in.legs_measured = in.legs_ready = true;
  in.accel_z_m_s2 = 9.80665F;
  V3Controller::Command cmd{};
  cmd.start = true;
  (void)ctrl.update(in, cmd);
  cmd.start = false;
  in.now_us = 1'300'000U;
  (void)ctrl.update(in, cmd);
  cmd.height_m = 0.136F;
  float worst_z = 0, peak_hip = 0, peak_knee = 0;
  bool roll_seen = false;
  for (int k = 1; k <= 8000; ++k) {
    in.now_us += 2000U;
    const float t = k*0.002F;
    in.pitch_rad = t > 3 ? 0.03F * std::sin(1.2F*t) : 0;
    in.pitch_rate_rad_s = t > 3 ? 0.036F * std::cos(1.2F*t) : 0;
    in.accel_y_m_s2 = -9.80665F * std::sin(0.04F);
    in.accel_z_m_s2 = 9.80665F * std::cos(0.04F);
    ctrl.setKneeExcitation(t > 4 ? 0.02F * std::sin(4*t) : 0);
    const auto out = ctrl.update(in, cmd);
    assert(out.state == V3Controller::State::Balancing);
    const float pitch = in.pitch_rad;
    float mean_z = 0;
    for (std::size_t side = 0; side < 2; ++side) {
      const float hip = ctrl.hipCommand() + out.leg_roll_hip_rad[side];
      const float knee = ctrl.kneeCommand() + out.leg_roll_knee_rad[side];
      const auto fk = WholeBodyModel::legHeight(LegHeight::hangHipCad()+hip, knee);
      mean_z += 0.5F * (-std::sin(pitch)*fk.x_m + std::cos(pitch)*fk.height_m);
      in.hip_rate_rad_s[side] = (hip-in.hip_rad[side])/in.dt_s;
      in.knee_rate_rad_s[side] = (knee-in.knee_rad[side])/in.dt_s;
      in.hip_rad[side] = hip;
      in.knee_rad[side] = knee;
    }
    const float target_z = -ctrl.legPose().axle_below_hip_m*std::cos(pitch);
    worst_z = std::max(worst_z, std::abs(mean_z-target_z));
    peak_hip = std::max(peak_hip, std::abs(ctrl.trunkHip()));
    peak_knee = std::max(peak_knee, std::abs(ctrl.kneeCommand()-ctrl.legPose().knee_offset_rad));
    roll_seen = roll_seen || std::abs(ctrl.rollSideHeight()) > 0.001F;
    in.leg_chain_rad = 0.5F*(in.hip_rad[0]+in.knee_rad[0]+in.hip_rad[1]+in.knee_rad[1]);
  }
  std::printf("trunk + pitch + roll + pose: mean world-Z residual %.9f m hip/knee peaks %.4f/%.4f rad\n",
              worst_z, peak_hip, peak_knee);
  assert(worst_z < 3.0e-6F && peak_hip > 0.01F && peak_knee > 0.001F && roll_seen);
}

// Check startup separately from plant dynamics: a pending deep crouch must
// not inherit the slow roll-reserve speed for its entire travel.
static void startupContract() {
  auto cfg = tap_fw::v3::defaultV3Config();
  cfg.engage.immediate = true;
  cfg.roll.enabled = true;
  cfg.roll.target_rad = 0;
  cfg.tier.trunk = cfg.tier.suspension = false;
  cfg.tier.model_check_rad = cfg.fall_pitch_rad = 2;
  cfg.height.max_pitch_error_rad = cfg.height.max_pitch_rate_rad_s = 1000;
  cfg.pid.held_dv_m_s = 1.0e6F;
  cfg.runaway_us = 1'000'000'000U;
  cfg.lqr_trim_gain = 0;
  V3Controller ctrl(cfg);
  V3Controller::Input in{};
  in.legs_measured = in.legs_ready = true;
  in.accel_z_m_s2 = 9.80665F;
  V3Controller::Command cmd{};
  cmd.start = true;
  (void)ctrl.update(in, cmd);
  cmd.start = false;
  in.now_us = 1'300'000U;
  (void)ctrl.update(in, cmd);
  in.now_us += 2000U;
  cmd.height_m = 0.084F;
  assert(ctrl.update(in, cmd).state == V3Controller::State::Balancing);
  const auto engaged = in.now_us;
  const float reserve = LegHeight::maxHeight() - cfg.roll.reserve_m;
  float last = LegHeight::forKnee(ctrl.kneeCommand()).axle_below_hip_m;
  float startup_peak = 0, normal_peak = 0;
  float normal_peak_t = 0, normal_peak_profile_v = 0;
  bool normal_seen = false;
  for (int k = 0; k < 5000; ++k) {
    in.now_us += 2000U;
    assert(ctrl.update(in, cmd).state == V3Controller::State::Balancing);
    const float h = LegHeight::forKnee(ctrl.kneeCommand()).axle_below_hip_m;
    const float speed = std::abs(h - last) / in.dt_s;
    if (in.now_us - engaged < 790'000U) {
      assert(std::abs(h - LegHeight::maxHeight()) < 1.0e-6F);
    }
    if (ctrl.heightTarget() > reserve - 0.001F) {
      assert(std::abs(ctrl.heightTarget() - reserve) < 1.0e-5F);
      startup_peak = std::max(startup_peak, speed);
    } else {
      normal_seen = true;
      if (speed > normal_peak) {
        normal_peak = speed;
        normal_peak_t = (in.now_us - engaged) * 1.0e-6F;
        normal_peak_profile_v = ctrl.heightSpeed();
      }
    }
    last = h;
    in.hip_rad = {ctrl.hipCommand(), ctrl.hipCommand()};
    in.knee_rad = {ctrl.kneeCommand(), ctrl.kneeCommand()};
    in.leg_chain_rad = ctrl.hipCommand() + ctrl.kneeCommand();
    in.pitch_rad -= ctrl.virtualPitch();
  }
  std::printf("startup reserve speed %.6f m/s; deep-crouch speed %.6f m/s "
              "at %.3f s (profile %.6f); final %.6f m\n",
              startup_peak, normal_peak, normal_peak_t, normal_peak_profile_v, last);
  assert(startup_peak <= cfg.height.startup_rate_m_s * 1.15F);
  assert(normal_peak <= cfg.height.rate_m_s * 1.15F);
  assert(normal_seen && normal_peak > 1.5F * cfg.height.startup_rate_m_s);
  assert(std::abs(last - cmd.height_m) < 0.0002F);
  cmd.disable = true;
  in.now_us += 2000U;
  assert(ctrl.update(in, cmd).state == V3Controller::State::Idle);
  cmd.disable = false;
  cmd.start = true;
  in.now_us += 2000U;
  (void)ctrl.update(in, cmd);
  assert(ctrl.kneeCommand() == 0);
  cmd.start = false;
  in.now_us += 1'300'000U;
  (void)ctrl.update(in, cmd);
  in.now_us += 2000U;
  assert(ctrl.update(in, cmd).state == V3Controller::State::Balancing);
  assert(std::abs(LegHeight::forKnee(ctrl.kneeCommand()).axle_below_hip_m -
                  LegHeight::maxHeight()) < 1.0e-6F);
}

// Kinematic contracts, not a physics plant. The closed-loop evidence lives
// in v3_tier_tests and uses delayed/noisy encoders and finite servo torques.
int main() {
  readyWindowContract();
  adaptiveHeightContract();
  workspaceAntiWindupContract();
  wheelCarrierContract();
  analyticHeightContract();
  axlePlaneContract();
  trunkPlaneContract();
  startupContract();
  auto cfg = tap_fw::v3::defaultV3Config();
  cfg.engage.immediate = true;
  cfg.height.startup_delay_s = 0;
  cfg.height.startup_rate_m_s = cfg.height.rate_m_s = 0.06F;
  cfg.height.startup_accel_m_s2 = cfg.height.accel_m_s2 = 0.2F;
  cfg.tier.trunk = false;
  cfg.tier.model_check_rad = 2;
  cfg.fall_pitch_rad = 2;
  cfg.pid.held_dv_m_s = 1.0e6F;
  cfg.runaway_us = 1'000'000'000U;
  cfg.lqr_trim_gain = 0;
  cfg.roll.target_rad = 0;
  cfg.roll.enabled = true;
  V3Controller ctrl(cfg);
  V3Controller::Input in{};
  in.legs_measured = in.legs_ready = true;
  V3Controller::Command cmd{};
  cmd.start = true;
  (void)ctrl.update(in, cmd);
  cmd.start = false;
  in.now_us = 1'300'000U;
  (void)ctrl.update(in, cmd);
  cmd.go = true;
  cmd.height_m = 0.136F;
  float worst_x = 0, worst_h = 0;
  bool asymmetric = false;
  for (int k = 1; k <= 5000; ++k) {
    in.now_us += 2000U;
    const float t = k * 0.002F;
    // Roll plus common pose changes AFTER the nominal height stops. The
    // previous cache keyed only on knee_cmd_rad_ misses these changes.
    ctrl.setKneeExcitation(t > 4 ? 0.04F * std::sin(4.0F * t) : 0);
    in.accel_y_m_s2 = -9.80665F * std::sin(0.06F);
    in.accel_z_m_s2 = 9.80665F * std::cos(0.06F);
    const auto out = ctrl.update(in, cmd);
    cmd.go = false;
    assert(out.state == V3Controller::State::Balancing);
    const auto common = WholeBodyModel::legHeight(
        LegHeight::hangHipCad() + ctrl.hipCommand(), ctrl.kneeCommand());
    asymmetric = asymmetric || std::abs(ctrl.rollSideHeight()) > 0.002F;
    for (std::size_t side = 0; side < 2; ++side) {
      const float hip = ctrl.hipCommand() + out.leg_roll_hip_rad[side];
      const float knee = ctrl.kneeCommand() + out.leg_roll_knee_rad[side];
      const auto leg = WholeBodyModel::legHeight(LegHeight::hangHipCad() + hip, knee);
      const float dh = (side == 0 ? 1 : -1) * ctrl.rollSideHeight();
      worst_x = std::max(worst_x, std::abs(leg.x_m - common.x_m));
      worst_h = std::max(worst_h, std::abs(-leg.height_m + common.height_m - dh));
      assert(knee >= LegHeight::kKneeMinRad - 1.0e-5F && knee <= 1.0e-5F);
      in.hip_rad[side] = hip;
      in.knee_rad[side] = knee;
    }
    in.leg_chain_rad = 0.5F * (in.hip_rad[0] + in.knee_rad[0] + in.hip_rad[1] + in.knee_rad[1]);
    // This fixture holds the virtual angle, so it tests geometry only.
    in.pitch_rad -= ctrl.virtualPitch();
  }
  std::printf("combined roll/common pose: x residual %.9f m height residual %.9f m\n", worst_x, worst_h);
  assert(asymmetric && worst_x < 2.0e-6F && worst_h < 2.0e-6F);

  // Supported-leg velocity with constant body-frame height: hip and knee
  // Jacobian terms cancel. Knee-only velocity falsely sees vertical motion.
  const float hip = LegHeight::hangHipCad() + 0.3F, knee = -0.5F;
  const auto leg = WholeBodyModel::legHeight(hip, knee);
  const float hip_rate = 0.7F;
  const float knee_rate = -leg.d_height_d_hip * hip_rate / leg.d_height_d_knee;
  const auto v = WholeBodyModel::imuVelocityThroughLeg(0, 0, hip, knee, hip_rate, knee_rate, 0);
  const float knee_only = -leg.d_height_d_knee * knee_rate;
  std::printf("constant-height velocity: full %.9f m/s knee-only %.6f m/s\n", v.vertical_m_s, knee_only);
  assert(std::abs(v.vertical_m_s) < 1.0e-6F && std::abs(knee_only) > 0.001F);
  ctrl.config().roll.enabled = false;
  ctrl.setKneeExcitation(0);
  in.accel_y_m_s2 = in.accel_x_m_s2 = 0;
  in.accel_z_m_s2 = 9.80665F;
  in.pitch_rad = in.pitch_rate_rad_s = 0;
  in.hip_rad = {hip - LegHeight::hangHipCad(), hip - LegHeight::hangHipCad()};
  in.knee_rad = {knee, knee};
  in.hip_rate_rad_s = {hip_rate, hip_rate};
  in.knee_rate_rad_s = {knee_rate, knee_rate};
  in.leg_chain_rad = hip - LegHeight::hangHipCad() + knee;
  // A constant instantaneous kinematic state isolates the observer's DC
  // response. Let its slow gravity estimate settle after the roll fixture.
  for (int k = 0; k < 6000; ++k) {
    in.now_us += 2000U;
    const auto out = ctrl.update(in, cmd);
    assert(out.state == V3Controller::State::Balancing);
  }
  std::printf("controller constant-height skyhook speed %.9f m/s\n", ctrl.bodyVerticalSpeed());
  // Gravity LPF is float at ~10 m/s² with alpha=0.001: its quantization
  // floor produces ~0.15 mm/s through the 0.3 s observer leak.
  assert(std::abs(ctrl.bodyVerticalSpeed()) < 2.0e-4F);
  std::puts("v3_coordination_tests passed");
}
