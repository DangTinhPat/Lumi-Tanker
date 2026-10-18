#include "tap_fw/v3/V3Controller.hpp"

#include "robot_control/BodyClearance.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/TunedConfig.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/BoardConfig.hpp"

namespace tap_fw::v3 {

// Sensor-derived COM/pitch, wheel prediction, and coordinated leg/roll control.

V3Controller::ComLine V3Controller::comLine(float hip_offset, float knee_offset) const noexcept {
  for (const auto& entry : com_cache_) {
    if (entry.valid && entry.hip == hip_offset && entry.knee == knee_offset &&
        entry.dx == config_.tier.body_com_dx_m) return entry.value;
  }
  namespace geo = robot_control::geometry;
  const float hip = LegHeight::hangHipCad() + hip_offset;
  const float knee = knee_offset;
  const auto body = robot_control::WholeBodyModel{}.compute(hip, knee, hip, knee);
  const auto jac = robot_control::WholeBodyModel::symmetricComOffsetJacobian(hip, knee);
  const float cx = body.com_offset_x_m +
      geo::parts[0].mass * config_.tier.body_com_dx_m / geo::total_mass_kg;
  const float cz = std::max(body.com_above_axle_m, 0.01F);
  const float den = cx * cx + cz * cz;
  const ComLine result{std::atan2(cx, cz),
          (cz * jac.d_com_offset_x_d_hip - cx * jac.d_com_above_axle_d_hip) / den,
          (cz * jac.d_com_offset_x_d_knee - cx * jac.d_com_above_axle_d_knee) / den,
          cz, body.pitch_inertia_at_axle_kg_m2};
  com_cache_[com_cache_next_] = {hip_offset, knee_offset, config_.tier.body_com_dx_m, result, true};
  com_cache_next_ = (com_cache_next_ + 1U) % com_cache_.size();
  return result;
}

float V3Controller::legModelError() const noexcept {
  // The sign guard/trace needs only the angle, not a second full Jacobian.
  namespace geo = robot_control::geometry;
  const float hip = LegHeight::hangHipCad() + hipCommand(), knee = kneeCommand();
  const auto model = robot_control::WholeBodyModel{}.compute(hip, knee, hip, knee);
  const float x = model.com_offset_x_m + geo::parts[0].mass *
      config_.tier.body_com_dx_m / geo::total_mass_kg;
  return c_meas_ - std::atan2(x, std::max(model.com_above_axle_m, 0.01F));
}

void V3Controller::updateVirtualPitch(const Input& in) {
  theta_v_ = in.pitch_rad;
  theta_v_rate_ = in.pitch_rate_rad_s;
  if (!config_.tier.virtual_pitch && !config_.tier.trunk) return;
  // Hang reference and the tier-2 gain at the height pose: recomputed only
  // when the model offset or the pose changes (the loop has ~0.4 ms spare).
  if (!(com_line_dx_ == config_.tier.body_com_dx_m) || !(j_hip_knee_ == pose_.knee_offset_rad)) {
    com_line_dx_ = config_.tier.body_com_dx_m;
    com_line_hang_ = comLine(0.0F, 0.0F).angle;
    j_hip_knee_ = pose_.knee_offset_rad;
    const ComLine pose = comLine(pose_.hip_offset_rad, pose_.knee_offset_rad);
    j_hip_ = pose.d_hip;
    c_pose_ = pose.angle;
  }
  if (!tierVirtual()) return;
  // Symmetric model on the mean of both legs (hand-arranged zeros differ a
  // little; the planar model has no roll anyway).
  hip_rate_meas_ = 0.5F * (in.hip_rate_rad_s[0] + in.hip_rate_rad_s[1]);
  knee_rate_meas_ = 0.5F * (in.knee_rate_rad_s[0] + in.knee_rate_rad_s[1]);
  hip_meas_ = 0.5F * (in.hip_rad[0] + in.hip_rad[1]);
  knee_meas_ = 0.5F * (in.knee_rad[0] + in.knee_rad[1]);
  const ComLine c = comLine(hip_meas_, knee_meas_);
  c_meas_ = c.angle;
  com_meas_height_m_ = c.height_m;
  theta_v_ = in.pitch_rad + c.angle - com_line_hang_;
  // Analytic rate from the driver velocities: differentiating the angle and
  // filtering it would add lag to the balancer's D channel.
  const Tier& t = config_.tier;
  float rate_gain = t.joint_rate_gain;
  if (t.joint_rate_gain_low >= 0.0F) {
    const float ratio = std::clamp(c.height_m / std::max(com_reference_m_, 0.01F), 0.1F, 1.0F);
    const float low_w = std::clamp((t.ratio_tall - ratio) / std::max(t.ratio_tall - t.ratio_low, 1.0e-3F),
                                   0.0F, 1.0F);
    rate_gain += low_w * (t.joint_rate_gain_low - t.joint_rate_gain);
  }
  theta_v_rate_ = in.pitch_rate_rad_s + rate_gain *
      (c.d_hip * hip_rate_meas_ + c.d_knee * knee_rate_meas_);
}

void V3Controller::updateImuVelocity(const Input& raw, bool free) {
  imu_velocity_ = ImuVelocity{};
  // Whole-body too: without it the pitch estimate read every acceleration
  // as tilt and kept ~0.026 rad of it seconds into the cruise (gyro-
  // dominant blend) - the WB then held the body that much off level and ran
  // ~15 % over the commanded speed (sim). The joints come from the raw
  // input (the tiers' hip_meas_ is only kept with the virtual pitch).
  const bool wb = wholeBody() && legs_measured_;
  if (!config_.tier.imu_kinematic || !free || !(tierVirtual() || wb)) return;
  // Roll makes the two legs asymmetric: average velocities AFTER FK, not
  // joint angles before FK. Use the same supported-leg kinematics as skyhook.
  robot_control::WholeBodyModel::PlanarVelocity v{};
  for (std::size_t side = 0; side < 2U; ++side) {
    const auto s = robot_control::WholeBodyModel::imuVelocityThroughLeg(
        raw.pitch_rad, raw.pitch_rate_rad_s,
        LegHeight::hangHipCad() + raw.hip_rad[side], raw.knee_rad[side],
        raw.hip_rate_rad_s[side], raw.knee_rate_rad_s[side], raw.wheel_rad_s[side],
        config_.wheel_radius_m);
    v.horizontal_m_s += 0.5F * s.horizontal_m_s;
    v.vertical_m_s += 0.5F * s.vertical_m_s;
  }
  imu_velocity_ = {v.horizontal_m_s, v.vertical_m_s, std::isfinite(v.horizontal_m_s) &&
                                                         std::isfinite(v.vertical_m_s)};
}

void V3Controller::updateWheelLegCoordination(const Input& in) noexcept {
  // A commanded height change already has a paired hip/knee trajectory
  // and balance feed-forward. Do not classify its own carrier motion as a
  // new external push. Use the existing tier law during that transition,
  // then smoothly hand back to disturbance damping once the plan is quiet.
  const bool moving = height_startup_ || knee_cmd_rad_ != kneeGoalRad() ||
      std::abs(height_speed_m_s_) > 1.0e-5F || std::abs(height_accel_m_s2_) > 1.0e-4F;
  if (moving || held_outer_off_ || held_quiet_) wheel_leg_quiet_since_us_ = 0U;
  else if (wheel_leg_quiet_since_us_ == 0U) wheel_leg_quiet_since_us_ = in.now_us;
  const bool settled = wheel_leg_quiet_since_us_ != 0U &&
      in.now_us - wheel_leg_quiet_since_us_ >= 1'000'000U;
  const float target = settled ? 1.0F : 0.0F;
  const float step = std::max(in.dt_s, 0.0F) / (settled ? 0.5F : 0.1F);
  wheel_leg_alpha_ += std::clamp(target-wheel_leg_alpha_, -step, step);
}

#ifdef TAP_SPLIT_HOST_PROTOTYPE
// Slow posture allocation and transient capture are separate tasks, following
// the task separation in wheel-legged whole-body control (IROS 2021,
// doi:10.1109/iros51168.2021.9636371). This bounded position-driver prototype
// is NOT full-body MPC; rotor-side FK remains an uncertain estimate.
float V3Controller::splitModelPosture() const noexcept {
  const float pitch = config_.tier.level_pitch_rad;
  const float depth = std::max(pose_.axle_below_hip_m * std::cos(pitch), LegHeight::minHeight());
  const float reach = LegHeight::maxHeight();
  const float xmax = std::sqrt(std::max(reach*reach-depth*depth, 0.0F));
  const float desired = com_line_hang_ + balanceRef() - pitch;
  float x = 0.0F;
  auto target = pose_;
  for (int i = 0; i < 5; ++i) {
    const auto at = LegHeight::forAxleOnPlane(x, depth, pitch);
    if (!at.ok) break;
    target.hip_offset_rad = at.hip_offset_rad;
    target.knee_offset_rad = at.knee_offset_rad;
    const float angle = comLine(at.hip_offset_rad, at.knee_offset_rad).angle;
    const auto next = LegHeight::forAxleOnPlane(std::min(x+0.0001F,xmax), depth,pitch);
    if (!next.ok || next.x_world_m-at.x_world_m < 1.0e-6F) break;
    const float jac = (comLine(next.hip_offset_rad,next.knee_offset_rad).angle-angle) /
                      (next.x_world_m-at.x_world_m);
    if (!std::isfinite(jac) || std::abs(jac)<0.1F) break;
    x = std::clamp(x+(desired-angle)/jac,-xmax,xmax);
  }
  return std::clamp(target.hip_offset_rad-pose_.hip_offset_rad,
                    -config_.tier.level_max_rad,config_.tier.level_max_rad);
}

void V3Controller::updatePosture(const Input& in, bool on, bool saturated) {
  const float dt = std::clamp(in.dt_s,0.0F,0.01F);
  const Split& p = config_.split;
  const float tau = std::max(p.posture_tau_s,1.5F);
  body_lp_ += std::clamp(dt*2.0F*3.14159265F*config_.tier.level_lpf_hz,0.0F,1.0F) *
              (body_pitch_-body_lp_);
  split_model_target_ = splitModelPosture();
  const float error = (body_lp_-config_.tier.level_pitch_rad) / std::max(j_hip_,0.2F);
  split_frozen_ = !on || saturated || split_alpha_b_>0.001F;
  // The learned wheel trim remains owned by LQR. A uses its bounded model
  // target, and integrates only body-level error (no second CoM integrator).
  const float increment = dt/tau*error;
  if (!split_frozen_ && (!axle_limited_ || increment*axle_limit_direction_<=0.0F))
    split_integral_ = std::clamp(split_integral_+increment,
                              -config_.tier.level_max_rad,config_.tier.level_max_rad);
  const float want = on ? std::clamp(split_model_target_+split_integral_,
                    -config_.tier.level_max_rad,config_.tier.level_max_rad) : 0.0F;
  const float rate = std::max(p.posture_rate_rad_s,0.0F)*dt;
  // tau governs the slow level adaptation. Do not filter the already
  // planned posture a second time: that delayed zero-offset correction.
  split_a_ += std::clamp(want-split_a_,-rate,rate);
}

void V3Controller::updateShock(const Input&, bool, float, float) {
  // Stage B is deliberately unavailable until the slow-posture gate passes.
  // A failed gate must not be hidden by turning the fast reflex back on.
  split_alpha_b_ = split_hold_s_ = split_b_ = 0.0F;
}

#endif

void V3Controller::updateLegLayers(const Input& in, bool active,
                                  std::array<float,2> wheel_cmd) {
#ifndef TAP_SPLIT_HOST_PROTOTYPE
  updateTrunk(in,active,wheel_cmd);
#else
  if (!config_.split.enabled) {
    split_was_enabled_=false;
    updateTrunk(in,active,wheel_cmd); return;
  }
  const float dt=std::clamp(in.dt_s,0.0F,0.01F);
  if (!split_was_enabled_) {
    split_a_=dhip_; split_b_=split_alpha_b_=split_hold_s_=split_quiet_s_=0.0F;
    split_integral_=0.0F; split_armed_=true; split_was_enabled_=true;
    body_lp_=body_pitch_;
  }
  // A is the continuously planned posture owner; it does not inherit the
  // old fast-reflex's two-second enable delay. Its own slew and the exact
  // workspace projection bound startup motion. Held/lifted blocks it.
  const bool on=active && config_.tier.trunk && tierVirtual() &&
    !held_outer_off_ && !held_quiet_;
  const float cap=std::max(config_.lqr.max_ground_velocity_m_s,0.05F)/config_.wheel_radius_m;
  const float wr=std::max(std::abs(wheel_cmd[0]),std::abs(wheel_cmd[1]))/cap;
  float plan=0.0F;
  if (config_.tier.preview && plan_n_>0U) plan=std::atan(planAt(config_.tier.preview_s).a/9.80665F);
  updateShock(in,on,wr,plan);
  updatePosture(in,on,wr>=config_.tier.wheel_sat_margin);
  split_preview_=on ? config_.tier.lean_share*plan/std::max(j_hip_,0.2F) : 0.0F;
  const float wanted=split_a_+split_b_+split_preview_;
  const float step=config_.tier.rate_rad_s*dt;
  dhip_+=std::clamp(wanted-dhip_,-step,step);
  trunk_alpha_=on ? 1.0F : 0.0F;
  level_i_=split_a_; level_beta_=on ? 1.0F : 0.0F;
  if (!active) { split_integral_=0.0F; split_b_=split_alpha_b_=0.0F; }
#endif
}

void V3Controller::updateTrunk(const Input& in, bool active,
                               std::array<float, 2> wheel_cmd_rad_s) {
  const Tier& t = config_.tier;
  // Settle after engage and again after every held/lifted spell: the
  // release catch is tier 1's alone.
  if (!active || held_outer_off_ || held_quiet_) trunk_block_us_ = in.now_us;
  const std::uint64_t since = std::max(state_since_us_, trunk_block_us_);
  const bool settled = in.now_us - since >=
      static_cast<std::uint64_t>(std::max(t.settle_s, 0.0F) * 1.0e6F);
  // The fast path sees whole-system lean, not body counter-rotation against the
  // joints. The slow level loop below sees the body itself, so static trim and
  // sensor/model offsets do not set its posture.
  const float lean = in.pitch_rad - (balanceRef() + ffPitch());
  float lean_plan_now = 0.0F, lean_plan_ahead = 0.0F;
  if (t.preview && tierVirtual() && plan_n_ > 0U) {
    const float g = 9.80665F;
    lean_plan_now = std::atan(planAt(t.preview_s).a / g);
    lean_plan_ahead = std::atan(planAt(std::max(t.preview_s - t.hip_advance_s, 0.0F)).a / g);
  }
  const float residual = lean - lean_plan_now;
  if (trunk_alpha_ <= 0.0F) {
    lean_lp_ = residual;
    lean_rate_f_ = 0.0F;
  } else if (t.hpf_s > 0.0F) {
    lean_lp_ += std::min(in.dt_s / t.hpf_s, 1.0F) * (residual - lean_lp_);
  } else {
    lean_lp_ = 0.0F;
  }
  const float rate_alpha = t.lead_lpf_hz > 0.0F
      ? std::clamp(2.0F * 3.14159265F * t.lead_lpf_hz * in.dt_s, 0.0F, 1.0F) : 1.0F;
  const float lean_rate_in = t.lead_gyro_only ? body_rate_ : in.pitch_rate_rad_s;
  lean_rate_f_ += rate_alpha * (lean_rate_in - lean_rate_f_);
  // Gain schedule from the CURRENT encoder/FK CoM height, not the requested
  // height. Smoothed over 50 ms so knee compliance/encoder noise does not modulate
  // the gains on every CAN sample. sqrt(l/g) is the pendulum's natural time scale
  // (Pratt et al., Capture Point, Humanoids 2006).
  const float observed_h = tierVirtual() ? com_meas_height_m_ : pose_.com_above_axle_m;
  const float measured_ratio = std::clamp(observed_h / std::max(com_reference_m_, 0.01F), 0.1F, 1.0F);
  if (!adaptive_init_ || !active) {
    adaptive_ratio_ = measured_ratio;
    adaptive_init_ = true;
  } else {
    adaptive_ratio_ += std::clamp(in.dt_s / 0.05F, 0.0F, 1.0F) * (measured_ratio-adaptive_ratio_);
  }
  const float ratio = adaptive_ratio_;
  const float span = std::max(t.ratio_tall - t.ratio_low, 1.0e-3F);
  const float low_w = std::clamp((t.ratio_tall - ratio) / span, 0.0F, 1.0F);
  const float exponent = t.height_exp_tall + low_w * (t.height_exp - t.height_exp_tall);
  const float share = t.lean_share * std::pow(ratio, std::max(exponent, 0.0F));
  const float jh = std::max(j_hip_, 0.2F);
  const float lead = t.lead_s * std::pow(ratio, std::max(t.lead_exp, 0.0F));
  const float plan_share = t.plan_full ? t.lean_share : share;
  const float disturbance_share = share * std::clamp(t.disturbance_share, 0.0F, 1.0F);
  const float fast_residual = residual - lean_lp_;
  const float disturbance_motion = std::max(std::abs(fast_residual)/0.02F,
                                            std::abs(lean_rate_f_)/0.20F);
  const float damping_weight = std::clamp((disturbance_motion - 0.1F)/0.9F, 0.0F, 1.0F);
  const float response = wheel_leg_alpha_ * damping_weight;
  const float rate_lead = lead * lean_rate_f_;
  const float plan_want = plan_share * lean_plan_ahead / jh;
  // Viscous damping idea: Hogan, Impedance Control Part I (1985),
  // doi:10.1115/1.3140702. This bounded position-driver proxy is not a torque
  // impedance controller or a passivity proof for delayed actuators. It is
  // allowed only with a resolved whole-system disturbance (response weight):
  // body/joint counter-rotation with a stationary axle->COM line keeps it at zero.
  // body_rate_ itself stays raw: held/armed detection and the reflex gate read it.
  if (trunk_alpha_ <= 0.0F || t.body_damp_lpf_hz <= 0.0F) {
    body_rate_damp_f_ = body_rate_;
  } else {
    body_rate_damp_f_ += std::clamp(2.0F * 3.14159265F * t.body_damp_lpf_hz * in.dt_s, 0.0F, 1.0F) *
        (body_rate_ - body_rate_damp_f_);
  }
  const float body_damping = std::max(t.body_damping_s, 0.0F) * std::sqrt(ratio) *
      response * body_rate_damp_f_ / jh;
  // Tier::hip_deadband_rad: small leans stay with the wheels.
  const float fast_signal = fast_residual + rate_lead;
  const float hip_db = std::max(t.hip_deadband_rad, 0.0F);
  const float fast_used = std::copysign(std::max(std::abs(fast_signal) - hip_db, 0.0F), fast_signal);
  float reflex_gate = 1.0F;
  if (t.reflex_gate) {
    if (trunk_alpha_ <= 0.0F) { reflex_w_ = 0.0F; gate_rate_lp_ = std::abs(body_rate_); }
    const float a = std::clamp(2.0F * 3.14159265F * 5.0F * in.dt_s, 0.0F, 1.0F);
    gate_rate_lp_ += a * (std::abs(body_rate_) - gate_rate_lp_);
    const float rate_m = (gate_rate_lp_ - t.reflex_rate_lo) /
        std::max(t.reflex_rate_hi - t.reflex_rate_lo, 1.0e-3F);
    const float lean_m = (std::abs(fast_residual) - t.reflex_lean_lo) /
        std::max(t.reflex_lean_hi - t.reflex_lean_lo, 1.0e-3F);
    const float want_w = std::clamp(std::max(rate_m, lean_m), 0.0F, 1.0F);
    const float up_g = in.dt_s / std::max(t.reflex_attack_s, 1.0e-3F);
    const float down_g = in.dt_s / std::max(t.reflex_release_s, 1.0e-3F);
    reflex_w_ += std::clamp(want_w - reflex_w_, -down_g, up_g);
    reflex_gate = reflex_w_;
  } else {
    reflex_w_ = 1.0F;
  }
  const float disturbance_want = reflex_gate * (disturbance_share * fast_used / jh +
      std::clamp(body_damping, -0.04F, 0.04F));
  // Full-authority desired hip offset (alpha = 1, unclamped): decides whether the
  // legs are running out of range, independent of how far alpha has faded
  // (see Tier::sat_margin).
  const float full_want = plan_want + disturbance_want;
  // Horizontal workspace at the current nominal gravity-frame depth. Dynamic
  // authority is limited here before the exact Cartesian projection in
  // coordinateTrunkAxle; the static level loop has its own limit and anti-windup.
  const float depth = std::max(pose_.axle_below_hip_m * std::cos(body_pitch_), LegHeight::minHeight());
  const float reach = LegHeight::maxHeight();
  const float xmax = std::sqrt(std::max((reach-depth)*(reach+depth), 0.0F));
  const float xbase = std::abs(pose_.axle_below_hip_m * std::sin(body_pitch_));
  const float travel = std::atan2(std::max(xmax-xbase, 0.0F), std::max(depth, 0.01F));
  const float max_rad = std::min(std::max(t.max_rad, 0.0F), travel);
  const float sched = t.adaptive_schedule ? std::sqrt(ratio) : 1.0F;
  const float adaptive_rate = std::max(t.rate_rad_s, 0.0F) * sched;
  const float level_tau = std::max(t.level_tau_s * sched, 0.5F);
  trunk_schedule_ = {ratio, lead, adaptive_rate, max_rad, jh, level_tau};
  const float margin = std::clamp(t.sat_margin, 0.0F, 0.99F);
  const float mag_ratio = std::abs(full_want) / std::max(max_rad, 1.0e-3F);
  // The fade is gated on the wheel command too (see Tier::sat_margin).
  const float wheel_cap = std::max(config_.lqr.max_ground_velocity_m_s, 0.05F) /
      std::max(config_.wheel_radius_m, 1.0e-4F);
  const float wheel_ratio = std::max(std::abs(wheel_cmd_rad_s[0] + wheel_carrier_rad_s_[0]),
                                    std::abs(wheel_cmd_rad_s[1] + wheel_carrier_rad_s_[1])) /
      wheel_cap;
  const bool wheel_saturating = wheel_ratio >= std::clamp(t.wheel_sat_margin, 0.0F, 1.0F);
  gimbal_frozen_ = wheel_saturating;
  const float range_ceiling = (!wheel_saturating || mag_ratio <= margin)
      ? 1.0F : std::clamp((1.0F - mag_ratio) / (1.0F - margin), 0.0F, 1.0F);
  // Not gated on !height_startup_: max_rad above already tracks the real
  // Cartesian workspace at the current height (near 0 at full extension, opening
  // as the reserve crouch proceeds); a second all-or-nothing gate only cost
  // settle time (v3_tier_tests V15/V17/V25).
  const bool on = active && settled && t.trunk && tierVirtual();
  const float target = on ? range_ceiling : 0.0F;
  const float up = t.ramp_s > 1.0e-3F ? in.dt_s / t.ramp_s : 1.0F;
  const float down = t.sat_fade_s > 1.0e-3F ? in.dt_s / t.sat_fade_s : 1.0F;
  const float diff = target - trunk_alpha_;
  trunk_alpha_ = std::clamp(trunk_alpha_ + std::clamp(diff, -down, up), 0.0F, 1.0F);
  const float dyn = std::clamp(trunk_alpha_ * full_want, -max_rad, max_rad);

  // Slow IMU body-level feedback (the "gimbal"), independent of the dynamic
  // virtual lean.
  const float lp = std::clamp(2.0F * 3.14159265F * t.level_lpf_hz * in.dt_s, 0.0F, 1.0F);
  body_lp_ += lp * (body_pitch_ - body_lp_);
  const bool level_on = on && t.level;
  const float lmax = std::max(t.level_max_rad, 0.0F);
  if (level_on && !level_init_) {
    // Seed from the measured error, not zero: level_beta_ below already ramps
    // 0 -> 1 over the same schedule as trunk_alpha_, so the output is bumpless
    // regardless of the seed; a zero seed costs a ~3*tau_s crawl (v3_tier_tests
    // V17/V25 settle-time bounds).
    level_i_ = std::clamp((body_lp_ - t.level_pitch_rad) / jh, -lmax, lmax);
    level_init_ = true;
  } else if (level_on && level_beta_ >= 1.0F && trunk_alpha_ >= 1.0F && !wheel_saturating) {
    const float increment = in.dt_s / level_tau *
        (body_lp_ - t.level_pitch_rad) / jh;
    // Do not wind farther into an unreachable axle target. Still allow
    // unwinding when the body error reverses, even at the workspace edge.
    if (!axle_limited_ || increment * axle_limit_direction_ <= 0.0F)
      level_i_ = std::clamp(level_i_ + increment, -lmax, lmax);
  }
  const float level_diff = (level_on ? 1.0F : 0.0F) - level_beta_;
  level_beta_ = std::clamp(level_beta_ + std::clamp(level_diff, -down, up), 0.0F, 1.0F);

  const float want = dyn + level_beta_ * level_i_;
  const float step = adaptive_rate * in.dt_s;
  dhip_ += std::clamp(want - dhip_, -step, step);
}

void V3Controller::predictWheels(const Input& raw, float* rate, float* pos) noexcept {
  const WheelPredict& w = config_.wheel_pred;
  const float dt = std::max(raw.dt_s, 1.0e-4F);
  auto ticks = [&](float s) {
    return std::min<std::size_t>(static_cast<std::size_t>(std::lround(std::max(s, 0.0F) / dt)), kWpBuf - 1U);
  };
  // Model of the wheel servo driven by the commands actually sent.
  wp_cmd_[wp_head_] = wp_last_cmd_;
  const std::size_t da = ticks(w.act_delay_s);
  const float u = wp_cmd_[(wp_head_ + kWpBuf - da) % kWpBuf];
  wp_w_model_ += std::min(dt / std::max(w.tau_s, 1.0e-3F), 1.0F) * (u - wp_w_model_);
  wp_p_model_ += wp_w_model_ * dt;
  wp_w_[wp_head_] = wp_w_model_;
  wp_p_[wp_head_] = wp_p_model_;
  const std::size_t dm = ticks(w.meas_delay_s);
  const std::size_t old = (wp_head_ + kWpBuf - dm) % kWpBuf;
  wp_head_ = (wp_head_ + 1U) % kWpBuf;
  if (!(w.enabled || (wholeBody() && w.with_wb))) return;
  *rate += wp_w_model_ - wp_w_[old];
  *pos += wp_p_model_ - wp_p_[old];
}

void V3Controller::updateRoll(const Input& in, bool active) {
  const Roll& r = config_.roll;
  // Body roll against GRAVITY: gyro, pulled toward the accelerometer with
  // the turn's centripetal pull (measured ground speed x measured yaw rate)
  // taken out. The first version estimated against the apparent vertical
  // directly - but the gyro cannot see a centripetal change, so at a turn's
  // entry the estimate read the lean-in as a tilt the wrong way for ~0.5 s
  // and the loop fought its own feed-forward (sim: still 0.005 rad off 4.9 s
  // into a 3 s turn). The lean the turn needs is the target instead.
  const float a_lp = std::clamp(2.0F * 3.14159265F * 5.0F * in.dt_s, 0.0F, 1.0F);
  roll_v_lp_ += a_lp * (ground_v_ - roll_v_lp_);
  roll_w_lp_ += a_lp * (in.yaw_rate_rad_s - roll_w_lp_);
  const float a_c = roll_v_lp_ * roll_w_lp_;  // centripetal [m/s^2], + = toward the left
  const float acc_roll = std::atan2(in.accel_y_m_s2 - a_c, std::max(in.accel_z_m_s2, 0.1F));
  const float amag = std::sqrt(in.accel_x_m_s2 * in.accel_x_m_s2 + in.accel_y_m_s2 * in.accel_y_m_s2 +
                               in.accel_z_m_s2 * in.accel_z_m_s2);
  if (!roll_init_) {
    roll_est_ = acc_roll;
    roll_init_ = true;
  } else {
    // The roll gyro arrives raw (the estimator only de-biases pitch): its
    // bias is learned from the same error (PI complementary filter, ~5 s).
    roll_est_ += (in.roll_rate_rad_s - roll_bias_) * in.dt_s;
    if (std::abs(amag - 9.80665F) < 2.5F) {
      const float tau = std::max(r.est_tau_s, 0.02F);
      const float err = acc_roll - roll_est_;
      roll_est_ += std::min(in.dt_s / tau, 1.0F) * err;
      roll_bias_ = std::clamp(roll_bias_ - in.dt_s / (tau * 5.0F) * err, -0.2F, 0.2F);
    }
  }
  // Filtered roll rate (bias removed): the reflex damping and the learning gate.
  roll_w_f_ += std::clamp(2.0F * 3.14159265F * 10.0F * in.dt_s, 0.0F, 1.0F) *
      ((in.roll_rate_rad_s - roll_bias_) - roll_w_f_);
  if (state_ != State::Balancing || !r.enabled) roll_fail_ = false;
  // Level reference: learned in place after free balancing has been calm for a
  // while, with the side offsets at zero (Roll::learn_ref). The same flat-floor
  // assumption as a manual calibration, but it follows each power-up.
  if (!active) {
    roll_ref_valid_ = false;
    roll_learn_n_ = 0;
    roll_learn_sum_ = 0.0F;
    roll_learn_t0_us_ = 0U;
    roll_active_since_us_ = 0U;
  } else if (r.learn_ref && !roll_ref_valid_) {
    if (roll_active_since_us_ == 0U) roll_active_since_us_ = in.now_us;
    const float since_s = static_cast<float>(in.now_us - roll_active_since_us_) * 1.0e-6F;
    if (since_s >= r.learn_delay_s) {
      const bool calm = std::abs(roll_w_f_) < r.learn_rate_rad_s &&
          std::abs(ground_v_) < r.learn_speed_m_s && std::abs(yaw_cmd_) < 1.0e-3F &&
          std::abs(roll_dh_) < 5.0e-4F;
      if (!calm) {
        roll_learn_n_ = 0;
        roll_learn_sum_ = 0.0F;
        roll_learn_t0_us_ = 0U;
      } else {
        if (roll_learn_t0_us_ == 0U) roll_learn_t0_us_ = in.now_us;
        roll_learn_sum_ += roll_est_;
        ++roll_learn_n_;
        if (static_cast<float>(in.now_us - roll_learn_t0_us_) * 1.0e-6F >= r.learn_window_s) {
          roll_ref_ = roll_learn_sum_ / static_cast<float>(roll_learn_n_);
          roll_ref_valid_ = true;
        }
      }
    }
  }
  const bool use_ref = r.learn_ref && roll_ref_valid_;
  const float ref = use_ref ? roll_ref_ : r.target_rad;
  const bool gimbal_open = !r.gimbal_link || (config_.tier.level && level_beta_ >= 0.99F);
  const bool level_ready = r.enabled && active && !roll_fail_ && gimbal_open &&
      (!r.learn_ref || roll_ref_valid_);
  roll_alpha_ += std::clamp((level_ready ? 1.0F : 0.0F) - roll_alpha_,
                            -in.dt_s / 0.5F, in.dt_s / std::max(r.ramp_s, 0.05F));
  // Lean-in target: the resultant of gravity and the centripetal pull
  // between the wheels (left turn, a_c > 0: left side down, roll < 0). With
  // cmd_preview it follows the commanded (slewed) yaw, which leads the wheels.
  const float a_lean = r.cmd_preview ? roll_v_lp_ * yaw_cmd_ : a_c;
  roll_target_ = ref - r.turn_ff * std::atan(std::clamp(a_lean / 9.80665F, -0.3F, 0.3F));
  roll_tgt_rate_f_ += std::clamp(2.0F * 3.14159265F * 10.0F * in.dt_s, 0.0F, 1.0F) *
      ((roll_target_ - roll_tgt_prev_) / std::max(in.dt_s, 1.0e-4F) - roll_tgt_rate_f_);
  roll_tgt_prev_ = roll_target_;
  const float half = 0.5F * std::max(r.track_m, 0.05F);
  // Feasible roll travel opens at the slow common-height rate. Keep the
  // normal roll slew so it can use that travel immediately on a cross slope.
  const float step = std::max(r.rate_m_s, 0.0F) * in.dt_s;
  if (!level_ready) {
    roll_sat_s_ = 0.0F;
    roll_dh_ -= std::clamp(roll_dh_, -step, step);  // back to even legs
    roll_i_ = roll_dh_;
    roll_e_lp_ = roll_est_ - roll_target_;
  } else {
    // Feed-forward: the side offset that sets the target roll on level
    // ground; the integral takes the ground and the legs' own error; the
    // proportional part reacts to a sudden tilt at once (gyro-fast).
    const float ff = half * std::tan(roll_target_ - ref);
    roll_e_lp_ += std::clamp(2.0F * 3.14159265F * std::max(r.err_lpf_hz, 0.1F) * in.dt_s, 0.0F, 1.0F) *
        ((roll_est_ - roll_target_) - roll_e_lp_);
    const float e = roll_e_lp_;  // + = left side too high
    // Shared hip/knee commands already include trunk, knee hold and
    // suspension. Reserve roll range around THAT pose, not the height plan.
    const auto common = robot_control::WholeBodyModel::legHeight(
        LegHeight::hangHipCad() + hipCommand(), kneeCommand());
    const float h = -common.height_m, x2 = common.x_m * common.x_m;
    const float hi = std::sqrt(std::max(LegHeight::maxHeight() * LegHeight::maxHeight() - x2, 0.0F));
    const float lo = std::sqrt(std::max(LegHeight::minHeight() * LegHeight::minHeight() - x2, 0.0F));
    const float room = std::max(std::min({r.max_dh_m, hi - h, h - lo}), 0.0F);
    const float tau = r.vel_ff ? r.tau_vff : r.tau_s;
    const float kp = r.vel_ff ? r.kp_vff : r.kp;
    if (!(r.gimbal_link && gimbal_frozen_)) {
      roll_i_ -= in.dt_s / std::max(tau, 0.02F) * half * std::sin(e);
    }
    roll_i_ = std::clamp(roll_i_, -room - ff, room - ff);
    const float p = -kp * half * std::sin(e);
    // Reflex: damping on the error rate (body rate minus the target's own rate).
    const float w_e = roll_w_f_ - roll_tgt_rate_f_;
    const float w_d = std::copysign(std::max(std::abs(w_e) - r.damp_db_rad_s, 0.0F), w_e);
    const float d = std::clamp(-r.damp_s * half * w_d, -r.damp_max_m, r.damp_max_m);
    const float want = std::clamp(roll_alpha_ * (roll_i_ + ff + p + d), -room, room);
    roll_dh_ += std::clamp(want - roll_dh_, -step, step);
    roll_dh_ = std::clamp(roll_dh_, -room, room);
    // Latch: pinned at the travel limit with the error still large = the loop
    // cannot level the body; fade out instead of fighting (Roll::latch).
    if (r.latch && room > 0.002F && std::abs(roll_dh_) >= 0.9F * room &&
        std::abs(roll_e_lp_) > r.fail_err_rad) {
      roll_sat_s_ += in.dt_s;
      if (roll_sat_s_ >= r.fail_s) roll_fail_ = true;
    } else {
      roll_sat_s_ = std::max(roll_sat_s_ - in.dt_s, 0.0F);
    }
  }
  // Side height offset -> joint offsets through the leg IK at the pose
  // (knee for the height, hip to keep the axle under the hip).
  // Exact per-side IK: retain the shared axle fore/aft coordinate while
  // changing its vertical distance. No cached linear Jacobian can become
  // stale when suspension/trunk changes the common pose at a fixed knee plan.
  const float hc = hipCommand(), kc = kneeCommand();
  const auto common = robot_control::WholeBodyModel::legHeight(LegHeight::hangHipCad() + hc, kc);
  std::array<float, 2> knee{}, hip{};
  for (std::size_t side = 0; side < 2U; ++side) {
    const float dh = (side == 0U ? 1.0F : -1.0F) * roll_dh_ * (r.sign < 0.0F ? -1.0F : 1.0F);
    if (std::abs(dh) < 1.0e-7F) continue;
    const float h = std::max(-common.height_m + dh, 0.0F);
    const auto p = LegHeight::forAxleOnPlane(common.x_m, h, 0.0F);
    if (!p.ok) continue;
    hip[side] = p.hip_offset_rad - hc;
    knee[side] = p.knee_offset_rad - kc;
  }
  for (std::size_t i = 0; i < 2U; ++i) {
    roll_knee_rate_[i] = r.vel_ff && in.dt_s > 0.0F ? (knee[i] - roll_knee_[i]) / in.dt_s : 0.0F;
    roll_hip_rate_[i] = r.vel_ff && in.dt_s > 0.0F ? (hip[i] - roll_hip_[i]) / in.dt_s : 0.0F;
  }
  roll_knee_ = knee;
  roll_hip_ = hip;
}

void V3Controller::coordinateTrunkAxle(float dt_s) noexcept {
  axle_hip_correction_ = 0.0F;
  axle_limited_ = false;
  axle_limit_direction_ = 0.0F;
  if (!kneeHoldOn()) { dknee_ = 0.0F; return; }
  const Tier& t = config_.tier;
  const bool follow = t.knee_follow_hz > 0.0F;
  // Below 2 micrometres of requested axle motion, avoid subtracting nearly
  // equal squared radii at full extension. Keep the nominal zero solution
  // (a following knee first walks back to it).
  if (std::abs(dhip_) < 1.0e-5F && !(follow && std::abs(dknee_) > 1.0e-5F)) {
    dknee_ = 0.0F;
    return;
  }
  // Keep the nominal pose's vertical projection at the CURRENT body pitch.
  // Thus dhip=0 introduces no new posture correction on the lifting stand
  // or during the max-height wheel catch. Cancel the arc caused by trunk
  // swing, including its pitch coupling, without making a second fast body
  // feedback loop. Height/suspension still own the nominal vertical target.
  const float h = pose_.axle_below_hip_m;
  // Preserve the existing tier's fore/aft calibration where its fixed-hip
  // solution is feasible. That provisional FK specifies only X; the paired
  // Cartesian solve below owns both final joints and the height constraint.
  const float provisional_knee = pose_.knee_offset_rad + kneeHold(dhip_);
  const auto provisional = robot_control::WholeBodyModel::legHeight(
      LegHeight::hangHipCad() + pose_.hip_offset_rad + dhip_, provisional_knee);
  const float requested_x = std::cos(body_pitch_)*provisional.x_m +
      std::sin(body_pitch_)*provisional.height_m;
  // At the deepest fold, h*cos(pitch) can fall below the supported minimum
  // radius. Keep the plane at least that deep so X can pass continuously
  // through zero instead of jumping between disconnected annulus branches.
  const float depth = std::max(h * std::cos(body_pitch_), LegHeight::minHeight());
  const auto target = LegHeight::forAxleOnPlane(requested_x, depth, body_pitch_);
  if (!target.ok) {
    axle_limited_ = true;
    if (!follow) { dknee_ = 0.0F; return; }
  }
  const float want_dknee = target.ok ? target.knee_offset_rad - pose_.knee_offset_rad : 0.0F;
  if (follow) {
    // dknee follows dhip^2: low-pass + rate limit it. The hip below is solved for
    // the knee actually sent, so the axle's fore/aft stays exact and only its
    // depth lags by the compensation's few millimetres.
    const float a = std::clamp(2.0F * 3.14159265F * t.knee_follow_hz * dt_s, 0.0F, 1.0F);
    const float step = std::max(t.knee_follow_rate_rad_s, 0.0F) * dt_s;
    dknee_ += std::clamp(a * (want_dknee - dknee_), -step, step);
    if (!target.ok) return;
    axle_hip_correction_ = LegHeight::hipForKneeAtX(
        pose_.knee_offset_rad + dknee_, target.x_world_m, body_pitch_) -
        pose_.hip_offset_rad - dhip_;
  } else {
    axle_hip_correction_ = target.hip_offset_rad - pose_.hip_offset_rad - dhip_;
    dknee_ = want_dknee;
  }
  axle_limited_ = target.limited;
  axle_limit_direction_ = target.x_world_m - requested_x;
}


}  // namespace tap_fw::v3
