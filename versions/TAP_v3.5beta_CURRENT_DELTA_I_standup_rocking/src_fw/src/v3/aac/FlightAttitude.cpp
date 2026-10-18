#include "tap_fw/v3/aac/FlightAttitude.hpp"

#include <algorithm>
#include <cmath>

#include "tap_fw/v3/aac/AirborneModel.hpp"

namespace tap_fw::v3 {

FlightAttitude::Output FlightAttitude::update(const Input& in) noexcept {
  const Config& c = config_;
  Output out{};
  // Where the legs are commanded now (plan + current change) and the touchdown pose with it.
  const float h_cmd = in.hip_plan_cad + hip_cmd_, k_cmd = in.knee_plan + knee_cmd_;
  const float h_td0 = in.hip_td_cad + hip_cmd_, k_td0 = in.knee_td + knee_cmd_;
  // Turn on the way: back from the measured pose onto the commanded path, along the plan (the change
  // carried along), then the change of the change at the touchdown pose (solved below).
  const float turn_back = bodyTurnFromLegPath(in.hip_meas_cad, in.knee_meas, h_cmd, k_cmd);
  float hips[Input::kSamples], knees[Input::kSamples];
  for (int i = 0; i < Input::kSamples; ++i) {
    hips[i] = in.plan_hip_cad[i] + hip_cmd_;
    knees[i] = in.plan_knee[i] + knee_cmd_;
  }
  const float free_rot = freeRotation(in.angular_momentum, hips, knees, Input::kSamples, in.time_left_s);
  const float line_now = comLineAngle(in.hip_meas_cad, in.knee_meas, in.body_com_dx_m);
  const float hang = in.body_pitch + line_now - in.theta_v;          // theta_v = body + line - hang
  const float body_base = in.body_pitch + free_rot + turn_back + in.plan_turn;
  const float line_td0 = comLineAngle(h_td0, k_td0, in.body_com_dx_m);
  const float theta_base = body_base + line_td0 - hang;             // with the current change kept
  // Body pitch of the robot standing balanced at the nominal touchdown pose.
  out.body_ref = in.theta_ref - comLineAngle(in.hip_td_cad, in.knee_td, in.body_com_dx_m) + hang;
  const float theta_goal = in.theta_ref + c.line_bias_rad;
  // Linear model in the change of d around the current one (exact turn/line sensitivities there).
  const TouchdownSensitivity s = touchdownSensitivity(h_td0, k_td0, in.body_com_dx_m);
  const float e_line0 = theta_base - theta_goal;
  const float e_body0 = body_base - out.body_ref;
  float w_body = c.w_body;
  float dh = hip_cmd_, dk = knee_cmd_;
  for (int attempt = 0; attempt < 4; ++attempt) {
    // Normal equations for x = (d - d_cmd):  min wl (e_l + S_t x)^2 + wb (e_b + S_b x)^2 + ws |d_cmd + x|^2
    const float wl = c.w_line, wb = w_body, ws = c.w_shape;
    const float a11 = wl * s.theta_hip * s.theta_hip + wb * s.body_hip * s.body_hip + ws;
    const float a12 = wl * s.theta_hip * s.theta_knee + wb * s.body_hip * s.body_knee;
    const float a22 = wl * s.theta_knee * s.theta_knee + wb * s.body_knee * s.body_knee + ws;
    const float b1 = -(wl * s.theta_hip * e_line0 + wb * s.body_hip * e_body0 + ws * hip_cmd_);
    const float b2 = -(wl * s.theta_knee * e_line0 + wb * s.body_knee * e_body0 + ws * knee_cmd_);
    const float det = a11 * a22 - a12 * a12;
    if (!(std::abs(det) > 1.0e-9F)) break;
    const float x1 = (b1 * a22 - b2 * a12) / det;
    const float x2 = (a11 * b2 - a12 * b1) / det;
    dh = std::clamp(hip_cmd_ + x1, -c.hip_change_max_rad, c.hip_change_max_rad);
    dk = std::clamp(knee_cmd_ + x2, -c.knee_change_max_rad, c.knee_change_max_rad);
    const float body_err = e_body0 + s.body_hip * (dh - hip_cmd_) + s.body_knee * (dk - knee_cmd_);
    if (std::abs(body_err) <= c.body_tilt_max_rad) break;
    w_body *= 4.0F;                                                 // keep the body within its limit
  }
  out.target_hip_rad = dh;
  out.target_knee_rad = dk;
  // Move the commanded change so that it is complete shortly before the touchdown.
  const float to_go = std::max(in.time_left_s - c.arrive_before_s, c.arrive_min_s);
  const float step = std::clamp(in.dt_s / to_go, 0.0F, 1.0F);
  const float max_step = c.change_rate_max_rad_s * in.dt_s;
  hip_cmd_ += std::clamp(step * (dh - hip_cmd_), -max_step, max_step);
  knee_cmd_ += std::clamp(step * (dk - knee_cmd_), -max_step, max_step);
  out.hip_change_rad = hip_cmd_;
  out.knee_change_rad = knee_cmd_;
  // Prediction with the solution (exact end segment), for telemetry and tests.
  const float end_turn = bodyTurnFromLegPath(h_td0, k_td0, in.hip_td_cad + dh, in.knee_td + dk);
  out.body_td = body_base + end_turn;
  out.theta_v_td = out.body_td + comLineAngle(in.hip_td_cad + dh, in.knee_td + dk, in.body_com_dx_m) - hang;
  return out;
}

}  // namespace tap_fw::v3
