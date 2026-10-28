#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "robot_control/GeneratedBodyEnvelope.hpp"
#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {

// Flat-floor body-clearance lower estimate from IMU attitude and leg encoders.
// Wheels cannot penetrate the floor: this bounds the root height from below.
// Ignoring cylinder half-width is conservative. Body component boxes bound the
// CAD hulls from outside; they never create contacts or change simulator physics.
// Accuracy still depends on attitude, CAD, wheel deformation and floor slope.
inline float bodyClearance(float pitch, float roll,
                           const std::array<float, 2>& hip_cad,
                           const std::array<float, 2>& knee_cad,
                           float radius) noexcept {
  if (!std::isfinite(pitch) || !std::isfinite(roll) ||
      !std::isfinite(radius) || radius <= 0.0F)
    return std::numeric_limits<float>::quiet_NaN();
  const std::array<float, 3> up{
      -std::sin(pitch), std::cos(pitch) * std::sin(roll),
      std::cos(pitch) * std::cos(roll)};
  const float extent = radius * std::sqrt(std::max(0.0F, 1.0F - up[1] * up[1]));
  float root_lower = -std::numeric_limits<float>::infinity();
  for (unsigned side = 0; side < 2; ++side) {
    if (!std::isfinite(hip_cad[side]) || !std::isfinite(knee_cad[side]))
      return std::numeric_limits<float>::quiet_NaN();
    const auto leg = WholeBodyModel::legHeight(hip_cad[side], knee_cad[side]);
    const auto& hip = geometry::origins[3 * side];
    const float axle_y = hip.y + geometry::origins[3 * side + 1].y +
                         geometry::origins[3 * side + 2].y;
    const float axle_up = up[0] * (hip.x + leg.x_m) + up[1] * axle_y +
                          up[2] * (hip.z + leg.height_m);
    root_lower = std::max(root_lower, extent - axle_up);
  }
  float body_lower = std::numeric_limits<float>::infinity();
  for (const auto& box : geometry::body_envelope) {
    float lower = 0.0F;
    for (unsigned axis = 0; axis < 3; ++axis)
      lower += up[axis] * (up[axis] >= 0.0F ? box.lo[axis] : box.hi[axis]);
    body_lower = std::min(body_lower, lower);
  }
  return root_lower + body_lower;
}

// First-order flat-floor velocity damper for a COMMON hip-rate command.
// Wheel translation does not directly change height on a flat floor. Work
// in (physical COM lean, common hip) coordinates, matching the catch allocator:
// pitch = COM_lean - COM_line(hip,knee). The wheel-owned COM rate is a measured
// disturbance; changing hip allocation also changes the implied body pitch.
// No contact, effort or simulator state is used.
struct BodyClearanceDamper {
  float clearance_m = 0.0F;
  float clearance_rate_m_s = 0.0F;
  float predicted_clearance_m = 0.0F;
  float hip_rate_lower_rad_s = 0.0F;
  float hip_rate_upper_rad_s = 0.0F;
  bool valid = false;
  bool feasible = false;
};

inline BodyClearanceDamper bodyClearanceDamper(
    float pitch, float roll, const std::array<float, 3>& gyro,
    const std::array<float, 2>& hips, const std::array<float, 2>& knees,
    const std::array<float, 2>& hip_rates, const std::array<float, 2>& knee_rates,
    float radius, float clearance_min_m, float approach_time_s, float lag_s,
    float hip_rate_max_rad_s, float hip_command_lead_rad,
    float com_hip_jacobian) noexcept {
  BodyClearanceDamper out;
  for (float v : {pitch, roll, radius, clearance_min_m, approach_time_s, lag_s,
                  hip_rate_max_rad_s, hip_command_lead_rad, com_hip_jacobian,
                  gyro[0], gyro[1], gyro[2], hips[0], hips[1],
                  knees[0], knees[1], hip_rates[0], hip_rates[1], knee_rates[0], knee_rates[1]})
    if (!std::isfinite(v)) return out;
  if (radius <= 0.0F || approach_time_s <= 0.0F || lag_s < 0.0F ||
      clearance_min_m < 0.0F || hip_rate_max_rad_s < 0.0F) return out;
  const std::array<float, 3> up{
      -std::sin(pitch), std::cos(pitch) * std::sin(roll),
      std::cos(pitch) * std::cos(roll)};
  // World vertical expressed in the body: d(up)/dt = up x body_gyro.
  const std::array<float, 3> du{
      up[1]*gyro[2] - up[2]*gyro[1], up[2]*gyro[0] - up[0]*gyro[2],
      up[0]*gyro[1] - up[1]*gyro[0]};
  const std::array<float, 3> up_pitch{
      -std::cos(pitch), -std::sin(pitch)*std::sin(roll),
      -std::sin(pitch)*std::cos(roll)};
  const float radial = std::sqrt(std::max(0.0F, 1.0F - up[1]*up[1]));
  // Cylinder extent derivative is singular with the wheel axis vertical.
  if (radial < 0.1F) return out;
  const float extent = radius * radial;
  const float extent_rate = -radius * up[1] * du[1] / radial;
  const float extent_pitch = -radius * up[1] * up_pitch[1] / radial;
  unsigned active = 0;
  float root = -std::numeric_limits<float>::infinity();
  std::array<WholeBodyModel::LegHeight, 2> legs;
  std::array<std::array<float, 3>, 2> axles;
  for (unsigned side = 0; side < 2; ++side) {
    legs[side] = WholeBodyModel::legHeight(hips[side], knees[side]);
    const auto& origin = geometry::origins[3*side];
    axles[side] = {origin.x + legs[side].x_m,
                  origin.y + geometry::origins[3*side+1].y + geometry::origins[3*side+2].y,
                  origin.z + legs[side].height_m};
    const auto& a = axles[side];
    const float lower = extent - (up[0]*a[0] + up[1]*a[1] + up[2]*a[2]);
    if (lower > root) { root = lower; active = side; }
  }
  const auto& leg = legs[active];
  const auto& axle = axles[active];
  const float jh = -(up[0]*leg.d_x_d_hip + up[2]*leg.d_height_d_hip);
  const float jk = -(up[0]*leg.d_x_d_knee + up[2]*leg.d_height_d_knee);
  const float mean_hip_rate = 0.5F * (hip_rates[0] + hip_rates[1]);
  // Retain the measured side difference and knee motion as disturbances.
  const float joints_external = jh*(hip_rates[active] - mean_hip_rate) + jk*knee_rates[active];
  out.clearance_m = out.predicted_clearance_m = std::numeric_limits<float>::infinity();
  out.hip_rate_lower_rad_s = -hip_rate_max_rad_s;
  out.hip_rate_upper_rad_s = hip_rate_max_rad_s;
  bool possible = true;
  // All 8 corners of all 6 fixed body boxes: no derivative of min(corner),
  // which would miss an approaching opposite end during a corner switch.
  for (const auto& box : geometry::body_envelope) {
    for (unsigned mask = 0; mask < 8; ++mask) {
      float c = root, external = extent_rate + joints_external;
      float jp = extent_pitch;
      for (unsigned axis = 0; axis < 3; ++axis) {
        const float point = (mask & (1U << axis)) ? box.hi[axis] : box.lo[axis];
        c += up[axis] * point;
        external += du[axis] * (point - axle[axis]);
        jp += up_pitch[axis] * (point - axle[axis]);
      }
      const float measured_rate = external + jh*mean_hip_rate;
      // Exact Jacobian coordinate change at this configuration:
      // dc = Jpitch*d(COM_lean) + (Jhip-Jpitch*Jcom_hip)*dhip + ... .
      // Keep the measured COM-rate disturbance fixed for the hip projection.
      // Freezing body gyro instead would ask for MORE hip rotation to raise
      // the root while ignoring the opposite body rotation of COM capture.
      const float allocation_jh = jh - jp*com_hip_jacobian;
      const float allocation_external = external + jp*com_hip_jacobian*mean_hip_rate;
      // Reserve clearance for an already-issued position target as well as
      // measured closing motion during lag. A zero new rate does not cancel
      // a target that is still ahead of the encoder in the position servo.
      const float predicted = c + lag_s*std::min(measured_rate, 0.0F) +
          std::min(allocation_jh*hip_command_lead_rad, 0.0F);
      if (c < out.clearance_m) {
        out.clearance_m = c;
        out.clearance_rate_m_s = measured_rate;
      }
      out.predicted_clearance_m = std::min(out.predicted_clearance_m, predicted);
      // Velocity damper: dc/dt >= -(c_pred-c_min)/T. One-dimensional
      // projection idea from Ames et al., IEEE TAC 2017,
      // doi:10.1109/TAC.2016.2638961. This kinematic/lag approximation is
      // NOT a certified CBF for the underactuated robot's full dynamics.
      const float rhs = -(predicted - clearance_min_m) / approach_time_s - allocation_external;
      if (std::abs(allocation_jh) < 1.0e-5F) {
        if (rhs > 0.0F) possible = false;
      } else if (allocation_jh > 0.0F) {
        out.hip_rate_lower_rad_s = std::max(out.hip_rate_lower_rad_s, rhs/allocation_jh);
      } else {
        out.hip_rate_upper_rad_s = std::min(out.hip_rate_upper_rad_s, rhs/allocation_jh);
      }
    }
  }
  out.valid = std::isfinite(out.clearance_m) && std::isfinite(out.clearance_rate_m_s) &&
      std::isfinite(out.predicted_clearance_m) && std::isfinite(out.hip_rate_lower_rad_s) &&
      std::isfinite(out.hip_rate_upper_rad_s);
  out.feasible = out.valid && possible && out.hip_rate_lower_rad_s <= out.hip_rate_upper_rad_s;
  return out;
}

}  // namespace robot_control
