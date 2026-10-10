#pragma once

#include <array>

namespace tap_fw::v3 {

// v3.5 jump planner. From the robot's own FK/IK (LegHeight, WholeBodyModel masses/CoM) and its
// actuator budget it works out HOW MUCH to dip, how fast to leave the ground and how to fold the
// legs in the air so that the wheels clear the floor by a target height, without turning the body.
//
//   wheel clearance at the apex = v^2 / 2g            CoM flight above the lift-off height
//                               + lift                wheels raised toward the CoM by the tuck
//
// The tuck follows the ZERO-REACTION leg path: in the air the angular momentum about the CoM stays
// zero, so a hip/knee motion turns the body at  -(L_h dhip + L_k dknee) / I.  Choosing
// dhip = -(L_k / L_h) dknee keeps the body level while the knee folds (L_h, L_k, I come from the link
// masses and inertias through the same FK as WholeBodyModel). The knee moves on a min-jerk profile
// whose peak acceleration stays inside the torque budget, so the legs can really follow it.
// A longer flight allows a deeper, gentler tuck; the planner takes the smallest CoM flight in
// [com_rise_min, com_rise_max] that reaches the clearance with the tuck the joints can perform.
struct JumpPlannerConfig {
  float target_clearance_m = 0.12F;   // wheel bottom to floor at the apex
  float com_rise_min_m = 0.04F;       // CoM flight above the lift-off height: smallest ...
  float com_rise_max_m = 0.08F;       //  ... and largest the planner may use
  float top_height_m = 0.165F;        // hip-to-axle height where the thrust ends
  float tuck_min_knee_rad = 0.25F;    // the knee never folds past this in the flight
  float knee_accel_rad_s2 = 347.2188F;   // peak knee acceleration of the min-jerk tuck (torque budget)
  float max_knee_rate_rad_s = 18.0F;
  float hold_s = 0.02F;               // dwell at the tucked pose
  float touchdown_margin_s = 0.0253F;   // legs are extended this long before touchdown
  float torque_limit_nm = 3.76F;      // per leg joint (8 A x 0.47 N.m/A)
  float torque_derate = 0.5F;         // share of it the thrust may use (the hips must also balance)
  float loss_factor = 1.3F;           // work margin for friction and actuator lag
  float min_dip_m = 0.02F;
  float min_crouch_height_m = 0.0F;   // 0 = LegHeight::minHeight() + 12 mm
};

struct JumpPlan {
  bool ok = false;
  const char* reason = "not planned";
  float crouch_height_m = 0.0F;       // dip: hip-to-axle height to start the thrust from
  float top_height_m = 0.0F;
  float dip_m = 0.0F;
  float takeoff_speed_m_s = 0.0F;     // CoM vertical speed at lift-off
  float com_rise_m = 0.0F;
  float flight_time_s = 0.0F;
  float predicted_tuck_lift_m = 0.0F;
  float predicted_clearance_m = 0.0F;
  float predicted_knee_travel_rad = 0.0F;
};

// The flight, planned at the moment the wheels leave the ground from what was MEASURED there: the
// CoM vertical speed v and the knee angle and rate (leg encoders through FK). Nothing in it depends on
// how the jump was started. T = 2 v / g (the legs are back at the top pose when the wheels touch).
//   knee   quintic from the measured (angle, rate) to the tuck, hold, min-jerk back to the top pose
//   hip    on the zero-reaction curve through the top pose, plus the measured offset fading out
struct FlightPlan {
  static constexpr int kGrid = 49;
  bool ok = false;
  const char* reason = "not planned";
  float vertical_speed_m_s = 0.0F;
  float flight_time_s = 0.0F;         // from the switch to the touchdown
  bool tuck_planned = false;          // planTuck() has run: kneeAt / hipAt follow the tuck
  float tuck_start_s = 0.0F;          // the tuck starts here (from the switch); before it the legs brake to the top pose
  float tuck_time_s = 0.0F;           // quintic from the measured state to the tuck
  float extend_start_s = 0.0F;        // min-jerk back to the top pose starts here (from the switch)
  float extend_time_s = 0.0F;
  float com_rise_m = 0.0F;
  float tuck_lift_m = 0.0F;           // wheels lifted toward the CoM at the apex
  float predicted_clearance_m = 0.0F;
  float peak_knee_accel_rad_s2 = 0.0F;
  float knee_start_rad = 0.0F;        // measured at the switch
  float knee_rate_start_rad_s = 0.0F;
  float hip_offset_cad_rad = 0.0F;    // measured hip minus the zero-reaction hip at the start (fades out)
  float knee_tuck_rad = 0.0F;
  float knee_top_rad = 0.0F;          // pose the legs return to
  float knee_lo_rad = 0.0F;           // grid of the zero-reaction curve through the top pose
  float knee_step_rad = 0.0F;
  std::array<float, kGrid> hip_cad_rad{};   // zero-reaction hip (CAD) on the grid
  std::array<float, kGrid> cab_m{};         // CoM above the axle on it
  // Builds the zero-reaction curve through the top pose (depends only on the configuration).
  bool buildCurve(const JumpPlannerConfig& config) noexcept;
  // Plans tuck, hold and extension from time t_now after the switch, from the measured state.
  bool planTuck(const JumpPlannerConfig& config, float vertical_speed_m_s, float t_now_s,
                float hip_cad_rad, float knee_rad, float knee_rate_rad_s) noexcept;
  [[nodiscard]] float zeroReactionHip(float knee_rad) const noexcept;
  [[nodiscard]] float cabAt(float knee_rad) const noexcept;
  [[nodiscard]] float kneeAt(float t_s) const noexcept;
  [[nodiscard]] float hipAt(float t_s) const noexcept;       // CAD angle
};

[[nodiscard]] FlightPlan planFlight(const JumpPlannerConfig& config, float vertical_speed_m_s,
                                    float hip_cad_rad, float knee_rad, float knee_rate_rad_s) noexcept;
[[nodiscard]] JumpPlan planJump(const JumpPlannerConfig& config) noexcept;

// Pitch rate (rad/s, about +Y like the CAD rotations) that a hip/knee motion causes in the air,
// from conservation of angular momentum about the CoM (both legs move together), and the hip
// rate that cancels a measured pitch rate: dhip = omega * I / L_h.
[[nodiscard]] float bodyRateFromLegMotion(float hip_cad_rad, float knee_rad, float hip_rate_rad_s,
                                          float knee_rate_rad_s) noexcept;
[[nodiscard]] float hipPerBodyRate(float hip_cad_rad, float knee_rad) noexcept;

// Whole-robot state that follows from the link model and the encoders/gyro (both legs together):
//   angular_momentum  about the CoM (about +Y)  L = I w + L_h dhip + L_k dknee;  conserved in the air
//   inertia           about the CoM
//   cab               CoM height above the axle
//   com_vz            CoM vertical speed relative to the axle (valid with the wheels on the ground)
struct BodyMomentum {
  float angular_momentum = 0.0F;
  float inertia = 0.0F;
  float cab = 0.0F;
  float com_vz = 0.0F;
};
// Angle the body turns when the legs move from one pose to another with zero total angular momentum.
[[nodiscard]] float bodyTurnFromLegPath(float hip0_cad, float knee0, float hip1_cad, float knee1) noexcept;
[[nodiscard]] BodyMomentum bodyMomentum(float hip_cad_rad, float knee_rad, float hip_rate_rad_s,
                                        float knee_rate_rad_s, float body_rate_rad_s) noexcept;

}  // namespace tap_fw::v3
