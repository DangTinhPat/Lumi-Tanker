#pragma once

#include <array>

#include "tap_fw/v3/aac/AirborneModel.hpp"

namespace tap_fw::v3 {

// v3.5 jump planner. From the robot's own FK/IK (LegHeight, WholeBodyModel masses/CoM) and its
// actuator budget it works out HOW MUCH to dip, how fast to leave the ground and how to fold the
// legs in the air so that the wheels clear the floor by a target height, without turning the body.
//
//   wheel clearance at the apex = v^2 / 2g            CoM flight above the lift-off height
//                               + lift                wheels raised toward the CoM by the tuck
//
// The tuck moves BOTH joints. In the air the angular momentum about the CoM stays zero, so a hip/knee
// motion turns the body at  -(L_h dhip + L_k dknee) / I  (L_h, L_k, I from the link masses and inertias
// through the same FK as WholeBodyModel). On the ZERO-REACTION curve dhip = -(L_k / L_h) dknee the body
// does not turn at all; the tuck path leaves that curve by  dev * s  (s = 0 at the top pose, 1 at the
// tuck), i.e. the hip pulls the thigh up further, which lifts the wheels more while the body turns by at
// most tuck_body_turn_rad. Out and back on the same path, so the turn is undone at the touchdown.
// Both joints follow min-jerk profiles whose peak accelerations stay inside their torque budgets.
// A longer flight allows a deeper, gentler tuck; the planner takes the smallest CoM flight in
// [com_rise_min, com_rise_max] that reaches the clearance with the tuck the joints can perform.
struct JumpPlannerConfig {
  float target_clearance_m = 0.17F;   // wheel bottom to floor at the apex
  float com_rise_min_m = 0.04F;       // CoM flight above the lift-off height: smallest ...
  float com_rise_max_m = 0.0824F;       //  ... and largest the planner may use
  float top_height_m = 0.165F;        // hip-to-axle height where the thrust ends
  float tuck_min_knee_rad = 0.25F;    // the knee never folds past this in the flight
  float knee_accel_rad_s2 = 1293.31F;   // peak knee acceleration of the min-jerk tuck (torque budget)
  float max_knee_rate_rad_s = 40.0F;
  float hip_accel_rad_s2 = 1408.92F;    // peak hip acceleration along the tuck path (torque budget)
  float tuck_body_turn_rad = 0.0F;    // the hip may pull the thigh up off the zero-reaction curve while
                                      // the body turns at most this much mid-flight (0 = body held level);
                                      // the legs come back on the same path, so the turn is undone at touchdown
  float hold_s = 0.0188F;                // dwell at the tucked pose
  float tuck_done_frac = 0.3528F;        // the tuck is complete at this fraction of the flight (0.5 = the CoM apex)
  float extend_fraction = 0.7981F;       // the legs are lowered this far back toward the top pose by the touchdown (0.7-0.9)
  float touchdown_margin_s = 0.0389F;   // legs are extended this long before touchdown
  float torque_limit_nm = 3.76F;      // per leg joint (8 A x 0.47 N.m/A)
  float torque_derate = 0.5F;         // share of it the thrust may use (the hips must also balance)
  float loss_factor = 1.3F;           // work margin for friction and actuator lag
  float min_dip_m = 0.02F;
  float min_crouch_height_m = 0.0F;   // 0 = LegHeight::minHeight() + 12 mm
};

// The flight, planned at the moment the wheels leave the ground from what was MEASURED there: the
// CoM vertical speed v and the knee angle and rate (leg encoders through FK). Nothing in it depends on
// how the jump was started. T = 2 v / g (the legs are back at the top pose when the wheels touch).
//   knee   quintic from the measured (angle, rate) to the tuck, hold, min-jerk back to the top pose
//   hip    on the tuck path (zero-reaction curve + dev * s) through the top pose, plus the measured
//          offset fading out
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
  float extend_fraction_ = 1.0F;      // share of the stroke tuck -> top pose done at the touchdown
  float knee_top_rad = 0.0F;          // pose the legs return to
  float knee_lo_rad = 0.0F;           // grid of the zero-reaction curve through the top pose
  float knee_step_rad = 0.0F;
  std::array<float, kGrid> hip_cad_rad{};   // zero-reaction hip (CAD) on the grid
  std::array<float, kGrid> cab_m{};         // CoM above the axle on it
  // For a tuck down to grid knee i: the hip pull-up dev (rad, off the zero-reaction curve at the tuck),
  // the wheel lift toward the CoM in the world frame at that pose and the body turn there.
  std::array<float, kGrid> dev_rad{};
  std::array<float, kGrid> lift_m{};
  std::array<float, kGrid> turn_rad{};
  // Body turn from the top pose along the planned tuck path to grid knee i (filled by planTuck).
  std::array<float, kGrid> path_turn_rad{};
  float hip_dev_rad = 0.0F;           // dev of the planned tuck
  float tuck_turn_rad = 0.0F;         // body turn at the tucked pose (mid-flight)
  float peak_hip_accel_rad_s2 = 0.0F;
  // Builds the curve and the tuck tables through the top pose (depends only on the configuration;
  // a few thousand FK evaluations: call it outside the control loop).
  bool buildCurve(const JumpPlannerConfig& config) noexcept;
  // Plans tuck, hold and extension from time t_now after the switch, from the measured state.
  bool planTuck(const JumpPlannerConfig& config, float vertical_speed_m_s, float t_now_s,
                float hip_cad_rad, float knee_rad, float knee_rate_rad_s) noexcept;
  [[nodiscard]] float zeroReactionHip(float knee_rad) const noexcept;
  [[nodiscard]] float kneeEnd() const noexcept {             // knee at the touchdown
    return knee_tuck_rad + extend_fraction_ * (knee_top_rad - knee_tuck_rad);
  }
  [[nodiscard]] float cabAt(float knee_rad) const noexcept;
  [[nodiscard]] float tuckHip(float knee_rad) const noexcept;  // CAD hip on the planned tuck path
  // Body turn along the planned tuck path between two knee angles (zero total angular momentum).
  [[nodiscard]] float pathTurn(float knee_from_rad, float knee_to_rad) const noexcept;
  [[nodiscard]] float kneeAt(float t_s) const noexcept;
  [[nodiscard]] float hipAt(float t_s) const noexcept;       // CAD angle
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
  float predicted_body_turn_rad = 0.0F;   // mid-flight, undone at the touchdown
  FlightPlan curve{};                 // tuck path tables, built here (outside the control loop)
};

[[nodiscard]] FlightPlan planFlight(const JumpPlannerConfig& config, float vertical_speed_m_s,
                                    float hip_cad_rad, float knee_rad, float knee_rate_rad_s) noexcept;
[[nodiscard]] JumpPlan planJump(const JumpPlannerConfig& config) noexcept;

}  // namespace tap_fw::v3
