#pragma once

namespace tap_fw::v3 {

// AirborneModel: the planar rigid-body model the airborne attitude control (AAC) is built on. Body,
// thigh, shin (with the wheel motor) and wheel, both legs moving together, masses/inertias/origins
// from robot_control::geometry (the same CAD chain as WholeBodyModel::compute). Stateless.
//
// In the air the angular momentum L about the CoM is conserved. With the leg joints q = (hip, knee):
//     L = I(q) w + L_h(q) dhip + L_k(q) dknee
// so the body turns at  w = (L - L_h dhip - L_k dknee) / I.  The virtual pitch the balance loop uses
// is  theta_v = body pitch + comLineAngle(q) - (angle at the hang pose): the direction of the CoM seen
// from the axle. Everything the AAC predicts about the touchdown follows from these two lines.
//
// Angles about +Y like the CAD rotations (and the IMU pitch); hip angles are CAD angles
// (LegHeight::hangHipCad() + core offset), knee angles are core offsets.

struct Vec2 {
  float x = 0.0F;
  float z = 0.0F;
};

struct LegInertia {
  float inertia = 0.0F;     // whole robot about its CoM
  float l_hip = 0.0F;       // angular momentum about the CoM per unit hip rate (body held)
  float l_knee = 0.0F;      // ... per unit knee rate
  Vec2 com_from_axle{};     // CoM relative to the axle, body frame (z = CoM height above the axle)
};
[[nodiscard]] LegInertia legInertia(float hip_cad, float knee) noexcept;

// Direction of the CoM seen from the axle in the body frame, atan2(x, z), with the controller's
// body CoM correction (V3Controller::Config::tier.body_com_dx_m); matches V3Controller::comLine.
[[nodiscard]] float comLineAngle(float hip_cad, float knee, float body_com_dx_m = 0.0F) noexcept;

// Body rate a hip/knee motion causes in the air with zero angular momentum, and I / L_h.
[[nodiscard]] float bodyRateFromLegMotion(float hip_cad, float knee, float hip_rate, float knee_rate) noexcept;
[[nodiscard]] float hipPerBodyRate(float hip_cad, float knee) noexcept;
// Angle the body turns when the legs move along the straight joint-space segment between two poses
// with zero angular momentum (Simpson rule on the 1-form -(L_h dh + L_k dk) / I).
[[nodiscard]] float bodyTurnFromLegPath(float hip0_cad, float knee0, float hip1_cad, float knee1) noexcept;

// Whole-robot state from the encoders and the gyro (both legs together):
//   angular_momentum  L about the CoM;   inertia;   cab = CoM height above the axle;
//   com_vz            CoM vertical speed relative to the axle (valid with the wheels on the ground)
struct BodyMomentum {
  float angular_momentum = 0.0F;
  float inertia = 0.0F;
  float cab = 0.0F;
  float com_vz = 0.0F;
  Vec2 com_from_axle{};   // body frame
};
[[nodiscard]] BodyMomentum bodyMomentum(float hip_cad, float knee, float hip_rate, float knee_rate,
                                        float body_rate) noexcept;

// Rotation a constant angular momentum produces over the time left while the legs go through the
// given poses (samples evenly spaced in time, first = now, last = touchdown): integral of L / I(q(t)).
[[nodiscard]] float freeRotation(float angular_momentum, const float* hip_cad, const float* knee, int samples,
                                 float time_left_s) noexcept;

// Touchdown attitude: body pitch and virtual pitch when the wheels touch, from the values now, the
// free rotation (L), the turn the remaining leg motion causes and the CoM line angles now / at touchdown.
struct TouchdownAttitude {
  float body = 0.0F;
  float theta_v = 0.0F;
};
[[nodiscard]] TouchdownAttitude predictTouchdown(float body_now, float theta_v_now, float free_rotation,
                                                 float leg_turn, float line_now, float line_td) noexcept;

// How the touchdown attitude changes with the touchdown pose (a small extra hip / knee motion just before
// the touchdown): d body = -(L_h dhip + L_k dknee) / I,  d theta_v = d body + d comLineAngle.
struct TouchdownSensitivity {
  float body_hip = 0.0F;
  float body_knee = 0.0F;
  float theta_hip = 0.0F;
  float theta_knee = 0.0F;
};
[[nodiscard]] TouchdownSensitivity touchdownSensitivity(float hip_cad, float knee, float body_com_dx_m = 0.0F) noexcept;

}  // namespace tap_fw::v3
