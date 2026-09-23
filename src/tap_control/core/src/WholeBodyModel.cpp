#include "robot_control/WholeBodyModel.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace robot_control {
namespace {
struct Point { float x, z; };
struct Rotation { float c, s; };
Rotation rotation(float angle) {return {std::cos(angle),std::sin(angle)};}
Point rotate(Point p, Rotation r) {
  return {r.c*p.x+r.s*p.z, -r.s*p.x+r.c*p.z};
}
Point add(Point a, Point b) { return {a.x+b.x, a.z+b.z}; }
Point derivative(Point p, Rotation r) {
  const Point rotated = rotate(p, r);
  return {rotated.z, -rotated.x};
}
}

WholeBodyModel::Result WholeBodyModel::compute(float lh, float lk,
                                               float rh, float rk) const {
  const std::array<float, 2> hips{lh, rh}, knees{lk, rk};
  std::array<Point, 7> com{};
  std::array<Point, 2> axles{};
  com[0] = {geometry::parts[0].x, geometry::parts[0].z};
  for (int side = 0; side < 2; ++side) {
    const auto upper=rotation(hips[side]);
    const auto lower = rotation(hips[side] + knees[side] +
                                geometry::knee_zero_rebase_rad);
    const auto& hip = geometry::origins[3*side];
    Point origin{hip.x, hip.z};
    for (int link = 0; link < 3; ++link) {
      const auto orientation=link ? lower : upper;
      if (link) {
        const auto& offset = geometry::origins[3*side+link];
        origin = add(origin, rotate({offset.x, offset.z},
                     link == 2 ? lower : upper));
      }
      const int index = 1+3*side+link;
      const auto& part = geometry::parts[index];
      com[index] = add(origin, rotate({part.x, part.z}, orientation));
    }
    axles[side] = origin;
  }
  const Point axle{(axles[0].x+axles[1].x)/2,
                   (axles[0].z+axles[1].z)/2};
  Result out{};
  float mx = 0.0F, mz = 0.0F;
  for (std::size_t i = 0; i < com.size(); ++i) {
    const auto& part = geometry::parts[i];
    const float dx = com[i].x-axle.x, dz = com[i].z-axle.z;
    mx += part.mass*dx;
    mz += part.mass*dz;
    out.pitch_inertia_at_axle_kg_m2 += part.iyy+part.mass*(dx*dx+dz*dz);
  }
  out.com_offset_x_m = mx/geometry::total_mass_kg;
  out.com_above_axle_m = mz/geometry::total_mass_kg;
  out.com_height_m = out.com_above_axle_m+config_.wheel_radius_m;
  out.com_line_angle_rad = std::atan2(
      out.com_offset_x_m, std::max(out.com_above_axle_m, 1.0e-4F));
  out.axle_height_difference_m = axles[0].z-axles[1].z;
  return out;
}

WholeBodyModel::LegHeight WholeBodyModel::legHeight(float hip,
                                                    float knee) noexcept {
  // Thigh (origins[1]) and shin (origins[2]) x,z offsets are identical for
  // both sides under native +Y axes (see header) - use the left-leg indices
  // for either leg. Hip origin z is 0 for both hips, so it drops out.
  const Point thigh{geometry::origins[1].x, geometry::origins[1].z};
  const Point shin{geometry::origins[2].x, geometry::origins[2].z};
  const auto upper = rotation(hip);
  const auto lower = rotation(hip + knee + geometry::knee_zero_rebase_rad);
  // For rotate(p, theta) = (c*px+s*pz, -s*px+c*pz): d/dtheta of the
  // x-component equals the z-component, and d/dtheta of the z-component
  // equals minus the x-component (standard 2D rotation derivative duality) -
  // used directly below instead of a separate finite-difference-derived
  // formula, and cross-checked by the FD unit test.
  const auto thigh_rot = rotate(thigh, upper);
  const auto shin_rot = rotate(shin, lower);
  const float x1 = thigh_rot.x, z1 = thigh_rot.z;
  const float x2 = shin_rot.x, z2 = shin_rot.z;
  WholeBodyModel::LegHeight out{};
  out.height_m = z1 + z2;
  out.d_height_d_hip = -(x1 + x2);
  out.d_height_d_knee = -x2;
  out.x_m = x1 + x2;
  out.d_x_d_hip = z1 + z2;
  out.d_x_d_knee = z2;
  return out;
}

WholeBodyModel::PlanarVelocity WholeBodyModel::imuVelocityThroughLeg(
    float pitch, float pitch_rate, float hip, float knee, float hip_rate,
    float knee_rate, float wheel_rate, float wheel_radius_m) noexcept {
  const auto leg = legHeight(hip, knee);
  const Point lever{geometry::imu_offset_x_m - leg.x_m,
                    geometry::imu_offset_z_m - leg.height_m};
  const Point lever_rate{
      -(leg.d_x_d_hip * hip_rate + leg.d_x_d_knee * knee_rate),
      -(leg.d_height_d_hip * hip_rate + leg.d_height_d_knee * knee_rate)};
  const auto body = rotation(pitch);
  const Point swing = derivative(lever, body);
  const Point stretch = rotate(lever_rate, body);
  const float axle_velocity =
      wheel_radius_m * (pitch_rate + hip_rate + knee_rate + wheel_rate);
  return {axle_velocity + pitch_rate * swing.x + stretch.x,
          pitch_rate * swing.z + stretch.z};
}

WholeBodyModel::SymmetricComJacobian
WholeBodyModel::symmetricComOffsetJacobian(float hip, float knee) noexcept {
  SymmetricComJacobian out{};
  if (!std::isfinite(hip) || !std::isfinite(knee)) return out;

  const auto upper = rotation(hip);
  const auto lower = rotation(hip + knee + geometry::knee_zero_rebase_rad);
  const auto& knee_origin = geometry::origins[1];
  const auto& wheel_origin = geometry::origins[2];
  const Point d_knee_d_hip = derivative(
      {knee_origin.x, knee_origin.z}, upper);
  const Point d_wheel_d_hip = add(
      d_knee_d_hip, derivative({wheel_origin.x, wheel_origin.z}, lower));
  const Point d_wheel_d_knee = derivative(
      {wheel_origin.x, wheel_origin.z}, lower);

  // The body is fixed while the mean axle moves.  All three link COMs occur
  // twice in a symmetric pose, so the axle-reference term carries the total
  // mass and each leg-local derivative is doubled.
  float d_mass_x_d_hip = -geometry::total_mass_kg * d_wheel_d_hip.x;
  float d_mass_x_d_knee = -geometry::total_mass_kg * d_wheel_d_knee.x;
  float d_mass_z_d_hip = -geometry::total_mass_kg * d_wheel_d_hip.z;
  float d_mass_z_d_knee = -geometry::total_mass_kg * d_wheel_d_knee.z;
  const auto d_thigh_d_hip = derivative(
      {geometry::parts[1].x, geometry::parts[1].z}, upper);
  const auto d_shin_d_hip = add(
      d_knee_d_hip, derivative({geometry::parts[2].x, geometry::parts[2].z}, lower));
  const auto d_shin_d_knee = derivative(
      {geometry::parts[2].x, geometry::parts[2].z}, lower);
  const auto d_wheel_com_d_hip = add(
      d_wheel_d_hip, derivative({geometry::parts[3].x, geometry::parts[3].z}, lower));
  const auto d_wheel_com_d_knee = add(
      d_wheel_d_knee, derivative({geometry::parts[3].x, geometry::parts[3].z}, lower));
  d_mass_x_d_hip += 2.0F *
      (geometry::parts[1].mass * d_thigh_d_hip.x +
       geometry::parts[2].mass * d_shin_d_hip.x +
       geometry::parts[3].mass * d_wheel_com_d_hip.x);
  d_mass_x_d_knee += 2.0F *
      (geometry::parts[2].mass * d_shin_d_knee.x +
       geometry::parts[3].mass * d_wheel_com_d_knee.x);
  d_mass_z_d_hip += 2.0F *
      (geometry::parts[1].mass * d_thigh_d_hip.z +
       geometry::parts[2].mass * d_shin_d_hip.z +
       geometry::parts[3].mass * d_wheel_com_d_hip.z);
  d_mass_z_d_knee += 2.0F *
      (geometry::parts[2].mass * d_shin_d_knee.z +
       geometry::parts[3].mass * d_wheel_com_d_knee.z);
  out.d_com_offset_x_d_hip = d_mass_x_d_hip / geometry::total_mass_kg;
  out.d_com_offset_x_d_knee = d_mass_x_d_knee / geometry::total_mass_kg;
  out.d_com_above_axle_d_hip = d_mass_z_d_hip / geometry::total_mass_kg;
  out.d_com_above_axle_d_knee = d_mass_z_d_knee / geometry::total_mass_kg;
  return out;
}

WholeBodyModel::LegGravityTorque WholeBodyModel::legGravityTorque(
    float hip_rad, float knee_rad, float gravity_m_s2) noexcept {
  LegGravityTorque out{};
  if (!std::isfinite(hip_rad) || !std::isfinite(knee_rad) ||
      !std::isfinite(gravity_m_s2)) {
    return out;
  }

  const auto rotated_x = [](float x, float z, float angle) {
    return std::cos(angle) * x + std::sin(angle) * z;
  };
  const auto dz_d_angle = [&](float x, float z, float angle) {
    return -rotated_x(x, z, angle);
  };
  const float lower = hip_rad + knee_rad + geometry::knee_zero_rebase_rad;
  const auto& knee_origin = geometry::origins[1];
  const auto& wheel_origin = geometry::origins[2];
  const auto& thigh = geometry::parts[1];
  const auto& shin = geometry::parts[2];
  const auto& wheel = geometry::parts[3];
  const float gravity_force = -std::max(gravity_m_s2, 0.0F);

  // Motor torque opposes J^T F.  The body mass is intentionally excluded:
  // its support-force split between the two hips is not identifiable from
  // the current sensor contract and must not be guessed here.
  const auto accumulate = [&](float mass, float dhip, float dknee) {
    const float force_z = mass * gravity_force;
    out.hip_nm -= dhip * force_z;
    out.knee_nm -= dknee * force_z;
  };
  accumulate(thigh.mass, dz_d_angle(thigh.x, thigh.z, hip_rad), 0.0F);
  accumulate(shin.mass,
             dz_d_angle(knee_origin.x, knee_origin.z, hip_rad) +
                 dz_d_angle(shin.x, shin.z, lower),
             dz_d_angle(shin.x, shin.z, lower));
  accumulate(wheel.mass,
             dz_d_angle(knee_origin.x, knee_origin.z, hip_rad) +
                 dz_d_angle(wheel_origin.x + wheel.x,
                            wheel_origin.z + wheel.z, lower),
             dz_d_angle(wheel_origin.x + wheel.x,
                        wheel_origin.z + wheel.z, lower));

  return out;
}

WholeBodyModel::LegGravityTorque WholeBodyModel::bodySupportTorque(
    float hip_rad, float knee_rad, float gravity_m_s2) noexcept {
  LegGravityTorque out{};
  if (!std::isfinite(hip_rad) || !std::isfinite(knee_rad) ||
      !std::isfinite(gravity_m_s2)) {
    return out;
  }

  const auto dz_d_angle = [](float x, float z, float angle) {
    return -(std::cos(angle) * x + std::sin(angle) * z);
  };
  const float lower = hip_rad + knee_rad + geometry::knee_zero_rebase_rad;
  const auto& knee_origin = geometry::origins[1];
  const auto& wheel_origin = geometry::origins[2];
  const float body_support_force_z =
      0.5F * geometry::parts[0].mass * std::max(gravity_m_s2, 0.0F);
  const float d_axle_z_d_hip =
      dz_d_angle(knee_origin.x, knee_origin.z, hip_rad) +
      dz_d_angle(wheel_origin.x, wheel_origin.z, lower);
  const float d_axle_z_d_knee =
      dz_d_angle(wheel_origin.x, wheel_origin.z, lower);
  // Motor torque opposes the upward support reaction projected through the
  // axle Jacobian. The equal split is the provisional Rule-2 design model;
  // dynamic left/right load transfer remains a calibration item.
  out.hip_nm = -d_axle_z_d_hip * body_support_force_z;
  out.knee_nm = -d_axle_z_d_knee * body_support_force_z;
  return out;
}

float WholeBodyModel::equilibriumPitch(const Result& r) noexcept {
  // Base pitch rotates COM about the axle, not the ground contact point.
  return std::atan2(-r.com_offset_x_m, std::max(r.com_above_axle_m, 0.01F));
}

WholeBodyModel::SymmetricPose WholeBodyModel::solveSymmetricHeight(
    float body_height_m, float hip_seed_rad, float knee_seed_rad,
    float hip_min_rad, float hip_max_rad, float knee_min_rad,
    float knee_max_rad) noexcept {
  SymmetricPose out{};
  if (!std::isfinite(body_height_m) || !std::isfinite(hip_seed_rad) ||
      !std::isfinite(knee_seed_rad) || !(hip_min_rad < hip_max_rad) ||
      !(knee_min_rad < knee_max_rad)) {
    return out;
  }

  const auto natural = legHeight(geometry::neutral_hip_rad,
                                 geometry::neutral_knee_rad);
  const float target_x = natural.x_m;
  const float target_z = geometry::wheel_radius_m - body_height_m;
  float hip = std::clamp(hip_seed_rad, hip_min_rad, hip_max_rad);
  float knee = std::clamp(knee_seed_rad, knee_min_rad, knee_max_rad);
  const float seed_height = geometry::wheel_radius_m -
      legHeight(hip, knee).height_m;
  // Near the straight-leg branch, unconstrained Newton can converge to the
  // other valid FK branch. That made a lower target from max stand command an
  // even higher knee pose. Keep the solve on the operator-requested branch.
  const bool lowering = body_height_m < seed_height;
  const float branch_knee_min = lowering ? knee_min_rad : knee_seed_rad;
  const float branch_knee_max = lowering ? knee_seed_rad : knee_max_rad;
  knee = std::clamp(knee, branch_knee_min, branch_knee_max);

  // Newton on the exact analytic FK/Jacobian.  The step clamp prevents an
  // out-of-branch jump when the requested height is close to a singular
  // straight-leg configuration.  The caller can reject the residual.
  for (int iteration = 0; iteration < 32; ++iteration) {
    const LegHeight current = legHeight(hip, knee);
    const float error_x = target_x - current.x_m;
    const float error_z = target_z - current.height_m;
    if (std::hypot(error_x, error_z) < 1.0e-7F) break;

    const float determinant = current.d_x_d_hip * current.d_height_d_knee -
                              current.d_x_d_knee * current.d_height_d_hip;
    if (!std::isfinite(determinant) || std::abs(determinant) < 1.0e-7F) {
      break;
    }
    float delta_hip = (error_x * current.d_height_d_knee -
                       current.d_x_d_knee * error_z) / determinant;
    float delta_knee = (current.d_x_d_hip * error_z -
                        error_x * current.d_height_d_hip) / determinant;
    const float step = std::max(std::abs(delta_hip), std::abs(delta_knee));
    if (step > 0.25F) {
      const float scale = 0.25F / step;
      delta_hip *= scale;
      delta_knee *= scale;
    }
    hip = std::clamp(hip + 0.75F * delta_hip, hip_min_rad, hip_max_rad);
    knee = std::clamp(knee + 0.75F * delta_knee,
                      branch_knee_min, branch_knee_max);
  }

  const LegHeight solved = legHeight(hip, knee);
  out.hip_rad = hip;
  out.knee_rad = knee;
  out.body_height_m = geometry::wheel_radius_m - solved.height_m;
  out.axle_x_m = solved.x_m;
  out.residual_m = std::hypot(target_x - solved.x_m,
                              target_z - solved.height_m);
  out.feasible = std::isfinite(out.residual_m) && out.residual_m <= 2.0e-5F;
  return out;
}

WholeBodyModel::SymmetricPose WholeBodyModel::solveSymmetricComHeight(
    float body_height_m, float com_target_x_m, float hip_seed_rad,
    float knee_seed_rad, float hip_min_rad, float hip_max_rad,
    float knee_min_rad, float knee_max_rad) noexcept {
  SymmetricPose out{};
  if (!std::isfinite(body_height_m) || !std::isfinite(com_target_x_m) ||
      !std::isfinite(hip_seed_rad) || !std::isfinite(knee_seed_rad) ||
      !(hip_min_rad < hip_max_rad) || !(knee_min_rad < knee_max_rad)) {
    return out;
  }

  float hip = std::clamp(hip_seed_rad, hip_min_rad, hip_max_rad);
  float knee = std::clamp(knee_seed_rad, knee_min_rad, knee_max_rad);
  for (int iteration = 0; iteration < 40; ++iteration) {
    const auto leg = legHeight(hip, knee);
    const auto body = WholeBodyModel{}.compute(hip, knee, hip, knee);
    const float height_error = body_height_m -
        (geometry::wheel_radius_m - leg.height_m);
    const float com_error = com_target_x_m - body.com_offset_x_m;
    if (std::hypot(height_error, com_error) < 1.0e-7F) break;
    const auto jacobian = symmetricComOffsetJacobian(hip, knee);
    const float height_hip = -leg.d_height_d_hip;
    const float height_knee = -leg.d_height_d_knee;
    const float determinant = height_hip *
            jacobian.d_com_offset_x_d_knee -
        height_knee * jacobian.d_com_offset_x_d_hip;
    if (!std::isfinite(determinant) || std::abs(determinant) < 1.0e-8F) {
      break;
    }
    float delta_hip = (height_error *
            jacobian.d_com_offset_x_d_knee -
        height_knee * com_error) / determinant;
    float delta_knee = (height_hip * com_error -
        height_error * jacobian.d_com_offset_x_d_hip) / determinant;
    const float step = std::max(std::abs(delta_hip), std::abs(delta_knee));
    if (step > 0.15F) {
      const float scale = 0.15F / step;
      delta_hip *= scale;
      delta_knee *= scale;
    }
    hip = std::clamp(hip + 0.8F * delta_hip, hip_min_rad, hip_max_rad);
    knee = std::clamp(knee + 0.8F * delta_knee, knee_min_rad, knee_max_rad);
  }
  const auto leg = legHeight(hip, knee);
  const auto body = WholeBodyModel{}.compute(hip, knee, hip, knee);
  const float height_error = body_height_m -
      (geometry::wheel_radius_m - leg.height_m);
  const float com_error = com_target_x_m - body.com_offset_x_m;
  out.hip_rad = hip;
  out.knee_rad = knee;
  out.body_height_m = geometry::wheel_radius_m - leg.height_m;
  out.axle_x_m = leg.x_m;
  out.residual_m = std::hypot(height_error, com_error);
  out.feasible = std::isfinite(out.residual_m) && out.residual_m <= 2.0e-5F;
  return out;
}

WholeBodyModel::SymmetricPose WholeBodyModel::solveSymmetricKnee(
    float knee_rad, float hip_seed_rad, float hip_min_rad, float hip_max_rad,
    float knee_min_rad, float knee_max_rad) noexcept {
  SymmetricPose out{};
  if (!std::isfinite(knee_rad) || !std::isfinite(hip_seed_rad) ||
      !(hip_min_rad < hip_max_rad) || !(knee_min_rad <= knee_rad) ||
      !(knee_rad <= knee_max_rad)) {
    return out;
  }

  const float target_x = legHeight(geometry::neutral_hip_rad,
                                   geometry::neutral_knee_rad).x_m;
  float hip = std::clamp(hip_seed_rad, hip_min_rad, hip_max_rad);
  for (int iteration = 0; iteration < 32; ++iteration) {
    const LegHeight current = legHeight(hip, knee_rad);
    const float error_x = target_x - current.x_m;
    if (std::abs(error_x) < 1.0e-7F) break;
    if (std::abs(current.d_x_d_hip) < 1.0e-7F) break;
    hip = std::clamp(hip + 0.75F * error_x / current.d_x_d_hip,
                     hip_min_rad, hip_max_rad);
  }
  const LegHeight solved = legHeight(hip, knee_rad);
  out.hip_rad = hip;
  out.knee_rad = knee_rad;
  out.body_height_m = geometry::wheel_radius_m - solved.height_m;
  out.axle_x_m = solved.x_m;
  out.residual_m = std::abs(target_x - solved.x_m);
  out.feasible = std::isfinite(out.residual_m) && out.residual_m <= 2.0e-5F;
  return out;
}
}  // namespace robot_control
