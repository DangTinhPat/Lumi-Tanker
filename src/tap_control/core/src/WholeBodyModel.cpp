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
}

WholeBodyModel::Result WholeBodyModel::compute(float lh, float lk,
                                               float rh, float rk) const {
  const std::array<float, 2> hips{lh, rh}, knees{lk, rk};
  std::array<Point, 7> com{};
  std::array<Point, 2> axles{};
  com[0] = {geometry::parts[0].x, geometry::parts[0].z};
  for (int side = 0; side < 2; ++side) {
    const auto upper=rotation(hips[side]);
    const auto lower=rotation(hips[side]+knees[side]);
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
  const auto lower = rotation(hip + knee);
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

float WholeBodyModel::equilibriumPitch(const Result& r) noexcept {
  // Base pitch rotates COM about the axle, not the ground contact point.
  return std::atan2(-r.com_offset_x_m, std::max(r.com_above_axle_m, 0.01F));
}
}  // namespace robot_control
