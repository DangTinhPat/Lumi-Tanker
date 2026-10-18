#include <mujoco/mujoco.h>
#include "robot_control/WholeBodyModel.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main(int argc, char** argv) {
  assert(argc == 2);
  char error[1024]{};
  auto* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model) { std::cerr << error << '\n'; return 1; }
  auto* data = mj_makeData(model);
  auto id = [&](mjtObj type, const char* name) {
    const int value = mj_name2id(model, type, name);
    assert(value >= 0);
    return value;
  };
  auto near = [](double a, double b) { return std::abs(a - b) < 2e-6; };
  assert(near(mj_getTotalmass(model), 3.02));
  for (int body = 1; body < model->nbody; ++body) {
    const auto* inertia = model->body_inertia + 3 * body;
    assert(model->body_mass[body] > 0);
    for (int i = 0; i < 3; ++i) {
      assert(inertia[i] > 0);
      assert(inertia[i] <= inertia[(i + 1) % 3] + inertia[(i + 2) % 3] + 1e-12);
    }
  }
  const std::array<int, 4> joints{
      id(mjOBJ_JOINT, "hip_left"), id(mjOBJ_JOINT, "knee_left"),
      id(mjOBJ_JOINT, "hip_right"), id(mjOBJ_JOINT, "knee_right")};
  for (int index = 0; index < static_cast<int>(joints.size()); ++index) {
    const int joint = joints[static_cast<std::size_t>(index)];
    assert(near(model->jnt_axis[3 * joint + 1], 1));
    const double expected_friction = index % 2 == 0 ? .05 : .1;
    assert(near(model->dof_frictionloss[model->jnt_dofadr[joint]], expected_friction));
    assert(model->dof_damping[model->jnt_dofadr[joint]] == 0);
  }
  assert(model->npair == 2);
  for (const char* name : {"left_hip_torque", "left_knee_torque", "right_hip_torque", "right_knee_torque"}) {
    const int actuator = id(mjOBJ_ACTUATOR, name);
    assert(near(model->actuator_ctrlrange[2 * actuator], -3.76));
    assert(near(model->actuator_ctrlrange[2 * actuator + 1], 3.76));
  }
  for (int pair = 0; pair < model->npair; ++pair) {
    assert(model->pair_dim[pair] == 3);
    assert(near(model->pair_friction[5 * pair], .8));
    assert(near(model->pair_friction[5 * pair + 1], .8));
  }
  for (const char* name : {"wheel_left", "wheel_right"}) {
    const int joint = id(mjOBJ_JOINT, name);
    assert(near(model->jnt_axis[3 * joint + 1], 1));
  }
  mj_resetDataKeyframe(model, data, id(mjOBJ_KEY, "spawn_extended"));
  mj_forward(model, data);
  const int ground = id(mjOBJ_GEOM, "ground");
  const int left_wheel = id(mjOBJ_GEOM, "left_wheel_collision");
  const int right_wheel = id(mjOBJ_GEOM, "right_wheel_collision");
  assert(data->ncon > 0);
  for (int i = 0; i < data->ncon; ++i) {
    const auto& contact = data->contact[i];
    assert(contact.geom1 == ground || contact.geom2 == ground);
    const int other = contact.geom1 == ground ? contact.geom2 : contact.geom1;
    assert(other == left_wheel || other == right_wheel);
    assert(contact.dist >= -1e-6);
  }
  // A collider must reach the floor at the CAD surface, including fallen poses.
  for (const char* body_name : {"gim_body", "left_thigh", "left_shin", "right_thigh", "right_shin"}) {
    const int body_id = id(mjOBJ_BODY, body_name);
    for (int axis : {0, 1}) for (int degrees : {-180, -120, -90, -60, -30, 0, 30, 60, 90, 120, 180}) {
      mj_resetData(model, data);
      const double half_angle = degrees * std::acos(-1.) / 360.;
      data->qpos[3] = std::cos(half_angle);
      data->qpos[4 + axis] = std::sin(half_angle);
      mj_forward(model, data);
      double visual_min = std::numeric_limits<double>::infinity();
      double collision_min = visual_min;
      for (int geom = 0; geom < model->ngeom; ++geom) {
        if (model->geom_bodyid[geom] != body_id) continue;
        const auto* rot = data->geom_xmat + 9 * geom;
        double minimum = data->geom_xpos[3 * geom + 2];
        if (model->geom_type[geom] == mjGEOM_MESH) {
          const int mesh = model->geom_dataid[geom];
          minimum = std::numeric_limits<double>::infinity();
          for (int v = model->mesh_vertadr[mesh]; v < model->mesh_vertadr[mesh] + model->mesh_vertnum[mesh]; ++v) {
            const auto* vert = model->mesh_vert + 3 * v;
            minimum = std::min(minimum, data->geom_xpos[3 * geom + 2]
                + rot[6]*vert[0] + rot[7]*vert[1] + rot[8]*vert[2]);
          }
          visual_min = std::min(visual_min, minimum);
        } else if (model->geom_type[geom] == mjGEOM_BOX) {
          for (int j = 0; j < 3; ++j) minimum -= std::abs(rot[6 + j]) * model->geom_size[3 * geom + j];
        } else {
          assert(model->geom_type[geom] == mjGEOM_CAPSULE);
          minimum -= model->geom_size[3 * geom] + std::abs(rot[8]) * model->geom_size[3 * geom + 1];
        }
        if (model->geom_contype[geom]) collision_min = std::min(collision_min, minimum);
      }
      if (std::abs(visual_min - collision_min) > .001) {
        std::cerr << body_name << " collider/CAD gap: axis=" << axis << " angle=" << degrees
                  << " gap=" << visual_min - collision_min << " m\n";
        return 1;
      }
    }
  }
  const std::array<std::array<float, 4>, 3> poses{{
      {0, 0, 0, 0}, {-.986111111F, 1.972222222F, -.986111111F, 1.972222222F},
      {-.3F, .9F, .5F, 1.2F}}};
  for (const auto& q : poses) {
    mj_resetData(model, data);
    data->qpos[2] = .5;
    for (int i = 0; i < 4; ++i) data->qpos[model->jnt_qposadr[joints[i]]] = q[i];
    mj_forward(model, data);
    const int left = id(mjOBJ_BODY, "left_wheel"), right = id(mjOBJ_BODY, "right_wheel");
    const double ax = .5 * (data->xpos[3 * left] + data->xpos[3 * right]);
    const double az = .5 * (data->xpos[3 * left + 2] + data->xpos[3 * right + 2]);
    const std::array<int, 2> wheels{left, right};
    const std::array<int, 2> thighs{id(mjOBJ_BODY, "left_thigh"), id(mjOBJ_BODY, "right_thigh")};
    for (int side = 0; side < 2; ++side) {
      const auto leg = robot_control::WholeBodyModel::legHeight(q[2 * side], q[2 * side + 1]);
      assert(near(leg.x_m, data->xpos[3 * wheels[side]] - data->xpos[3 * thighs[side]]));
      assert(near(leg.height_m, data->xpos[3 * wheels[side] + 2] - data->xpos[3 * thighs[side] + 2]));
    }
    double mx = 0, mz = 0, iyy = 0;
    for (int body = 1; body < model->nbody; ++body) {
      const double x = data->xipos[3 * body] - ax, z = data->xipos[3 * body + 2] - az;
      mx += model->body_mass[body] * x;
      mz += model->body_mass[body] * z;
      // The principal inertia is expressed in the body_iquat frame (fullinertia rotates it): take its pitch (y) axis.
      const auto* principal = model->body_inertia + 3 * body;
      mjtNum inertial_rotation[9];
      mju_quat2Mat(inertial_rotation, model->body_iquat + 4 * body);
      double pitch_inertia = 0;
      for (int axis = 0; axis < 3; ++axis)
        pitch_inertia += inertial_rotation[3 + axis] * inertial_rotation[3 + axis] * principal[axis];
      iyy += pitch_inertia + model->body_mass[body] * (x*x + z*z);
    }
    const auto com = robot_control::WholeBodyModel{}.compute(q[0], q[1], q[2], q[3]);
    assert(near(com.com_offset_x_m, mx / mj_getTotalmass(model)));
    assert(near(com.com_above_axle_m, mz / mj_getTotalmass(model)));
    if (!near(com.pitch_inertia_at_axle_kg_m2, iyy)) {
      std::cerr << "axle inertia mismatch: controller=" << com.pitch_inertia_at_axle_kg_m2
                << " MuJoCo=" << iyy << '\n';
      return 1;
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  std::cout << "PASS GIM mass/inertia, COM/FK, friction and body/leg CAD contact envelopes\n";
}
