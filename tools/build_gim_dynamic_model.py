#!/usr/bin/env python3
"""Build the articulated MuJoCo model from STEP component exports."""

import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "src/tap_sim/model"
MANIFEST = MODEL_DIR / "gim_components.json"
ASSETS = MODEL_DIR / "gim_components"
OUTPUT = MODEL_DIR / "gim_dynamic.xml"

BODY = {7, 8, 13, 14, 15, 17, 18, 19, 20, 21, 22, 24,
        31, 32, 35, 36, 37, 39, 40, 44, 45, 46, 47,
        *range(48, 60)}
GROUPS = {
    "body": BODY,
    "left_thigh": {9, 10, 11, 12, 16},
    "left_shin": {1, 2, 3, 4, 5, 23},
    "left_wheel": {6},
    "right_thigh": {33, 34, 38, 42, 43},
    "right_shin": {26, 27, 28, 29, 30, 41},
    "right_wheel": {25},
}
# CAD shell hulls retain the rounded outline; the exposed hip mounts extend
# laterally beyond the shell. Internal electronics/motors need no extra hull.
BODY_CONTACT_PARTS = {7, 8, 31, 32, 50, 55}
CAD_CONTACT_PARTS = (BODY_CONTACT_PARTS | GROUPS["left_thigh"] | GROUPS["left_shin"]
                     | GROUPS["right_thigh"] | GROUPS["right_shin"])
MATERIAL = {
    "structure": "gim_structure", "motor": "gim_motor",
    "wheel": "gim_wheel", "electronics": "gim_electronics",
    "battery": "gim_battery", "spring": "gim_spring",
    "hardware": "gim_hardware",
}


def vec(values):
    return " ".join(f"{v:.9g}" for v in values)


def main():
    doc = json.loads(MANIFEST.read_text(encoding="utf-8"))
    parts = {p["number"]: p for p in doc["components"]}
    assigned = set().union(*GROUPS.values())
    if assigned != set(parts) or sum(map(len, GROUPS.values())) != len(assigned):
        raise ValueError("each CAD component must belong to exactly one rigid link")

    meshes, geoms = [], {name: [] for name in GROUPS}
    for number in sorted(parts):
        part = parts[number]
        center = part["center_m"]
        category = part["category"]
        stem = part["mesh_stem"]
        files = sorted(ASSETS.glob(stem + "_*.stl"))
        if not files:
            raise FileNotFoundError(f"missing exported CAD mesh: {stem}")
        for index, path in enumerate(files):
            mesh = f"cad_{number:02d}_{index}"
            contact = int(number in CAD_CONTACT_PARTS)
            meshes.append(f'    <mesh name="{mesh}" file="gim_components/{path.name}" '
                          'scale="0.001 0.001 0.001"/>')
            geoms[next(group for group, ids in GROUPS.items() if number in ids)].append(
                (number, f'      <geom name="cad_{number:02d}_{index}" type="mesh" '
                 f'mesh="{mesh}" material="{MATERIAL[category]}" '
                 f'pos="{vec(center)}" contype="{contact}" conaffinity="{contact}"/>'))

    # Keep the CAD mesh coordinates exact by expressing every component center
    # relative to the corresponding neutral joint frame.
    origins = {
        "body": (0.0, 0.0, 0.0),
        "left_thigh": (0.0, 0.1025, 0.00828),
        "left_shin": (-0.09816, 0.1130, -0.01080),
        "left_wheel": (0.0, 0.1327, -0.02988),
        "right_thigh": (0.0, -0.1025, 0.00828),
        "right_shin": (-0.09816, -0.1130, -0.01080),
        "right_wheel": (0.0, -0.1327, -0.02988),
    }
    for name, items in geoms.items():
        fixed = []
        for number, line in items:
            center = parts[number]["center_m"]
            local = [center[i] - origins[name][i] for i in range(3)]
            fixed.append(line.replace(f'pos="{vec(center)}"', f'pos="{vec(local)}"'))
        geoms[name] = fixed

    body_geom = "\n".join(geoms["body"])
    left_thigh = "\n".join(geoms["left_thigh"])
    left_shin = "\n".join(geoms["left_shin"])
    left_wheel = "\n".join(geoms["left_wheel"])
    right_thigh = "\n".join(geoms["right_thigh"])
    right_shin = "\n".join(geoms["right_shin"])
    right_wheel = "\n".join(geoms["right_wheel"])
    material_defs = "\n".join((
        '    <material name="gim_structure" rgba="0.14 0.62 0.84 1"/>',
        '    <material name="gim_motor" rgba="0.24 0.26 0.29 1"/>',
        '    <material name="gim_wheel" rgba="0.08 0.09 0.10 1"/>',
        '    <material name="gim_electronics" rgba="0.12 0.48 0.24 1"/>',
        '    <material name="gim_battery" rgba="0.72 0.20 0.14 1"/>',
        '    <material name="gim_spring" rgba="0.96 0.34 0.12 1"/>',
        '    <material name="gim_hardware" rgba="0.36 0.38 0.42 1"/>'))
    knee = 1.9722222222222222
    hip = -knee / 2
    axle_z = (0.00828 - math.sin(hip) * -0.09816 + math.cos(hip) * -0.01908
              - math.sin(hip + knee) * 0.09816 + math.cos(hip + knee) * -0.01908)
    spawn_qpos = vec((0, 0, 0.040 - axle_z, 1, 0, 0, 0,
                      hip, knee, 0, hip, knee, 0))
    hip_home = math.radians(74.8)
    knee_home = 0.0
    home_qpos = vec((0, 0, 0.04172, 1, 0, 0, 0, hip_home, knee_home, 0,
                     hip_home, knee_home, 0))
    xml = f'''<mujoco model="gim6010_v3_dynamic">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.0005" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic"/>
  <visual><global azimuth="135" elevation="-20" fovy="45"/><headlight ambient="0.65 0.65 0.65" diffuse="0.8 0.8 0.8" specular="0.25 0.25 0.25"/></visual>
  <default>
    <joint damping="0.02" armature="0.0001"/>
    <geom friction="1.0 0.01 0.001" solref="0.006 1" solimp="0.95 0.99 0.001"/>
  </default>
  <asset>
    <texture name="ground_grid_tex" type="2d" builtin="checker" rgb1="0.46 0.48 0.50" rgb2="0.28 0.30 0.32" mark="edge" markrgb="0.68 0.70 0.72" width="512" height="512"/>
    <material name="ground_grid" texture="ground_grid_tex" texuniform="true" texrepeat="8 8" reflectance="0.12"/>
{chr(10).join(meshes)}
{material_defs}
  </asset>
  <worldbody>
    <light name="key" pos="0 -1 1.5" dir="0 1 -1"/>
    <geom name="ground" type="plane" pos="0 0 0" size="2 2 0.1" material="ground_grid"/>
    <body name="gim_body" pos="0 0 0.06988">
      <freejoint name="root"/>
      <inertial pos="-0.00509198 0.00012916 0.01377824" mass="2.4" fullinertia="0.00896273 0.00482998 0.00936716 -0.0000152898 0.0000180070 0.0000002404"/>
{body_geom}
      <site name="imu_site" pos="0 0 0.0561" size="0.004"/>
      <body name="left_thigh" pos="0 0.1025 0.00828">
        <joint name="hip_left" type="hinge" axis="0 1 0" damping="0" frictionloss="0.1"/>
        <inertial pos="-0.04908 0 -0.00954" mass="0.05" diaginertia="0.00042 0.0000417 0.00042"/>
{left_thigh}
        <body name="left_shin" pos="-0.09816 0.0105 -0.01908">
          <joint name="knee_left" type="hinge" axis="0 1 0" range="0 1.9722222222" limited="true" damping="0" frictionloss="0.1"/>
          <inertial pos="0.08998 0 -0.01749" mass="0.24" diaginertia="0.001 0.000220 0.001"/>
{left_shin}
          <body name="left_wheel" pos="0.09816 0.0197 -0.01908">
            <joint name="wheel_left" type="hinge" axis="0 1 0" damping="0.002" frictionloss="0.01"/>
            <inertial pos="0 0 0" mass="0.02" diaginertia="0.000008 0.000016 0.000008"/>
{left_wheel}
            <geom name="left_wheel_collision" type="cylinder" size="0.040 0.0085" quat="0.7071067812 0.7071067812 0 0" material="gim_wheel" contype="1" conaffinity="1"/>
          </body>
        </body>
      </body>
      <body name="right_thigh" pos="0 -0.1025 0.00828">
        <joint name="hip_right" type="hinge" axis="0 1 0" damping="0" frictionloss="0.1"/>
        <inertial pos="-0.04908 0 -0.00954" mass="0.05" diaginertia="0.00042 0.0000417 0.00042"/>
{right_thigh}
        <body name="right_shin" pos="-0.09816 -0.0105 -0.01908">
          <joint name="knee_right" type="hinge" axis="0 1 0" range="0 1.9722222222" limited="true" damping="0" frictionloss="0.1"/>
          <inertial pos="0.08998 0 -0.01749" mass="0.24" diaginertia="0.001 0.000220 0.001"/>
{right_shin}
          <body name="right_wheel" pos="0.09816 -0.0197 -0.01908">
            <joint name="wheel_right" type="hinge" axis="0 1 0" damping="0.002" frictionloss="0.01"/>
            <inertial pos="0 0 0" mass="0.02" diaginertia="0.000008 0.000016 0.000008"/>
{right_wheel}
            <geom name="right_wheel_collision" type="cylinder" size="0.040 0.0085" quat="0.7071067812 0.7071067812 0 0" material="gim_wheel" contype="1" conaffinity="1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <contact>
    <pair geom1="ground" geom2="left_wheel_collision" condim="3" friction="0.8 0.8 0 0 0" solref="0.006 1" solimp="0.95 0.99 0.001"/>
    <pair geom1="ground" geom2="right_wheel_collision" condim="3" friction="0.8 0.8 0 0 0" solref="0.006 1" solimp="0.95 0.99 0.001"/>
  </contact>
  <actuator>
    <motor name="left_hip_torque" joint="hip_left" ctrllimited="true" ctrlrange="-3.76 3.76"/>
    <motor name="left_knee_torque" joint="knee_left" ctrllimited="true" ctrlrange="-3.76 3.76"/>
    <motor name="right_hip_torque" joint="hip_right" ctrllimited="true" ctrlrange="-3.76 3.76"/>
    <motor name="right_knee_torque" joint="knee_right" ctrllimited="true" ctrlrange="-3.76 3.76"/>
    <motor name="left_wheel_torque" joint="wheel_left" ctrllimited="true" ctrlrange="-0.15 0.15"/>
    <motor name="right_wheel_torque" joint="wheel_right" ctrllimited="true" ctrlrange="-0.15 0.15"/>
  </actuator>
  <sensor>
    <accelerometer name="mpu6500_accel" site="imu_site"/>
    <gyro name="mpu6500_gyro" site="imu_site"/>
    <jointpos name="enc_left_hip_pos" joint="hip_left"/><jointvel name="enc_left_hip_vel" joint="hip_left"/>
    <jointpos name="enc_left_knee_pos" joint="knee_left"/><jointvel name="enc_left_knee_vel" joint="knee_left"/>
    <jointpos name="enc_left_wheel_pos" joint="wheel_left"/><jointvel name="enc_left_wheel_vel" joint="wheel_left"/>
    <jointpos name="enc_right_hip_pos" joint="hip_right"/><jointvel name="enc_right_hip_vel" joint="hip_right"/>
    <jointpos name="enc_right_knee_pos" joint="knee_right"/><jointvel name="enc_right_knee_vel" joint="knee_right"/>
    <jointpos name="enc_right_wheel_pos" joint="wheel_right"/><jointvel name="enc_right_wheel_vel" joint="wheel_right"/>
  </sensor>
  <keyframe>
    <key name="spawn_extended" qpos="{spawn_qpos}"/>
    <key name="cad_neutral" qpos="0 0 0.06988 1 0 0 0 0 0 0 0 0 0"/>
    <key name="home" qpos="{home_qpos}"/>
  </keyframe>
</mujoco>
'''
    OUTPUT.write_text(xml, encoding="utf-8")
    print(f"wrote {OUTPUT} with 6 hinges and {len(parts)} articulated CAD components")


if __name__ == "__main__":
    main()
