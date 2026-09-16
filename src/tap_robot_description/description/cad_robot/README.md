# CAD Robot Preview

```bash
colcon build --symlink-install --packages-select tap_robot_description
source install/setup.bash
ros2 launch tap_robot_description cad_preview.launch.py
```

RViz displays the assembled robot; Joint State Publisher sliders move the two
hips, two knees and two continuous wheel joints. Center restores the CAD pose.
The body is fixed for inspection. This is a kinematic preview, not a balance or
contact simulation. Ground contact is only aligned in the zero pose.

Runtime dependencies include `rviz2`, `joint_state_publisher` and
`joint_state_publisher_gui` (declared in package.xml). On a standard Jazzy system,
install package dependencies with `rosdep install --from-paths
src/tap_robot_description --ignore-src -r -y`. No machine-specific paths are stored.

## Source And Coordinates

The supplied `3D.rar` contains `Lap rap he mo phong.stl`, which separates into
seven connected components: a body, two thighs, two shins and two wheels.
The separately supplied `Banh xe .stl` (space before extension) is the body;
`Banh xe.stl` is a wheel. Both sides are extracted directly from the assembly.

STL has no unit metadata. CAD coordinates are provisionally interpreted as mm:
wheel diameter 60 mm, wheel thickness 25 mm, wheel-centre spacing 309 mm.
CAD coordinates map to ROS as `(x, y, z) = (-Z, X, -Y) / 1000`.
Meshes are expressed in metres relative to their joint centres, with the
original leg assembly pose represented by all six joint positions equal to zero.
The body alone is rotated -90 degrees about the hip axis (ROS Y), with its COM
and inertia frame rotated consistently. Hip, knee and wheel frames are unchanged.

Joint centres were inferred from concentric circular mesh features. In CAD mm,
left hip is `(125, 0, 0)`, knee `(127, 104.76742, 14.10935)` and wheel
`(154.5, 204.76742, -32.52141)`; right centres negate CAD X. All axes are ROS +Y.
The knee centre uses the shared 4 mm radius feature in thigh and shin. Confirm
these centres and the mechanical joint arrangement against the original CAD
before using them for hardware control.

Supplier masses and estimated COM/inertia tensors are now included for all seven
parts. See [PHYSICS.md](PHYSICS.md) for values, assumptions and regeneration.
Hip/knee limits and effort/velocity values remain display placeholders.
Simulation-only collision shapes and controller interfaces are defined by
`simulation.urdf.xacro` and the `tap_sim` package; preview remains kinematic.
The original wheel_leg Gazebo model and controllers are not included here.

## Regenerate Meshes

Only regeneration needs `numpy` and `trimesh`. From the repository root, pass the
extracted assembly STL to:

```bash
python3 tools/import_cad_preview.py '/path/to/Lap rap he mo phong.stl'
```

For a headless transform preview use launch arguments `gui:=false rviz:=false`.
