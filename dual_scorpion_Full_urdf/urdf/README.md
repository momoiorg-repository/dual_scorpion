# URDF and ROS 2 Description

This folder contains a 17-link, 16-joint URDF description with visual STL meshes
and 370 convex OBJ collision meshes.

## Files

- `scorpion.urdf` uses relative mesh paths and is suitable for standalone URDF viewers.
- `scorpion_ros.urdf` uses `package://scorpion_description/` mesh paths for ROS 2.
- `scorpion.srdf` records the 16 intended parent-child collision exclusions.
- `collision_config.json` records the collision policy in machine-readable form.
- `initial_state.json` stores the Fusion display pose in radians.
- `CMakeLists.txt`, `package.xml`, and `launch/` provide a ROS 2 description package.

## Use as a ROS 2 package

The launch file requires `robot_state_publisher`,
`joint_state_publisher_gui`, and `rviz2`. Create a ROS 2 workspace and copy this
folder into it as the `scorpion_description` package:

```sh
mkdir -p ~/scorpion_ws/src
cp -R <path-to-repository>/dual_scorpion_Full_urdf/urdf \
  ~/scorpion_ws/src/scorpion_description
cd ~/scorpion_ws
source /opt/ros/<distribution>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select scorpion_description
source install/setup.bash
ros2 launch scorpion_description display.launch.py
```

The launch file starts RViz with its default layout. Add a RobotModel display and
select `/robot_description` to inspect the model. No calibrated joint limits,
collision checking, controller, or hardware driver is supplied here.

## Collision policy

The URDF contains self-collision geometry, but URDF itself has no standard field
for disabling specific link pairs. When using MoveIt, load `scorpion.srdf` so the
16 adjacent pairs remain excluded. For other consumers, reproduce the exclusions
listed in `collision_config.json` while keeping collisions enabled between
non-adjacent links.

The collision meshes are convex approximations. Use the original STEP or exact
CAD solids for detailed interference analysis.
