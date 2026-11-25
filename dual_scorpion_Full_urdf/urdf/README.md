# URDF and ROS 2

This folder contains the URDF model, meshes, SRDF, and ROS 2 package files for
Dual Scorpion.

## Files

- `scorpion.urdf` uses relative mesh paths.
- `scorpion_ros.urdf` uses `package://scorpion_description/` mesh paths.
- `scorpion.srdf` defines the intended adjacent-link collision exclusions.
- `collision_config.json` defines the collision policy.
- `CMakeLists.txt`, `package.xml`, and `launch/` provide the ROS 2 package.

## Run in ROS 2

Create a ROS 2 workspace and copy this folder as the `scorpion_description`
package:

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

Add a RobotModel display in RViz and select `/robot_description`.

## Notes

- URDF does not apply SRDF exclusions automatically. Apply them in the target
  simulator or motion-planning stack.
- The collision meshes are convex approximations of the visual model.
- Joint limits and controller values are provisional.
