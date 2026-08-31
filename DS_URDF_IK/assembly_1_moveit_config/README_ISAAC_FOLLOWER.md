# Isaac Sim and follower simultaneous MoveIt integration

Use `demo_isaac_follower.launch.py` when one MoveIt `Execute` should move both:

- Isaac Sim, through `sensor_msgs/msg/JointState` on `/joint_command`
- the physical LeRobot `dual_scorpion_follower`, through
  `DualScorpionFollower.send_action()`

This launch provides the same MoveIt actions as the Isaac-only and
follower-only launches:

```text
/left_arm_controller/follow_joint_trajectory
/right_arm_controller/follow_joint_trajectory
```

Do not run this launch at the same time as `demo_isaac.launch.py` or
`demo_follower.launch.py`; they provide the same action names.

## Isaac Sim setup

Start Isaac Sim and create the existing ROS 2 bridge graph first:

```python
exec(open("/home/syun/open_pj/dual_scorpion/DS_URDF_IK/assembly_1_moveit_config/scripts/create_isaac_moveit_graph.py").read())
```

Then press Play and confirm:

```bash
ros2 topic info /joint_states -v
ros2 topic info /joint_command -v
```

`/joint_states` should have an Isaac publisher, and `/joint_command` should have
an Isaac subscriber.

## Run

Use the LeRobot virtualenv so the ROS node can import both `rclpy` and LeRobot
dependencies.
If you need to rebuild first, run colcon from the repository root with
`--base-paths DS_URDF_IK` so colcon does not try to parse the LeRobot Python
package:

```bash
cd /home/syun/open_pj/dual_scorpion
source /opt/ros/jazzy/setup.bash
colcon --log-base DS_URDF_IK/log build --symlink-install --base-paths DS_URDF_IK \
  --build-base DS_URDF_IK/build \
  --install-base DS_URDF_IK/install \
  --packages-select assembly_1 assembly_1_moveit_config
```

```bash
cd /home/syun/open_pj/dual_scorpion/DS_URDF_IK
source /opt/ros/jazzy/setup.bash
source /home/syun/open_pj/dual_scorpion/.venv/bin/activate
source install/setup.bash

ros2 launch assembly_1_moveit_config demo_isaac_follower.launch.py \
  joint_command_topic:=/joint_command \
  lerobot_root:=/home/syun/open_pj/dual_scorpion \
  left_arm_port:=/dev/tty.usbmodemFOLLOWER_L \
  right_arm_port:=/dev/tty.usbmodemFOLLOWER_R \
  robot_id:=scorpion_follower_parallel_gripper
```

The follower bridge sends all left and right follower joints on every MoveIt
trajectory update, matching the `lerobot-teleoperate` behavior. If MoveIt plans
only `left_arm`, the right arm is held at its latest known position instead of
being sent as an empty command.

`revolute_4` is flipped by default with:

```bash
joint_signs:='{"revolute_4": -1}'
```

The bridge also applies a default zero-center offset for the left elbow joint:

```bash
{"revolute_4": 85.0}
```

LeRobot degrees use the calibrated range midpoint as zero, while these URDF
joints can have large non-zero range centers. The right elbow (`revolute_12`)
is left without default sign or offset correction because its mirror direction
depends on the physical calibration. If the physical follower is still offset
from Isaac/MoveIt, override the measured degree offsets, for example:

```bash
joint_offsets_deg:='{"revolute_4": 78.0, "revolute_12": -45.0}'
```

By default, `publish_follower_joint_states:=false`, so MoveIt uses Isaac Sim's
`/joint_states` as the current robot state. This avoids two publishers fighting
over the same joint names. Set `publish_follower_joint_states:=true` only when
Isaac is not publishing `/joint_states` and you intentionally want follower
hardware state to drive MoveIt's current state.

Also keep `subscribe_external_joint_states:=true` for normal simultaneous use.
The bridge uses Isaac's `/joint_states` to preserve non-commanded joints in the
full `/joint_command` message. Before pressing `Execute`, align the physical
follower and Isaac Sim to the same starting pose.

## Checks

```bash
ros2 action list | grep follow_joint_trajectory
ros2 topic info /joint_command -v
ros2 topic echo /joint_command
```

In RViz, `Plan` computes one trajectory and `Execute` sends each interpolated
point to both Isaac Sim and the physical follower.
