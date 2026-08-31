# Dual Scorpion follower MoveIt integration

Use `demo_follower.launch.py` when MoveIt should execute trajectories on the
physical LeRobot `dual_scorpion_follower` instead of Isaac Sim.

The launch keeps the same MoveIt controller/action names as the Isaac bridge:

```text
/left_arm_controller/follow_joint_trajectory
/right_arm_controller/follow_joint_trajectory
```

It publishes the physical follower state to `/joint_states` and converts MoveIt
radian commands into `DualScorpionFollower.send_action()` commands.

## Build

```bash
cd /home/syun/open_pj/dual_scorpion
source /opt/ros/jazzy/setup.bash
colcon --log-base DS_URDF_IK/log build --symlink-install --base-paths DS_URDF_IK \
  --build-base DS_URDF_IK/build \
  --install-base DS_URDF_IK/install \
  --packages-select assembly_1 assembly_1_moveit_config
source DS_URDF_IK/install/setup.bash
```

Do not run plain `colcon build` from the repository root. It will try to inspect
the LeRobot Python package in `.`. Use `--base-paths DS_URDF_IK` or run colcon
from inside `DS_URDF_IK`.

## Run

Pass the same follower ports and calibration id that you use with
`lerobot-calibrate` / `lerobot-teleoperate`.
Use the LeRobot virtualenv so the ROS node can import both `rclpy` and LeRobot
dependencies.

```bash
cd /home/syun/open_pj/dual_scorpion/DS_URDF_IK
source /opt/ros/jazzy/setup.bash
source /home/syun/open_pj/dual_scorpion/.venv/bin/activate
source install/setup.bash

ros2 launch assembly_1_moveit_config demo_follower.launch.py \
  lerobot_root:=/home/syun/open_pj/dual_scorpion \
  left_arm_port:=/dev/tty.usbmodemFOLLOWER_L \
  right_arm_port:=/dev/tty.usbmodemFOLLOWER_R \
  robot_id:=scorpion_follower_parallel_gripper
```

If your calibration was saved with the default `robot.id=None`, omit
`robot_id:=...`.

## Useful launch arguments

- `max_relative_target:=""` is the default, matching `lerobot-teleoperate`.
  Set `max_relative_target:=12.0` if you want an extra LeRobot safety clamp.
- `calibrate_on_connect:=false` avoids interactive calibration from a ROS launch.
  Keep it false for normal use and run `lerobot-calibrate` separately first.
- `publish_rate_hz:=30.0` controls interpolation and `/joint_states` publishing.
- `joint_signs:='{"revolute_4": -1}'` is the default because this physical joint
  moves opposite to the URDF/MoveIt positive direction. The right elbow
  `revolute_12` is not sign-flipped by default.
- The bridge applies a default zero-center offset `{"revolute_4": 85.0}`
  because LeRobot degrees are centered on the calibrated range midpoint. The
  right elbow `revolute_12` has no default offset because its mirror direction
  depends on the physical calibration. Use
  `joint_offsets_deg:='{"revolute_12": -45.0}'` only if a measured correction is
  needed.
- `gripper_min_percent:=0.0 gripper_max_percent:=100.0` maps the URDF gripper
  joint limits to the LeRobot gripper range. Swap the two values if the gripper
  moves in the opposite direction.

## Checks

```bash
ros2 action list | grep follow_joint_trajectory
ros2 topic echo /joint_states --once
```

In RViz, use the MotionPlanning panel. `Plan` computes the path and `Execute`
sends it directly to the follower hardware.
