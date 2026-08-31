# Isaac Sim integration

Use `demo_isaac.launch.py` when Isaac Sim should receive MoveIt trajectory commands.
Use `demo.launch.py` only for local RViz / Fake Controller checks.

## Working Procedure

Use this order.

1. Start Isaac Sim from a terminal where ROS 2 Jazzy is sourced.

```bash
cd ~/isaacsim
source /opt/ros/jazzy/setup.bash
./isaac-sim.sh
```

2. Open or import the robot in Isaac Sim.

3. Select the robot articulation root in the Isaac Sim Stage.

4. Open `Window > Script Editor` and run:

```python
exec(open("/home/syun/DS_URDF_IK/assembly_1_moveit_config/scripts/create_isaac_moveit_graph.py").read())
```

This creates `/ROS2_MoveIt_Bridge` with:

- `ROS2 Publish Joint State` on `joint_states`
- `ROS2 Subscribe Joint State` on `joint_command`
- `Isaac Articulation Controller` connected to the subscribed position command

5. Press Play in Isaac Sim.

6. In a ROS 2 terminal, verify that Isaac Sim is connected:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic info /joint_states -v
ros2 topic info /joint_command -v
```

Expected:

```text
/joint_states   Publisher count: 1 or more
/joint_command  Subscription count: 1 or more
```

7. Start MoveIt for Isaac Sim:

If you have not built the ROS packages yet, build only the ROS workspace under
`DS_URDF_IK`:

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
source install/setup.bash
ros2 launch assembly_1_moveit_config demo_isaac.launch.py joint_command_topic:=/joint_command
```

8. In RViz, use the MotionPlanning panel. `Plan` computes the path, and `Execute` sends the trajectory to Isaac Sim through `/joint_command`.

## Required Isaac Sim State

- ROS 2 Bridge extension is enabled.
- Isaac Sim is in Play mode while checking topics and executing trajectories.
- Isaac Sim publishes robot joint states to `/joint_states`.
- Isaac Sim subscribes to `sensor_msgs/msg/JointState` commands on `/joint_command`.
- The subscriber drives the robot Articulation Controller in position mode.
- Isaac Sim and ROS 2 use the same `ROS_DOMAIN_ID`.
- Isaac joint names match the URDF joint names, for example `revolute_1` through `revolute_16`.

## Quick checks

```bash
ros2 topic echo /joint_states --once
ros2 topic info /joint_states -v
ros2 topic info /joint_command -v
ros2 topic echo /joint_command
ros2 action list | grep follow_joint_trajectory
```

`/joint_command` must show at least one subscription from Isaac Sim before
Isaac can react to RViz Execute commands. If the subscription count is `0`,
fix the Isaac Sim Action Graph topic name and Articulation Controller wiring
first.

`/joint_states` must show a publisher from Isaac Sim. If it has no publisher,
MoveIt cannot validate Execute against the simulated robot state.

Expected actions:

```text
/left_arm_controller/follow_joint_trajectory
/right_arm_controller/follow_joint_trajectory
```

## Troubleshooting

If `isaac_follow_joint_trajectory` prints this warning:

```text
no subscribers on /joint_command; Isaac Sim is not receiving commands;
no recent external /joint_states received; Isaac Sim is not publishing robot state
```

the ROS 2 side is running, but Isaac Sim is not connected. Re-run the Script
Editor command, press Play, and re-check `/joint_states` and `/joint_command`.

If Execute aborts with:

```text
Failed to validate trajectory: couldn't receive full current joint state within 1s
```

MoveIt is not receiving fresh `/joint_states` from Isaac Sim. Check that Play is
active and `/joint_states` has a publisher from Isaac Sim.

If Isaac uses `/clock` and all ROS nodes should use simulation time, launch with:

```bash
ros2 launch assembly_1_moveit_config demo_isaac.launch.py use_sim_time:=true joint_command_topic:=/joint_command
```
