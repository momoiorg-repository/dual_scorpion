# Dual Scorpion with wrist cameras

Both wrists carry the integrated SO-101 mount and an InnoMaker U20CAM-1080P-S1 model. The model includes the right and left gripper servos, `r_link_8` and `l_link_8`, and the 16 original movable joints `joint0_R` through `joint7_R` and `joint0_L` through `joint7_L`. Each camera has a mechanical link and a ROS optical frame. The nominal camera settings are 1920 × 1080, 106° horizontal field of view, and 30 Hz for the Gazebo publishers.

Open the standard robot from [default/](../default/README.md). Its meshes and USD layers are also used by the camera configuration. Keep both folders together when copying or mounting the simulation models. For a Git clone with LFS pointers, run `git lfs pull` before loading either configuration.

## MuJoCo

Open [mujoco/scorpion.xml](mujoco/scorpion.xml) as a native MJCF. The camera names are `r_wrist_camera` and `l_wrist_camera`. The runner retains the stock actuators and the `neutral` and `fusion_current` keyframes. In the viewer, `O` commands both grippers open and `N` returns them to the starting target.

```sh
uv run --no-project --with 'mujoco==4.0.1' mujoco/run.py --headless --steps 1000
```

On macOS, launch the interactive viewer via MuJoCo's `mjpython`.

```sh
uv run --no-project --with 'mujoco==4.0.1' mjpython mujoco/run.py
```

## Isaac Sim

Open [isaac_sim/scorpion.usd](isaac_sim/scorpion.usd) in Isaac Sim. The editable [scorpion.usda](isaac_sim/scorpion.usda) composes the standard articulation with the new wrist geometry and fixed camera joints. The alternate [scorpion_current_pose.usda](isaac_sim/scorpion_current_pose.usda) applies the captured pose. Select either named camera under `/Scorpion/r_wrist_camera_link` or `/Scorpion/l_wrist_camera_link`, or use its prim path when creating a render product.

```sh
"$ISAAC_SIM_ROOT/python.sh" isaac_sim/run.py
```

## URDF and ROS 2

Use [urdf/scorpion.urdf](urdf/scorpion.urdf) from tools that support relative mesh paths. For ROS 2 use [urdf/scorpion_ros.urdf](urdf/scorpion_ros.urdf) and the shared `scorpion_description` package. [urdf/scorpion.srdf](urdf/scorpion.srdf) records the 18 directly connected collision exclusions.

Run from `with_camera/` with ROS 2 sourced. A workspace with `scorpion_description` already installed can start at the launch command.

```sh
scorpion_camera_package="$PWD"
mkdir -p ~/scorpion_ws
cd ~/scorpion_ws
colcon build --base-paths "$scorpion_camera_package/../default/urdf" --packages-select scorpion_description
. install/setup.sh
ros2 launch "$scorpion_camera_package/urdf/display.launch.py"
```

Launch with `gui:=false` for headless joint states and TF. Launch with `publish_joint_states:=false` when a simulator already publishes `/joint_states`.

## Genesis

The [Genesis runner](genesis/run.py) uses the shared camera URDF, its fixed frames and precomputed collision shapes. Set `--camera-output` to save one RGB image from each moving wrist. Omit it for a physics-only run without a camera renderer.

```sh
uv run --no-project --with 'genesis-world==1.4.4' --with torch \
  genesis/run.py --headless --steps 50 --camera-output /tmp/scorpion-genesis-rgb
```

## Gazebo Jetty

The [world](gazebo_jetty/worlds/scorpion.sdf) loads the [camera-equipped SDF model](gazebo_jetty/models/scorpion_wrist_cameras/model.sdf). The runner registers the shared mesh locations and enables Assimp so DART can use the convex OBJ collision meshes. The world starts in the neutral pose and keeps the standard position controller topics. Camera frames and image topics use `r_wrist_camera` and `l_wrist_camera` with the same optical frame names as ROS 2.

```sh
./gazebo_jetty/run.sh
```

With a `ros_gz_bridge` build compatible with Gazebo Jetty, the supplied [bridge configuration](gazebo_jetty/ros_gz_bridge.yaml) connects the two RGB streams, their camera info, and joint states to ROS 2. Start the ROS launch with `publish_joint_states:=false` to use the Gazebo poses for TF.

```sh
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="$PWD/gazebo_jetty/ros_gz_bridge.yaml"
```

## Regenerate the models

[build_models.py](build_models.py) combines the [canonical wrist-camera URDF](../default/urdf/variants/wrist_cameras/scorpion_wrist_cameras.urdf) and adjacent-link rules with the standard simulator scenes. The Genesis runner reads the shared camera URDF directly. From `with_camera/`, omit `--check` to update the generated files.

```sh
uv run --no-project --with 'usd-core==26.5' --with 'trimesh==5.2.2' --with scipy \
  build_models.py --check
```
