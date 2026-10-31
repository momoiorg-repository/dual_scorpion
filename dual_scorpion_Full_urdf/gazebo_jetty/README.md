# Gazebo Sim

This folder contains a self-contained SDF 1.12 robot model and demo world for
Gazebo Sim (modern Gazebo, command `gz sim`).

## Run

Install Gazebo Sim and the joint-controller system plugin for your distribution,
then run from this folder:

```sh
sh run.sh
```

The script locates its own directory, adds `models/` to `GZ_SIM_RESOURCE_PATH`,
and opens `worlds/scorpion.sdf`. The world starts the robot in its neutral pose.

## Model

- `models/scorpion/model.sdf` contains 17 links, 16 revolute joints, one fixed
  world joint, visual meshes, and 370 convex collision meshes.
- Each revolute joint has a provisional `JointPositionController`.
- `initial_state.json` records the Fusion display pose; it is not applied
  automatically by `run.sh`.

The SDF and all referenced mesh paths were parsed and checked structurally.
Gazebo Sim itself was not launched during generation and validation, so runtime
physics, plugin loading, and rendering are not yet verified.
