# Dual Scorpion URDF

This directory contains URDF and simulation assets for Dual Scorpion. The model
has a fixed base, 17 links, and 16 revolute joints, with seven joints on each arm.

It can be used with ROS 2, MuJoCo, Genesis, Isaac Sim, and Gazebo Sim.

## Contents

| Directory | Format |
| --- | --- |
| `urdf/` | URDF, ROS 2 description package, SRDF, visual meshes, and collision meshes |
| `mujoco/` | MuJoCo MJCF model |
| `genesis/` | Genesis URDF assets and runner |
| `isaac_sim/` | OpenUSD assets and runner |
| `gazebo_jetty/` | Gazebo Sim SDF model and runner |

Each simulator directory includes its own meshes and usage instructions.

## Quick start

From this directory:

```sh
cd mujoco
python3 run.py
```

For headless runs:

```sh
cd mujoco
python3 run.py --headless --steps 1000
```

See the README in the relevant simulator directory for dependencies and launch
commands.

## Model notes

- The model consists of a fixed `base_link`, right-arm links `r_link_1` to
  `r_link_8`, and left-arm links `l_link_1` to `l_link_8`.
- Mesh coordinates and model units are in meters.
- `neutral` sets all joint coordinates to zero. Captured-pose data is provided
  for comparison and is not a calibrated home position.
- Collision geometry is enabled, with the directly connected parent-child link
  pairs excluded. Apply the supplied SRDF or collision configuration in the
  target simulator.
- Joint limits and controller gains are provisional. Replace them with measured
  values before using the model for hardware control.
