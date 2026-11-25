# Gazebo Sim

This folder contains an SDF model and demo world for Gazebo Sim.

## Run

From this folder:

```sh
sh run.sh
```

The script configures the model resource path and opens
`worlds/scorpion.sdf` with the robot in its neutral pose.

## Model files

- `models/scorpion/model.sdf` contains the robot links, joints, collisions,
  visuals, and position controllers.
- `initial_state.json` contains an alternative captured pose and is not loaded
  automatically by `run.sh`.

## Notes

Joint limits and controller values are provisional. Gazebo Sim runtime behavior
has not been verified yet.
