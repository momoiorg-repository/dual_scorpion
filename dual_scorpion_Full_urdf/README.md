# Dual Scorpion simulation models

- [default](default/README.md) contains the original Dual Scorpion, with 17 links and 16 movable joints. The [standard URDF](default/urdf/scorpion.urdf) is in this folder.
- [with_camera](with_camera/README.md) adds an InnoMaker camera to each wrist. It retains the left and right grippers and their opening joints. Start from the [camera URDF](with_camera/urdf/scorpion.urdf).

Both folders provide entry points for MuJoCo, Isaac Sim, ROS 2, Genesis, and Gazebo Jetty. Keep both folders together to share the meshes and USD layers. Run `git lfs pull` after cloning to populate the original meshes and USD assets.

The former top-level simulator paths are compatibility symlinks for local checkouts that support symlinks. Use the paths under `default/` for GitHub file links and new integrations.
