# Dual Scorpion with wrist cameras

The camera configuration lives in this directory and shares the original arm meshes in [`urdf/meshes`](../../meshes). Keep the `dual_scorpion_Full_urdf` directory together when copying the models. The original entry points for LeLab, ROS 2, MuJoCo and Genesis remain at their established paths.

## Choose a description

- [scorpion_wrist_cameras.urdf](scorpion_wrist_cameras.urdf) uses relative paths for general URDF loaders, including Genesis. The LeLab static model endpoint also serves it at `/scorpion-urdf/variants/wrist_cameras/scorpion_wrist_cameras.urdf`. LeLab's built-in Scorpion selector continues to load the original model.
- [scorpion_wrist_cameras_ros.urdf](scorpion_wrist_cameras_ros.urdf) uses `package://scorpion_description/` paths. Once the [`urdf` ROS 2 package](../../README.md) is built, open it with `ros2 launch scorpion_description display_wrist_cameras.launch.py`. The Gazebo sensor definitions are included for consumers of the ROS 2 description.
- [scorpion_wrist_cameras_mujoco.urdf](scorpion_wrist_cameras_mujoco.urdf) uses MuJoCo compiler settings and the existing base OBJ. [run_mujoco.py](run_mujoco.py) adds both lens views and applies the exclusions from the matching [SRDF](scorpion_wrist_cameras.srdf).

Run the MuJoCo viewer from this directory on macOS.

```sh
uv run --no-project --with 'mujoco==4.0.1' mjpython run_mujoco.py
```

Use the camera selector for `l_wrist_camera` or `r_wrist_camera`. Press `O` for the 30° open view or `N` for the neutral pose. To check the model without a viewer, run `uv run --no-project --with 'mujoco==4.0.1' python run_mujoco.py --headless`.

Edit the general URDF to update the shared configuration, then run `uv run --no-project python export_variants.py`. Use `--check` to detect outdated generated files.

## Geometry and verification

The configuration retains all 16 continuous joints from [scorpion.urdf](../../scorpion.urdf). Each wrist carries the repaired integrated mount, the original joint 6 drive horn, the original ST3215 gripper motor case and an InnoMaker U20CAM-1080P-S1 camera. `r_link_8` and `l_link_8` retain the moving jaws and their output and idler horns. The camera is fixed to link 7, so it follows wrist roll while the original `joint7_R` and `joint7_L` can open and close the jaws independently.

- The mount visuals come from the repaired Fusion solid exported alongside [the STEP](../../../../STEP/Wrist_Cam_Mount_32x32_UVC_Module_SO101_Integrated_v1.step). Thirteen corresponding cylindrical faces on each existing wrist establish the side-specific placement. The collision meshes use 20 convex approximations per mount.
- The [camera STEP](../../../../STEP/InnoMaker_U20CAM_1080P-S1.step) contains the PCB, USB connector, lens housing and front glass used in the URDF. The modeled camera uses the manufacturer's 32 × 32 mm PCB, Ø2.2 mm board holes and M12 lens specification. The 27 mm pitch comes from the mount. PCB thickness, housing depth and mass are modeled estimates. The Fusion project reference `Camera Model` has a 38 × 38 mm PCB. [InnoMaker U20CAM-1080P-S1](https://github.com/INNO-MAKER/U20CAM-1080P-S1)
- The [ST3215 servo STEP](../../../../STEP/ST3215_Wrist_Gripper_Servo.step) contains the Fusion servo assembly and both horns. The URDF reuses the original Scorpion servo case locations, horn locations and CAD-derived component masses.
- The `l_wrist_camera_optical_frame` and `r_wrist_camera_optical_frame` frames use Z forward, X right and Y down. The Gazebo camera elements specify 1920 × 1080 at 30 Hz and a nominal horizontal field of view of 106°. [ROS REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
- The MuJoCo preview applies the model's parent-child contact exclusions. At the neutral pose the center of each camera view is clear, with both jaws visible at the bottom. The nominal CAD meshes also retain approximately 305 mm³ of overlap between each printed shell and its motor case, within 0.01 mm³ of the original model. Physical mounting, wiring and optical calibration remain unverified.
