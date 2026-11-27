"""Load the wrist-camera URDF with native MuJoCo cameras and its adjacent-link rules."""

import argparse
import math
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

DIRECTORY = Path(__file__).resolve().parent
URDF = DIRECTORY / "scorpion_wrist_cameras_mujoco.urdf"
SRDF = DIRECTORY / "scorpion_wrist_cameras.srdf"


def load_model() -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(URDF))
    for pair in ET.parse(SRDF).findall("disable_collisions"):
        spec.add_exclude(bodyname1=pair.attrib["link1"], bodyname2=pair.attrib["link2"])
    for geom in spec.geoms:
        if geom.contype or geom.conaffinity:
            geom.group = 3

    for sensor in ET.parse(URDF).findall("gazebo/sensor[@type='camera']"):
        camera = sensor.find("camera")
        width = int(camera.findtext("image/width"))
        height = int(camera.findtext("image/height"))
        hfov = float(camera.findtext("horizontal_fov"))
        vfov = math.degrees(2 * math.atan(math.tan(hfov / 2) * height / width))
        frame = spec.body(camera.findtext("optical_frame_id"))
        # REP-103 looks along +Z. MuJoCo looks along -Z with +Y up.
        frame.add_camera(name=sensor.attrib["name"], quat=(0, 1, 0, 0), fovy=vfov)
    return spec.compile()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=1000)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")

    model = load_model()
    data = mujoco.MjData(model)
    if args.headless:
        for _ in range(args.steps):
            mujoco.mj_step(model, data)
            if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
                raise RuntimeError(f"Non-finite joint state at {data.time:.4f} seconds")
        print(f"MuJoCo {mujoco.__version__} loaded {model.nv} DOF and {model.ncam} wrist cameras")
        print(f"Completed {args.steps} simulation steps with finite joint positions and velocities")
        return

    from mujoco import viewer

    desired_angle = 0.0

    def pose_key(key: int) -> None:
        nonlocal desired_angle
        if key in (ord("O"), ord("N")):
            desired_angle = math.radians(-30.0) if key == ord("O") else 0.0

    gripper_indices = [model.joint(f"joint7_{side}").qposadr[0] for side in ("R", "L")]
    displayed_angle = 0.0
    mujoco.mj_forward(model, data)
    with viewer.launch_passive(model, data, key_callback=pose_key) as window:
        with window.lock():
            window.opt.geomgroup[3] = 0
            window.cam.lookat[:] = [0, 0.11, 0.21]
            window.cam.azimuth = 120
            window.cam.elevation = -18
            window.cam.distance = 0.85
        while window.is_running():
            next_angle = desired_angle
            if next_angle != displayed_angle:
                with window.lock():
                    data.qpos[gripper_indices] = next_angle
                    mujoco.mj_forward(model, data)
                displayed_angle = next_angle
            window.sync()
            time.sleep(1 / 60)


if __name__ == "__main__":
    main()
