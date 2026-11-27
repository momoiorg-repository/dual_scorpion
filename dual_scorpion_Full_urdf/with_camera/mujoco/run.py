"""Run the native wrist-camera MJCF with the stock Dual Scorpion actuators."""

import argparse
import math
import time
from pathlib import Path

import mujoco
import numpy as np

MODEL = Path(__file__).resolve().with_name("scorpion.xml")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--pose", choices=("neutral", "fusion_current"), default="neutral")
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(args.pose).id)
    mujoco.mj_forward(model, data)

    if args.headless:
        for _ in range(args.steps):
            mujoco.mj_step(model, data)
            if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
                raise RuntimeError(f"Non-finite joint state at {data.time:.4f} s")
        print(
            f"MuJoCo {mujoco.__version__}: {model.nv} DOF, {model.nu} actuators, "
            f"{model.ncam} cameras, {args.steps} finite steps ({args.pose})"
        )
        return

    from mujoco import viewer

    gripper_ids = [model.actuator(f"joint7_{side}_position").id for side in ("R", "L")]
    gripper_targets = data.ctrl[gripper_ids].copy()
    opening = False

    def pose_key(key: int) -> None:
        nonlocal opening
        if key in (ord("O"), ord("N")):
            opening = key == ord("O")

    with viewer.launch_passive(model, data, key_callback=pose_key) as window:
        with window.lock():
            window.opt.geomgroup[3] = 0
            window.cam.lookat[:] = [0, 0.08, 0.23]
            window.cam.distance = 1.2
            window.cam.azimuth = 135
            window.cam.elevation = -20
        while window.is_running():
            start = time.monotonic()
            with window.lock():
                data.ctrl[gripper_ids] = math.radians(-30) if opening else gripper_targets
                mujoco.mj_step(model, data)
            window.sync()
            time.sleep(max(0, model.opt.timestep - (time.monotonic() - start)))


if __name__ == "__main__":
    main()
