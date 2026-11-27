"""Run the shared wrist-camera URDF in Genesis and optionally capture both cameras."""

import argparse
import json
import math
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from xml.etree import ElementTree as ET

import numpy as np

DIRECTORY = Path(__file__).resolve().parent
MODEL = DIRECTORY.parent / "urdf/scorpion.urdf"
INITIAL_STATE = DIRECTORY.parents[1] / "default/genesis/initial_state.json"
JOINT_NAMES = [f"joint{i}_{side}" for side in ("R", "L") for i in range(8)]


def add_wrist_cameras(scene, robot):
    from genesis.options.sensors.camera import RasterizerCameraOptions

    cameras = {}
    for sensor in ET.parse(MODEL).findall("gazebo/sensor[@type='camera']"):
        camera = sensor.find("camera")
        frame = robot.get_link(camera.findtext("optical_frame_id"))
        width = int(camera.findtext("image/width"))
        height = int(camera.findtext("image/height"))
        hfov = float(camera.findtext("horizontal_fov"))
        vfov = math.degrees(2 * math.atan(math.tan(hfov / 2) * height / width))
        # The URDF optical frame looks along +Z with +Y down, while the rasterizer uses -Z with +Y up.
        options = RasterizerCameraOptions(
            entity_idx=robot.idx,
            link_idx_local=frame.idx_local,
            offset_T=np.diag((1.0, -1.0, -1.0, 1.0)),
            res=(width, height),
            fov=vfov,
            near=float(camera.findtext("clip/near")),
            far=float(camera.findtext("clip/far")),
        )
        cameras[sensor.attrib["name"]] = scene.add_sensor(options), height, width
    return cameras


def run(args):
    import genesis as gs

    gs.init(backend=gs.cpu, logging_level="warning")
    try:
        scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=0.002, substeps=2),
            rigid_options=gs.options.RigidOptions(
                enable_self_collision=True,
                enable_neutral_collision=True,
                enable_adjacent_collision=False,
                max_collision_pairs=512,
            ),
            viewer_options=gs.options.ViewerOptions(camera_pos=(1, -1, 0.8), camera_lookat=(0, 0, 0.23)),
            show_viewer=not args.headless,
        )
        material = gs.materials.Rigid(friction=0.6)
        scene.add_entity(gs.morphs.Plane(pos=(0, 0, -0.03)), material=material)
        robot = scene.add_entity(
            gs.morphs.URDF(
                file=str(MODEL),
                fixed=True,
                merge_fixed_links=False,
                recompute_inertia=False,
                convexify=False,
                decimate=False,
                watertighten=None,
            ),
            material=material,
        )
        cameras = add_wrist_cameras(scene, robot) if args.camera_output else {}
        scene.build()

        dofs = [robot.get_joint(name).dofs_idx_local[0] for name in JOINT_NAMES]
        state = json.loads(INITIAL_STATE.read_text(encoding="utf-8")) if args.pose == "fusion_current" else {}
        q = np.array([state.get(name, 0.0) for name in JOINT_NAMES])
        robot.set_dofs_position(q, dofs_idx_local=dofs)
        robot.set_dofs_kp(np.full(len(dofs), 10.0), dofs_idx_local=dofs)
        robot.set_dofs_kv(np.full(len(dofs), 0.5), dofs_idx_local=dofs)
        limits = np.ones(len(dofs))
        robot.set_dofs_force_range(-limits, limits, dofs_idx_local=dofs)
        robot.control_dofs_position(q, dofs_idx_local=dofs)
        for _ in range(args.steps):
            scene.step()
        if not np.isfinite(robot.get_dofs_position().detach().cpu().numpy()).all():
            raise RuntimeError("Non-finite joint state")
        print(
            f"Genesis {gs.__version__}: {robot.n_links} links, {robot.n_dofs} DOF, "
            f"{args.steps} finite steps ({args.pose})"
        )

        if cameras:
            from PIL import Image

            args.camera_output.mkdir(parents=True, exist_ok=True)
            for name, (sensor, height, width) in cameras.items():
                image = sensor.read().rgb.detach().cpu().numpy().reshape(height, width, 3)
                path = args.camera_output / f"{name}.png"
                Image.fromarray(image).save(path)
                print(path)
    finally:
        gs.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--pose", choices=("neutral", "fusion_current"), default="neutral")
    parser.add_argument(
        "--camera-output", type=Path, help="Save one RGB frame from each wrist after stepping"
    )
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")

    with TemporaryDirectory(prefix="scorpion_camera_genesis_") as cache:
        os.environ.setdefault("GS_CACHE_FILE_PATH", cache)
        os.environ.setdefault("TI_OFFLINE_CACHE_FILE_PATH", str(Path(cache) / "taichi"))
        run(args)


if __name__ == "__main__":
    main()
