"""MuJoCo kinematic preview of the existing URDFs and guarded Telegrip IK.

PyBullet stays in DIRECT mode for the established FK/IK implementation.
MuJoCo receives the validated joint targets; no gravity/contact stepping is done.
"""

import logging
import os
import threading
import xml.etree.ElementTree as ET
from contextlib import nullcontext
from copy import deepcopy
from pathlib import Path

import numpy as np

from .ik import DualSimulation

LOG = logging.getLogger(__name__)
COLORS = {"left": [0.15, 0.55, 0.95, 1], "right": [0.95, 0.55, 0.15, 1]}
JOINTS = [f"joint{i}" for i in range(7)] + ["gripper"]
# The Telegrip export's left chain follows the assembly's right branch
# (ds_urdf_v4_1 / ds_urdf_v4); its right chain follows the left branch
# (left_link_2 / left_link_5). Link names alone are insufficient to match CAD.
ASSEMBLY_LINKS = {
    "left": {
        "base_link": "base_link",
        "left_link_1": "right_link_1",
        "ds_urdf_v4_1": "ds_urdf_v4_1",
        "left_link_3": "right_link_3",
        "left_link_4": "right_link_4",
        "ds_urdf_v4": "ds_urdf_v4",
        "left_link_6": "right_link_6",
        "left_link_7": "right_link_7",
        "left_gripper_link": "right_gripper_link",
    },
    "right": {
        "right_link_1": "left_link_1",
        "right_link_2": "left_link_2",
        "right_link_3": "left_link_3",
        "right_link_4": "left_link_4",
        "right_link_5": "left_link_5",
        "right_link_6": "left_link_6",
        "right_link_7": "left_link_7",
        "right_gripper_link": "left_gripper_link",
    },
}


def mesh_path(filename, urdf_path):
    if filename.startswith("package://"):
        package, relative = filename[len("package://") :].split("/", 1)
        package_root = urdf_path.parent.parent
        if package != package_root.name:
            raise ValueError(f"Cannot resolve ROS package {package} from {urdf_path}")
        path = package_root / relative
    else:
        path = urdf_path.parent / filename
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"URDF mesh missing: {path}")
    return path


def add_assembly_visuals(tree, side, source_path):
    """Copy CAD visuals in local link coordinates onto the corrected IK tree."""
    source_path = Path(source_path).resolve()
    sources = {link.attrib["name"]: link for link in ET.parse(source_path).getroot().findall("link")}
    for link in tree.findall("link"):
        name = link.attrib["name"]
        if link.find("visual") is not None or name == "tcp_link" or (side == "right" and name == "base_link"):
            continue
        donor_name = ASSEMBLY_LINKS[side].get(name)
        if donor_name not in sources or sources[donor_name].find("visual") is None:
            raise ValueError(f"No assembly visual mapping for {side}/{name}; check robot.visual_urdf")
        for original in sources[donor_name].findall("visual"):
            visual = deepcopy(original)
            visual.set("name", f"cad_{name}")
            for mesh in visual.findall(".//mesh"):
                mesh.set("filename", str(mesh_path(mesh.attrib["filename"], source_path)))
            link.append(visual)


def build_model(urdfs, mujoco, visual_urdf=None):
    """Import both original URDF trees, preserving axes, limits and mount offsets."""
    scene = mujoco.MjSpec.from_string(
        '<mujoco model="Dual Scorpion mocopi preview">'
        '<option gravity="0 0 0"/>'
        '<visual><headlight ambient="0.4 0.4 0.4"/><global offwidth="960" offheight="720"/></visual>'
        '<worldbody><light pos="0 -1 2"/>'
        '<geom name="floor" type="plane" size="1 1 0.02" rgba="0.25 0.28 0.32 1"/>'
        "</worldbody></mujoco>"
    )
    for side in ("left", "right"):
        path = Path(urdfs[side]).resolve()
        tree = ET.parse(path).getroot()
        if visual_urdf:
            add_assembly_visuals(tree, side, visual_urdf)
        extension = tree.find("mujoco")
        if extension is None:
            extension = ET.SubElement(tree, "mujoco")
        compiler = extension.find("compiler")
        if compiler is None:
            compiler = ET.SubElement(extension, "compiler")
        # These URDFs have tiny masses and one zero inertia. Bounds are for
        # compilation of the preview only; original files remain untouched.
        compiler.attrib.update(
            fusestatic="false",
            discardvisual="false",
            balanceinertia="true",
            boundmass="0.000001",
            boundinertia="0.000000001",
            strippath="false",
        )
        for mesh in tree.findall(".//mesh"):
            mesh.set("filename", str(mesh_path(mesh.attrib["filename"], path)))
        arm = mujoco.MjSpec.from_string(ET.tostring(tree, encoding="unicode"))
        scene.attach(arm, frame=scene.worldbody.add_frame(), prefix=f"{side}/")
        tcp = scene.body(f"{side}/tcp_link")
        tcp.add_site(name=f"{side}/actual", size=[0.015, 0, 0], rgba=[0.2, 0.9, 0.3, 1])
        elbow_link = tree.find("joint[@name='joint4']/child").attrib["link"]
        scene.body(f"{side}/{elbow_link}").add_site(
            name=f"{side}/elbow_actual", size=[0.008, 0, 0], rgba=[0.9, 0.3, 0.8, 1]
        )
        target = scene.worldbody.add_body(name=f"{side}/target", mocap=True)
        target.add_geom(
            name=f"{side}/target_marker",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.018, 0, 0],
            contype=0,
            conaffinity=0,
            rgba=[*COLORS[side][:3], 0.45],
        )
        elbow_target = scene.worldbody.add_body(name=f"{side}/elbow_target", mocap=True)
        elbow_target.add_geom(
            name=f"{side}/elbow_target_marker",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.012, 0, 0],
            contype=0,
            conaffinity=0,
            rgba=[*COLORS[side][:3], 0],
        )
    return scene.compile()


class MujocoSimulation(DualSimulation):
    def __init__(self, config, gui=True):
        # Select the bundled X11 library before MuJoCo imports GLFW. This
        # desktop's native Wayland/libdecor path crashes during initialization.
        if gui and os.environ.get("DISPLAY"):
            os.environ.setdefault("PYGLFW_LIBRARY_VARIANT", "x11")
        try:
            import mujoco
        except ImportError as exc:
            raise ImportError(
                "MuJoCo is not installed. Run: uv pip install --python .venv/bin/python "
                "-e './telegrip_teleoperation[mujoco]' ; or use --sim-backend pybullet"
            ) from exc

        self.mujoco = mujoco
        self.viewer = None
        self.postures = {}
        self.viewer_threads = []
        self.model = build_model(config["urdf"], mujoco, config.get("visual_urdf"))
        self.data = mujoco.MjData(self.model)
        self.qpos_indices = {
            side: np.array([self.model.joint(f"{side}/{name}").qposadr[0] for name in JOINTS])
            for side in ("left", "right")
        }
        super().__init__(config, gui=False)
        try:
            self.targets = {side: target.copy() for side, target in self.initial_ee.items()}
            self._sync_state()
            if gui and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
                LOG.warning("No display available; MuJoCo running headless")
            elif gui:
                import mujoco.viewer

                existing_threads = set(threading.enumerate())
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self.viewer_threads = [
                    thread
                    for thread in threading.enumerate()
                    if thread not in existing_threads and thread.daemon
                ]
                with self.viewer.lock():
                    self.viewer.cam.lookat[:] = [0, 0, 0.25]
                    self.viewer.cam.distance = 1.3
                    self.viewer.cam.azimuth = 160
                    self.viewer.cam.elevation = -25
                self.viewer.sync()
        except Exception:
            self.close()
            raise

    def _sync_state(self):
        with self.viewer.lock() if self.viewer else nullcontext():
            for side, arm in self.arms.items():
                self.data.qpos[self.qpos_indices[side]] = np.deg2rad(arm.last_valid_target)
                marker = self.model.body(f"{side}/target").mocapid[0]
                self.data.mocap_pos[marker] = self.targets[side][:3, 3]
                posture = self.postures.get(side)
                elbow_marker = self.model.body(f"{side}/elbow_target").mocapid[0]
                self.model.geom(f"{side}/elbow_target_marker").rgba[3] = 0.45 if posture else 0
                if posture:
                    self.data.mocap_pos[elbow_marker] = posture.elbow
            self.data.qvel[:] = 0
            self.mujoco.mj_forward(self.model, self.data)
        if self.viewer:
            self.viewer.sync()

    def update(self, targets, valid=True, postures=None):
        super().update(targets, valid, postures)
        if valid:
            for side, arm in self.arms.items():
                if arm.valid:
                    self.postures[side] = deepcopy((postures or {}).get(side))
        self._sync_state()

    def is_running(self):
        return self.viewer is None or self.viewer.is_running()

    def close(self):
        try:
            if self.viewer:
                self.viewer.close()
                # Handle.close() requests exit asynchronously. Let its render
                # threads finish before GLFW's interpreter-exit termination.
                for thread in self.viewer_threads:
                    thread.join(timeout=10)
                self.viewer = None
        finally:
            super().close()
