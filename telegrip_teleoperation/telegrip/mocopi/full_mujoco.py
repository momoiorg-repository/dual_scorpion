"""Kinematic mocopi preview using the full native Scorpion MuJoCo model.

The supplied CAD export has no measured joint limits. The +/-180 degree
optimization bounds below are preview-only and do not authorize hardware use.
"""

import logging
import os
import threading
from contextlib import nullcontext
from copy import deepcopy
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .geometry import pose
from .ik import SafeIK
from .mujoco_sim import COLORS, add_orientation_axes

LOG = logging.getLogger(__name__)
SUFFIX = {"left": "L", "right": "R"}


def full_model_path(config):
    """Resolve the MJCF path; configs without the key fail with a clear message."""
    path = config.get("mujoco_full_model")
    if not path:
        raise ValueError("robot.mujoco_full_model is not set; add it to the YAML or pass --mujoco-model")
    return path


# Preview poses for this CAD frame. They are not calibrated hardware positions.
FULL_START_POSES = {
    "work": {side: [80, 0, 0, -30, 0, 0, 0, 0] for side in SUFFIX},
    "backwards": {side: [0, 0, 0, -30, 0, 0, 0, 0] for side in SUFFIX},
}


def build_full_model(path, mujoco):
    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Full MuJoCo model not found: {path}")
    spec = mujoco.MjSpec.from_file(str(path))
    for side, suffix in SUFFIX.items():
        tcp = spec.body(f"{suffix.lower()}_link_8")
        tcp.add_site(name=f"{side}/tcp_site", size=[0.002, 0, 0], rgba=[0, 0, 0, 0])
        add_orientation_axes(tcp, f"{side}/actual_axis", mujoco, 0.07, 1)
        spec.body(f"{suffix.lower()}_link_5").add_site(
            name=f"{side}/elbow_actual", size=[0.008, 0, 0], rgba=[0.9, 0.3, 0.8, 1]
        )
        target = spec.worldbody.add_body(name=f"{side}/target", mocap=True)
        add_orientation_axes(target, f"{side}/target_axis", mujoco, 0.11, 0.6)
        elbow = spec.worldbody.add_body(name=f"{side}/elbow_target", mocap=True)
        elbow.add_geom(
            name=f"{side}/elbow_target_marker",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.012, 0, 0],
            contype=0,
            conaffinity=0,
            rgba=[*COLORS[side][:3], 0],
        )
    return spec.compile()


class FullModelFK:
    """Use a private MuJoCo data instance so IK probes do not move the viewer."""

    def __init__(self, model, side, mujoco):
        self.model, self.mujoco = model, mujoco
        self.data = mujoco.MjData(model)
        suffix = SUFFIX[side]
        self.qpos = np.array([model.joint(f"joint{i}_{suffix}").qposadr[0] for i in range(8)], dtype=int)
        self.dofs = np.array([model.joint(f"joint{i}_{suffix}").dofadr[0] for i in range(7)], dtype=int)
        self.site = model.site(f"{side}/tcp_site").id
        self.shoulder = model.body(f"{suffix.lower()}_link_1").id
        self.upper = model.body(f"{suffix.lower()}_link_3").id
        self.elbow = model.body(f"{suffix.lower()}_link_5").id

    def set_joints(self, joints):
        joints = np.asarray(joints, dtype=float)
        if joints.shape not in ((7,), (8,)) or not np.isfinite(joints).all():
            raise ValueError("Expected seven finite arm angles and an optional gripper angle")
        self.data.qpos[self.qpos[:7]] = np.deg2rad(joints[:7])
        # The last joint is the gripper, which this prototype never follows.
        self.data.qpos[self.qpos[7]] = 0
        self.mujoco.mj_kinematics(self.model, self.data)
        self.mujoco.mj_comPos(self.model, self.data)

    def compute(self, joints):
        self.set_joints(joints)
        position = self.data.site_xpos[self.site].copy()
        rotation = self.data.site_xmat[self.site].reshape(3, 3).copy()
        return position, Rotation.from_matrix(rotation).as_quat()

    def arm_state(self, joints):
        self.set_joints(joints)
        return (
            self.data.xpos[self.shoulder].copy(),
            self.data.xpos[self.elbow].copy(),
            self.data.xmat[self.upper].reshape(3, 3).copy(),
        )

    def jacobian(self):
        position, rotation = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        self.mujoco.mj_jacSite(self.model, self.data, position, rotation, self.site)
        return position[:, self.dofs], rotation[:, self.dofs]


class FullModelIK:
    """Position-priority MuJoCo Jacobian IK with preview-only numeric bounds."""

    def __init__(self, model, side, mujoco, *, max_step_deg, tolerance_m):
        self.fk_solver = FullModelFK(model, side, mujoco)
        self.max_step = float(max_step_deg)
        self.tolerance = float(tolerance_m)
        self.last_solve_info = {}

    def arm_state(self, joints):
        return self.fk_solver.arm_state(joints)

    def solve(self, target_position, target_orientation_quat, current_angles_deg):
        target_position = np.asarray(target_position, dtype=float)
        current = np.asarray(current_angles_deg, dtype=float)
        if (
            target_position.shape != (3,)
            or not np.isfinite(target_position).all()
            or current.shape != (8,)
            or not np.isfinite(current).all()
        ):
            raise ValueError("Invalid full-model IK target or state")
        initial = np.deg2rad(current[:7])
        lower, upper = np.full(7, -np.pi), np.full(7, np.pi)
        initial = np.clip(initial, lower + 1e-9, upper - 1e-9)
        desired_rotation = (
            Rotation.from_quat(target_orientation_quat).as_matrix()
            if target_orientation_quat is not None
            else None
        )

        def run(use_orientation):
            cache = {}

            def evaluate(values):
                if "q" not in cache or not np.array_equal(cache["q"], values):
                    self.fk_solver.set_joints(np.rad2deg(values))
                    data = self.fk_solver.data
                    position = data.site_xpos[self.fk_solver.site].copy()
                    rotation = data.site_xmat[self.fk_solver.site].reshape(3, 3).copy()
                    jp, jr = self.fk_solver.jacobian()
                    residual = [position - target_position]
                    jac = [jp]
                    if use_orientation:
                        residual.append(
                            0.08 * Rotation.from_matrix(rotation @ desired_rotation.T).as_rotvec()
                        )
                        jac.append(0.08 * jr)
                    # Favor a nearby solution in the redundant arm.
                    residual.append(0.002 * (values - initial))
                    jac.append(0.002 * np.eye(7))
                    cache.update(q=values.copy(), residual=np.concatenate(residual), jac=np.vstack(jac))
                return cache["residual"], cache["jac"]

            result = least_squares(
                lambda values: evaluate(values)[0],
                initial,
                jac=lambda values: evaluate(values)[1],
                bounds=(lower, upper),
                max_nfev=35,
                ftol=1e-7,
                xtol=1e-7,
                gtol=1e-7,
            )
            position, _ = self.fk_solver.compute(np.rad2deg(result.x))
            return result.x, float(np.linalg.norm(position - target_position))

        solution, residual = run(desired_rotation is not None)
        position_only = False
        if desired_rotation is not None and residual > self.tolerance:
            solution, residual = run(False)
            position_only = True
        self.last_solve_info = {
            "residual_final_m": residual,
            "workspace_clamped": False,
            "used_position_only": position_only,
            "model": "full_mujoco_preview_unmeasured_limits",
        }
        return np.clip(
            np.rad2deg(solution),
            np.maximum(-180, current[:7] - self.max_step),
            np.minimum(180, current[:7] + self.max_step),
        )


class FullMujocoSimulation:
    """Mocopi-compatible viewer for the supplied full native MJCF model."""

    def __init__(self, config, gui=True):
        if gui and os.environ.get("DISPLAY"):
            os.environ.setdefault("PYGLFW_LIBRARY_VARIANT", "x11")
        try:
            import mujoco
        except ImportError as exc:
            raise ImportError("MuJoCo is required for the full model preview") from exc

        self.mujoco, self.viewer, self.viewer_threads = mujoco, None, []
        self.postures, self.targets, self.initial_ee, self.arms = {}, {}, {}, {}
        self.model = build_full_model(full_model_path(config), mujoco)
        self.data = mujoco.MjData(self.model)
        preset = config.get("start_pose", "config")
        if preset not in ("work", "backwards", "config"):
            raise ValueError("robot.start_pose must be work, backwards or config")
        initial = config["initial_joints_deg"] if preset == "config" else FULL_START_POSES[preset]
        self.qpos_indices = {}
        for side in SUFFIX:
            joints = np.asarray(initial[side], dtype=float)
            if joints.shape != (8,) or not np.isfinite(joints).all() or joints[7] != 0:
                raise ValueError("Initial pose needs seven arm angles and a zero gripper angle")
            lower, upper = np.r_[np.full(7, -180.0), 0.0], np.r_[np.full(7, 180.0), 0.0]
            if np.any(joints < lower) or np.any(joints > upper):
                raise ValueError(f"{side} initial pose exceeds preview bounds")
            solver = FullModelIK(
                self.model,
                side,
                mujoco,
                max_step_deg=config["max_joint_step_deg"],
                tolerance_m=config["position_tolerance_m"],
            )
            arm = SafeIK(
                solver, joints, lower, upper, config["max_joint_step_deg"], config["position_tolerance_m"]
            )
            self.arms[side] = arm
            self.qpos_indices[side] = solver.fk_solver.qpos
            position, quaternion = solver.fk_solver.compute(joints)
            self.initial_ee[side] = pose(position, quaternion)
            self.targets[side] = self.initial_ee[side].copy()
        LOG.warning("Full CAD model: joint limits and motor mapping are unverified; simulation only")
        try:
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
                    self.viewer.cam.lookat[:] = [0, 0.08, 0.23]
                    self.viewer.cam.distance = 1.2
                    self.viewer.cam.azimuth = 135
                    self.viewer.cam.elevation = -20
                    self.viewer.opt.geomgroup[3] = 0
                self.viewer.sync()
        except Exception:
            self.close()
            raise

    def _sync_state(self):
        with self.viewer.lock() if self.viewer else nullcontext():
            for side, arm in self.arms.items():
                self.data.qpos[self.qpos_indices[side][:7]] = np.deg2rad(arm.last_valid_target[:7])
                self.data.qpos[self.qpos_indices[side][7]] = 0
                marker = self.model.body(f"{side}/target").mocapid[0]
                self.data.mocap_pos[marker] = self.targets[side][:3, 3]
                self.data.mocap_quat[marker] = Rotation.from_matrix(self.targets[side][:3, :3]).as_quat()[
                    [3, 0, 1, 2]
                ]
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
        for side, arm in self.arms.items():
            if valid:
                arm.solve(targets[side], (postures or {}).get(side))
                if arm.valid:
                    self.targets[side] = targets[side].copy()
                    self.postures[side] = deepcopy((postures or {}).get(side))
            else:
                arm.valid, arm.status, arm.posture_info = False, "tracking lost: HOLD", {"mode": "HOLD"}
        self._sync_state()

    def is_running(self):
        return self.viewer is None or self.viewer.is_running()

    def close(self):
        if self.viewer:
            self.viewer.close()
            for thread in self.viewer_threads:
                thread.join(timeout=10)
            self.viewer = None
