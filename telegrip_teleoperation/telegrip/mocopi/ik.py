"""Adapter to existing Telegrip PyBullet FK/IK and visualization, degrees at boundary."""

import logging

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import pose, validate_pose

LOG = logging.getLogger(__name__)
START_POSES = {
    "neutral": {side: [0, 0, 0, 0, 0, 0, 0, 0] for side in ("left", "right")},
    "work": {side: [20, 0, 0, 0, 0, 0, 0, 0] for side in ("left", "right")},
    "backwards": {
        "left": [-20, 0, -80, 0, 0, 0, 0, 0],
        "right": [-20, 0, 80, 0, 0, 0, 0, 0],
    },
}


class SafeIK:
    def __init__(self, solver, initial, lower, upper, max_step_deg=0.5, tolerance_m=0.025):
        self.solver = solver
        self.last_valid_target = np.asarray(initial, dtype=float).copy()
        self.lower = np.asarray(lower)
        self.upper = np.asarray(upper)
        self.max_step = max_step_deg
        self.tolerance = tolerance_m
        self.valid = False
        self.status = "uninitialized"
        self.posture_info = {"mode": "hand-only"}
        self.posture_solver = None

    def check_candidate(self, candidate):
        if candidate.shape != (7,) or not np.isfinite(candidate).all():
            raise ValueError("IK returned invalid joint array")
        if np.any(candidate < self.lower[:7] - 1e-6) or np.any(candidate > self.upper[:7] + 1e-6):
            raise ValueError("IK violated joint limits")
        if np.any(np.abs(candidate - self.last_valid_target[:7]) > self.max_step + 1e-6):
            raise ValueError("IK joint jump")

    def solve(self, target, posture=None):
        try:
            target = validate_pose(target)
            candidate = np.asarray(
                self.solver.solve(
                    target[:3, 3],
                    None if posture is not None else Rotation.from_matrix(target[:3, :3]).as_quat(),
                    self.last_valid_target.copy(),
                ),
                dtype=float,
            )
            info = self.solver.last_solve_info
            residual = info.get("residual_final_m", np.inf)
            if "error" in info or not np.isfinite(residual) or residual > self.tolerance:
                raise ValueError(f"IK not reachable: {info}")
            self.check_candidate(candidate)
            self.posture_info = {"mode": "hand-only"}
            if posture is not None:
                from .posture import ArmPostureIK

                if self.posture_solver is None:
                    self.posture_solver = ArmPostureIK(self.solver, self.lower, self.upper, self.max_step)
                candidate, self.posture_info = self.posture_solver.refine(
                    candidate, self.last_valid_target, posture, target
                )
            self.check_candidate(candidate)
            self.last_valid_target[:7] = candidate
            self.valid = True
            self.status = (
                "position+upper-arm"
                if posture is not None
                else "position-only"
                if info.get("used_position_only")
                else "OK"
            )
        except Exception as exc:
            self.valid = False
            self.status = str(exc)
            self.posture_info = {"mode": "HOLD"}
        return self.last_valid_target.copy()


class ResearchSafeIK(SafeIK):
    """Strict exploration modes that leave SafeIK's default path unchanged."""

    def __init__(
        self,
        solver,
        initial,
        lower,
        upper,
        max_step_deg=0.5,
        tolerance_m=0.025,
        *,
        ik_mode,
        orientation_tolerance_deg,
    ):
        super().__init__(solver, initial, lower, upper, max_step_deg, tolerance_m)
        if ik_mode not in ("position-only", "hand-6dof", "hand-6dof-upper-arm"):
            raise ValueError(f"Unsupported research IK mode: {ik_mode}")
        if not np.isfinite(orientation_tolerance_deg) or orientation_tolerance_deg <= 0:
            raise ValueError("orientation_tolerance_deg must be positive")
        self.ik_mode = ik_mode
        self.orientation_tolerance = float(orientation_tolerance_deg)
        # The initial alignment uses the same 6-DoF solve for hand-6dof and
        # hand-6dof-upper-arm.  Per-frame calls require the posture target.
        self.posture_optional = False
        self.last_solve = {}

    def solve(self, target, posture=None):
        self.last_solve = {
            "mode": self.ik_mode,
            "success": False,
            "fallback": False,
            "failure_kind": None,
            "position_error_m": None,
            "orientation_error_deg": None,
        }
        try:
            target = validate_pose(target)
            use_orientation = self.ik_mode != "position-only"
            if self.ik_mode == "hand-6dof-upper-arm" and posture is None and not self.posture_optional:
                raise ValueError("hand-6dof-upper-arm requires a posture target")
            if self.ik_mode != "hand-6dof-upper-arm" and posture is not None:
                raise ValueError(f"{self.ik_mode} must not receive an upper-arm posture target")
            candidate = np.asarray(
                self.solver.solve(
                    target[:3, 3],
                    Rotation.from_matrix(target[:3, :3]).as_quat() if use_orientation else None,
                    self.last_valid_target.copy(),
                ),
                dtype=float,
            )
            info = self.solver.last_solve_info
            position_error = float(info.get("residual_final_m", np.inf))
            orientation_error = info.get("orientation_error_deg")
            self.last_solve.update(
                fallback=bool(info.get("used_position_only")),
                position_error_m=position_error,
                orientation_error_deg=orientation_error,
            )
            if "error" in info or not np.isfinite(position_error) or position_error > self.tolerance:
                self.last_solve["failure_kind"] = "unreachable"
                raise ValueError(f"IK not reachable: {info}")
            if use_orientation and (
                orientation_error is None
                or not np.isfinite(orientation_error)
                or orientation_error > self.orientation_tolerance
            ):
                self.last_solve["failure_kind"] = "orientation_error"
                raise ValueError(
                    f"IK orientation error {orientation_error} exceeds {self.orientation_tolerance} deg"
                )
            if info.get("used_position_only"):
                self.last_solve["failure_kind"] = "fallback"
                raise ValueError("Research IK mode forbids position-only fallback")
            self.check_candidate(candidate)
            pre_position, pre_quaternion = self.solver.fk_solver.compute(candidate)
            pre_position_error = float(np.linalg.norm(pre_position - target[:3, 3]))
            pre_orientation_error = float(
                np.rad2deg(
                    (
                        Rotation.from_quat(pre_quaternion).inv() * Rotation.from_matrix(target[:3, :3])
                    ).magnitude()
                )
            )
            self.posture_info = {"mode": "hand-only"}
            if posture is not None:
                from .posture import ArmPostureIK

                if self.posture_solver is None:
                    self.posture_solver = ArmPostureIK(self.solver, self.lower, self.upper, self.max_step)
                candidate, self.posture_info = self.posture_solver.refine(
                    candidate, self.last_valid_target, posture, target
                )
            self.check_candidate(candidate)
            final_position, final_quaternion = self.solver.fk_solver.compute(candidate)
            final_position_error = float(np.linalg.norm(final_position - target[:3, 3]))
            final_orientation_error = float(
                np.rad2deg(
                    (
                        Rotation.from_quat(final_quaternion).inv() * Rotation.from_matrix(target[:3, :3])
                    ).magnitude()
                )
            )
            posture_position_delta = float(np.linalg.norm(final_position - pre_position))
            posture_orientation_delta = float(
                np.rad2deg(
                    (
                        Rotation.from_quat(final_quaternion).inv() * Rotation.from_quat(pre_quaternion)
                    ).magnitude()
                )
            )
            self.last_solve.update(
                position_error_m=final_position_error,
                orientation_error_deg=final_orientation_error,
                pre_refinement_position_error_m=pre_position_error,
                pre_refinement_orientation_error_deg=pre_orientation_error,
                posture_tcp_position_delta_m=posture_position_delta,
                posture_tcp_orientation_delta_deg=posture_orientation_delta,
            )
            if final_position_error > self.tolerance:
                self.last_solve["failure_kind"] = "posture_tcp_error"
                raise ValueError("Posture refinement exceeded the TCP position tolerance")
            if use_orientation and final_orientation_error > self.orientation_tolerance:
                self.last_solve["failure_kind"] = "posture_orientation_error"
                raise ValueError("Posture refinement exceeded the TCP orientation tolerance")
            self.last_valid_target[:7] = candidate
            self.valid = True
            self.status = "OK"
            self.last_solve["success"] = True
        except Exception as exc:
            self.valid = False
            self.status = str(exc)
            self.posture_info = {"mode": "HOLD"}
            if self.last_solve["failure_kind"] is None:
                message = str(exc)
                self.last_solve["failure_kind"] = (
                    "joint_limit"
                    if "joint limits" in message
                    else "joint_jump"
                    if "joint jump" in message
                    else "solver_error"
                )
        return self.last_valid_target.copy()


class DualSimulation:
    def __init__(self, config, gui=True):
        from ..core.kinematics import IKSolver
        from ..core.visualizer import PyBulletVisualizer

        preset = config.get("start_pose", "config")
        if preset not in ("work", "backwards", "neutral", "config"):
            raise ValueError("robot.start_pose must be work, backwards, neutral or config")
        initial_joints = config["initial_joints_deg"] if preset == "config" else START_POSES[preset]
        self.viz = PyBulletVisualizer(config["urdf"], use_gui=gui, log_level="info")
        if not self.viz.setup():
            self.viz.disconnect()
            raise RuntimeError("Existing Dual Scorpion simulation initialization failed")
        self.arms = {}
        self.initial_ee = {}
        self.targets = {}
        try:
            for side in ("left", "right"):
                solver = IKSolver(
                    self.viz.physics_client,
                    self.viz.robot_ids[side],
                    self.viz.joint_indices[side],
                    self.viz.end_effector_link_indices[side],
                    self.viz.joint_limits_min_deg[side],
                    self.viz.joint_limits_max_deg[side],
                    arm_name=side,
                    max_joint_step_deg=config["max_joint_step_deg"],
                    position_tolerance_m=config["position_tolerance_m"],
                )
                initial = np.array(initial_joints[side], dtype=float)
                if initial.shape != (8,) or not np.isfinite(initial).all():
                    raise ValueError("Initial joint configuration must contain 8 finite degree values")
                if np.any(initial < solver.joint_limits_min_deg) or np.any(
                    initial > solver.joint_limits_max_deg
                ):
                    raise ValueError(f"{side} initial pose violates URDF limits")
                self.arms[side] = SafeIK(
                    solver,
                    initial,
                    solver.joint_limits_min_deg,
                    solver.joint_limits_max_deg,
                    config["max_joint_step_deg"],
                    config["position_tolerance_m"],
                )
                position, quaternion = solver.fk_solver.compute(initial)
                self.initial_ee[side] = pose(position, quaternion)
                self.viz.update_robot_pose(initial, side)
        except Exception:
            self.close()
            raise

    def update(self, targets, valid=True, postures=None):
        for side, arm in self.arms.items():
            if valid:
                joints = arm.solve(targets[side], (postures or {}).get(side))
                if arm.valid:
                    self.targets[side] = targets[side].copy()
            else:
                joints = arm.last_valid_target
                arm.valid = False
                arm.status = "tracking lost: HOLD"
                arm.posture_info = {"mode": "HOLD"}
            # FK verification mutates the Bullet robot temporarily; always
            # restore the last command, including after failed IK.
            self.viz.update_robot_pose(joints, side)
            if side in self.targets:
                self.viz.update_marker_position(side + "_target", self.targets[side][:3, 3])
                actual, _ = arm.solver.fk_solver.compute(joints)
                self.viz.update_marker_position(side + "_goal", actual)
        if self.viz.gui_active:
            import pybullet as p

            text = " | ".join(f"{side}: {arm.status[:90]}" for side, arm in self.arms.items())
            self.label = p.addUserDebugText(
                text,
                [0.0, 0.0, 1.1],
                textColorRGB=[0, 0.7, 0] if all(a.valid for a in self.arms.values()) else [1, 0, 0],
                replaceItemUniqueId=getattr(self, "label", -1),
            )
        # Kinematic preview: do not step gravity/contact dynamics, which would
        # move supposedly frozen resetJointState arms.

    def close(self):
        self.viz.disconnect()

    def is_running(self):
        import pybullet as p

        return bool(p.isConnected(self.viz.physics_client))
