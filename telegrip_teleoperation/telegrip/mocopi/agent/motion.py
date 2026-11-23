"""Deterministic Cartesian-to-joint planning using the existing URDF/PyBullet FK-IK stack."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from ..config import load_config
from ..dataset.real import JOINTS, validate_plan
from ..dataset.replay import model_fingerprint
from ..geometry import pose, validate_pose
from ..ik import DualSimulation
from .scene import SceneCollisionChecker, SceneGeometry

SIDES = ("left", "right")


class MotionPlanningError(RuntimeError):
    pass


class KinematicsService:
    """Plan and validate real-follower trajectories without stepping MuJoCo.

    This is a planning backend only. Execution is performed by the existing
    Feetech follower adapter, not by the kinematic preview.
    """

    def __init__(
        self,
        robot_config_path: str | Path,
        *,
        simulation: Any | None = None,
        scene: SceneGeometry | None = None,
    ):
        self.full_config = load_config(Path(robot_config_path))
        self.config = self.full_config["robot"]
        self.simulation = simulation or DualSimulation(self.config, gui=False)
        self.scene = scene
        self.collision_checker = (
            SceneCollisionChecker(self.simulation, scene)
            if scene is not None and scene.execution_allowed
            else None
        )

    def close(self) -> None:
        if self.collision_checker is not None:
            self.collision_checker.close()
        self.simulation.close()

    def __enter__(self) -> KinematicsService:
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def fk(self, side: str, joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if side not in SIDES:
            raise ValueError("side must be left or right")
        values = np.asarray(joints, dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError("joint angles must contain seven finite values")
        return self.simulation.arms[side].solver.fk_solver.compute(np.r_[values, 0.0])

    def pose_vector(self, side: str, joints: np.ndarray) -> list[float]:
        position, quaternion = self.fk(side, joints)
        return np.r_[position, quaternion].tolist()

    def scene_status(self) -> dict[str, Any]:
        if self.scene is None:
            return {
                "configured": False,
                "status": "missing",
                "execution_allowed": False,
                "reason": "measured table/stick scene is required before arm motion",
            }
        return {
            "configured": True,
            "status": self.scene.status,
            "execution_allowed": self.scene.execution_allowed,
            **self.scene.to_dict(),
        }

    def collision_report(self, joints: dict[str, np.ndarray]) -> dict[str, Any]:
        if self.collision_checker is None:
            return {
                "configured": self.scene is not None,
                "checked": False,
                "execution_allowed": self.scene.execution_allowed if self.scene else False,
                "count": None,
                "reason": "measured scene geometry is required before real motion",
            }
        return self.collision_checker.check_pose(joints)

    def solve_to_target(
        self,
        side: str,
        start_joints: np.ndarray,
        target: np.ndarray,
        *,
        max_steps: int = 100,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        target = validate_pose(target)
        arm = self.simulation.arms[side]
        arm.last_valid_target[:7] = np.asarray(start_joints, dtype=float)
        arm.last_valid_target[7] = 0.0
        for _ in range(max_steps):
            candidate = arm.solve(target)
            info = arm.solver.last_solve_info
            residual = info.get("residual_final_m")
            if arm.valid and np.isfinite(residual) and residual <= arm.tolerance:
                return np.asarray(candidate[:7], dtype=float), dict(info)
        raise MotionPlanningError(
            f"{side} IK did not converge within {max_steps} bounded steps: {arm.status}"
        )

    def build_motion_plan(
        self,
        start_joints: dict[str, np.ndarray],
        targets: dict[str, np.ndarray | None],
        *,
        fps: int = 30,
        source: str = "real",
        task: str = "agent motion",
    ) -> dict[str, Any]:
        if source not in {"real", "fake"}:
            raise ValueError("source must be real or fake")
        if not 0 < fps <= 120:
            raise ValueError("fps must be in (0, 120]")
        if set(start_joints) != set(SIDES):
            raise ValueError("start_joints must contain left and right")
        if self.scene is not None and not self.scene.execution_allowed:
            raise MotionPlanningError("measured scene geometry is required before real motion")
        if self.collision_checker is not None:
            initial_collision = self.collision_checker.check_pose(start_joints)
            if initial_collision["count"]:
                raise MotionPlanningError(
                    f"initial pose violates measured scene clearance: {initial_collision}"
                )

        start_pose: dict[str, np.ndarray] = {}
        final_joints: dict[str, np.ndarray] = {}
        final_pose: dict[str, np.ndarray] = {}
        ik_info: dict[str, Any] = {}
        for side in SIDES:
            start = np.asarray(start_joints[side], dtype=float)
            position, quaternion = self.fk(side, start)
            start_pose[side] = pose(position, quaternion)
            if targets.get(side) is None:
                final_joints[side] = start.copy()
                final_pose[side] = start_pose[side].copy()
            else:
                final_joints[side], ik_info[side] = self.solve_to_target(side, start, targets[side])
                final_position, final_quaternion = self.fk(side, final_joints[side])
                final_pose[side] = pose(final_position, final_quaternion)

        max_joint_delta = max(
            float(np.max(np.abs(final_joints[side] - np.asarray(start_joints[side], dtype=float))))
            for side in SIDES
        )
        configured_step = float(self.config["max_joint_step_deg"])
        intervals = max(1, math.ceil(max_joint_delta / configured_step)) if max_joint_delta > 1e-9 else 1
        frames = []
        for frame_index in range(intervals + 1):
            alpha = frame_index / intervals
            frame_joints: dict[str, np.ndarray] = {}
            frame_targets: dict[str, list[float]] = {}
            for side in SIDES:
                start = np.asarray(start_joints[side], dtype=float)
                frame_joints[side] = start + alpha * (final_joints[side] - start)
                frame_target = _interpolate_pose(start_pose[side], final_pose[side], alpha)
                frame_targets[side] = np.r_[
                    frame_target[:3, 3], Rotation.from_matrix(frame_target[:3, :3]).as_quat()
                ].tolist()
            frames.append(
                {
                    "time_s": frame_index / fps,
                    "arms": {
                        side: {
                            "joint_deg": frame_joints[side].tolist(),
                            "target_pose": frame_targets[side],
                            "simulated_pose": self.pose_vector(side, frame_joints[side]),
                            "valid": True,
                            "workspace_clamped": False,
                            "retarget_clamped": False,
                        }
                        for side in SIDES
                    },
                }
            )

        plan = {
            "schema": "dual_scorpion.mocopi.robot_plan.v1",
            "source": source,
            "task": task,
            "model_kind": "legacy_urdf",
            "urdf_sha256": model_fingerprint(self.full_config),
            "joint_names": list(JOINTS),
            "gripper_included": False,
            "fps": fps,
            "expected_frames": len(frames),
            "frames": frames,
            "failures": [],
            "max_joint_step_deg": min(0.5, 15.0 / fps, configured_step),
            "hardware_max_joint_step_deg": configured_step,
            "ik_info": ik_info,
            "scene_schema": None if self.scene is None else "dual_scorpion.agent.scene.v1",
            "scene_sha256": None if self.scene is None else self.scene.sha256,
            "scene_status": None if self.scene is None else self.scene.status,
            "collision_checked": self.collision_checker is not None,
            "collision_margin_m": None if self.scene is None else self.scene.collision_margin_m,
            "startup_behavior": {
                "auto_home": False,
                "initial_state_source": "measured_encoders",
                "hold_strategy": "present_position_before_torque_enable",
            },
        }
        if self.collision_checker is not None:
            self.validate_scene_trajectory(plan)
        return plan

    def validate_scene_trajectory(self, plan: dict[str, Any]) -> dict[str, Any]:
        if self.collision_checker is None:
            raise MotionPlanningError("measured scene geometry is required before real motion")
        reports = []
        for frame in plan.get("frames", []):
            joints = {
                side: np.asarray(frame["arms"][side]["joint_deg"], dtype=float)
                for side in SIDES
            }
            report = self.collision_checker.check_pose(joints)
            report["frame"] = len(reports)
            reports.append(report)
            if report["count"]:
                plan["scene_collision"] = report
                raise MotionPlanningError(
                    f"trajectory frame {len(reports) - 1} violates measured scene clearance: {report}"
                )
        plan["scene_collision"] = {
            "configured": True,
            "checked": True,
            "frames": len(reports),
            "count": 0,
            "clearance_violation": False,
            "penetrating": False,
            "min_distance_m": min(
                (item["min_distance_m"] for item in reports if item["min_distance_m"] is not None),
                default=None,
            ),
        }
        return plan["scene_collision"]

    def validate(self, plan: dict[str, Any], *, allow_synthetic: bool = False) -> dict[str, Any]:
        return validate_plan(
            plan,
            self.full_config,
            self.simulation,
            allow_synthetic=allow_synthetic,
            max_sim_error_m=float(self.config["position_tolerance_m"]),
        )


def _interpolate_pose(start: np.ndarray, target: np.ndarray, alpha: float) -> np.ndarray:
    result = np.eye(4)
    result[:3, 3] = (1.0 - alpha) * start[:3, 3] + alpha * target[:3, 3]
    start_rotation = Rotation.from_matrix(start[:3, :3])
    target_rotation = Rotation.from_matrix(target[:3, :3])
    slerp = Slerp([0.0, 1.0], Rotation.concatenate([start_rotation, target_rotation]))
    result[:3, :3] = slerp([alpha]).as_matrix()[0]
    return result
