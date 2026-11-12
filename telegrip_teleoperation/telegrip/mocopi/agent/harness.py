"""Real-robot orchestration for plans, cameras, WorldState, and execution."""

from __future__ import annotations

import contextlib
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from ..geometry import pose
from .cameras import CameraFrame, CameraService
from .config import HarnessConfig
from .motion import KinematicsService
from .robot import RealRobotAdapter, _gripper_position_state
from .safety import AuthorizationManager, SafetyManager
from .scene import SceneGeometry
from .schema import PlanValidationError, TaskPlan, validate_task_plan
from .skills import SkillRegistry, built_in_skill_registry
from .world import ArmState, CameraState, ObjectState, StateSource, WorldState

SIDES = ("left", "right")


@dataclass
class ExecutionJob:
    execution_id: str
    plan_id: str
    motion_plan_id: str
    report_path: Path
    status: str
    error: str | None = None
    report: dict[str, Any] | None = None
    before_images: list[dict[str, Any]] = field(default_factory=list)
    after_images: list[dict[str, Any]] = field(default_factory=list)
    thread: threading.Thread | None = None


class ScorpionAgentHarness:
    def __init__(
        self,
        config: HarnessConfig,
        *,
        robot: RealRobotAdapter | None = None,
        cameras: CameraService | None = None,
        kinematics: KinematicsService | None = None,
        authorization: AuthorizationManager | None = None,
        registry: SkillRegistry | None = None,
    ):
        self.config = config
        self.registry = registry or built_in_skill_registry()
        self.robot = robot or (
            RealRobotAdapter(
                left_port=config.robot.left_port,
                right_port=config.robot.right_port,
                robot_id=config.robot.robot_id,
                calibration_dir=config.robot.calibration_dir,
                max_step_deg=config.robot.max_step_deg,
                gripper=config.gripper,
            )
            if config.robot is not None
            else None
        )
        self.cameras = cameras or CameraService(config.cameras)
        scene = SceneGeometry.from_file(config.scene_geometry_path) if config.scene_geometry_path else None
        self.kinematics = kinematics or KinematicsService(config.robot_config_path, scene=scene)
        token_file = config.authorization_token_file
        self.authorization = authorization or AuthorizationManager(token_file)
        self.safety = SafetyManager(self.authorization)
        self.objects: dict[str, ObjectState] = {}
        self.plans: dict[str, TaskPlan] = {}
        self.motion_plans: dict[str, dict[str, Any]] = {}
        self.motion_plan_plan_ids: dict[str, str] = {}
        self.jobs: dict[str, ExecutionJob] = {}

    def close(self) -> None:
        self.cameras.close()
        self.kinematics.close()
        if self.robot is not None:
            self.robot.disconnect(disable_torque=False)

    def __enter__(self) -> ScorpionAgentHarness:
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def status(self) -> dict[str, Any]:
        if self.robot is None:
            robot_status = {"connected": False, "configured": False}
        elif isinstance(self.robot, RealRobotAdapter):
            robot_status = self.robot.configuration_status()
        else:
            robot_status = self.robot.status()
        return {
            "schema": "dual_scorpion.agent.status.v1",
            "robot": robot_status,
            "cameras": self.cameras.list_cameras(),
            "skills": self.registry.specs(),
            "motion_authorization_configured": self.authorization.token_file is not None,
            "motion_authorization_ready": self.authorization.required,
            "motion_authorization_required": self.authorization.token_file is not None,
            "stop_requested": self.safety.stop_requested,
            "active_executions": len(self.jobs),
            "scene": self.kinematics.scene_status(),
            "startup_behavior": {
                "auto_home": False,
                "initial_state_source": "encoder_reading_only",
                "start_pose_config": self.kinematics.full_config["robot"].get("start_pose"),
                "retarget_start_pose_config": self.kinematics.full_config["robot"].get(
                    "retarget_start_pose"
                ),
                "note": "start_pose presets initialize kinematics only; harness planning starts from measured encoders",
            },
            "hardware_commands_sent_by_constructor": 0,
        }

    def connect_robot(self, authorization_token: str | None) -> dict[str, Any]:
        self.safety.authorize_motion(authorization_token)
        if self.robot is None:
            raise RuntimeError("real robot adapter is not configured")
        calibration = self.robot.validate_calibration()
        if calibration["status"] != "valid":
            raise RuntimeError(f"real robot calibration is invalid: {calibration['errors']}")
        self.robot.connect()
        reading = self.robot.read()
        return {
            "status": "connected",
            "robot": self.robot.status(),
            "calibration": calibration,
            "startup_behavior": {
                "auto_home": False,
                "initial_state_source": "encoder_reading_only",
                "hold_strategy": "present_position_before_torque_enable",
            },
            "timestamp_monotonic_ns": reading.timestamp_monotonic_ns,
            "joints_deg": {side: list(values) for side, values in reading.joints_deg.items()},
        }

    def world_state(self) -> WorldState:
        now = time.monotonic_ns()
        arms: dict[str, ArmState] = {}
        if self.robot is not None and self.robot.is_connected:
            reading = self.robot.read()
            gripper_positions = getattr(reading, "gripper_positions", None) or {}
            for side in SIDES:
                joints = np.asarray(reading.joints_deg[side], dtype=float)
                position, quaternion = self.kinematics.fk(side, joints)
                arms[side] = ArmState(
                    side=side,
                    source=StateSource.MEASURED,
                    joint_deg=tuple(float(value) for value in joints),
                    tcp_position_m=tuple(float(value) for value in position),
                    tcp_orientation_xyzw=tuple(float(value) for value in quaternion),
                    connected=reading.connected,
                    calibrated=reading.calibrated,
                    timestamp_monotonic_ns=reading.timestamp_monotonic_ns,
                    gripper_state=(
                        _gripper_position_state(
                            reading.gripper_positions[side],
                            self.config.gripper,
                        )
                        if side in gripper_positions
                        else "unknown"
                    ),
                    gripper_position=(
                        float(reading.gripper_positions[side])
                        if side in gripper_positions
                        else None
                    ),
                )
        else:
            for side in SIDES:
                arms[side] = ArmState(
                    side=side,
                    source=StateSource.UNKNOWN,
                    joint_deg=(0.0,) * 7,
                    tcp_position_m=None,
                    tcp_orientation_xyzw=None,
                    connected=False,
                    calibrated=False,
                    timestamp_monotonic_ns=0,
                )

        cameras: dict[str, CameraState] = {}
        for camera_status in self.cameras.list_cameras():
            cameras[camera_status["camera_id"]] = CameraState(
                camera_id=camera_status["camera_id"],
                connected=bool(camera_status["connected"]),
                last_frame_monotonic_ns=camera_status["last_frame_monotonic_ns"],
                width=int(camera_status["width"]),
                height=int(camera_status["height"]),
                coordinate_frame=camera_status["coordinate_frame"],
                last_error=camera_status["last_error"],
            )
        return WorldState(
            updated_monotonic_ns=now,
            arms=arms,
            cameras=cameras,
            objects=dict(self.objects),
            task_progress={"submitted_plans": len(self.plans), "active_executions": len(self.jobs)},
            metadata={
                "hardware_access": self.robot is not None,
                "vision_object_detection": False,
                "metric_pose_estimation": False,
                "gripper_control": "integrated_software_only_physical_execution_not_verified",
                "scene_collision_required": True,
                "scene_collision_ready": self.kinematics.scene_status()["execution_allowed"],
                "llm_semantic_estimates_allowed": True,
            },
        )

    def update_object_estimates(self, estimates: list[dict[str, Any]]) -> dict[str, Any]:
        parsed = [ObjectState.from_dict(item) for item in estimates]
        for item in parsed:
            self.objects[item.id] = item
        return {"updated_object_ids": [item.id for item in parsed], "objects": {key: value.to_dict() for key, value in self.objects.items()}}

    def capture_images(
        self,
        camera_ids: list[str] | None = None,
        *,
        max_age_s: float | None = None,
    ) -> list[CameraFrame]:
        return self.cameras.capture(camera_ids, max_age_s=max_age_s)

    def read_gripper_state(self, arm: str) -> dict[str, Any]:
        if self.robot is None:
            raise RuntimeError("real robot adapter is not configured")
        return self.robot.read_gripper_state(arm)

    def control_gripper(
        self,
        *,
        action: str,
        arms: str,
        position: float | None = None,
        authorization_token: str | None = None,
    ) -> dict[str, Any]:
        if self.robot is None:
            raise RuntimeError("real robot adapter is not configured")
        if action not in {"open", "close", "set_position", "hold", "release", "grasp"}:
            raise ValueError("unsupported gripper action")
        sides = _arms_to_sides(arms)
        if action == "set_position" and position is None:
            raise ValueError("set_position requires a normalized position")
        if self.safety.stop_requested:
            return {
                "status": "stopped",
                "action": action,
                "results": [
                    {"side": side, "operation": action, "status": "stopped"}
                    for side in sides
                ],
            }
        self.safety.authorize_motion(authorization_token)
        scene_status = self.kinematics.scene_status()
        if not scene_status["execution_allowed"]:
            raise RuntimeError("measured table/stick scene geometry is required before gripper motion")
        reading = self.robot.read()
        current_joints = {side: np.asarray(reading.joints_deg[side], dtype=float) for side in SIDES}
        current_collision = self.kinematics.collision_report(current_joints)
        if not current_collision["checked"] or current_collision.get("count"):
            raise RuntimeError(f"current pose is not clear of the measured scene: {current_collision}")
        results = []
        for side in sides:
            if self.safety.stop_requested:
                results.append({"side": side, "operation": action, "status": "stopped"})
                continue
            if action == "open":
                result = self.robot.open_gripper(side, stop_event=self.safety.stop_event)
            elif action == "close":
                result = self.robot.close_gripper(side, stop_event=self.safety.stop_event)
            elif action == "set_position":
                result = self.robot.set_gripper_position(
                    side,
                    float(position),
                    stop_event=self.safety.stop_event,
                )
            elif action == "hold":
                result = self.robot.hold_gripper(side, stop_event=self.safety.stop_event)
            elif action == "release":
                result = self.robot.release_gripper(side, stop_event=self.safety.stop_event)
            else:
                result = self.robot.grasp(side, stop_event=self.safety.stop_event)
            results.append(result)
        return {
            "status": "complete" if all(item.get("status") == "complete" for item in results) else "stopped",
            "action": action,
            "results": results,
        }

    def submit_task_plan(self, plan_dict: dict[str, Any]) -> dict[str, Any]:
        try:
            plan = TaskPlan.from_dict(plan_dict)
        except PlanValidationError as exc:
            return {
                "plan_id": None,
                "status": "rejected",
                "errors": [str(exc)],
                "plan": plan_dict,
            }
        world = self.world_state()
        errors = validate_task_plan(
            plan,
            available_skills=self.registry.available_names(),
            world=world,
        )
        plan_id = "plan-" + uuid.uuid4().hex[:12]
        if not errors:
            self.plans[plan_id] = plan
        return {
            "plan_id": plan_id,
            "status": "valid" if not errors else "rejected",
            "errors": errors,
            "plan": plan.to_dict(),
        }

    def plan_motion(self, plan_id: str, *, fps: int = 30) -> dict[str, Any]:
        if plan_id not in self.plans:
            raise KeyError(f"unknown plan {plan_id!r}")
        plan = self.plans[plan_id]
        world = self.world_state()
        if self.robot is None or not self.robot.is_connected:
            raise RuntimeError("robot must be connected before motion planning")
        reading = self.robot.read()
        current_joints = {side: np.asarray(reading.joints_deg[side], dtype=float) for side in SIDES}
        scene_status = self.kinematics.scene_status()
        if not scene_status["execution_allowed"]:
            raise RuntimeError("measured table/stick scene geometry is required before motion planning")
        initial_collision = self.kinematics.collision_report(current_joints)
        if not initial_collision["checked"] or initial_collision.get("count"):
            raise RuntimeError(f"initial pose is not clear of the measured scene: {initial_collision}")
        segments: list[dict[str, Any]] = []
        ik_info: dict[str, Any] = {}
        for step in plan.steps:
            if step.skill != "move_arm_to_pose":
                continue
            targets = self._targets_for_step(step, current_joints, world)
            segment = self.kinematics.build_motion_plan(
                current_joints,
                targets,
                fps=fps,
                source="real",
                task=f"{plan.goal} / {step.id}",
            )
            validation = self.kinematics.validate(segment, allow_synthetic=False)
            if validation.get("status") == "rejected" or validation.get("errors"):
                raise RuntimeError(f"motion segment {step.id!r} failed validation: {validation}")
            segments.append(segment)
            ik_info[step.id] = segment.get("ik_info", {})
            for side in SIDES:
                current_joints[side] = np.asarray(segment["frames"][-1]["arms"][side]["joint_deg"], dtype=float)

        if not segments:
            raise RuntimeError("plan contains no executable move_arm_to_pose steps")
        combined_frames: list[dict[str, Any]] = []
        for segment in segments:
            segment_frames = segment["frames"]
            start_index = 0 if not combined_frames else 1
            combined_frames.extend(segment_frames[start_index:])
        for index, frame in enumerate(combined_frames):
            frame["time_s"] = index / fps
        motion_plan = dict(segments[0])
        motion_plan["frames"] = combined_frames
        motion_plan["expected_frames"] = len(combined_frames)
        motion_plan["task"] = plan.goal
        motion_plan["ik_info"] = ik_info
        validation = self.kinematics.validate(motion_plan, allow_synthetic=False)
        motion_plan_id = "motion-" + uuid.uuid4().hex[:12]
        self.motion_plans[motion_plan_id] = motion_plan
        self.motion_plan_plan_ids[motion_plan_id] = plan_id
        return {
            "motion_plan_id": motion_plan_id,
            "status": "valid",
            "validation": validation,
            "frame_count": len(combined_frames),
        }

    def execute_motion(
        self,
        motion_plan_id: str,
        authorization_token: str | None,
        *,
        capture_before_after: bool = False,
    ) -> dict[str, Any]:
        if motion_plan_id not in self.motion_plans:
            raise KeyError(f"unknown motion plan {motion_plan_id!r}")
        self.safety.authorize_motion(authorization_token)
        if self.robot is None:
            raise RuntimeError("real robot adapter is not configured")
        if not self.robot.is_connected:
            self.robot.connect()
        reading = self.robot.read()
        current_joints = {side: np.asarray(reading.joints_deg[side], dtype=float) for side in SIDES}
        scene_status = self.kinematics.scene_status()
        if (
            not scene_status["execution_allowed"]
            or motion_plan_id not in self.motion_plans
            or self.motion_plans[motion_plan_id].get("scene_sha256")
            != scene_status.get("sha256")
            or self.motion_plans[motion_plan_id].get("collision_checked") is not True
        ):
            raise RuntimeError("motion plan and measured scene do not match; regenerate the plan")
        current_collision = self.kinematics.collision_report(current_joints)
        if not current_collision["checked"] or current_collision.get("count"):
            raise RuntimeError(f"current pose is not clear of the measured scene: {current_collision}")
        motion_plan = self.motion_plans[motion_plan_id]
        plan_id = self.motion_plan_plan_ids.get(motion_plan_id, "unknown")
        execution_id = "exec-" + uuid.uuid4().hex[:12]
        report_path = self.config.execution_report_dir / f"{execution_id}.json"
        report_path.parent.mkdir(parents=True, exist_ok=True)
        job = ExecutionJob(
            execution_id=execution_id,
            plan_id=plan_id,
            motion_plan_id=motion_plan_id,
            report_path=report_path,
            status="running",
        )
        self.jobs[execution_id] = job

        def save_images(prefix: str, frames: list[Any]) -> list[dict[str, Any]]:
            rows = []
            for frame in frames:
                image_path = report_path.parent / f"{execution_id}-{prefix}-{frame.camera_id}-{frame.timestamp_monotonic_ns}.jpg"
                image_path.write_bytes(frame.image_bytes)
                rows.append({**frame.metadata(), "path": str(image_path)})
            return rows

        if capture_before_after:
            job.before_images = save_images("before", self.capture_images())

        def run() -> None:
            try:
                report = self.robot.execute_plan(
                    motion_plan,
                    self.kinematics.full_config,
                    self.kinematics.simulation,
                    report_path,
                    stop_event=self.safety.stop_event,
                )
                job.report = report
                job.status = "complete"
                if capture_before_after:
                    job.after_images = save_images("after", self.capture_images())
            except BaseException as exc:
                job.error = str(exc)
                job.status = "failed"
                if capture_before_after:
                    with contextlib.suppress(Exception):
                        job.after_images = save_images("after", self.capture_images())

        thread = threading.Thread(target=run, name=f"scorpion-exec-{execution_id}", daemon=True)
        job.thread = thread
        thread.start()
        return {
            "execution_id": execution_id,
            "status": "running",
            "report_path": str(report_path),
            "before_images": job.before_images,
        }

    def execute_task_plan(
        self,
        plan_id: str,
        authorization_token: str | None,
        *,
        capture_before_after: bool = False,
    ) -> dict[str, Any]:
        if plan_id not in self.plans:
            raise KeyError(f"unknown plan {plan_id!r}")
        self.safety.authorize_motion(authorization_token)
        plan = self.plans[plan_id]
        execution_id = "task-" + uuid.uuid4().hex[:12]
        report_path = self.config.execution_report_dir / f"{execution_id}.json"
        report_path.parent.mkdir(parents=True, exist_ok=True)
        job = ExecutionJob(
            execution_id=execution_id,
            plan_id=plan_id,
            motion_plan_id="task-plan",
            report_path=report_path,
            status="running",
        )
        self.jobs[execution_id] = job

        def save_images(prefix: str) -> list[dict[str, Any]]:
            frames = self.capture_images()
            rows = []
            for frame in frames:
                image_path = report_path.parent / f"{execution_id}-{prefix}-{frame.camera_id}-{frame.timestamp_monotonic_ns}.jpg"
                image_path.write_bytes(frame.image_bytes)
                rows.append({**frame.metadata(), "path": str(image_path)})
            return rows

        if capture_before_after:
            job.before_images = save_images("before")

        def run() -> None:
            step_results: list[dict[str, Any]] = []
            try:
                ordered_steps = _dependency_order(plan.steps)
                completed: set[str] = set()
                for step in ordered_steps:
                    if self.safety.stop_requested:
                        raise RuntimeError("operator stop requested")
                    missing = [dependency for dependency in step.depends_on if dependency not in completed]
                    if missing:
                        raise RuntimeError(f"step {step.id!r} has incomplete dependencies: {missing}")
                    if step.skill == "move_arm_to_pose":
                        result = self._execute_motion_step(step, plan, authorization_token)
                    elif step.skill in {"set_gripper", "hold_gripper", "release_gripper", "grasp"}:
                        result = self._execute_gripper_step(step, authorization_token)
                    elif step.skill == "read_gripper_state":
                        result = {
                            "status": "complete",
                            "results": [
                                self.robot.read_gripper_state(side)
                                for side in _arms_to_sides(step.arms)
                            ],
                        }
                    else:
                        raise RuntimeError(f"unsupported task-plan skill {step.skill!r}")
                    step_results.append(
                        {
                            "step_id": step.id,
                            "skill": step.skill,
                            "status": result.get("status", "complete"),
                            "result": result,
                        }
                    )
                    if result.get("status") not in {"complete", "valid"}:
                        raise RuntimeError(f"step {step.id!r} did not complete")
                    completed.add(step.id)
                job.report = {
                    "schema": "dual_scorpion.agent.task_execution.v1",
                    "status": "complete",
                    "plan_id": plan_id,
                    "steps": step_results,
                }
                job.status = "complete"
                if capture_before_after:
                    job.after_images = save_images("after")
            except BaseException as exc:
                job.error = str(exc)
                job.status = "failed"
                job.report = {
                    "schema": "dual_scorpion.agent.task_execution.v1",
                    "status": "failed",
                    "plan_id": plan_id,
                    "steps": step_results,
                    "error": str(exc),
                }

        thread = threading.Thread(target=run, name=f"scorpion-task-{execution_id}", daemon=True)
        job.thread = thread
        thread.start()
        return {
            "execution_id": execution_id,
            "status": "running",
            "report_path": str(report_path),
            "before_images": job.before_images,
        }

    def stop_motion(self) -> dict[str, Any]:
        self.safety.request_stop()
        return {"stop_requested": True, "active_executions": [job.execution_id for job in self.jobs.values() if job.status == "running"]}

    def execution_report(self, execution_id: str) -> dict[str, Any]:
        if execution_id not in self.jobs:
            raise KeyError(f"unknown execution {execution_id!r}")
        job = self.jobs[execution_id]
        if job.report is None and job.report_path.is_file():
            import json

            job.report = json.loads(job.report_path.read_text(encoding="utf-8"))
        return {
            "execution_id": job.execution_id,
            "plan_id": job.plan_id,
            "motion_plan_id": job.motion_plan_id,
            "status": job.status,
            "error": job.error,
            "before_images": job.before_images,
            "after_images": job.after_images,
            "report_path": str(job.report_path),
            "report": job.report,
        }

    def _targets_for_step(
        self,
        step: Any,
        current_joints: dict[str, np.ndarray],
        world: WorldState,
    ) -> dict[str, np.ndarray | None]:
        sides = ["left", "right"] if step.arms == "both" else [step.arms]
        targets: dict[str, np.ndarray | None] = {"left": None, "right": None}
        for side in sides:
            current_position, current_quaternion = self.kinematics.fk(side, current_joints[side])
            if "relative_position_m" in step.parameters:
                offset = np.asarray(step.parameters["relative_position_m"], dtype=float)
                target_position = current_position + offset
            elif "target_object_id" in step.parameters:
                object_state = world.objects[step.parameters["target_object_id"]]
                target_position = np.asarray(object_state.position_m, dtype=float)
                arm_offsets = step.parameters.get("arm_position_offsets_m", {})
                if side in arm_offsets:
                    target_position = target_position + np.asarray(arm_offsets[side], dtype=float)
                if "position_offset_m" in step.parameters:
                    target_position = target_position + np.asarray(step.parameters["position_offset_m"], dtype=float)
            else:
                validation = step.parameters.get("pose_validation", {})
                if not isinstance(validation, dict) or not validation.get("method") or "uncertainty_m" not in validation:
                    raise RuntimeError(f"step {step.id!r} has invalid pose_validation")
                target_position = np.asarray(step.parameters["target_position_m"], dtype=float)
            orientation = step.parameters.get("target_orientation_xyzw")
            target_quaternion = (
                np.asarray(orientation, dtype=float)
                if orientation is not None
                else np.asarray(current_quaternion, dtype=float)
            )
            if target_quaternion.shape != (4,) or not np.isfinite(target_quaternion).all():
                raise RuntimeError(f"step {step.id!r} has invalid target orientation")
            if abs(np.linalg.norm(target_quaternion) - 1.0) > 1e-5:
                raise RuntimeError(f"step {step.id!r} target orientation is not a unit quaternion")
            targets[side] = pose(target_position, target_quaternion)
        return targets

    def _execute_motion_step(
        self,
        step: Any,
        plan: TaskPlan,
        authorization_token: str | None,
    ) -> dict[str, Any]:
        raw_plan = plan.to_dict()
        raw_plan["steps"] = [step.to_dict()]
        raw_plan["success_conditions"] = []
        temporary_plan = TaskPlan.from_dict(raw_plan)
        submitted = self.submit_task_plan(temporary_plan.to_dict())
        if submitted["status"] != "valid":
            raise RuntimeError(f"motion step {step.id!r} is invalid: {submitted['errors']}")
        temporary_plan_id = submitted["plan_id"]
        motion = self.plan_motion(temporary_plan_id)
        execution = self.execute_motion(
            motion["motion_plan_id"],
            authorization_token,
            capture_before_after=False,
        )
        job = self.jobs[execution["execution_id"]]
        job.thread.join()
        report = self.execution_report(execution["execution_id"])
        self.plans.pop(temporary_plan_id, None)
        self.motion_plans.pop(motion["motion_plan_id"], None)
        self.motion_plan_plan_ids.pop(motion["motion_plan_id"], None)
        if report["status"] != "complete":
            raise RuntimeError(f"motion step {step.id!r} failed: {report['error']}")
        return {
            "status": "complete",
            "execution_id": execution["execution_id"],
            "report": report,
        }

    def _execute_gripper_step(
        self,
        step: Any,
        authorization_token: str | None,
    ) -> dict[str, Any]:
        action = {
            "set_gripper": step.parameters.get("action"),
            "hold_gripper": "hold",
            "release_gripper": "release",
            "grasp": "grasp",
        }[step.skill]
        return self.control_gripper(
            action=action,
            arms=step.arms,
            position=step.parameters.get("position"),
            authorization_token=authorization_token,
        )


def _arms_to_sides(arms: str) -> list[str]:
    if arms == "both":
        return ["left", "right"]
    if arms in {"left", "right"}:
        return [arms]
    raise ValueError("arms must be left, right, or both")


def _dependency_order(steps: list[Any]) -> list[Any]:
    remaining = {step.id: step for step in steps}
    complete: set[str] = set()
    ordered: list[Any] = []
    while remaining:
        ready = [
            step
            for step in remaining.values()
            if all(dependency in complete for dependency in step.depends_on)
        ]
        if not ready:
            raise RuntimeError("task-plan dependencies contain a cycle")
        ready.sort(key=lambda step: step.id)
        for step in ready:
            ordered.append(step)
            complete.add(step.id)
            del remaining[step.id]
    return ordered
