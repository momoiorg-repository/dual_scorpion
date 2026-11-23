"""Dual Scorpion encoder capture and ScorpionAgentHarness safety integration."""

from __future__ import annotations

import math
import time
from pathlib import Path
from threading import Event, RLock
from typing import TYPE_CHECKING, Any

import numpy as np

from ..core import ActionValidation, check_feedback_age, resolve_joint_chunk
from ..schema import (
    JointLimit,
    PolicyError,
    PolicyOutput,
    RobotCapabilities,
    RobotObservation,
    StaleResponseError,
    positive,
)
from .common import state_vector

if TYPE_CHECKING:
    from telegrip.mocopi.agent.config import HarnessConfig
    from telegrip.mocopi.agent.harness import ScorpionAgentHarness
    from telegrip.mocopi.agent.robot import RealRobotAdapter, RobotReading

SIDES = ("right", "left")  # DualScorpionFollower and LeRobot dataset feature order.
JOINTS = tuple(f"joint{i}" for i in range(7))
FEATURES = tuple(f"{side}_{motor}.pos" for side in SIDES for motor in (*JOINTS, "gripper"))
CAMERAS = ("head", "right_wrist", "left_wrist")
POSITION_UNITS = {name: "lerobot_0_100" if name.endswith("gripper.pos") else "deg" for name in FEATURES}
ARM_SPEED_DEG_S = 15.0
ARM_STEP_DEG = 0.5
CONTROL_FPS = 30


class DualScorpionAdapter:
    def __init__(
        self,
        *,
        config: Path | None = None,
        camera_ids: dict[str, str] | None = None,
        verified_arm_binding: dict[str, Any] | None = None,
    ):
        self._config: HarnessConfig | None = None
        if config is not None:
            try:
                from telegrip.mocopi.agent.config import HarnessConfig
            except ImportError as exc:
                raise PolicyError(
                    "Dual Scorpion hardware needs Telegrip and the mocopi and feetech extras"
                ) from exc
            self._config = HarnessConfig.from_file(config)
            positive(self._config.max_feedback_age_s, "harness max_feedback_age_s")
        if camera_ids is None:
            camera_ids = {name: name for name in CAMERAS}
        if (
            not isinstance(camera_ids, dict)
            or set(camera_ids) != set(CAMERAS)
            or any(not isinstance(value, str) or not value for value in camera_ids.values())
            or len(set(camera_ids.values())) != len(CAMERAS)
        ):
            raise PolicyError(
                "Dual Scorpion camera_ids must map head, right_wrist and left_wrist to distinct IDs"
            )
        self._camera_ids = dict(camera_ids)
        configured = None if self._config is None else {camera.camera_id for camera in self._config.cameras}
        cameras = tuple(name for name in CAMERAS if configured is None or camera_ids[name] in configured)
        self._capabilities = RobotCapabilities(
            "dual_scorpion",
            FEATURES,
            POSITION_UNITS,
            cameras,
            FEATURES,
            POSITION_UNITS,
            max_action_horizon=239 if self._config is None else self._config.max_plan_frames - 1,
            max_control_fps=CONTROL_FPS,
        )
        if verified_arm_binding is not None and (
            not isinstance(verified_arm_binding, dict)
            or set(verified_arm_binding) != {"calibration_sha256", "urdf_sha256", "scene_sha256"}
            or not _is_sha256(verified_arm_binding["calibration_sha256"])
            or not _is_sha256(verified_arm_binding["scene_sha256"])
            or not isinstance(verified_arm_binding["urdf_sha256"], dict)
            or set(verified_arm_binding["urdf_sha256"]) != set(SIDES)
            or not all(_is_sha256(value) for value in verified_arm_binding["urdf_sha256"].values())
        ):
            raise PolicyError(
                "verified_arm_binding must contain the calibration, legacy URDF, and scene SHA-256 hashes"
            )
        self._binding = verified_arm_binding
        self._harness: ScorpionAgentHarness | None = None
        self._hardware: RealRobotAdapter | None = None
        self._live: RobotObservation | None = None
        self._lock = RLock()
        self._stopped = Event()

    def get_capabilities(self) -> RobotCapabilities:
        return self._capabilities

    def _robot(self) -> RealRobotAdapter | None:
        if self._config is None or self._config.robot is None:
            return None
        if self._hardware is None:
            from telegrip.mocopi.agent.robot import RealRobotAdapter

            spec = self._config.robot
            self._hardware = RealRobotAdapter(
                left_port=spec.left_port,
                right_port=spec.right_port,
                robot_id=spec.robot_id,
                calibration_dir=spec.calibration_dir,
                max_step_deg=spec.max_step_deg,
                gripper=self._config.gripper,
            )
        return self._hardware

    def _services(self) -> ScorpionAgentHarness:
        if self._config is None:
            raise PolicyError("Dual Scorpion requires a local HarnessConfig for hardware capture")
        if self._harness is None:
            from telegrip.mocopi.agent.harness import ScorpionAgentHarness

            harness = ScorpionAgentHarness(self._config, robot=self._robot())
            self._harness = harness
            if self._stopped.is_set():
                harness.safety.request_stop()
        return self._harness

    def doctor(self) -> dict[str, Any]:
        """Inspect local device paths and calibration files without opening serial ports."""
        spec = None if self._config is None else self._config.robot
        hardware = self._robot()
        calibration = None if hardware is None else hardware.validate_calibration()
        cameras = {}
        if self._config is not None:
            by_id = {camera.camera_id: camera for camera in self._config.cameras}
            for name, camera_id in self._camera_ids.items():
                camera = by_id.get(camera_id)
                device = None if camera is None else camera.device
                path = f"/dev/video{device}" if isinstance(device, int) else device
                cameras[name] = {
                    "device": device,
                    "present": path is not None and Path(path).exists(),
                }
        scene_status = "unconfigured"
        if self._config is not None and self._config.scene_geometry_path is not None:
            from telegrip.mocopi.agent.scene import SceneGeometry

            try:
                scene = SceneGeometry.from_file(self._config.scene_geometry_path)
                scene_status = scene.status if _measured_scene(scene) else "insufficient_geometry"
            except (ValueError, OSError, TypeError):
                scene_status = "invalid"
        return {
            "serial_ports": {}
            if spec is None
            else {side: Path(getattr(spec, f"{side}_port")).exists() for side in SIDES},
            "missing_camera_config": [
                name for name in CAMERAS if cameras.get(name, {}).get("device") is None
            ],
            "cameras": cameras,
            "calibration_status": "unconfigured" if calibration is None else calibration["status"],
            "calibration_errors": [] if calibration is None else calibration["errors"],
            "scene_status": scene_status,
            "verified_arm_binding_configured": self._binding is not None,
        }

    def get_observation(self, task: str | None = None) -> RobotObservation:
        with self._lock:
            self._live = None
            hardware = self._robot()
            if hardware is None:
                raise PolicyError("Dual Scorpion capture requires a configured, calibrated follower")
            if set(self._capabilities.images) != set(CAMERAS):
                raise PolicyError(
                    f"Dual Scorpion cameras missing from HarnessConfig: {sorted(set(CAMERAS) - set(self._capabilities.images))}"
                )
            harness = self._services()
            if not hardware.is_connected:
                # RealRobotAdapter.connect opens the buses and checks their IDs. It does not call
                # DualScorpionFollower.connect, which configures the motors or can run calibration.
                hardware.connect()
            images, captured_at = {}, {}
            for name, camera_id in self._camera_ids.items():
                captured_at[name], images[name] = harness.cameras.read_rgb(camera_id)
            reading = hardware.read()
            observation = RobotObservation(
                hardware.robot_id,
                self._capabilities.robot_type,
                reading.timestamp_monotonic_ns,
                _state_from_reading(reading),
                FEATURES,
                POSITION_UNITS,
                images,
                captured_at,
                task=task,
                metadata={"source": "scorpion_agent_harness_encoders"},
            )
            check_feedback_age(observation, harness.config.max_feedback_age_s, cameras=CAMERAS)
            self._live = observation
            return observation

    def _limits(self) -> tuple[dict[str, JointLimit], bool]:
        hardware = self._robot()
        calibration_ok = hardware is not None and hardware.validate_calibration()["status"] == "valid"
        gripper = None if self._config is None else self._config.gripper
        gripper_min, gripper_max, gripper_step = (0.0, 100.0, 2.0)
        if gripper is not None:
            gripper_min = max(gripper_min, gripper.min_position)
            gripper_max = min(gripper_max, gripper.max_position)
            gripper_step = gripper.max_step
        step = ARM_STEP_DEG if hardware is None else min(ARM_STEP_DEG, hardware.max_step_deg)
        limits = {}
        if calibration_ok:
            from telegrip.mocopi.dataset.hardware_dataset import calibration_joint_limits

        for side in SIDES:
            for joint in JOINTS:
                minimum, maximum = -180.0, 180.0
                if calibration_ok:
                    window = calibration_joint_limits(hardware.follower, side, joint)
                    minimum, maximum = window["min_motor_deg"], window["max_motor_deg"]
                limits[f"{side}_{joint}.pos"] = JointLimit(minimum, maximum, ARM_SPEED_DEG_S, step)
            limits[f"{side}_gripper.pos"] = JointLimit(
                gripper_min,
                gripper_max,
                CONTROL_FPS * gripper_step,
                gripper_step,
            )
        return limits, calibration_ok

    def validate_action(self, output: PolicyOutput, observation: RobotObservation) -> ActionValidation:
        with self._lock:
            validation, _ = self._validate(output, observation)
            return validation

    def _validate(
        self,
        output: PolicyOutput,
        observation: RobotObservation,
    ) -> tuple[ActionValidation, dict[str, Any] | None]:
        limits, calibrated = self._limits()
        positions = resolve_joint_chunk(output, observation, self._capabilities, limits)
        reference = state_vector(observation, FEATURES)
        blockers = []
        checks = {
            "joint_limits": "calibration_checked" if calibrated else "nominal_only",
            "gripper_range": "passed",
            "velocity_and_continuity": "passed",
            "measured_scene_and_self_collision": "unchecked",
            "feedback_freshness": "offline_or_external",
            "operation_grant": "required_at_execution",
        }
        if not calibrated:
            blockers.append("verified follower motor calibration required")
        live = observation is self._live
        if live:
            check_feedback_age(observation, self._config.max_feedback_age_s, cameras=CAMERAS)
            checks["feedback_freshness"] = "passed"
        else:
            blockers.append("execution requires this process's fresh read-only capture")
        stopped = self._stopped.is_set() or (
            self._harness is not None and self._harness.safety.stop_requested
        )
        checks["emergency_stop"] = "latched" if stopped else "clear"
        if stopped:
            blockers.append("local emergency stop is latched")
        if np.any(positions[:, [7, 15]] != reference[[7, 15]]):
            blockers.append("the harness arm executor cannot apply gripper changes in a policy chunk")

        plan = None
        if self._config is None or self._config.scene_geometry_path is None:
            blockers.append("measured table and support-stick scene geometry required")
        elif not calibrated or self._binding is None:
            blockers.append("verified legacy-URDF motor mapping and collision envelope required")
        elif not float(output.fps).is_integer():
            blockers.append("the harness arm executor requires integer fps")
        else:
            harness = self._services()
            self._check_binding(harness)
            waypoints = [_arms(row) for row in np.vstack((reference, positions))]
            plan = harness.kinematics.build_joint_plan(
                waypoints,
                fps=int(output.fps),
                task=f"policy {output.policy_id} {output.request_id} {output.observation_id}",
            )
            from telegrip.mocopi.dataset.real import plan_hash

            checks["motion_plan_id"] = plan_hash(plan)
            checks["joint_limits"] = "calibration_and_urdf_checked"
            checks["measured_scene_and_self_collision"] = "passed"
        return ActionValidation(len(positions), checks, tuple(blockers)), plan

    def _check_binding(self, harness: ScorpionAgentHarness) -> None:
        from telegrip.mocopi.agent.scene import SceneGeometry
        from telegrip.mocopi.dataset.hardware_dataset import calibration_sha256
        from telegrip.mocopi.dataset.replay import model_fingerprint

        scene = SceneGeometry.from_file(harness.config.scene_geometry_path)
        if not _measured_scene(scene):
            raise PolicyError("execution requires measured table and support-stick collision geometry")
        if scene.sha256 != harness.kinematics.scene_status().get("sha256"):
            raise PolicyError("measured scene changed since planning services started")
        if self._binding != {
            "calibration_sha256": calibration_sha256(harness.robot.follower),
            "urdf_sha256": model_fingerprint(harness.kinematics.full_config),
            "scene_sha256": scene.sha256,
        }:
            raise PolicyError(
                "legacy URDF, motor calibration or scene differs from the local verified binding"
            )

    def execute_action(self, output: PolicyOutput, observation: RobotObservation) -> None:
        """Execute through the harness only with a one-shot grant for this exact local arm plan."""
        with self._lock:
            validation, plan = self._validate(output, observation)
            if validation.blockers or plan is None:
                raise PolicyError("execution blocked: " + "; ".join(validation.blockers))
            harness = self._services()
            age_s = harness.config.max_feedback_age_s
            limits, calibrated = self._limits()
            if not calibrated:
                raise PolicyError("follower motor calibration changed since validation")
            bounds = {
                side: np.array(
                    [
                        (limits[f"{side}_{joint}.pos"].minimum, limits[f"{side}_{joint}.pos"].maximum)
                        for joint in JOINTS
                    ]
                ).T
                for side in SIDES
            }
            check_feedback_age(observation, age_s, cameras=CAMERAS)
            if not np.array_equal(
                _state_from_reading(harness.robot.read())[[7, 15]],
                state_vector(observation, FEATURES)[[7, 15]],
            ):
                raise PolicyError("gripper feedback changed since the captured observation")
            plan_id = validation.checks["motion_plan_id"]
            grant = harness.safety.authorize_arm(plan_id, list(SIDES))
            self._live = None  # A captured sample can authorize at most one attempt.
            step = min(harness.robot.max_step_deg, ARM_STEP_DEG, ARM_SPEED_DEG_S / output.fps)

            def guard(index: int, feedback_ns: int, current: dict, desired: dict) -> None:
                if any(
                    np.any(current[side] < bounds[side][0]) or np.any(current[side] > bounds[side][1])
                    for side in SIDES
                ):
                    raise PolicyError("live feedback is outside motor calibration limits")
                if any(np.max(np.abs(desired[side] - current[side])) > step for side in SIDES):
                    raise PolicyError("live feedback would require an unsafe waypoint step")
                harness.kinematics.check_joint_segment(current, desired)
                now = time.monotonic_ns()
                if (
                    not 0 <= now - feedback_ns <= age_s * 1e9
                    or now - observation.timestamp_ns > (age_s + index / output.fps) * 1e9
                ):
                    raise StaleResponseError("policy chunk or live encoder feedback expired")
                if time.time() >= grant.expires_at_unix:
                    raise PolicyError("local operation grant expired during motion")

            harness.motion_plans[plan_id] = plan
            try:
                result = harness.execute_motion(plan_id, authorization_grant=grant, before_command=guard)
                job = harness.jobs[result["execution_id"]]
                try:
                    job.thread.join()
                except KeyboardInterrupt:
                    self.stop()
                    job.thread.join()
                    raise
                if job.status != "complete":
                    raise PolicyError(f"ScorpionAgentHarness stopped the policy plan: {job.error}")
            finally:
                harness.motion_plans.pop(plan_id, None)

    def stop(self) -> None:
        self._stopped.set()
        if self._harness is not None:
            self._harness.safety.request_stop()

    def close(self) -> None:
        with self._lock:
            self._live = None
            if self._harness is not None:
                self._harness.close()
                self._harness = None


def _state_from_reading(reading: RobotReading) -> np.ndarray:
    if not reading.connected or not reading.calibrated or reading.errors:
        raise PolicyError("Dual Scorpion follower feedback is disconnected, uncalibrated or invalid")
    grippers = reading.gripper_positions
    if grippers is None or any(side not in grippers or not math.isfinite(grippers[side]) for side in SIDES):
        raise PolicyError("Dual Scorpion capture requires both calibrated gripper readings")
    return np.array(
        [value for side in SIDES for value in (*reading.joints_deg[side], grippers[side])], dtype=np.float32
    )


def _arms(row: np.ndarray) -> dict[str, np.ndarray]:
    return {side: row[index * 8 : index * 8 + 7] for index, side in enumerate(SIDES)}


def _measured_scene(scene) -> bool:
    return scene.execution_allowed and {"table", "support_stick"} <= {row.role for row in scene.obstacles}


def _is_sha256(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)
