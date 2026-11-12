"""Hardware-free tests for the real-robot Dual Scorpion agent harness."""

from __future__ import annotations

import base64
import json
import threading
import time
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from telegrip.mocopi.agent.cameras import CameraService
from telegrip.mocopi.agent.config import CameraSpec, GripperSpec, HarnessConfig
from telegrip.mocopi.agent.harness import ScorpionAgentHarness
from telegrip.mocopi.agent.mcp import ScorpionMcpServer
from telegrip.mocopi.agent.motion import KinematicsService
from telegrip.mocopi.agent.robot import (
    RealRobotAdapter,
    load_calibration_document,
    validate_calibration_document,
)
from telegrip.mocopi.agent.safety import AuthorizationManager
from telegrip.mocopi.agent.scene import SceneGeometry
from telegrip.mocopi.agent.schema import PLAN_SCHEMA_ID
from telegrip.mocopi.agent.world import StateSource
from telegrip.mocopi.dataset.real import JOINTS, execute_plan

CONFIG = Path(__file__).parents[2] / "config" / "mocopi" / "tracking.yaml"


def _plan(**parameter_overrides: Any) -> dict[str, Any]:
    parameters = {
        "relative_position_m": [0.01, 0.0, 0.0],
        "target_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    parameters.update(parameter_overrides)
    return {
        "schema": PLAN_SCHEMA_ID,
        "goal": "Move both arms one centimeter forward.",
        "goal_state": {"type": "relative_tcp_motion"},
        "objects": [],
        "environment": {},
        "arm_roles": {"left": "mover", "right": "mover"},
        "steps": [
            {
                "id": "move",
                "skill": "move_arm_to_pose",
                "arms": "both",
                "depends_on": [],
                "parameters": parameters,
            }
        ],
        "preconditions": ["Both arms are connected and calibrated."],
        "success_conditions": [
            {"type": "tcp_position_error", "step_id": "move", "max_value": 0.025},
            {"type": "joint_tracking_error", "step_id": "move", "max_value": 5.0},
            {"type": "no_stop"},
        ],
        "failure_conditions": [{"type": "ik_failure"}],
        "safety_constraints": ["Reject stale feedback.", "Never command the gripper."],
        "replan_policy": {"max_attempts": 3},
        "provenance": {"planner": "test"},
    }


class FakeKinematics:
    def __init__(self):
        self.full_config = {"robot": {"max_joint_step_deg": 0.5}}
        self.config = {"max_joint_step_deg": 0.5}
        self.simulation = None

    def close(self):
        pass

    def fk(self, side: str, joints: np.ndarray):
        assert side in {"left", "right"}
        return np.array([0.2, 0.0, 0.3]), np.array([0.0, 0.0, 0.0, 1.0])

    def build_motion_plan(self, *args: Any, **kwargs: Any):
        return {
            "schema": "dual_scorpion.mocopi.robot_plan.v1",
            "frames": [
                {
                    "arms": {
                        "left": {"joint_deg": [0.0] * 7},
                        "right": {"joint_deg": [0.0] * 7},
                    }
                }
            ],
            "ik_info": {},
            "scene_sha256": "test-scene",
            "collision_checked": True,
        }

    def validate(self, *args: Any, **kwargs: Any):
        return {"status": "valid", "errors": []}

    def scene_status(self):
        return {
            "configured": True,
            "status": "measured",
            "execution_allowed": True,
            "sha256": "test-scene",
        }

    def collision_report(self, joints):
        return {
            "configured": True,
            "checked": True,
            "execution_allowed": True,
            "count": 0,
            "clearance_violation": False,
            "penetrating": False,
            "min_distance_m": 0.1,
            "contacts": [],
        }


class FakeRobot:
    def __init__(self):
        self.is_calibrated = True

    @property
    def is_connected(self):
        return True

    def status(self):
        return {"connected": True, "calibrated": True}

    def read(self):
        return SimpleNamespace(
            connected=True,
            calibrated=True,
            joints_deg={"left": tuple([0.0] * 7), "right": tuple([0.0] * 7)},
            timestamp_monotonic_ns=1,
        )

    def execute_plan(self, *args: Any, **kwargs: Any):
        return {
            "status": "complete",
            "frames": [],
            "failure": None,
            "metrics": {"hardware_commands_sent": 0},
        }


class FakeGripperBus:
    def __init__(
        self,
        initial_position: float = 0.0,
        *,
        stall_position: float | None = None,
        load: float = 0.0,
        current: float = 0.0,
        fail_reads: bool = False,
    ):
        from lerobot.motors import MotorCalibration, MotorNormMode

        self.is_connected = True
        self.is_calibrated = True
        self.position = float(initial_position)
        self.stall_position = stall_position
        self.load = load
        self.current = current
        self.fail_reads = fail_reads
        self.commands = []
        self.enabled = []
        self.active_writes = 0
        self.max_active_writes = 0
        self.calibration = {
            **{joint: MotorCalibration(index + 1, 0, 0, 0, 4095) for index, joint in enumerate(JOINTS)},
            "gripper": MotorCalibration(8, 0, 0, 0, 4095),
        }
        self.motors = {
            **{joint: SimpleNamespace(model="sts3215") for joint in JOINTS},
            "gripper": SimpleNamespace(model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
        }

    def sync_read(self, data_name, motors=None, normalize=True):
        if self.fail_reads:
            raise ConnectionError("fake gripper bus disconnected")
        if data_name == "Present_Position":
            return {"gripper": self.position}
        if data_name == "Present_Load":
            return {"gripper": self.load}
        if data_name == "Present_Current":
            return {"gripper": self.current}
        raise KeyError(data_name)

    def sync_write(self, data_name, values, normalize=True):
        if data_name != "Goal_Position":
            raise KeyError(data_name)
        target = float(values["gripper"])
        self.active_writes += 1
        self.max_active_writes = max(self.max_active_writes, self.active_writes)
        time.sleep(0.01)
        self.commands.append(target)
        if (
            self.stall_position is not None
            and ((target > self.position and self.stall_position <= target)
                 or (target < self.position and self.stall_position >= target))
        ):
            self.position = float(self.stall_position)
        else:
            self.position = target
        self.active_writes -= 1

    def enable_torque(self, motors):
        self.enabled.extend(motors)

    def disconnect(self, disable_torque=True):
        self.is_connected = False


class FakeGripperFollower:
    def __init__(self, left: FakeGripperBus, right: FakeGripperBus):
        self.left_bus = left
        self.right_bus = right
        self.is_calibrated = True

    @property
    def is_connected(self):
        return self.left_bus.is_connected and self.right_bus.is_connected

    def get_observation(self):
        result = {
            f"{side}_{joint}.pos": 0.0
            for side in ("left", "right")
            for joint in JOINTS
        }
        result["left_gripper.pos"] = self.left_bus.position
        result["right_gripper.pos"] = self.right_bus.position
        return result


class FakeCombinedRobot(FakeRobot):
    def __init__(self, gripper: GripperSpec | None = None):
        super().__init__()
        self.adapter = RealRobotAdapter(
            left_port="/dev/null",
            right_port="/dev/null",
            robot_id="fake",
            gripper=gripper or GripperSpec(max_step=50.0, settle_s=0.001, grasp_timeout_s=0.2),
            follower=FakeGripperFollower(FakeGripperBus(0.0), FakeGripperBus(10.0)),
        )

    def read(self):
        reading = self.adapter.read()
        return SimpleNamespace(
            connected=True,
            calibrated=True,
            joints_deg=reading.joints_deg,
            timestamp_monotonic_ns=reading.timestamp_monotonic_ns,
            gripper_positions=reading.gripper_positions,
        )

    def read_gripper_state(self, side):
        return self.adapter.read_gripper_state(side)

    def open_gripper(self, side, *, stop_event=None):
        return self.adapter.open_gripper(side, stop_event=stop_event)

    def close_gripper(self, side, *, stop_event=None):
        return self.adapter.close_gripper(side, stop_event=stop_event)

    def set_gripper_position(self, side, position, *, stop_event=None):
        return self.adapter.set_gripper_position(side, position, stop_event=stop_event)

    def hold_gripper(self, side, *, stop_event=None):
        return self.adapter.hold_gripper(side, stop_event=stop_event)

    def release_gripper(self, side, *, stop_event=None):
        return self.adapter.release_gripper(side, stop_event=stop_event)

    def grasp(self, side, *, stop_event=None):
        return self.adapter.grasp(side, stop_event=stop_event)


class FakeCamera:
    def read(self):
        image = np.zeros((4, 6, 3), dtype=np.uint8)
        image[:, :, 2] = 255
        return 123.0, image

    def close(self):
        pass


def _harness(tmp_path: Path, *, robot: Any | None = None) -> ScorpionAgentHarness:
    config = HarnessConfig(
        robot_config_path=CONFIG,
        cameras=(),
        authorization_token_file=None,
        max_feedback_age_s=0.5,
        max_plan_frames=240,
        execution_report_dir=tmp_path / "reports",
        gripper=GripperSpec(max_step=50.0, settle_s=0.001, grasp_timeout_s=0.2),
    )
    return ScorpionAgentHarness(
        config,
        robot=robot or FakeRobot(),
        cameras=None,
        kinematics=FakeKinematics(),
        authorization=AuthorizationManager(token="operator-token"),
    )


def test_plan_schema_rejects_direct_motor_commands_and_unavailable_skills(tmp_path):
    harness = _harness(tmp_path)
    plan = _plan(joint_deg=[0.0] * 7)
    result = harness.submit_task_plan(plan)
    assert result["status"] == "rejected"
    assert any("direct motor fields" in error for error in result["errors"])

    unavailable = _plan()
    unavailable["steps"][0]["parameters"] = {}
    unavailable["steps"][0]["skill"] = "not_a_real_skill"
    result = harness.submit_task_plan(unavailable)
    assert result["status"] == "rejected"
    assert any("unavailable skill" in error for error in result["errors"])


def test_semantic_object_estimate_requires_metric_validation(tmp_path):
    harness = _harness(tmp_path)
    with pytest.raises(ValueError, match="coordinate_validation"):
        harness.update_object_estimates(
            [
                {
                    "id": "cup",
                    "label": "cup",
                    "source": StateSource.LLM_SEMANTIC,
                    "position_m": [0.1, 0.0, 0.2],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "confidence": 0.5,
                    "timestamp_monotonic_ns": 1,
                }
            ]
        )

    harness.update_object_estimates(
        [
            {
                "id": "cup",
                "label": "cup",
                "source": StateSource.LLM_SEMANTIC,
                "position_m": None,
                "orientation_xyzw": None,
                "confidence": 0.5,
                "timestamp_monotonic_ns": 1,
            }
        ]
    )
    world = harness.world_state()
    assert world.objects["cup"].source == StateSource.LLM_SEMANTIC
    assert world.objects["cup"].position_m is None


def test_camera_service_returns_mcp_image_content(tmp_path):
    from telegrip.mocopi.agent.cameras import CameraService
    from telegrip.mocopi.agent.config import CameraSpec

    spec = CameraSpec(
        camera_id="head",
        device="/dev/null",
        width=6,
        height=4,
        fps=30,
        coordinate_frame="head_camera_uncalibrated",
    )
    service = CameraService((spec,), backend_factory=lambda camera_spec: FakeCamera())
    frames = service.capture(["head"])
    assert frames[0].width == 6 and frames[0].height == 4
    assert frames[0].mime_type == "image/jpeg"
    content = frames[0].to_mcp_content()
    assert content["type"] == "image"
    assert base64.b64decode(content["data"])
    assert frames[0].metadata()["camera_id"] == "head"


def test_mcp_lists_tools_and_requires_motion_authorization(tmp_path):
    spec = CameraSpec(camera_id="head", device="/dev/null", width=6, height=4, fps=30)
    harness = _harness(tmp_path)
    harness.cameras = CameraService((spec,), backend_factory=lambda camera_spec: FakeCamera())
    server = ScorpionMcpServer(harness)
    initialized = server.handle({"jsonrpc": "2.0", "id": 0, "method": "initialize"})
    assert initialized["result"]["protocolVersion"] == "2025-06-18"
    listed = server.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/list"})
    names = {tool["name"] for tool in listed["result"]["tools"]}
    assert {"get_status", "capture_images", "submit_task_plan", "execute_motion", "stop_motion"} <= names

    captured = server.handle(
        {
            "jsonrpc": "2.0",
            "id": 2,
            "method": "tools/call",
            "params": {"name": "capture_images", "arguments": {"camera_ids": ["head"]}},
        }
    )
    assert captured["result"]["content"][0]["type"] == "image"
    assert base64.b64decode(captured["result"]["content"][0]["data"])
    assert captured["result"]["structuredContent"]["frames"][0]["camera_id"] == "head"

    result = harness.submit_task_plan(_plan())
    plan_id = result["plan_id"]
    motion = harness.plan_motion(plan_id)
    response = server.handle(
        {
            "jsonrpc": "2.0",
            "id": 2,
            "method": "tools/call",
            "params": {
                "name": "execute_motion",
                "arguments": {"motion_plan_id": motion["motion_plan_id"], "authorization_token": None},
            },
        }
    )
    assert "error" in response
    assert "authorization" in response["error"]["message"]


def test_motion_planner_generates_and_validates_fk_ik_trajectory():
    with KinematicsService(CONFIG) as kinematics:
        start = {side: np.zeros(7) for side in ("left", "right")}
        targets = {}
        for side in ("left", "right"):
            position, quaternion = kinematics.fk(side, start[side])
            targets[side] = np.eye(4)
            targets[side][:3, 3] = position + np.array([0.01, 0.0, 0.0])
            targets[side][:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        plan = kinematics.build_motion_plan(
            start,
            targets,
            fps=30,
            source="fake",
            task="unit test relative motion",
        )
        validation = kinematics.validate(plan, allow_synthetic=True)
        assert plan["gripper_included"] is False
        assert plan["frames"] and plan["frames"][0]["arms"]["left"]["joint_deg"] == [0.0] * 7
        assert validation["frames"] == len(plan["frames"])
        assert validation["fps"] == 30


def _write_scene(path: Path, *, table_center: list[float], status: str = "measured") -> Path:
    path.write_text(
        f"""
schema: dual_scorpion.agent.scene.v1
status: {status}
coordinate_frame: measured_workbench
source: local_test_measurement
measured_at: "2026-11-12T00:00:00Z"
collision_margin_m: 0.002
arm_collision_radius_m: 0.01
robot_to_scene:
  position_m: [0.0, 0.0, 0.0]
  orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
required_obstacles: [table, support_stick]
obstacles:
  - name: measured_table
    role: table
    shape: box
    measurement_status: measured
    center_m: {table_center}
    half_size_m: [1.0, 1.0, 0.01]
  - name: support_stick
    role: support_stick
    shape: cylinder
    measurement_status: measured
    center_m: [0.0, 1.0, 0.0]
    rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
    length_m: 0.45
    radius_m: 0.01
""",
        encoding="utf-8",
    )
    return path


def test_measured_table_and_stick_scene_enables_trajectory_collision_check(tmp_path):
    scene = SceneGeometry.from_file(
        _write_scene(tmp_path / "scene.yaml", table_center=[0.0, 0.0, -1.0])
    )
    assert scene.execution_allowed is True
    with KinematicsService(CONFIG, scene=scene) as kinematics:
        start = {side: np.zeros(7) for side in ("left", "right")}
        initial = kinematics.collision_report(start)
        assert initial["configured"] is True and initial["count"] == 0
        targets = {}
        for side in ("left", "right"):
            position, quaternion = kinematics.fk(side, start[side])
            targets[side] = np.eye(4)
            targets[side][:3, 3] = position + np.array([0.01, 0.0, 0.0])
            targets[side][:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        plan = kinematics.build_motion_plan(start, targets, fps=30, source="fake")
        assert plan["collision_checked"] is True
        assert plan["scene_collision"]["count"] == 0
        assert plan["startup_behavior"]["auto_home"] is False


def test_initial_table_contact_rejects_motion_before_ik_or_execution(tmp_path):
    scene = SceneGeometry.from_file(
        _write_scene(tmp_path / "blocking-scene.yaml", table_center=[0.0, 0.0, 0.14])
    )
    with KinematicsService(CONFIG, scene=scene) as kinematics:
        start = {side: np.zeros(7) for side in ("left", "right")}
        report = kinematics.collision_report(start)
        assert report["clearance_violation"] is True
        with pytest.raises(Exception, match="initial pose violates measured scene clearance"):
            kinematics.build_motion_plan(
                start,
                {"left": np.eye(4), "right": np.eye(4)},
                fps=30,
                source="fake",
            )


def test_unmeasured_scene_is_parsed_but_never_execution_allowed(tmp_path):
    scene = SceneGeometry.from_file(
        _write_scene(
            tmp_path / "template-scene.yaml",
            table_center=[0.0, 0.0, -1.0],
            status="template_not_for_execution",
        )
    )
    assert scene.execution_allowed is False
    with KinematicsService(CONFIG, scene=scene) as kinematics:
        assert kinematics.scene_status()["execution_allowed"] is False
        with pytest.raises(Exception, match="measured scene geometry is required"):
            kinematics.build_motion_plan(
                {"left": np.zeros(7), "right": np.zeros(7)},
                {"left": np.eye(4), "right": np.eye(4)},
                fps=30,
                source="fake",
            )


class _StopBus:
    def __init__(self, joints: list[float]):
        self.is_connected = False
        self.calibration = {name: SimpleNamespace(range_min=0, range_max=4095) for name in JOINTS}
        self.motors = {name: SimpleNamespace(model="sts3215") for name in JOINTS}
        self.model_resolution_table = {"sts3215": 4096}
        self.joints = joints

    def connect(self):
        self.is_connected = True

    def read(self, field: str, name: str, normalize: bool = False):
        assert field == "Operating_Mode" and not normalize
        return 0

    def enable_torque(self, names):
        pass

    def disconnect(self, disable_torque: bool):
        self.is_connected = False


class _StopRobot:
    def __init__(self, plan):
        self.left_bus = _StopBus(plan["frames"][0]["arms"]["left"]["joint_deg"])
        self.right_bus = _StopBus(plan["frames"][0]["arms"]["right"]["joint_deg"])
        self.config = SimpleNamespace(max_relative_target=0.5, use_degrees=True)
        self.is_calibrated = True

    @property
    def is_connected(self):
        return self.left_bus.is_connected and self.right_bus.is_connected

    def get_observation(self):
        return {
            f"{side}_{name}.pos": float(getattr(self, side + "_bus").joints[index])
            for side in ("left", "right")
            for index, name in enumerate(JOINTS)
        }

    def send_action(self, action):
        for side in ("left", "right"):
            for index, name in enumerate(JOINTS):
                getattr(self, side + "_bus").joints[index] = float(action[f"{side}_{name}.pos"])
        return dict(action)


def test_real_execution_honors_local_stop_request(tmp_path):
    with KinematicsService(CONFIG) as kinematics:
        start = {side: np.zeros(7) for side in ("left", "right")}
        targets = {}
        for side in ("left", "right"):
            position, quaternion = kinematics.fk(side, start[side])
            targets[side] = np.eye(4)
            targets[side][:3, 3] = position + np.array([0.01, 0.0, 0.0])
            targets[side][:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        plan = kinematics.build_motion_plan(start, targets, fps=30, source="fake")
        robot = _StopRobot(plan)
        stop_event = threading.Event()
        stop_event.set()
        report_path = tmp_path / "stopped.json"
        with pytest.raises(RuntimeError, match="Operator stop requested"):
            execute_plan(
                plan,
                kinematics.full_config,
                kinematics.simulation,
                robot,
                report_path,
                realtime=False,
                allow_synthetic=True,
                stop_event=stop_event,
            )
        report = json.loads(report_path.read_text())
        assert report["status"] == "aborted"
        assert "Operator stop requested" in report["failure"]
        assert report["gripper_commanded"] is False
        assert not robot.left_bus.is_connected and not robot.right_bus.is_connected


def test_agent_skill_structure_and_references_exist():
    skill_root = Path(__file__).parents[2] / ".agents" / "skills" / "dual-scorpion-real-agent"
    skill_path = skill_root / "SKILL.md"
    text = skill_path.read_text(encoding="utf-8")
    assert text.startswith("---\n")
    assert "name: dual-scorpion-real-agent" in text
    assert "description:" in text
    assert (skill_root / "references" / "mcp-tools.md").is_file()
    assert (skill_root / "references" / "safety.md").is_file()
    for tool_name in (
        "get_status",
        "capture_images",
        "submit_task_plan",
        "execute_motion",
        "read_gripper_state",
        "control_gripper",
        "execute_task_plan",
        "stop_motion",
    ):
        assert tool_name in (skill_root / "references" / "mcp-tools.md").read_text(encoding="utf-8")
    assert "unverified" in text


def test_example_agent_config_resolves_paths_without_hardcoding_devices():
    root = Path(__file__).parents[2]
    config = HarnessConfig.from_file(root / "config" / "mocopi" / "agent.example.yaml")
    assert config.robot_config_path == (root / "config" / "mocopi" / "tracking.yaml").resolve()
    assert [camera.camera_id for camera in config.cameras] == ["camera"]
    assert config.cameras[0].device == "/dev/video2"
    assert config.gripper.open_position == 0.0
    assert config.gripper.closed_position == 45.0
    assert all(camera.device for camera in config.cameras)
    assert config.authorization_token_file is not None
    assert config.robot is not None
    assert config.robot.robot_id == "dual_scorpion_follower_2026_1026"
    assert config.robot.calibration_dir is None
    assert config.scene_geometry_path is None


def _write_calibration(path: Path, robot_id: str = "test_dual_scorpion") -> Path:
    from lerobot.motors import MotorCalibration

    document = {}
    for side in ("left", "right"):
        for index in range(7):
            document[f"{side}_joint{index}"] = asdict(
                MotorCalibration(
                    id=index + 1,
                    drive_mode=0,
                    homing_offset=100 + index,
                    range_min=10,
                    range_max=4000,
                )
            )
        document[f"{side}_gripper"] = asdict(
            MotorCalibration(id=8, drive_mode=0, homing_offset=0, range_min=0, range_max=4095)
        )
    calibration_path = path / f"{robot_id}.json"
    calibration_path.write_text(json.dumps(document), encoding="utf-8")
    return calibration_path


def test_lerobot_calibration_file_is_validated_without_recalibration(tmp_path):
    calibration_path = _write_calibration(tmp_path)
    document = load_calibration_document(calibration_path)
    report = validate_calibration_document(
        document,
        robot_id="test_dual_scorpion",
        path=calibration_path,
    )
    assert report["status"] == "valid"
    assert report["arm_joint_count"] == 14
    assert report["gripper_entries"] == ["left_gripper", "right_gripper"]

    wrong_id = validate_calibration_document(
        document,
        robot_id="different_robot",
        path=calibration_path,
    )
    assert wrong_id["status"] == "invalid"
    assert any("does not match robot ID" in error for error in wrong_id["errors"])

    broken = dict(document)
    broken["left_joint0"] = asdict(broken["left_joint0"])
    broken["left_joint0"]["drive_mode"] = 3
    invalid_direction = validate_calibration_document(
        broken,
        robot_id="test_dual_scorpion",
        path=calibration_path,
    )
    assert invalid_direction["status"] == "invalid"
    assert any("drive_mode" in error for error in invalid_direction["errors"])

    broken_gripper = dict(document)
    broken_gripper["left_gripper"] = asdict(broken_gripper["left_gripper"])
    broken_gripper["left_gripper"]["id"] = 9
    invalid_gripper_id = validate_calibration_document(
        broken_gripper,
        robot_id="test_dual_scorpion",
        path=calibration_path,
    )
    assert invalid_gripper_id["status"] == "invalid"
    assert any("left_gripper motor ID 9" in error for error in invalid_gripper_id["errors"])

    broken_gripper["left_gripper"] = dict(broken_gripper["left_gripper"], id=8, range_min=4000, range_max=100)
    invalid_gripper_range = validate_calibration_document(
        broken_gripper,
        robot_id="test_dual_scorpion",
        path=calibration_path,
    )
    assert invalid_gripper_range["status"] == "invalid"
    assert any("left_gripper calibrated range is invalid" in error for error in invalid_gripper_range["errors"])

    broken_gripper["left_gripper"] = dict(
        broken_gripper["left_gripper"],
        range_min=0,
        range_max=4095,
        drive_mode=3,
    )
    invalid_gripper_direction = validate_calibration_document(
        broken_gripper,
        robot_id="test_dual_scorpion",
        path=calibration_path,
    )
    assert invalid_gripper_direction["status"] == "invalid"
    assert any("left_gripper drive_mode" in error for error in invalid_gripper_direction["errors"])


def test_harness_constructs_real_adapter_and_reports_calibration_without_connecting(tmp_path):
    from telegrip.mocopi.agent.config import RobotSpec

    calibration_dir = tmp_path
    _write_calibration(calibration_dir, robot_id="test_dual_scorpion")
    config = HarnessConfig(
        robot_config_path=CONFIG,
        cameras=(),
        authorization_token_file=None,
        max_feedback_age_s=0.5,
        max_plan_frames=240,
        execution_report_dir=tmp_path / "reports",
        robot=RobotSpec(
            robot_id="test_dual_scorpion",
            left_port="/dev/null",
            right_port="/dev/null",
            calibration_dir=calibration_dir,
        ),
    )
    harness = ScorpionAgentHarness(config, kinematics=FakeKinematics())
    status = harness.status()
    assert status["robot"]["configured"] is not False
    assert status["robot"]["auto_home_on_startup"] is False
    assert status["robot"]["initial_state_source"] == "encoder_reading_only"
    assert status["robot"]["calibration"]["status"] == "valid"
    assert status["robot"]["connected"] is False


def test_connect_reads_current_encoders_and_never_homes_or_commands_motors(tmp_path):
    class NoHomeRobot:
        def __init__(self):
            self.connected = False
            self.actions = []

        @property
        def is_connected(self):
            return self.connected

        def validate_calibration(self):
            return {"status": "valid", "errors": []}

        def connect(self):
            self.connected = True

        def disconnect(self, *, disable_torque=False):
            self.connected = False

        def status(self):
            return {"connected": self.connected}

        def read(self):
            return SimpleNamespace(
                connected=True,
                calibrated=True,
                joints_deg={"left": tuple([1.0] * 7), "right": tuple([-1.0] * 7)},
                timestamp_monotonic_ns=123,
            )

    robot = NoHomeRobot()
    harness = _harness(tmp_path, robot=robot)
    result = harness.connect_robot("operator-token")
    assert result["status"] == "connected"
    assert result["startup_behavior"]["auto_home"] is False
    assert result["joints_deg"]["left"] == [1.0] * 7
    assert result["joints_deg"]["right"] == [-1.0] * 7
    assert robot.actions == []


def test_gripper_open_close_set_hold_release_and_read_state(tmp_path):
    robot = FakeCombinedRobot()
    harness = _harness(tmp_path, robot=robot)

    opened = harness.control_gripper(
        action="open",
        arms="left",
        authorization_token="operator-token",
    )
    assert opened["status"] == "complete"
    assert opened["results"][0]["final_position"] == pytest.approx(0.0)

    closed = harness.control_gripper(
        action="close",
        arms="right",
        authorization_token="operator-token",
    )
    assert closed["status"] == "complete"
    assert closed["results"][0]["final_position"] == pytest.approx(45.0)

    set_result = harness.control_gripper(
        action="set_position",
        arms="left",
        position=25.0,
        authorization_token="operator-token",
    )
    assert set_result["results"][0]["final_position"] == pytest.approx(25.0)

    hold_result = harness.control_gripper(
        action="hold",
        arms="left",
        authorization_token="operator-token",
    )
    assert hold_result["results"][0]["final_position"] == pytest.approx(25.0)

    release_result = harness.control_gripper(
        action="release",
        arms="right",
        authorization_token="operator-token",
    )
    assert release_result["results"][0]["final_position"] == pytest.approx(0.0)

    left_state = harness.read_gripper_state("left")
    right_state = harness.read_gripper_state("right")
    assert left_state["position"] == pytest.approx(25.0)
    assert left_state["state"] == "intermediate"
    assert right_state["position"] == pytest.approx(0.0)
    assert right_state["state"] == "open"
    assert left_state["calibration"]["id"] == 8
    assert left_state["calibration"]["normalization"] == "range_0_100"
    assert robot.adapter.follower.left_bus.commands
    assert robot.adapter.follower.right_bus.commands


def test_gripper_rejects_out_of_range_and_operates_left_right_independently(tmp_path):
    robot = FakeCombinedRobot()
    harness = _harness(tmp_path, robot=robot)
    with pytest.raises(ValueError, match="outside"):
        harness.control_gripper(
            action="set_position",
            arms="left",
            position=101.0,
            authorization_token="operator-token",
        )

    result = harness.control_gripper(
        action="set_position",
        arms="both",
        position=20.0,
        authorization_token="operator-token",
    )
    assert result["status"] == "complete"
    assert robot.adapter.follower.left_bus.position == pytest.approx(20.0)
    assert robot.adapter.follower.right_bus.position == pytest.approx(20.0)
    assert robot.adapter.follower.left_bus.commands == [20.0]
    assert robot.adapter.follower.right_bus.commands == [20.0]


def test_gripper_stop_and_communication_errors_are_reported(tmp_path):
    robot = FakeCombinedRobot()
    harness = _harness(tmp_path, robot=robot)
    harness.safety.request_stop()
    stopped = harness.control_gripper(
        action="close",
        arms="both",
        authorization_token="operator-token",
    )
    assert stopped["status"] == "stopped"
    assert robot.adapter.follower.left_bus.commands == []
    assert robot.adapter.follower.right_bus.commands == []
    harness.safety.clear_stop()

    robot.adapter.follower.left_bus.fail_reads = True
    with pytest.raises(ConnectionError, match="fake gripper bus disconnected"):
        harness.control_gripper(
            action="close",
            arms="left",
            authorization_token="operator-token",
        )


def test_grasp_is_unverified_without_threshold_evidence(tmp_path):
    robot = FakeCombinedRobot()
    robot.adapter.follower.left_bus.stall_position = 30.0
    robot.adapter.follower.left_bus.load = 100.0
    robot.adapter.follower.left_bus.current = 100.0
    harness = _harness(tmp_path, robot=robot)
    result = harness.control_gripper(
        action="grasp",
        arms="left",
        authorization_token="operator-token",
    )
    row = result["results"][0]
    assert row["position_reached"] is False
    assert row["grasp_verified"] is False
    assert row["grasp_state"] == "unverified"
    assert row["verification_basis"] == ["encoder_position_only"]


def test_grasp_can_use_configured_load_and_current_threshold_evidence(tmp_path):
    spec = GripperSpec(
        open_position=0.0,
        closed_position=45.0,
        position_tolerance=2.0,
        max_step=50.0,
        settle_s=0.001,
        grasp_timeout_s=0.2,
        load_threshold=10.0,
        current_threshold=10.0,
    )
    robot = FakeCombinedRobot(spec)
    robot.adapter.follower.left_bus.stall_position = 30.0
    robot.adapter.follower.left_bus.load = 100.0
    robot.adapter.follower.left_bus.current = 100.0
    harness = _harness(tmp_path, robot=robot)
    result = harness.control_gripper(
        action="grasp",
        arms="left",
        authorization_token="operator-token",
    )
    row = result["results"][0]
    assert row["position_reached"] is False
    assert row["grasp_verified"] is True
    assert row["grasp_state"] == "threshold_verified"
    assert "configured_present_load_threshold" in row["verification_basis"]


def test_concurrent_gripper_commands_are_serialized_by_the_shared_bus_lock(tmp_path):
    robot = FakeCombinedRobot()
    adapter = robot.adapter
    threads = [
        threading.Thread(target=adapter.set_gripper_position, args=("left", 20.0)),
        threading.Thread(target=adapter.set_gripper_position, args=("left", 40.0)),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=2)
    assert adapter.follower.left_bus.max_active_writes == 1
    assert adapter.follower.left_bus.position in {20.0, 40.0}


def test_task_plan_combines_arm_motion_and_gripper_with_dependencies(tmp_path):
    robot = FakeCombinedRobot()
    harness = _harness(tmp_path, robot=robot)
    plan = _plan()
    plan["steps"] = [
        plan["steps"][0],
        {
            "id": "close_left",
            "skill": "set_gripper",
            "arms": "left",
            "depends_on": ["move"],
            "parameters": {"action": "close"},
        },
    ]
    plan["success_conditions"] = [
        {"type": "gripper_command_complete", "step_id": "close_left"},
    ]
    submitted = harness.submit_task_plan(plan)
    assert submitted["status"] == "valid"

    execution = harness.execute_task_plan(
        submitted["plan_id"],
        "operator-token",
    )
    job = harness.jobs[execution["execution_id"]]
    job.thread.join(timeout=3)
    report = harness.execution_report(execution["execution_id"])
    assert report["status"] == "complete"
    assert [step["step_id"] for step in report["report"]["steps"]] == ["move", "close_left"]
    assert all(step["status"] == "complete" for step in report["report"]["steps"])
    assert robot.adapter.follower.left_bus.position == pytest.approx(45.0)


def test_mcp_exposes_gripper_state_and_control_tools(tmp_path):
    robot = FakeCombinedRobot()
    harness = _harness(tmp_path, robot=robot)
    server = ScorpionMcpServer(harness)
    listed = server.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/list"})
    names = {tool["name"] for tool in listed["result"]["tools"]}
    assert {"read_gripper_state", "control_gripper", "execute_task_plan"} <= names

    state = server.handle(
        {
            "jsonrpc": "2.0",
            "id": 2,
            "method": "tools/call",
            "params": {"name": "read_gripper_state", "arguments": {"arm": "left"}},
        }
    )
    assert state["result"]["structuredContent"]["position"] == pytest.approx(0.0)

    controlled = server.handle(
        {
            "jsonrpc": "2.0",
            "id": 3,
            "method": "tools/call",
            "params": {
                "name": "control_gripper",
                "arguments": {
                    "action": "close",
                    "arms": "left",
                    "authorization_token": "operator-token",
                },
            },
        }
    )
    assert controlled["result"]["structuredContent"]["status"] == "complete"
    assert robot.adapter.follower.left_bus.position == pytest.approx(45.0)


def test_execution_job_can_capture_before_and_after_images(tmp_path):
    from telegrip.mocopi.agent.cameras import CameraService
    from telegrip.mocopi.agent.config import CameraSpec
    from telegrip.mocopi.agent.safety import AuthorizationManager

    config = HarnessConfig(
        robot_config_path=CONFIG,
        cameras=(),
        authorization_token_file=None,
        max_feedback_age_s=0.5,
        max_plan_frames=240,
        execution_report_dir=tmp_path / "reports",
    )
    spec = CameraSpec(camera_id="head", device="/dev/null", width=6, height=4, fps=30)
    harness = ScorpionAgentHarness(
        config,
        robot=FakeRobot(),
        cameras=CameraService((spec,), backend_factory=lambda camera_spec: FakeCamera()),
        kinematics=FakeKinematics(),
        authorization=AuthorizationManager(token="operator-token"),
    )
    plan_result = harness.submit_task_plan(_plan())
    motion = harness.plan_motion(plan_result["plan_id"])
    result = harness.execute_motion(
        motion["motion_plan_id"],
        "operator-token",
        capture_before_after=True,
    )
    job = harness.jobs[result["execution_id"]]
    job.thread.join(timeout=2)
    report = harness.execution_report(result["execution_id"])
    assert report["status"] == "complete"
    assert len(report["before_images"]) == 1
    assert len(report["after_images"]) == 1
    assert Path(report["before_images"][0]["path"]).is_file()
    assert Path(report["after_images"][0]["path"]).is_file()
