"""Hardware-free tests for the real-robot Dual Scorpion agent harness."""

from __future__ import annotations

import base64
import json
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from telegrip.mocopi.agent.cameras import CameraService
from telegrip.mocopi.agent.config import CameraSpec, HarnessConfig
from telegrip.mocopi.agent.harness import ScorpionAgentHarness
from telegrip.mocopi.agent.mcp import ScorpionMcpServer
from telegrip.mocopi.agent.motion import KinematicsService
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
        }

    def validate(self, *args: Any, **kwargs: Any):
        return {"status": "valid", "errors": []}


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
    )
    return ScorpionAgentHarness(
        config,
        robot=robot or FakeRobot(),
        cameras=None,
        kinematics=FakeKinematics(),
        authorization=None,
    )


def test_plan_schema_rejects_direct_motor_commands_and_unavailable_skills(tmp_path):
    harness = _harness(tmp_path)
    plan = _plan(joint_deg=[0.0] * 7)
    result = harness.submit_task_plan(plan)
    assert result["status"] == "rejected"
    assert any("direct motor fields" in error for error in result["errors"])

    unavailable = _plan()
    unavailable["steps"][0]["parameters"] = {}
    unavailable["steps"][0]["skill"] = "set_gripper"
    unavailable["steps"][0]["parameters"] = {"arm": "left", "closed": True}
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
    for tool_name in ("get_status", "capture_images", "submit_task_plan", "execute_motion", "stop_motion"):
        assert tool_name in (skill_root / "references" / "mcp-tools.md").read_text(encoding="utf-8")
    assert "unavailable" in text


def test_example_agent_config_resolves_paths_without_hardcoding_devices():
    root = Path(__file__).parents[2]
    config = HarnessConfig.from_file(root / "config" / "mocopi" / "agent.example.yaml")
    assert config.robot_config_path == (root / "config" / "mocopi" / "tracking.yaml").resolve()
    assert [camera.camera_id for camera in config.cameras] == ["head", "left_wrist", "right_wrist"]
    assert all(camera.device for camera in config.cameras)
    assert config.authorization_token_file is not None


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
