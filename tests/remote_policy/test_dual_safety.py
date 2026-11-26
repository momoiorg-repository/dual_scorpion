"""Hardware-free tests of the actual harness and its policy motion authorization."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from lerobot.remote_policy.adapters.dual_scorpion import CAMERAS, FEATURES, JOINTS, SIDES, DualScorpionAdapter
from lerobot.remote_policy.adapters.mock import MockPolicy
from lerobot.remote_policy.core import InferenceBackend, LocalTransport, PolicyClient
from lerobot.remote_policy.schema import ABSOLUTE, PolicyCapabilities, PolicyError

REPO = Path(__file__).resolve().parents[2]


def _scene(tmp_path):
    path = tmp_path / "scene.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "schema": "dual_scorpion.agent.scene.v1",
                "status": "measured",
                "coordinate_frame": "test",
                "source": "test",
                "measured_at": "test",
                "collision_margin_m": 0.002,
                "arm_collision_radius_m": 0.01,
                "robot_to_scene": {"position_m": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]},
                "required_obstacles": ["table", "support_stick"],
                "obstacles": [
                    {
                        "name": "table",
                        "role": "table",
                        "shape": "box",
                        "measurement_status": "measured",
                        "center_m": [0, 0, -1],
                        "half_size_m": [1, 1, 0.01],
                    },
                    {
                        "name": "support_stick",
                        "role": "support_stick",
                        "shape": "cylinder",
                        "measurement_status": "measured",
                        "center_m": [0, 1, 0],
                        "length_m": 0.45,
                        "radius_m": 0.01,
                    },
                ],
            }
        )
    )
    return path


def test_dual_arm_motion_needs_fresh_local_capture_and_exact_one_shot_grant(tmp_path, monkeypatch):
    pytest.importorskip("telegrip")
    pytest.importorskip("pybullet")
    pytest.importorskip("rustypot")
    from telegrip.mocopi.agent.cameras import CameraService
    from telegrip.mocopi.agent.robot import RealRobotAdapter, RobotReading
    from telegrip.mocopi.agent.safety import AuthorizationError
    from telegrip.mocopi.agent.scene import SceneGeometry
    from telegrip.mocopi.config import load_config
    from telegrip.mocopi.dataset.replay import model_fingerprint

    robot_config = REPO / "config/mocopi/tracking.yaml"
    calibration = tmp_path / "safety-test.json"
    calibration.write_text(
        json.dumps(
            {
                f"{side}_{name}": {
                    "id": index,
                    "drive_mode": 0,
                    "homing_offset": 0,
                    "range_min": 0,
                    "range_max": 4095,
                }
                for side in SIDES
                for index, name in enumerate((*JOINTS, "gripper"), 1)
            }
        )
    )
    scene = _scene(tmp_path)
    config = tmp_path / "harness.yaml"
    config.write_text(
        yaml.safe_dump(
            {
                "robot_config": str(robot_config),
                "scene_geometry": str(scene),
                "max_feedback_age_s": 2.0,
                "robot": {
                    "id": calibration.stem,
                    "left_port": "fake-left",
                    "right_port": "fake-right",
                    "calibration_dir": str(tmp_path),
                },
                "cameras": [{"id": name, "device": index} for index, name in enumerate(CAMERAS)],
                "execution_report_dir": str(tmp_path / "reports"),
            }
        )
    )
    binding = {
        "scene_sha256": SceneGeometry.from_file(scene).sha256,
        "calibration_sha256": hashlib.sha256(calibration.read_bytes()).hexdigest(),
        "urdf_sha256": model_fingerprint(load_config(robot_config)),
    }
    moving = []

    def fake_execute(self, plan, robot_config, simulation, report_path, *, stop_event, before_command):
        assert not stop_event.is_set()
        assert len(plan["frames"]) == 3
        assert plan["gripper_included"] is False
        for frame, waypoint in enumerate(plan["frames"]):
            pose = {side: np.array(waypoint["arms"][side]["joint_deg"]) for side in SIDES}
            before_command(frame, time.monotonic_ns(), pose, pose)
        moving.append(plan)
        return {"status": "complete"}

    monkeypatch.setattr(RealRobotAdapter, "connect", lambda self: None)
    monkeypatch.setattr(
        RealRobotAdapter,
        "read",
        lambda self: RobotReading(
            True,
            True,
            dict.fromkeys(SIDES, (0.0,) * 7),
            time.monotonic_ns(),
            {},
            dict.fromkeys(SIDES, 40.0),
        ),
    )
    monkeypatch.setattr(RealRobotAdapter, "execute_plan", fake_execute)
    monkeypatch.setattr(
        CameraService,
        "read_rgb",
        lambda self, camera: (time.monotonic_ns(), np.zeros((2, 2, 3), dtype=np.uint8)),
    )
    robot = DualScorpionAdapter(config=config, verified_arm_binding=binding)
    try:
        observation = robot.get_observation("hold")
        caps = robot.get_capabilities()
        policy = MockPolicy(
            PolicyCapabilities(
                "hold",
                ("dual_scorpion",),
                CAMERAS,
                FEATURES,
                caps.state_units,
                FEATURES,
                caps.action_units,
                2,
                ABSOLUTE,
                30,
            )
        )
        output = PolicyClient(LocalTransport(InferenceBackend(policy))).predict(observation, robot=caps)
        result = robot.validate_action(output, observation)
        assert result.checks["measured_scene_and_self_collision"] == "passed" and not result.blockers
        assert moving == []
        with pytest.raises(AuthorizationError, match="no local operation grant"):
            robot.execute_action(output, observation)
        assert moving == []

        harness = robot._services()
        harness.authorization.issue_grant(
            operation="arm",
            arms=list(SIDES),
            actions=["move_arm_to_pose"],
            plan_id=result.checks["motion_plan_id"],
            expires_in_s=2,
        )
        robot.execute_action(output, observation)
        assert len(moving) == 1
        with pytest.raises(PolicyError, match="fresh read-only capture"):
            robot.execute_action(output, observation)
        assert len(moving) == 1
        assert harness.authorization.status()["active_grant"] is None
    finally:
        robot.close()


def test_pybullet_checks_self_and_table_contacts(tmp_path):
    pytest.importorskip("telegrip")
    pytest.importorskip("pybullet")
    from telegrip.mocopi.agent.motion import KinematicsService, MotionPlanningError
    from telegrip.mocopi.agent.scene import SceneGeometry

    scene_path = _scene(tmp_path)
    config = REPO / "config/mocopi/tracking.yaml"
    start = {side: np.zeros(7) for side in SIDES}
    with KinematicsService(config, scene=SceneGeometry.from_file(scene_path)) as service:
        checker = service.collision_checker
        result = checker.check_pose(start, include_self=True)
        assert result["self_checked"] and result["count"] == 0
        # Make proxies from both arms coincide without changing any devices.
        original_update = checker._update_proxy
        first = next(proxy for proxy in checker.proxy_bodies if proxy["arm"] == "left")

        def overlap(proxy, joints):
            original_update(proxy, joints)
            if proxy["arm"] == "right" and proxy["segment"] == first["segment"]:
                position, orientation = checker.p.getBasePositionAndOrientation(
                    first["body_id"],
                    physicsClientId=checker.client,
                )
                checker.p.resetBasePositionAndOrientation(
                    proxy["body_id"],
                    position,
                    orientation,
                    physicsClientId=checker.client,
                )

        checker._update_proxy = overlap
        with pytest.raises(MotionPlanningError, match="right_segment"):
            service.check_joint_segment(start, start)
    scene = yaml.safe_load(scene_path.read_text())
    scene["obstacles"][0]["center_m"][2] = 0.14
    scene_path.write_text(yaml.safe_dump(scene))
    with (
        KinematicsService(config, scene=SceneGeometry.from_file(scene_path)) as service,
        pytest.raises(MotionPlanningError, match="table"),
    ):
        service.check_joint_segment(start, start)
