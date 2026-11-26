"""Tests for the zero-CAD Scorpion visual mapping wizard backend."""

from pathlib import Path
from unittest.mock import Mock

import pytest

from lelab import hardware, scorpion_mapping, scorpion_view
from lelab.utils import config as cfg


@pytest.fixture
def mock_follower(tmp_path, monkeypatch):
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.robots.dual_scorpion_follower import DualScorpionFollower

    cal_dir = tmp_path / "calibration" / "robots" / "dual_scorpion_follower"
    cal_dir.mkdir(parents=True)
    cal_file = cal_dir / "follower.json"
    cal_file.write_text("{}", encoding="utf-8")

    monkeypatch.setattr(hardware, "HF_LEROBOT_CALIBRATION", tmp_path / "calibration")

    joints = [f"joint{i}" for i in range(7)] + ["gripper"]
    motors = {j: Motor(i + 1, "sts3215", MotorNormMode.DEGREES) for i, j in enumerate(joints)}
    calibration = {j: MotorCalibration(i + 1, 0, 1000, 0, 4095) for i, j in enumerate(joints)}

    robot = Mock(spec=DualScorpionFollower)
    robot.id = "follower"
    robot.calibration_fpath = cal_file
    robot.action_features = {f"{side}_{j}.pos": float for side in ("right", "left") for j in joints}
    robot.left_bus = Mock()
    robot.left_bus.motors = motors
    robot.left_bus.calibration = calibration
    robot.left_bus.model_resolution_table = {"sts3215": 4096}
    robot.right_bus = Mock()
    robot.right_bus.motors = motors
    robot.right_bus.calibration = calibration
    robot.right_bus.model_resolution_table = {"sts3215": 4096}
    robot.get_observation.return_value = {f"{side}_{j}.pos": 12.5 for side in ("right", "left") for j in joints}

    monkeypatch.setattr("lerobot.robots.make_robot_from_config", lambda _: robot)
    return robot


def test_mapping_session_connects_follower_disables_torque_and_reads(client, mock_follower):
    res = client.post(
        "/scorpion-mapping/start",
        json={
            "follower_left_port": "/dev/mock-l",
            "follower_right_port": "/dev/mock-r",
            "follower_config": "follower.json",
        },
    ).json()
    assert res["success"]
    mock_follower.connect.assert_called_once_with(calibrate=False)
    mock_follower.left_bus.disable_torque.assert_called_once()
    mock_follower.right_bus.disable_torque.assert_called_once()

    pos = client.get("/scorpion-mapping/positions").json()
    assert pos["success"] and pos["active"]
    assert pos["positions"]["right_joint0.pos"] == 12.5

    # Concurrent calibration or teleop must be rejected while mapping session holds ports.
    refused = client.post(
        "/start-calibration",
        json={
            "device_type": "robot",
            "port": "",
            "config_file": "follower",
            "robot_model": "dual_scorpion",
        },
    ).json()
    assert not refused["success"]

    stop = client.post("/scorpion-mapping/stop").json()
    assert stop["success"]
    mock_follower.disconnect.assert_called_once()


def test_save_mapping_from_wizard_creates_verified_yaml_and_links_robot(
    client, tmp_path, tmp_lerobot_home, mock_follower, monkeypatch
):
    model_dir = tmp_path / "model"
    (model_dir / "mujoco").mkdir(parents=True)
    xml_copy = model_dir / "mujoco" / "scorpion.xml"
    xml_copy.write_text((scorpion_view.SCORPION_MODEL_DIR / "mujoco" / "scorpion.xml").read_text())
    (model_dir.parent / "telegrip_teleoperation/telegrip/mocopi/dataset").mkdir(parents=True)
    (model_dir.parent / "telegrip_teleoperation/telegrip/mocopi/dataset/hardware_dataset.py").write_text("")
    monkeypatch.setattr(scorpion_mapping, "SCORPION_MODEL_DIR", model_dir)
    monkeypatch.setattr(scorpion_view, "SCORPION_MODEL_DIR", model_dir)
    monkeypatch.setattr(scorpion_mapping, "supports_verified_view_mapping", lambda: True)
    monkeypatch.setattr(scorpion_view, "supports_verified_view_mapping", lambda: True)

    cfg.save_robot_record(
        "scorp1",
        {
            "robot_model": "dual_scorpion",
            "follower_left_port": "/dev/mock-l",
            "follower_right_port": "/dev/mock-r",
            "follower_config": "follower.json",
        },
        allow_create=True,
    )

    joints = {}
    for side in ("right", "left"):
        for i in range(7):
            joints[f"{side}_joint{i}"] = {
                "model_reference_deg": 0.0,
                "motor_reference_deg": 10.0,
                "start_motor_deg": 10.0,
                "moved_motor_deg": 35.0 if side == "left" else -15.0,
                "expected_model_direction": 1,
            }

    res = client.post(
        "/scorpion-mapping/save",
        json={
            "robot_name": "scorp1",
            "follower_left_port": "/dev/mock-l",
            "follower_right_port": "/dev/mock-r",
            "follower_config": "follower.json",
            "joints": joints,
            "view_gripper_deg": {
                "right": {"closed": 0.0, "open": 45.0},
                "left": {"closed": 0.0, "open": 45.0},
            },
        },
    ).json()

    assert res["success"]
    assert res["verified"] is True
    assert res["verification_status"] == "physically_verified"
    assert res["filename"] == "follower-mapping.yaml"

    saved_yaml = Path(model_dir.parent / "config" / "mocopi" / "hardware" / "follower-mapping.yaml")
    assert saved_yaml.is_file()

    record = cfg.get_robot_record("scorp1")
    assert record["view_mapping"] == "follower-mapping.yaml"
    assert record["visual_mapping"]["left"]["sign"] == [1] * 7
    assert record["visual_mapping"]["right"]["sign"] == [-1] * 7

    # Generated YAML must load cleanly through read_view_mapping!
    loaded = scorpion_view.read_view_mapping("follower-mapping.yaml", mock_follower)
    assert loaded["verified"] is True
    assert "follower-mapping.yaml" in client.get("/scorpion-view-mappings").json()["mappings"]
