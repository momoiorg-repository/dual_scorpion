"""The web workflow uses the registered Scorpion devices and real dataset format."""

import math
import xml.etree.ElementTree as ET
from threading import Event
from unittest.mock import Mock

import pytest

from lelab import calibrate, hardware, record, replay, teleoperate
from lelab.scorpion_view import scorpion_joint_frame

PORTS = {
    "leader_left_port": "/dev/mock-leader-l",
    "leader_right_port": "/dev/mock-leader-r",
    "follower_left_port": "/dev/mock-follower-l",
    "follower_right_port": "/dev/mock-follower-r",
}
PAIR = {
    **PORTS,
    "robot_model": "dual_scorpion",
    "leader_config": "leader.json",
    "follower_config": "follower.json",
}


@pytest.fixture
def fake_pair(tmp_path, monkeypatch):
    from lerobot.robots.dual_scorpion_follower import DualScorpionFollower
    from lerobot.teleoperators import Teleoperator

    monkeypatch.setattr(hardware, "HF_LEROBOT_CALIBRATION", tmp_path / "calibration")
    actual = DualScorpionFollower(
        hardware.scorpion_follower(PORTS["follower_left_port"], PORTS["follower_right_port"], "follower")
    )
    positions = dict.fromkeys(actual.action_features, 0.0)
    follower = Mock(spec=DualScorpionFollower)
    follower.name = actual.name
    follower.action_features = actual.action_features
    follower.observation_features = actual.action_features
    follower.cameras = {}
    follower.get_joint_positions.return_value = positions
    follower.get_observation.side_effect = positions.copy
    follower.send_action.side_effect = lambda action: action
    leader = Mock(spec=Teleoperator)
    leader.get_action.side_effect = positions.copy

    monkeypatch.setattr("lerobot.robots.make_robot_from_config", lambda _: follower)
    monkeypatch.setattr("lerobot.teleoperators.make_teleoperator_from_config", lambda _: leader)
    return follower, leader


def test_dual_scorpion_record_config(tmp_path, monkeypatch):
    monkeypatch.setattr(hardware, "HF_LEROBOT_CALIBRATION", tmp_path)
    legacy = tmp_path / "robots/so101_dual_follower"
    legacy.mkdir(parents=True)
    (legacy / "follower.json").write_text("{}")
    config = record.create_record_config(
        record.RecordingRequest(
            **PAIR,
            dataset_repo_id="local/mock",
            single_task="Test",
            cameras={"head": {"type": "opencv", "camera_index": 0, "width": 640, "height": 480, "fps": 30}},
        )
    )
    assert config.robot.type == "dual_scorpion_follower"
    assert config.robot.calibration_dir == legacy
    assert (config.robot.left_arm_port, config.robot.right_arm_port) == (
        PORTS["follower_left_port"],
        PORTS["follower_right_port"],
    )
    assert config.robot.require_calibrated
    assert config.robot.cameras["head"].width == 640
    assert (config.teleop.left_arm_port, config.teleop.right_arm_port) == (
        PORTS["leader_left_port"],
        PORTS["leader_right_port"],
    )
    with pytest.raises(ValueError, match="different serial port"):
        hardware.scorpion_pair(
            teleoperate.TeleoperateRequest(**{**PAIR, "leader_right_port": PORTS["follower_right_port"]})
        )


def test_stored_mapping_allows_raw_view_on_branches_without_mapping_tools(monkeypatch):
    from lelab import scorpion_view

    monkeypatch.setattr(scorpion_view, "supports_verified_view_mapping", lambda: False)
    positions = {"right_joint0.pos": 12.0}
    mapping = scorpion_view.read_view_mapping("prior-branch.yaml", robot=None)

    frame = scorpion_view.scorpion_joint_frame(positions, mapping)

    assert frame["joints"] == {}
    assert frame["motor_positions"] == positions


def test_teleoperation_streams_joints_and_reserves_ports(client, fake_pair):
    follower, leader = fake_pair
    with client.websocket_connect("/ws/joint-data") as websocket:
        assert client.post("/move-arm", json=PAIR).json()["success"]
        update = websocket.receive_json()
        assert update["type"] == "joint_update"
        assert update["robot_model"] == "dual_scorpion"
        assert len(update["motor_positions"]) == 16
        assert update["joints"] == {}
        refused = client.post(
            "/start-calibration",
            json={
                "device_type": "robot",
                "port": "",
                "config_file": "another",
                "robot_model": "dual_scorpion",
            },
        ).json()
        assert not refused["success"]
        assert client.post("/stop-teleoperation").json()["success"]
    follower.connect.assert_called_once_with(calibrate=False)
    leader.connect.assert_called_once_with(calibrate=False)
    follower.disconnect.assert_called_once()
    leader.disconnect.assert_called_once()


def test_replay_rejects_bad_pose_and_then_releases_ports(client, fake_pair, monkeypatch):
    follower, _ = fake_pair
    base = dict.fromkeys(follower.action_features, 0.0)
    poses = [{**base, "left_joint4.pos": 10.0}]
    monkeypatch.setattr(replay, "_episode_actions", lambda *_: (poses, 30))
    episode = {**PAIR, "dataset_repo_id": "local/episode", "episode_index": 0}
    failed = client.post("/start-replay", json=episode).json()
    assert not failed["success"] and "first pose" in failed["message"]
    follower.send_action.assert_not_called()
    follower.disconnect.assert_called_once()

    poses[:] = [base]
    assert client.post("/start-replay", json=episode).json()["success"]
    replay.replay_thread.join(timeout=3)
    follower.send_action.assert_called_once_with(base)
    assert follower.disconnect.call_count == 2
    assert not replay.replay_active


def test_record_and_visualize_dual_scorpion_data(client, tmp_lerobot_home, fake_pair):
    follower, _ = fake_pair
    collected = Event()

    def remember(action):
        # End the take after several real record_loop frames.
        if follower.send_action.call_count >= 4:
            collected.set()
        return action

    follower.send_action.side_effect = remember
    response = client.post(
        "/start-recording",
        json={
            **PAIR,
            "dataset_repo_id": "local/scorpion",
            "single_task": "Pick up the block",
            "num_episodes": 1,
            "episode_time_s": 5,
            "reset_time_s": 0,
            "video": False,
            "fps": 20,
        },
    ).json()
    assert response["success"]
    try:
        assert collected.wait(timeout=5)
        assert client.post("/recording-exit-early").json()["success"]
        record.recording_thread.join(timeout=8)
        assert not record.recording_active
        repo_id = response["dataset_id"]
        report = client.get("/dataset-episode", params={"repo_id": repo_id, "episode_index": 0}).json()
        assert report["success"] and report["robot_type"] == "dual_scorpion_follower"
        frames = client.get(
            "/dataset-scorpion-joints", params={"repo_id": repo_id, "episode_index": 0}
        ).json()["frames"]
        assert len(frames) >= 3
        assert frames[0]["motor_positions"] == dict.fromkeys(follower.action_features, 0.0)
    finally:
        record.stop_recording_and_wait()


def test_dual_calibration_writes_each_bus_and_prefixed_file(tmp_path):
    manager = calibrate.CalibrationManager()
    manager._current_request = calibrate.CalibrationRequest(
        "robot", "", "new-calibration", robot_model="dual_scorpion"
    )
    manager.device = Mock()
    manager.device.calibration_fpath = tmp_path / "new-calibration.json"
    for bus in (manager.device.left_bus, manager.device.right_bus):
        bus.motors = {"joint0": Mock(id=1)}
        bus.set_half_turn_homings.return_value = {"joint0": 50}
    manager._step_homing()
    manager._mins = {"left_joint0": 100, "right_joint0": 200}
    manager._maxes = {"left_joint0": 500, "right_joint0": 600}
    manager._complete_calibration()
    assert manager.device.calibration["right_joint0"].range_min == 200
    assert manager.device.left_bus.write_calibration.call_args.args[0]["joint0"].range_min == 100
    manager.device._save_calibration.assert_called_once()


def test_bundled_urdf_and_measured_coordinates(client):
    response = client.get("/scorpion-urdf/scorpion.urdf")
    assert response.status_code == 200
    urdf = ET.fromstring(response.content)
    names = {joint.attrib["name"] for joint in urdf.findall("./joint")}
    assert {f"joint{i}_{side}" for side in ("L", "R") for i in range(8)} <= names
    mesh = urdf.find(".//visual/geometry/mesh").attrib["filename"]
    assert int(client.head(f"/scorpion-urdf/{mesh}").headers["content-length"]) > 128

    frame = scorpion_joint_frame(
        {"left_joint0.pos": 10, "right_joint0.pos": 10, "left_gripper.pos": 25},
        {
            "sides": {
                "left": {"sign": [-1] * 7, "offset_deg": [40] * 7},
                "right": {"sign": [1] * 7, "offset_deg": [0] * 7},
            },
            "view_gripper_deg": {"left": {"closed": 0, "open": 40}},
        },
    )
    assert frame["joints"] == pytest.approx(
        {"joint0_L": math.radians(30), "joint0_R": math.radians(10), "joint7_L": math.radians(10)}
    )
