"""Demonstration capture, conversion and playback integration tests."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "telegrip_teleoperation"))
from telegrip.mocopi.config import load_config
from telegrip.mocopi.dataset.capture import SCHEMA, capture_session

CONFIG = Path(__file__).resolve().parents[2] / "config/mocopi/tracking.yaml"


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def test_fake_capture_preserves_separate_camera_and_mocopi_clocks(tmp_path):
    cv2 = pytest.importorskip("cv2")
    cfg = load_config(CONFIG)
    cfg["camera"].update(width=96, height=64, fps=20)
    session = tmp_path / "demo"
    metadata = capture_session(cfg, session, 0.35, "pick up a cup", fake=True)
    assert metadata["schema"] == SCHEMA and metadata["status"] == "complete"
    assert metadata["source"] == "fake" and metadata["head_pose_source"] == "mocopi"
    assert metadata["gripper_recorded"] is False and metadata["robot_ik_at_capture"] is False
    assert metadata == json.loads((session / "session.json").read_text())
    images, motion = rows(session / "camera.jsonl"), rows(session / "mocopi.jsonl")
    assert len(images) == metadata["camera_frames"] >= 3
    assert len(motion) == metadata["mocopi_frames"] >= 3
    assert all(np.all(np.diff([x["timestamp_s"] for x in stream]) > 0) for stream in (images, motion))
    assert {"0", "10", "12", "13", "14", "16", "17", "18"} <= motion[0]["bones"].keys()
    assert "gripper" not in json.dumps(motion[0])
    assert cv2.imread(str(session / images[0]["file"])).shape == (64, 96, 3)
    with pytest.raises(FileExistsError):
        capture_session(cfg, session, 0.2, "must not overwrite", fake=True)
    assert json.loads((session / "session.json").read_text())["task"] == "pick up a cup"


def test_capture_failure_is_not_marked_as_complete(tmp_path):
    class BrokenCamera:
        closed = False

        def read(self):
            raise RuntimeError("camera disconnected")

        def close(self):
            self.closed = True

    camera = BrokenCamera()
    path = tmp_path / "broken"
    with pytest.raises(RuntimeError, match="camera disconnected"):
        capture_session(load_config(CONFIG), path, 0.1, "test", fake=True, camera=camera)
    assert camera.closed
    assert json.loads((path / "session.json").read_text())["status"] == "aborted"


def test_gaps_split_episodes_instead_of_compressing_time():
    from types import SimpleNamespace

    from telegrip.mocopi.dataset.session import aligned_segments

    times = np.array([0, 0.05, 0.10, 0.45, 0.50, 0.55])
    segments, report = aligned_segments(SimpleNamespace(camera_times=times, motion_times=times), 20, 0.04)
    assert len(segments) == 2
    assert [len(segment) for segment in segments] == [3, 3]
    assert report == {"grid_samples": 12, "unsynchronized_samples": 6}
    np.testing.assert_allclose([segment[0]["grid_s"] for segment in segments], [0, 0.45])


def test_quaternion_sign_is_continuous_across_180_degrees():
    from scipy.spatial.transform import Rotation
    from telegrip.mocopi.dataset.session import pose_vectors
    from telegrip.mocopi.sources import fake_pair

    previous, quaternions = {}, []
    for degree in (179, 181):
        sample, _ = fake_pair(0, 1)
        sample.bones[14][:3, :3] = Rotation.from_euler("z", degree, degrees=True).as_matrix()
        row = {"bones": {str(k): value.tolist() for k, value in sample.bones.items()}}
        quaternions.append(pose_vectors(row, previous)["left_hand"][3:])
    assert np.dot(*quaternions) > 0.99


def test_quality_metrics_workspace_and_real_lerobot_roundtrip(tmp_path):
    pytest.importorskip("cv2")
    dataset_type = pytest.importorskip("lerobot.datasets.lerobot_dataset").LeRobotDataset
    from telegrip.mocopi.dataset.convert import convert_sessions
    from telegrip.mocopi.dataset.evaluate import session_metrics

    cfg = load_config(CONFIG)
    cfg["camera"].update(width=96, height=64, fps=20)
    session = tmp_path / "raw"
    capture_session(cfg, session, 0.65, "move cup", fake=True)
    workspace = {
        "left": {"min_m": [-10, -10, -10], "max_m": [10, 10, 10]},
        "right": {"min_m": [10, 10, 10], "max_m": [11, 11, 11]},
    }
    report = session_metrics(session, fps=20, max_sync_s=0.08, workspace=workspace)
    assert report["usable_dataset_frames"] >= 4
    assert report["camera_to_mocopi_abs_offset_ms"]["max"] >= 0
    assert report["hands"]["left"]["outside_workspace_frames"] == 0
    assert report["hands"]["right"]["outside_workspace_frames"] == report["mocopi_frames"]
    output = tmp_path / "dataset"
    manifest = convert_sessions([session], output, "local/mocopi-test", fps=20)
    assert not manifest["gripper_recorded"] and not manifest["robot_ik_at_capture"]
    assert manifest["sessions"][0]["source"] == "fake"
    assert manifest["action_semantics"].startswith("next measured human")
    dataset = dataset_type("local/mocopi-test", root=output, video_backend="pyav")
    assert len(dataset) == report["usable_dataset_frames"]
    assert dataset.num_episodes == report["usable_segments"]
    assert dataset[0]["observation.images.head"].shape == (3, 64, 96)
    assert dataset[0]["observation.state"].shape == (49,)
    assert dataset[0]["action"].shape == (14,)
    assert "gripper" not in str(dataset.meta.features).lower()
    assert dataset[0]["task"] == "move cup"
    for index in range(len(dataset) - 1):
        current, next_frame = dataset[index], dataset[index + 1]
        if current["episode_index"] == next_frame["episode_index"]:
            expected = np.concatenate(
                [next_frame["observation.state"][21:28], next_frame["observation.state"][42:49]]
            )
            np.testing.assert_allclose(current["action"], expected, atol=1e-6)
    with pytest.raises(FileExistsError):
        convert_sessions([session], output, "local/mocopi-test", fps=20)

    pytest.importorskip("pybullet")
    pytest.importorskip("mujoco")
    from telegrip.mocopi.dataset.replay import model_fingerprint, replay_sim

    # Replays are derived from human poses, and never add an invented gripper command.
    plan, simulation = replay_sim(cfg, output, gui=False, realtime=False)
    assert simulation["frames"] > 0 and not simulation["ik_failures"]
    assert not plan["gripper_included"] and plan["source"] == "fake"
    assert all(
        len(frame["arms"][side]["joint_deg"]) == 7 for frame in plan["frames"] for side in ("left", "right")
    )
    assert simulation["tracking"]["left"]["position_m"]["max"] >= 0
    assert (
        simulation["tracking"]["right"]["max_joint_step_deg"]["max"]
        <= cfg["robot"]["max_joint_step_deg"] + 1e-6
    )

    # An independently selected/updated URDF is detected and used; no assembly
    # CAD meshes are needed to show the fallback kinematic skeleton.
    original_hash = model_fingerprint(cfg)
    custom = tmp_path / "left_changed.urdf"
    contents = Path(cfg["robot"]["urdf"]["left"]).read_text()
    assert "0.2923 0.0015 0.3967" in contents
    custom.write_text(contents.replace("0.2923 0.0015 0.3967", "0.2933 0.0015 0.3967", 1))
    cfg["robot"]["urdf"]["left"] = str(custom)
    cfg["robot"]["visual_urdf"] = None
    changed, changed_report = replay_sim(cfg, output, gui=False, realtime=False)
    assert changed["urdf_sha256"]["left"] != original_hash["left"]
    assert changed["urdf_sha256"]["right"] == original_hash["right"]
    assert not changed_report["ik_failures"]


def test_real_plan_preflight_and_fake_follower_never_command_gripper(tmp_path):
    from copy import deepcopy
    from types import SimpleNamespace

    pytest.importorskip("pybullet")
    from telegrip.mocopi.dataset.real import JOINTS, execute_plan, validate_plan
    from telegrip.mocopi.dataset.replay import _pose_vector, model_fingerprint
    from telegrip.mocopi.geometry import pose
    from telegrip.mocopi.ik import DualSimulation

    cfg = load_config(CONFIG)
    simulation = DualSimulation(cfg["robot"], gui=False)
    try:
        initial = {side: arm.last_valid_target.copy() for side, arm in simulation.arms.items()}
        plan = {
            "schema": "dual_scorpion.mocopi.robot_plan.v1",
            "source": "fake",
            "fps": 30,
            "expected_frames": 4,
            "urdf_sha256": model_fingerprint(cfg),
            "joint_names": list(JOINTS),
            "gripper_included": False,
            "frames": [],
            "failures": [],
        }
        for index in range(4):
            frame = {"time_s": index / 30, "arms": {}}
            for side, arm in simulation.arms.items():
                joint = initial[side].copy()
                joint[0] += 0.2 * index
                position, quaternion = arm.solver.fk_solver.compute(joint)
                target = _pose_vector(pose(position, quaternion))
                frame["arms"][side] = {
                    "joint_deg": joint[:7].tolist(),
                    "target_pose": target,
                    "simulated_pose": target,
                    "valid": True,
                    "workspace_clamped": False,
                    "retarget_clamped": False,
                }
            plan["frames"].append(frame)
        with pytest.raises(ValueError, match="Synthetic"):
            validate_plan(plan, cfg, simulation)
        assert validate_plan(plan, cfg, simulation, allow_synthetic=True)["frames"] == 4
        bad = deepcopy(plan)
        bad["frames"][1]["arms"]["left"]["joint_deg"][0] += 30
        with pytest.raises(ValueError, match="limit"):
            validate_plan(bad, cfg, simulation, allow_synthetic=True)
        bad = deepcopy(plan)
        bad["urdf_sha256"]["left"] = "changed"
        with pytest.raises(ValueError, match="URDF differs"):
            validate_plan(bad, cfg, simulation, allow_synthetic=True)
        bad = deepcopy(plan)
        del bad["frames"][1]["arms"]["left"]["retarget_clamped"]
        with pytest.raises(ValueError, match="Unsafe"):
            validate_plan(bad, cfg, simulation, allow_synthetic=True)

        class FakeBus:
            def __init__(self):
                self.is_connected = False
                self.calibration = {name: SimpleNamespace(range_min=0, range_max=4095) for name in JOINTS}
                self.motors = {name: SimpleNamespace(model="sts3215") for name in JOINTS}
                self.model_resolution_table = {"sts3215": 4096}
                self.enabled = []

            def connect(self):
                self.is_connected = True

            def read(self, field, name, normalize=False):
                assert field == "Operating_Mode" and name in JOINTS and not normalize
                return 0

            def enable_torque(self, names):
                self.enabled.extend(names)

            def disconnect(self, disable_torque):
                assert not disable_torque
                self.is_connected = False

        class FakeRobot:
            is_calibrated = True

            def __init__(self, stalled=False):
                self.left_bus, self.right_bus = FakeBus(), FakeBus()
                self.config = SimpleNamespace(max_relative_target=0.5, use_degrees=True)
                self.current = {
                    f"{side}_{name}.pos": plan["frames"][0]["arms"][side]["joint_deg"][i]
                    for side in ("left", "right")
                    for i, name in enumerate(JOINTS)
                }
                self.stalled, self.commands = stalled, []

            @property
            def is_connected(self):
                return self.left_bus.is_connected and self.right_bus.is_connected

            def get_observation(self):
                return dict(self.current)

            def send_action(self, action):
                self.commands.append(dict(action))
                if not self.stalled:
                    self.current.update(action)
                return dict(action)

        robot = FakeRobot()
        report = execute_plan(
            plan, cfg, simulation, robot, tmp_path / "real-report.json", realtime=False, allow_synthetic=True
        )
        assert report["status"] == "complete" and len(report["frames"]) == len(plan["frames"])
        assert not report["gripper_commanded"]
        assert robot.left_bus.enabled == list(JOINTS) and robot.right_bus.enabled == list(JOINTS)
        assert all(
            len(command) == 14 and all("gripper" not in key for key in command) for command in robot.commands
        )
        stalled = FakeRobot(stalled=True)
        with pytest.raises(RuntimeError, match="fell behind"):
            execute_plan(
                plan,
                cfg,
                simulation,
                stalled,
                tmp_path / "stalled.json",
                max_tracking_error_deg=0.1,
                allow_synthetic=True,
                realtime=False,
            )
        assert json.loads((tmp_path / "stalled.json").read_text())["status"] == "aborted"
        wrong_start = FakeRobot()
        wrong_start.current["left_joint0.pos"] += 3
        with pytest.raises(RuntimeError, match="start pose"):
            execute_plan(
                plan,
                cfg,
                simulation,
                wrong_start,
                tmp_path / "wrong-start.json",
                allow_synthetic=True,
                realtime=False,
            )
        assert not wrong_start.left_bus.enabled and not wrong_start.commands
    finally:
        simulation.close()


def test_real_adapter_constructs_existing_driver_with_only_seven_motors(tmp_path):
    pytest.importorskip("scservo_sdk")
    pytest.importorskip("serial")
    pytest.importorskip("deepdiff")
    from telegrip.mocopi.dataset.real import JOINTS, arm_only_follower

    calibration = {}
    for side in ("left", "right"):
        for index, name in enumerate((*JOINTS, "gripper"), start=1):
            calibration[f"{side}_{name}"] = {
                "id": index,
                "drive_mode": int(side == "left" and name == "joint1"),
                "homing_offset": 0,
                "range_min": 0,
                "range_max": 4095,
            }
    (tmp_path / "robot-test.json").write_text(json.dumps(calibration))
    robot = arm_only_follower("/dev/not-open-left", "/dev/not-open-right", "robot-test", 0.5, tmp_path)
    for bus in (robot.left_bus, robot.right_bus):
        assert set(bus.motors) == set(JOINTS)
        assert set(bus.calibration) == set(JOINTS)
        assert bus.ids == list(range(1, 8))
        assert not bus.is_connected
    # The production driver's existing drive-mode calibration is preserved.
    assert robot.left_bus._unnormalize({2: 30.0})[2] < 2047
    assert robot.right_bus._unnormalize({2: 30.0})[2] > 2047
