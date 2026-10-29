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
