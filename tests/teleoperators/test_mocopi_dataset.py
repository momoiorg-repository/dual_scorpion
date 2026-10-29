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
