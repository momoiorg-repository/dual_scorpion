"""Full-CAD joint dataset mapping and hardware replay safety tests."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "telegrip_teleoperation"))
from telegrip.mocopi.config import load_config
from telegrip.mocopi.dataset.fingerprint import file_sha256
from telegrip.mocopi.dataset.hardware_dataset import (
    MAPPING_SCHEMA,
    execute_joint_episode,
    fit_joint_mapping,
    load_joint_episode,
    replay_joint_dataset,
    select_reference_frames,
    verify_dataset_mapping,
)
from telegrip.mocopi.full_mujoco import full_model_path

CONFIG = Path(__file__).resolve().parents[2] / "config/mocopi/tracking.yaml"
LOCAL_DATASET = Path(__file__).resolve().parents[2] / "outputs/mocopi/datasets/episode-001-robot-final12"


def _mapping_document(robot_id, calibration_hash, signs=None, offsets=None):
    signs = np.ones(7, dtype=int) if signs is None else np.asarray(signs, dtype=int)
    offsets = np.zeros(7) if offsets is None else np.asarray(offsets, dtype=float)
    return {
        "schema": MAPPING_SCHEMA,
        "robot_id": robot_id,
        "calibration_sha256": calibration_hash,
        "verified": True,
        "model_kind": "full_mujoco",
        "created_at": "test",
        "verification": {},
        "sides": {
            side: {"sign": signs.tolist(), "offset_deg": offsets.tolist()} for side in ("right", "left")
        },
    }


def _fake_episode(model_joints, fps=15):
    frames = []
    for index, joints in enumerate(model_joints):
        frames.append(
            SimpleNamespace(
                plan_frame=index,
                source_frame=index,
                time_s=index / fps,
                joints={side: np.asarray(joints[side], dtype=float) for side in ("right", "left")},
                target_pose={
                    side: np.array([0.25, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]) for side in ("right", "left")
                },
                status=np.array([1, 0, 0, 1, 0, 0], dtype=float),
                collision=np.zeros(2),
                warning=np.zeros(2),
                final_action=False,
            )
        )
    return SimpleNamespace(
        root=Path("."),
        repo_id="local/test",
        episode=0,
        fps=fps,
        frames=frames,
        metadata={
            "mujoco_model_sha256": file_sha256(full_model_path(load_config(CONFIG)["robot"])),
            "obstacles": [],
            "ik": {"preview_max_joint_step_deg": 20.0},
        },
        info={},
        dataset_sha256="test",
        manifest_sha256="test",
        warning_frames=[],
    )


def _fake_robot(tmp_path, *, stalled=False, measurements=None):
    tmp_path.mkdir(parents=True, exist_ok=True)
    calibration = {
        f"{side}_joint{i}": {
            "id": i + 1,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 0,
            "range_max": 4095,
        }
        for side in ("right", "left")
        for i in range(7)
    }
    calibration_path = tmp_path / "robot-test.json"
    calibration_path.write_text(json.dumps(calibration))

    class FakeBus:
        def __init__(self):
            self.is_connected = False
            self.calibration = {f"joint{i}": SimpleNamespace(range_min=0, range_max=4095) for i in range(7)}
            self.motors = {f"joint{i}": SimpleNamespace(model="sts3215") for i in range(7)}
            self.model_resolution_table = {"sts3215": 4096}
            self.torque_enabled = False

        def connect(self):
            self.is_connected = True

        def read(self, field, name, normalize=False):
            assert field == "Operating_Mode" and not normalize
            return 0

        def enable_torque(self, names):
            assert list(names) == [f"joint{i}" for i in range(7)]
            self.torque_enabled = True

        def disable_torque(self, names):
            pass

        def disconnect(self, disable_torque):
            assert not disable_torque
            self.is_connected = False

    class FakeRobot:
        id = "robot-test"
        is_calibrated = True
        calibration_fpath = calibration_path
        config = SimpleNamespace(max_relative_target=0.5, use_degrees=True)

        def __init__(self):
            self.left_bus, self.right_bus = FakeBus(), FakeBus()
            self.stalled = stalled
            self.measurements = list(measurements) if measurements is not None else None
            self.raw_commands = []
            self.current = {f"{side}_joint{i}.pos": 0.0 for side in ("right", "left") for i in range(7)}

        @property
        def is_connected(self):
            return self.left_bus.is_connected and self.right_bus.is_connected

        def get_observation(self):
            if self.measurements is not None:
                return dict(self.measurements.pop(0))
            return dict(self.current)

        def send_action(self, action):
            self.raw_commands.append(dict(action))
            sent = {}
            for key, value in action.items():
                present = self.current[key]
                clipped = float(np.clip(value, present - 0.5, present + 0.5))
                sent[key] = clipped
                if not self.stalled:
                    self.current[key] = clipped
            return sent

    return FakeRobot()


def test_fit_joint_mapping_recovers_signs_and_offsets():
    model = []
    for index in range(3):
        model.append(
            {
                "right": np.array(
                    [
                        10.0 * index,
                        8.0 * index,
                        12.0 * index,
                        6.0 * index,
                        15.0 * index,
                        9.0 * index,
                        11.0 * index,
                    ]
                ),
                "left": np.array(
                    [
                        9.0 * index,
                        10.0 * index,
                        7.0 * index,
                        13.0 * index,
                        8.0 * index,
                        12.0 * index,
                        6.0 * index,
                    ]
                ),
            }
        )
    episode = _fake_episode(model)
    signs = np.array([1, 1, -1, -1, 1, -1, 1])
    offsets = np.array([2, -3, 4, -5, 6, -7, 8], dtype=float)
    measured = [
        {side: signs * episode.frames[index].joints[side] + offsets for side in ("right", "left")}
        for index in (0, 1, 2)
    ]
    document, report = fit_joint_mapping(
        episode,
        measured,
        [0, 1, 2],
        robot_id="robot-test",
        calibration_hash="hash",
    )
    assert report["status"] == "ok" and document is not None
    for side in ("right", "left"):
        assert document["sides"][side]["sign"] == signs.tolist()
        np.testing.assert_allclose(document["sides"][side]["offset_deg"], offsets, atol=1e-9)


def test_fit_joint_mapping_rejects_wrong_correspondence_and_short_range():
    episode = _fake_episode(
        [
            {
                "right": np.zeros(7),
                "left": np.zeros(7),
            },
            {
                "right": np.array([10, 8, 12, 6, 15, 9, 11], dtype=float),
                "left": np.array([9, 10, 7, 13, 8, 12, 6], dtype=float),
            },
            {
                "right": np.array([20, 16, 24, 12, 30, 18, 22], dtype=float),
                "left": np.array([18, 20, 14, 26, 16, 24, 12], dtype=float),
            },
        ]
    )
    wrong = [
        {
            "right": episode.frames[index].joints["left"] * 0.4,
            "left": episode.frames[index].joints["right"] * 0.4,
        }
        for index in (0, 1, 2)
    ]
    document, report = fit_joint_mapping(
        episode, wrong, [0, 1, 2], robot_id="robot-test", calibration_hash="hash"
    )
    assert document is None and report["status"] == "failed"
    assert any("slope" in failure for failure in report["failures"])

    still = _fake_episode([{side: np.zeros(7) for side in ("right", "left")} for _ in range(3)])
    measured = [{side: np.zeros(7) for side in ("right", "left")} for _ in range(3)]
    document, report = fit_joint_mapping(
        still, measured, [0, 1, 2], robot_id="robot-test", calibration_hash="hash"
    )
    assert document is None and any("range" in failure for failure in report["failures"])


@pytest.mark.skipif(not LOCAL_DATASET.is_dir(), reason="local full-CAD joint dataset is not present")
def test_local_episode_loading_and_dry_run(tmp_path):
    episode = load_joint_episode(LOCAL_DATASET)
    assert len(episode.frames) == 249
    assert episode.frames[-1].final_action and episode.frames[-1].target_pose["left"] is None
    calibration_path = tmp_path / "robot-test.json"
    calibration_path.write_text(
        json.dumps(
            {
                f"{side}_joint{i}": {
                    "id": i + 1,
                    "drive_mode": 0,
                    "homing_offset": 0,
                    "range_min": 0,
                    "range_max": 4095,
                }
                for side in ("right", "left")
                for i in range(7)
            }
        )
    )
    mapping_path = tmp_path / "mapping.yaml"
    mapping_path.write_text(yaml.safe_dump(_mapping_document("robot-test", file_sha256(calibration_path))))
    result = replay_joint_dataset(
        load_config(CONFIG),
        LOCAL_DATASET,
        mapping_path=mapping_path,
        robot_id="robot-test",
        calibration_dir=tmp_path,
    )
    assert result["mode"] == "validation_only"
    assert result["frames"] == 249
    assert result["validation"]["preview_step_limit_deg"] == 20.0
    assert result["gripper_commanded"] is False


def test_execute_mapped_episode_uses_interpolation_and_never_commands_gripper(tmp_path):
    pytest.importorskip("mujoco")
    episode = _fake_episode(
        [
            {"right": np.zeros(7), "left": np.zeros(7)},
            {"right": np.array([5.0] * 7), "left": np.array([4.0] * 7)},
            {"right": np.array([10.0] * 7), "left": np.array([8.0] * 7)},
        ]
    )
    robot = _fake_robot(tmp_path)
    document = {
        "path": "test-mapping",
        "document": _mapping_document("robot-test", file_sha256(robot.calibration_fpath)),
    }
    report = execute_joint_episode(
        episode,
        document,
        load_config(CONFIG),
        robot,
        tmp_path / "real.json",
        realtime=False,
    )
    assert report["status"] == "complete"
    assert len(report["frames"]) == 2
    assert report["gripper_commanded"] is False
    assert report["control_steps"] > 2
    assert all("gripper" not in key for command in robot.raw_commands for key in command)
    assert (
        max(
            abs(command[f"right_joint{i}.pos"] - previous[f"right_joint{i}.pos"])
            for command, previous in zip(robot.raw_commands[1:], robot.raw_commands[:-1], strict=True)
            for i in range(7)
        )
        <= 0.5 + 1e-9
    )

    stalled = _fake_robot(tmp_path / "stalled", stalled=True)
    with pytest.raises(RuntimeError, match="fell behind"):
        execute_joint_episode(
            episode,
            {
                "path": "test-mapping",
                "document": _mapping_document("robot-test", file_sha256(stalled.calibration_fpath)),
            },
            load_config(CONFIG),
            stalled,
            tmp_path / "stalled.json",
            realtime=False,
        )
    stalled_report = json.loads((tmp_path / "stalled.json").read_text())
    assert stalled_report["status"] == "aborted" and stalled_report["failure"]


def test_select_reference_frames_is_spread_and_unique():
    frames = [SimpleNamespace() for _ in range(20)]
    assert select_reference_frames(frames) == [0, 5, 10, 14, 19]
    with pytest.raises(ValueError, match="three"):
        select_reference_frames(frames[:2])


def test_verify_dataset_mapping_reads_manual_poses_without_enabling_torque(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    model = []
    for index in range(3):
        model.append(
            {
                "right": np.array(
                    [
                        10.0 * index,
                        8.0 * index,
                        12.0 * index,
                        6.0 * index,
                        15.0 * index,
                        9.0 * index,
                        11.0 * index,
                    ]
                ),
                "left": np.array(
                    [
                        9.0 * index,
                        10.0 * index,
                        7.0 * index,
                        13.0 * index,
                        8.0 * index,
                        12.0 * index,
                        6.0 * index,
                    ]
                ),
            }
        )
    episode = _fake_episode(model)
    signs = np.array([1, 1, -1, -1, 1, -1, 1])
    offsets = np.array([2, -3, 4, -5, 6, -7, 8], dtype=float)
    measurements = []
    for index in (0, 1, 2):
        measurements.append(
            {
                f"{side}_joint{i}.pos": float(signs[i] * episode.frames[index].joints[side][i] + offsets[i])
                for side in ("right", "left")
                for i in range(7)
            }
        )
    robot = _fake_robot(tmp_path, measurements=measurements)
    monkeypatch.setattr("builtins.input", lambda prompt="": "")
    output = tmp_path / "verified.yaml"
    report = verify_dataset_mapping(
        episode,
        robot,
        output,
        reference_frames=[0, 1, 2],
        report=tmp_path / "verification.json",
    )
    assert report["status"] == "ok"
    document = yaml.safe_load(output.read_text())
    assert document["verified"] is True and document["robot_id"] == "robot-test"
    for side in ("right", "left"):
        assert document["sides"][side]["sign"] == signs.tolist()
        np.testing.assert_allclose(document["sides"][side]["offset_deg"], offsets, atol=1e-9)
    assert not robot.left_bus.torque_enabled and not robot.right_bus.torque_enabled
