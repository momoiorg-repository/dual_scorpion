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
from telegrip.mocopi.dataset.fingerprint import dataset_fingerprint, file_sha256
from telegrip.mocopi.dataset.hardware_dataset import (
    JOINT_VERIFICATION_SCHEMA,
    MAPPING_SCHEMA,
    METHOD_MULTI_POSE,
    build_pose_context,
    evaluate_initial_transition,
    execute_joint_episode,
    fit_joint_from_reference,
    fit_joint_mapping,
    fit_joint_verification,
    inspect_joint_mapping,
    load_joint_episode,
    replay_joint_dataset,
    select_reference_frames,
    validate_mapped_episode,
    validate_mapping_document,
    validate_mapping_for_robot,
    verify_dataset_mapping,
    verify_dataset_mapping_jointwise,
)
from telegrip.mocopi.dataset.real import JOINTS
from telegrip.mocopi.dataset.real_dataset import export_hardware_replay_dataset
from telegrip.mocopi.full_mujoco import full_model_path

CONFIG = Path(__file__).resolve().parents[2] / "config/mocopi/tracking.yaml"
LOCAL_DATASET = Path(__file__).resolve().parents[2] / "outputs/mocopi/datasets/episode-001-robot-final12"


def _joint_records(side, signs, offsets, *, verified=True, record_verified=None, motor_ids=None):
    record_verified = verified if record_verified is None else record_verified
    records = {}
    for index, joint in enumerate(JOINTS):
        records[joint] = {
            "schema": JOINT_VERIFICATION_SCHEMA,
            "side": side,
            "joint": joint,
            "motor_id": int(motor_ids[index]) if motor_ids is not None else index + 1,
            "motor_model": "sts3215",
            "method": METHOD_MULTI_POSE,
            "verified": record_verified,
            "model_axis": [1.0, 0.0, 0.0],
            "calibration": {"min_motor_deg": -180.0, "max_motor_deg": 180.0},
            "model_angles_deg": [0.0, 10.0],
            "motor_angles_deg": [float(offsets[index]), float(signs[index] * 10 + offsets[index])],
            "model_range_deg": 10.0,
            "slope": float(signs[index]),
            "sign": int(signs[index]),
            "offset_deg": float(offsets[index]),
            "rms_deg": 0.0,
            "max_residual_deg": 0.0,
            "scale_basis": "fitted",
            "scale_verified": True,
            "evidence": {},
            "failures": [],
        }
    return records


def _mapping_document(
    robot_id,
    calibration_hash,
    signs=None,
    offsets=None,
    *,
    verified=True,
    include_records=True,
    record_verified=None,
    motor_ids=None,
):
    signs = np.ones(7, dtype=int) if signs is None else np.asarray(signs, dtype=int)
    offsets = np.zeros(7) if offsets is None else np.asarray(offsets, dtype=float)
    sides = {}
    for side in ("right", "left"):
        side_doc = {"sign": signs.tolist(), "offset_deg": offsets.tolist()}
        if include_records:
            side_doc["joint_verification"] = _joint_records(
                side,
                signs,
                offsets,
                verified=verified,
                record_verified=record_verified,
                motor_ids=motor_ids,
            )
        sides[side] = side_doc
    return {
        "schema": MAPPING_SCHEMA,
        "robot_id": robot_id,
        "calibration_sha256": calibration_hash,
        "mujoco_model_sha256": file_sha256(full_model_path(load_config(CONFIG)["robot"])),
        "verified": verified,
        "model_kind": "full_mujoco",
        "created_at": "test",
        "verification": {},
        "sides": sides,
    }


def _fake_episode(model_joints, fps=15, preview_step=20.0):
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
            "ik": {"preview_max_joint_step_deg": preview_step},
        },
        info={},
        dataset_sha256="test",
        manifest_sha256="test",
        warning_frames=[],
    )


def _fake_robot(
    tmp_path,
    *,
    stalled=False,
    measurements=None,
    range_min=0,
    range_max=4095,
    current=None,
):
    tmp_path.mkdir(parents=True, exist_ok=True)
    calibration = {
        f"{side}_joint{i}": {
            "id": i + 1,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": range_min,
            "range_max": range_max,
        }
        for side in ("right", "left")
        for i in range(7)
    }
    calibration_path = tmp_path / "robot-test.json"
    calibration_path.write_text(json.dumps(calibration))

    class FakeBus:
        def __init__(self):
            self.is_connected = False
            self.calibration = {
                f"joint{i}": SimpleNamespace(
                    id=i + 1,
                    drive_mode=0,
                    homing_offset=0,
                    range_min=range_min,
                    range_max=range_max,
                )
                for i in range(7)
            }
            self.motors = {f"joint{i}": SimpleNamespace(model="sts3215", id=i + 1) for i in range(7)}
            self.model_resolution_table = {"sts3215": 4096}
            self.torque_enabled = False
            self.disconnect_disable_torque = None

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
            self.disconnect_disable_torque = disable_torque
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
            self.current = {
                f"{side}_joint{i}.pos": float((current or {}).get(side, np.zeros(7))[i])
                for side in ("right", "left")
                for i in range(7)
            }

        @property
        def is_connected(self):
            return self.left_bus.is_connected and self.right_bus.is_connected

        def get_observation(self):
            if self.measurements:
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
        episode, measured, [0, 1, 2], robot_id="robot-test", calibration_hash="hash"
    )
    assert report["status"] == "ok" and document is not None
    for side in ("right", "left"):
        assert document["sides"][side]["sign"] == signs.tolist()
        np.testing.assert_allclose(document["sides"][side]["offset_deg"], offsets, atol=1e-9)
        assert set(document["sides"][side]["joint_verification"]) == set(JOINTS)
        assert all(record["verified"] for record in document["sides"][side]["joint_verification"].values())


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


def test_fit_joint_verification_recovers_sign_offset_and_rejects_bad_ranges():
    record = fit_joint_verification(
        [0.0, 10.0, 20.0],
        [5.0, -5.0, -15.0],
        side="right",
        joint="joint0",
        motor_id=1,
        motor_model="sts3215",
    )
    assert record["verified"] is True
    assert record["sign"] == -1 and record["offset_deg"] == 5.0
    assert record["max_residual_deg"] == 0.0

    short = fit_joint_verification(
        [0.0, 1.0],
        [0.0, 1.0],
        side="right",
        joint="joint0",
        motor_id=1,
        min_model_range_deg=5.0,
    )
    assert short["verified"] is False
    assert any("range" in failure for failure in short["failures"])

    bad = fit_joint_verification(
        [0.0, 10.0, 20.0],
        [0.0, 4.0, 8.0],
        side="right",
        joint="joint0",
        motor_id=1,
        max_residual_deg=1.0,
    )
    assert bad["verified"] is False
    assert any("slope" in failure or "residual" in failure for failure in bad["failures"])


def test_fit_joint_from_reference_requires_bidirectional_motion():
    good = fit_joint_from_reference(
        10.0,
        -5.0,
        -15.0,
        5.0,
        side="right",
        joint="joint0",
        motor_id=1,
        min_test_deg=5.0,
    )
    assert good["verified"] is True and good["sign"] == -1
    assert good["offset_deg"] == 5.0

    same_direction = fit_joint_from_reference(
        10.0,
        0.0,
        10.0,
        8.0,
        side="right",
        joint="joint0",
        motor_id=1,
        min_test_deg=5.0,
    )
    assert same_direction["verified"] is False
    assert any("reverse" in failure for failure in same_direction["failures"])

    too_small = fit_joint_from_reference(
        10.0,
        0.0,
        2.0,
        -2.0,
        side="right",
        joint="joint0",
        motor_id=1,
        min_test_deg=5.0,
    )
    assert too_small["verified"] is False
    assert sum("moved only" in failure for failure in too_small["failures"]) == 2

    other_moved = fit_joint_from_reference(
        10.0,
        0.0,
        10.0,
        -10.0,
        side="right",
        joint="joint0",
        motor_id=1,
        min_test_deg=5.0,
        other_joint_violations={"left_joint0": 12.0},
    )
    assert other_moved["verified"] is False
    assert any("other joints" in failure for failure in other_moved["failures"])


@pytest.mark.skipif(not LOCAL_DATASET.is_dir(), reason="local full-CAD joint dataset is not present")
def test_replay_dataset_refuses_unverified_or_incomplete_mapping(tmp_path):
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
    mapping_path.write_text(
        yaml.safe_dump(
            _mapping_document(
                "robot-test",
                file_sha256(calibration_path),
                verified=False,
                record_verified=False,
            )
        )
    )
    with pytest.raises(ValueError, match="not verified"):
        replay_joint_dataset(
            load_config(CONFIG),
            LOCAL_DATASET,
            mapping_path=mapping_path,
            robot_id="robot-test",
            calibration_dir=tmp_path,
        )

    mapping_path.write_text(
        yaml.safe_dump(
            _mapping_document(
                "robot-test",
                file_sha256(calibration_path),
                include_records=False,
            )
        )
    )
    with pytest.raises(ValueError, match="per-joint"):
        replay_joint_dataset(
            load_config(CONFIG),
            LOCAL_DATASET,
            mapping_path=mapping_path,
            robot_id="robot-test",
            calibration_dir=tmp_path,
        )


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
    before = dataset_fingerprint(LOCAL_DATASET, "mocopi_joint_dataset.json")
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
    assert result["joint_inspection"]["all_joints_verified"] is True
    assert result["pose_context"]["model_zero_deg"]["right"] == [0.0] * 7
    assert result["pose_context"]["calibration_zero"]["right"]["motor_deg"] == [0.0] * 7
    assert len(result["joint_inspection"]["left_right_pairing"]) == 7
    assert result["unverified"]["joints"] == []
    assert dataset_fingerprint(LOCAL_DATASET, "mocopi_joint_dataset.json") == before


def test_dry_run_rejects_missing_or_unverified_joint_records(tmp_path):
    pytest.importorskip("mujoco")
    robot = _fake_robot(tmp_path)
    missing = _mapping_document(
        "robot-test",
        file_sha256(robot.calibration_fpath),
        include_records=False,
    )
    with pytest.raises(ValueError, match="per-joint"):
        validate_mapping_document(missing)

    unverified = _mapping_document(
        "robot-test",
        file_sha256(robot.calibration_fpath),
        verified=True,
        record_verified=False,
    )
    with pytest.raises(ValueError, match="unverified joint"):
        validate_mapping_document(unverified)


def test_mapping_rejects_wrong_motor_id(tmp_path):
    pytest.importorskip("mujoco")
    robot = _fake_robot(tmp_path)
    document = _mapping_document(
        "robot-test",
        file_sha256(robot.calibration_fpath),
        motor_ids=[99] + list(range(2, 8)),
    )
    with pytest.raises(ValueError, match="motor id"):
        validate_mapping_for_robot(document, robot)


def test_validate_mapped_episode_uses_calibration_half_range(tmp_path):
    episode = _fake_episode(
        [
            {"right": np.zeros(7), "left": np.zeros(7)},
            {"right": np.full(7, 90.0), "left": np.full(7, 90.0)},
        ],
        preview_step=180.0,
    )
    robot = _fake_robot(tmp_path, range_min=1000, range_max=3000)
    document = _mapping_document("robot-test", file_sha256(robot.calibration_fpath))
    with pytest.raises(ValueError, match="half-range"):
        validate_mapped_episode(episode, document, robot)


def test_joint_wise_verification_recovers_signs_offsets_and_ids(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    episode = _fake_episode([{side: np.zeros(7) for side in ("right", "left")} for _ in range(2)])
    signs = np.array([1, -1, 1, -1, 1, -1, 1])
    offsets = np.array([2.0, -3.0, 4.0, -5.0, 6.0, -7.0, 8.0])
    reference = {f"{side}_joint{i}.pos": float(offsets[i]) for side in ("right", "left") for i in range(7)}
    measurements = [dict(reference)]
    for side in ("right", "left"):
        for i in range(7):
            positive = dict(reference)
            negative = dict(reference)
            returned = dict(reference)
            positive[f"{side}_joint{i}.pos"] += float(signs[i] * 10.0)
            negative[f"{side}_joint{i}.pos"] -= float(signs[i] * 10.0)
            measurements.extend([positive, negative, returned])
    robot = _fake_robot(tmp_path, measurements=measurements)
    monkeypatch.setattr("builtins.input", lambda prompt="": "y" if "直結" in prompt else "")
    output = tmp_path / "jointwise.yaml"
    report = verify_dataset_mapping_jointwise(
        episode,
        robot,
        output,
        config=load_config(CONFIG),
        reference_frame=0,
        sign_test_deg=5.0,
        report=tmp_path / "jointwise-report.json",
    )
    assert report["status"] == "ok"
    document = yaml.safe_load(output.read_text())
    assert document["verified"] is True
    assert document["verification_method"] == "reference_pose_bidirectional_direction"
    for side in ("right", "left"):
        assert document["sides"][side]["sign"] == signs.tolist()
        np.testing.assert_allclose(document["sides"][side]["offset_deg"], offsets, atol=1e-9)
        for i, joint in enumerate(JOINTS):
            record = document["sides"][side]["joint_verification"][joint]
            assert record["motor_id"] == i + 1
            assert record["scale_verified"] is False
            assert record["verified"] is True


def test_inspection_and_pose_context_distinguish_all_four_poses(tmp_path):
    pytest.importorskip("mujoco")
    episode = _fake_episode(
        [
            {"right": np.zeros(7), "left": np.zeros(7)},
            {"right": np.full(7, 10.0), "left": np.full(7, 10.0)},
        ]
    )
    robot = _fake_robot(tmp_path)
    document = _mapping_document(
        "robot-test",
        file_sha256(robot.calibration_fpath),
        signs=np.array([1, -1, 1, 1, -1, 1, -1]),
        offsets=np.full(7, 20.0),
    )
    config = load_config(CONFIG)
    inspection = inspect_joint_mapping(episode, document, robot, config)
    assert inspection["all_joints_verified"] is True
    assert inspection["sides"]["right"][0]["model_joint"] == "joint0_R"
    assert inspection["sides"]["left"][0]["model_joint"] == "joint0_L"
    assert inspection["sides"]["right"][0]["motor_id"] == 1
    assert inspection["sides"]["right"][0]["model_at_calibration_zero_deg"] == pytest.approx(-20.0)
    assert len(inspection["left_right_pairing"]) == 7

    current = {"right": np.zeros(7) + 1.0, "left": np.zeros(7) + 1.0}
    context = build_pose_context(episode, document, robot, config, current=current)
    assert context["model_zero_deg"]["right"] == [0.0] * 7
    assert context["calibration_zero"]["right"]["model_deg"][0] == pytest.approx(-20.0)
    assert context["dataset_initial_pose"]["right"]["mapped_motor_deg"][0] == pytest.approx(20.0)
    assert context["robot_current_pose"]["right"]["motor_deg"][0] == pytest.approx(1.0)
    assert context["start_error_deg"]["max"] == pytest.approx(19.0)


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
    assert len(report["frames"]) == 3
    assert report["gripper_commanded"] is False
    assert report["control_steps"] > 2
    assert report["initial_transition"]["required"] is False
    assert report["pose_context"]["start_error_deg"]["max"] == 0.0
    assert all("gripper" not in key for command in robot.raw_commands for key in command)
    assert (
        max(
            abs(command[f"right_joint{i}.pos"] - previous[f"right_joint{i}.pos"])
            for command, previous in zip(robot.raw_commands[1:], robot.raw_commands[:-1], strict=True)
            for i in range(7)
        )
        <= 0.5 + 1e-9
    )
    assert report["max_observed_speed_deg_s"] <= 5.0 + 1e-6
    assert robot.right_bus.disconnect_disable_torque is False

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


def test_execute_refuses_start_mismatch_without_explicit_initial_move(tmp_path):
    pytest.importorskip("mujoco")
    episode = _fake_episode(
        [
            {"right": np.zeros(7), "left": np.zeros(7)},
            {"right": np.full(7, 5.0), "left": np.full(7, 4.0)},
            {"right": np.full(7, 10.0), "left": np.full(7, 8.0)},
        ]
    )
    robot = _fake_robot(tmp_path / "refuse", current={"right": np.full(7, -10.0), "left": np.full(7, -10.0)})
    document = {
        "path": "test-mapping",
        "document": _mapping_document("robot-test", file_sha256(robot.calibration_fpath)),
    }
    with pytest.raises(RuntimeError, match="start pose"):
        execute_joint_episode(
            episode,
            document,
            load_config(CONFIG),
            robot,
            tmp_path / "refuse.json",
            realtime=False,
        )
    refused = json.loads((tmp_path / "refuse.json").read_text())
    assert refused["status"] == "aborted"
    assert refused["start_pose"]["max_error_deg"] == pytest.approx(10.0)
    assert refused["hold_sent"] is True
    assert robot.raw_commands
    assert all(abs(value - (-10.0)) < 1e-9 for command in robot.raw_commands for value in command.values())

    allowed = _fake_robot(
        tmp_path / "allowed", current={"right": np.full(7, -10.0), "left": np.full(7, -10.0)}
    )
    allowed_document = {
        "path": "test-mapping",
        "document": _mapping_document("robot-test", file_sha256(allowed.calibration_fpath)),
    }
    report = execute_joint_episode(
        episode,
        allowed_document,
        load_config(CONFIG),
        allowed,
        tmp_path / "allowed.json",
        realtime=False,
        allow_initial_move=True,
    )
    assert report["status"] == "complete"
    assert report["initial_transition"]["applied"] is True
    assert report["initial_transition"]["final_start_error_deg"] <= 2.0
    assert report["pose_context_after_initial_transition"]["start_error_deg"]["max"] <= 2.0
    assert report["max_observed_speed_deg_s"] <= 5.0 + 1e-6


def test_initial_transition_evaluation_reports_modeled_collisions(tmp_path):
    pytest.importorskip("mujoco")
    from telegrip.mocopi.full_mujoco import FullMujocoSimulation

    episode = _fake_episode(
        [
            {"right": np.zeros(7), "left": np.zeros(7)},
            {"right": np.full(7, 10.0), "left": np.full(7, 10.0)},
        ]
    )
    robot = _fake_robot(tmp_path)
    document = _mapping_document("robot-test", file_sha256(robot.calibration_fpath))
    simulation = FullMujocoSimulation(load_config(CONFIG)["robot"], gui=False)
    try:
        current = {"right": np.full(7, -10.0), "left": np.full(7, -10.0)}
        transition = evaluate_initial_transition(
            current,
            episode,
            document,
            robot,
            max_step_deg=0.5,
            max_speed_deg_s=5.0,
            control_hz=20.0,
            simulation=simulation,
        )
        assert transition["collision_checked"] is True
        assert transition["safe"] is True
        assert transition["steps"] >= 40
    finally:
        simulation.close()


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
        config=load_config(CONFIG),
    )
    assert report["status"] == "ok"
    document = yaml.safe_load(output.read_text())
    assert document["verified"] is True and document["robot_id"] == "robot-test"
    for side in ("right", "left"):
        assert document["sides"][side]["sign"] == signs.tolist()
        np.testing.assert_allclose(document["sides"][side]["offset_deg"], offsets, atol=1e-9)
        assert document["sides"][side]["joint_verification"]["joint0"]["motor_id"] == 1
    assert not robot.left_bus.torque_enabled and not robot.right_bus.torque_enabled


@pytest.mark.skipif(not LOCAL_DATASET.is_dir(), reason="local full-CAD joint dataset is not present")
def test_hardware_report_converts_to_lerobot_with_separate_spaces(tmp_path):
    pytest.importorskip("datasets")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    episode = load_joint_episode(LOCAL_DATASET)
    report = {
        "schema": "dual_scorpion.mocopi.hardware_dataset_replay.v1",
        "status": "complete",
        "dataset": str(LOCAL_DATASET),
        "dataset_sha256": episode.dataset_sha256,
        "repo_id": episode.repo_id,
        "episode": 0,
        "robot_id": "robot-test",
        "calibration_sha256": "calibration-test",
        "mapping": "mapping-test",
        "mujoco_model_sha256": episode.metadata["mujoco_model_sha256"],
        "gripper_commanded": False,
        "source_frames": len(episode.frames),
        "control_steps": len(episode.frames) - 1,
        "time_scale": 1.0,
        "frames": [],
    }
    for index, frame in enumerate(episode.frames):
        report["frames"].append(
            {
                "plan_frame": frame.plan_frame,
                "source_frame": frame.source_frame,
                "source_time_s": frame.time_s,
                "elapsed_s": frame.time_s,
                "control_steps": index,
                "arms": {
                    side: {
                        "planned_model_joint_deg": frame.joints[side].tolist(),
                        "mapped_motor_joint_deg": frame.joints[side].tolist(),
                        "sent_motor_joint_deg": frame.joints[side].tolist(),
                        "measured_motor_joint_deg": frame.joints[side].tolist(),
                        "measured_model_joint_deg": frame.joints[side].tolist(),
                        "max_joint_error_deg": 0.0,
                        "tcp_position_error_m": 0.0,
                        "tcp_orientation_error_deg": 0.0,
                        "target_pose_source": "dataset",
                    }
                    for side in ("right", "left")
                },
            }
        )
    report_path = tmp_path / "hardware-report.json"
    report_path.write_text(json.dumps(report))
    output = tmp_path / "hardware-feedback"
    manifest = export_hardware_replay_dataset(
        [(LOCAL_DATASET, report_path)], output, "local/hardware-feedback-test"
    )
    assert manifest["schema"] == "dual_scorpion.mocopi.hardware_replay_feedback_dataset.v1"
    assert manifest["source_report_schema"] == "dual_scorpion.mocopi.hardware_dataset_replay.v1"
    assert manifest["state_semantics"].startswith("Measured encoder")
    assert manifest["commanded_semantics"].startswith("Motor command")
    dataset = LeRobotDataset("local/hardware-feedback-test", root=output)
    assert len(dataset) == len(episode.frames) - 1
    row = dataset[0]
    assert row["observation.state"].shape == (14,)
    assert row["observation.planned.state"].shape == (14,)
    assert row["observation.commanded.state"].shape == (14,)
    assert row["observation.motor.target_state"].shape == (14,)
    assert row["observation.model.measured_state"].shape == (14,)
    assert row["observation.execution.time"].shape == (4,)
    assert row["observation.execution.frame"].shape == (4,)
