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
from telegrip.mocopi.dataset.__main__ import main as mocopi_dataset_cli
from telegrip.mocopi.dataset.fingerprint import dataset_fingerprint, file_sha256
from telegrip.mocopi.dataset.hardware_dataset import (
    DEFAULT_FULL_CAD_SIGNS_DRIVE_MODE_0,
    FT_DATASET_SCHEMA,
    JOINT_VERIFICATION_SCHEMA,
    MAPPING_SCHEMA,
    METHOD_CALIBRATION_MIDPOINT,
    METHOD_MULTI_POSE,
    METHOD_PER_JOINT_MULTI_ANGLE,
    METHOD_PER_JOINT_REFERENCE,
    METHOD_PER_JOINT_UNIDIRECTIONAL,
    METHOD_UNMEASURED,
    build_calibration_mapping,
    build_pose_context,
    evaluate_initial_transition,
    execute_joint_episode,
    export_ft_dataset,
    fit_joint_from_reference,
    fit_joint_from_unidirectional_reference,
    fit_joint_mapping,
    fit_joint_verification,
    inspect_dataset_mapping,
    inspect_joint_mapping,
    load_joint_episode,
    replay_joint_dataset,
    select_reference_frames,
    validate_mapped_episode,
    validate_mapping_document,
    validate_mapping_for_robot,
    verify_dataset_mapping,
    verify_dataset_mapping_jointwise,
    verify_joint_direction_unidirectional,
)
from telegrip.mocopi.dataset.real import JOINTS, arm_only_follower
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
    initial_torque_enabled=False,
    is_calibrated=True,
    drive_mode=0,
    homing_offset=0,
    left_connect_error=None,
):
    tmp_path.mkdir(parents=True, exist_ok=True)
    calibration = {
        f"{side}_joint{i}": {
            "id": i + 1,
            "drive_mode": drive_mode,
            "homing_offset": homing_offset,
            "range_min": range_min,
            "range_max": range_max,
        }
        for side in ("right", "left")
        for i in range(7)
    }
    calibration_path = tmp_path / "robot-test.json"
    calibration_path.write_text(json.dumps(calibration))

    class FakeBus:
        def __init__(self, *, connect_error=None):
            self.is_connected = False
            self.is_calibrated = is_calibrated
            self._connect_error = connect_error
            self.calibration = {
                f"joint{i}": SimpleNamespace(
                    id=i + 1,
                    drive_mode=drive_mode,
                    homing_offset=homing_offset,
                    range_min=range_min,
                    range_max=range_max,
                )
                for i in range(7)
            }
            self.motors = {f"joint{i}": SimpleNamespace(model="sts3215", id=i + 1) for i in range(7)}
            self.model_resolution_table = {"sts3215": 4096}
            self.torque_enabled = initial_torque_enabled
            self.disconnect_disable_torque = None
            self.enable_torque_calls = []
            self.disable_torque_calls = []
            self.disconnect_calls = []

        def connect(self):
            if self._connect_error is not None:
                raise self._connect_error
            self.is_connected = True

        def read(self, field, name, normalize=False):
            assert field == "Operating_Mode" and not normalize
            assert name != "gripper"
            return 0

        def enable_torque(self, names):
            names_list = list(names)
            assert names_list == [f"joint{i}" for i in range(7)]
            assert "gripper" not in names_list
            self.enable_torque_calls.append(names_list)
            self.torque_enabled = True

        def disable_torque(self, names=None):
            names_list = list(names) if names is not None else None
            if names_list is not None:
                assert "gripper" not in names_list
            self.disable_torque_calls.append(names_list)
            self.torque_enabled = False

        def disconnect(self, disable_torque=True):
            self.disconnect_disable_torque = disable_torque
            self.disconnect_calls.append(disable_torque)
            if disable_torque:
                self.torque_enabled = False
            self.is_connected = False

    class FakeRobot:
        id = "robot-test"
        calibration_fpath = calibration_path
        config = SimpleNamespace(max_relative_target=0.5, use_degrees=True)

        def __init__(self):
            self.left_bus = FakeBus(connect_error=left_connect_error)
            self.right_bus = FakeBus()
            self.stalled = stalled
            self.measurements = list(measurements) if measurements is not None else None
            self.raw_commands = []
            self.current = {
                f"{side}_joint{i}.pos": float((current or {}).get(side, np.zeros(7))[i])
                for side in ("right", "left")
                for i in range(7)
            }

        @property
        def is_calibrated(self):
            return is_calibrated

        @property
        def is_connected(self):
            return self.left_bus.is_connected and self.right_bus.is_connected

        def get_observation(self):
            if self.measurements:
                return dict(self.measurements.pop(0))
            return dict(self.current)

        def send_action(self, action):
            assert all("gripper" not in key for key in action)
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


def _make_joint_dataset(src_root, model_hash):
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    # Build a synthetic source kinematic joint dataset (14-dim q_model + head image + kinematic cols).
    joint_names = [f"{s}_joint{i}.pos" for s in ("right", "left") for i in range(7)]
    src_ds = LeRobotDataset.create(
        repo_id="local/src-joint-test",
        fps=15,
        features={
            "observation.images.head": {
                "dtype": "video",
                "shape": (3, 48, 64),
                "names": ["channels", "height", "width"],
            },
            "observation.state": {"dtype": "float32", "shape": (14,), "names": joint_names},
            "action": {"dtype": "float32", "shape": (14,), "names": joint_names},
            "observation.robot.target_pose": {"dtype": "float32", "shape": (14,), "names": None},
            "observation.robot.status": {"dtype": "float32", "shape": (6,), "names": None},
            "observation.robot.collision": {"dtype": "float32", "shape": (2,), "names": None},
            "observation.robot.warning": {"dtype": "float32", "shape": (2,), "names": None},
            "observation.sim.frame": {"dtype": "int64", "shape": (2,), "names": None},
            "observation.robot.plan_time_s": {"dtype": "float64", "shape": (1,), "names": None},
        },
        root=src_root,
        robot_type="dual_scorpion_kinematic",
        use_videos=True,
        rgb_encoder=RGBEncoderConfig(vcodec="h264"),
    )
    q_frames = [
        np.zeros(14, dtype=np.float32),
        np.linspace(1.0, 7.0, 14, dtype=np.float32),
        np.linspace(2.0, 10.0, 14, dtype=np.float32),
        np.linspace(3.0, 12.0, 14, dtype=np.float32),
    ]
    for idx, (s0, s1) in enumerate(zip(q_frames[:-1], q_frames[1:], strict=True)):
        img = np.full((3, 48, 64), 0.5, dtype=np.float32)
        src_ds.add_frame(
            {
                "task": "pick cup",
                "observation.images.head": img,
                "observation.state": s0,
                "action": s1,
                "observation.robot.target_pose": np.tile(
                    np.array([0.25, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0], dtype=np.float32), 2
                ),
                "observation.robot.status": np.array([1, 0, 0, 1, 0, 0], dtype=np.float32),
                "observation.robot.collision": np.zeros(2, dtype=np.float32),
                "observation.robot.warning": np.zeros(2, dtype=np.float32),
                "observation.sim.frame": np.array([idx, idx], dtype=np.int64),
                "observation.robot.plan_time_s": np.array([idx / 15.0], dtype=np.float64),
            }
        )
    src_ds.save_episode()
    src_ds.finalize()
    src_meta = {
        "schema": "dual_scorpion.mocopi.kinematic_joint_dataset.v1",
        "repo_id": "local/src-joint-test",
        "model_kind": "full_mujoco",
        "robot_state_source": "kinematic_simulation_not_real_encoders",
        "joint_names": joint_names,
        "joint_unit": "degree",
        "gripper_recorded": False,
        "gripper_commanded": False,
        "gripper_feature_included": False,
        "quaternion_order": "xyzw",
        "mujoco_model_sha256": model_hash,
        "obstacles": [],
        "ik": {"preview_max_joint_step_deg": 20.0},
    }
    (src_root / "meta" / "mocopi_joint_dataset.json").write_text(json.dumps(src_meta), encoding="utf-8")
    return src_meta, q_frames


def test_replay_dataset_refuses_unverified_or_incomplete_mapping(tmp_path):
    pytest.importorskip("datasets")
    model_hash = file_sha256(full_model_path(load_config(CONFIG)["robot"]))
    dataset = tmp_path / "dataset"
    _make_joint_dataset(dataset, model_hash)
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
            dataset,
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
            dataset,
            mapping_path=mapping_path,
            robot_id="robot-test",
            calibration_dir=tmp_path,
        )


@pytest.mark.parametrize("use_local_dataset", [False, True])
def test_local_episode_loading_and_dry_run(tmp_path, use_local_dataset):
    pytest.importorskip("datasets")
    config = load_config(CONFIG)
    model_hash = file_sha256(full_model_path(config["robot"]))
    dataset = LOCAL_DATASET if use_local_dataset else tmp_path / "dataset"
    if use_local_dataset:
        if not dataset.is_dir():
            pytest.skip("local full-CAD joint dataset is not present")
    else:
        _make_joint_dataset(dataset, model_hash)
    episode = load_joint_episode(dataset)
    assert len(episode.frames) == (249 if use_local_dataset else 4)
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
    before = dataset_fingerprint(dataset, "mocopi_joint_dataset.json")
    if episode.metadata["mujoco_model_sha256"] != model_hash:
        with pytest.raises(ValueError, match="MuJoCo model differs"):
            replay_joint_dataset(config, dataset, mapping_path=mapping_path, robot_id="robot-test", calibration_dir=tmp_path)
        assert dataset_fingerprint(dataset, "mocopi_joint_dataset.json") == before
        return
    result = replay_joint_dataset(
        config,
        dataset,
        mapping_path=mapping_path,
        robot_id="robot-test",
        calibration_dir=tmp_path,
    )
    assert result["mode"] == "validation_only"
    assert result["frames"] == len(episode.frames)
    assert result["validation"]["preview_step_limit_deg"] == 20.0
    assert result["gripper_commanded"] is False
    assert result["joint_inspection"]["all_joints_verified"] is True
    assert result["pose_context"]["model_zero_deg"]["right"] == [0.0] * 7
    assert result["pose_context"]["calibration_zero"]["right"]["motor_deg"] == [0.0] * 7
    assert len(result["joint_inspection"]["left_right_pairing"]) == 7
    assert result["unverified"]["joints"] == []
    assert dataset_fingerprint(dataset, "mocopi_joint_dataset.json") == before


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
    # Each joint is measured independently at its own known model angle while other joints stay in arbitrary poses.
    model_refs = np.array([0.0, 15.0, -20.0, 90.0, 0.0, 30.0, -10.0])
    measurements = []
    prompts_seen = []
    for side in ("right", "left"):
        for i in range(7):
            # Arbitrary non-zero base pose (unrelated to any dataset whole-body frame).
            arbitrary_base = {
                f"{s}_joint{j}.pos": float(17.0 * (s_i + 1) - 3.5 * j)
                for s_i, s in enumerate(("right", "left"))
                for j in range(7)
            }
            positive = dict(arbitrary_base)
            negative = dict(arbitrary_base)
            positive[f"{side}_joint{i}.pos"] += float(signs[i] * 10.0)
            negative[f"{side}_joint{i}.pos"] -= float(signs[i] * 10.0)
            reference_pose = dict(arbitrary_base)
            reference_pose[f"{side}_joint{i}.pos"] = float(signs[i] * model_refs[i] + offsets[i])
            measurements.extend([arbitrary_base, positive, negative, reference_pose])

    angle_iter = iter([str(model_refs[i]) for _side in ("right", "left") for i in range(7)])

    def _fake_input(prompt=""):
        prompts_seen.append(prompt)
        if "トルクを解除" in prompt or "直結" in prompt:
            return "y"
        if "既知のモデル角度" in prompt:
            return next(angle_iter)
        if "測定根拠" in prompt:
            return "CAD基準/90度スコヤ実測"
        return ""

    robot = _fake_robot(tmp_path, measurements=measurements, initial_torque_enabled=True)
    monkeypatch.setattr("builtins.input", _fake_input)
    output = tmp_path / "jointwise.yaml"
    report = verify_dataset_mapping_jointwise(
        episode,
        robot,
        output,
        config=load_config(CONFIG),
        sign_test_deg=5.0,
        report=tmp_path / "jointwise-report.json",
    )
    assert report["status"] == "ok"
    assert not any("参照フレーム" in p for p in prompts_seen)
    assert not any("全身" in p and "不要" not in p for p in prompts_seen)
    assert robot.right_bus.disable_torque_calls and robot.left_bus.disable_torque_calls
    document = yaml.safe_load(output.read_text())
    assert document["verified"] is True
    assert document["verification_method"] == METHOD_PER_JOINT_REFERENCE
    for side in ("right", "left"):
        assert document["sides"][side]["sign"] == signs.tolist()
        np.testing.assert_allclose(document["sides"][side]["offset_deg"], offsets, atol=1e-9)
        for i, joint in enumerate(JOINTS):
            record = document["sides"][side]["joint_verification"][joint]
            assert record["motor_id"] == i + 1
            assert record["scale_verified"] is False
            assert record["verified"] is True
            assert record["verification_evidence"]["measurement_basis"] == "CAD基準/90度スコヤ実測"


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
    robot = _fake_robot(tmp_path, measurements=measurements, initial_torque_enabled=True)
    monkeypatch.setattr("builtins.input", lambda prompt="": "y" if "トルクを解除" in prompt else "")
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
    assert not robot.left_bus.enable_torque_calls and not robot.right_bus.enable_torque_calls
    assert robot.left_bus.disable_torque_calls and robot.right_bus.disable_torque_calls
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


def _measurement_entry(sign, offset, *, model_ref=90.0, multi_angle=False, basis="90度スコヤ実測"):
    if multi_angle:
        angles = [-15.0, 30.0]
        return {
            "direction_test": {
                "baseline_motor_deg": float(offset),
                "positive_motor_deg": float(offset + sign * 12.0),
                "negative_motor_deg": float(offset - sign * 12.0),
            },
            "model_angles_deg": angles,
            "motor_angles_deg": [float(sign * a + offset) for a in angles],
            "measurement_basis": basis,
        }
    return {
        "direction_test": {
            "baseline_motor_deg": float(offset + 15.0),
            "positive_motor_deg": float(offset + 15.0 + sign * 12.0),
            "negative_motor_deg": float(offset + 15.0 - sign * 12.0),
        },
        "direct_drive_confirmed": True,
        "measurement_basis": basis,
        "model_reference_deg": float(model_ref),
        "motor_reference_deg": float(sign * model_ref + offset),
    }


def _install_mock_feetech_bus(
    monkeypatch,
    tmp_path,
    *,
    robot_id="robot-mock",
    initial_torque_enabled=False,
    is_calibrated=True,
    second_connect_error=None,
    current=None,
):
    from lerobot.motors.motors_bus import Motor, MotorNormMode

    cal_dir = tmp_path / "calibrations"
    cal_dir.mkdir(parents=True, exist_ok=True)
    cal_data = {
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
    cal_data["right_gripper"] = {
        "id": 8,
        "drive_mode": 0,
        "homing_offset": 0,
        "range_min": 0,
        "range_max": 4095,
    }
    cal_data["left_gripper"] = dict(cal_data["right_gripper"])
    (cal_dir / f"{robot_id}.json").write_text(json.dumps(cal_data), encoding="utf-8")

    created_buses = []
    default_pose = {f"joint{i}": float((current or {}).get(i, 5.0 * (i + 1))) for i in range(7)}

    class MockFeetechMotorsBus:
        def __init__(self, port, motors, calibration=None):
            self.port = port
            self.motors = dict(motors)
            self.calibration = dict(calibration or {})
            self.is_connected = False
            self.model_resolution_table = {"sts3215": 4096}
            self.torque_enabled = initial_torque_enabled
            self.enable_torque_calls = []
            self.disable_torque_calls = []
            self.disconnect_calls = []
            self.sync_read_calls = []
            self.sync_write_calls = []
            self.read_calls = []
            self.write_calls = []
            self.pose = dict(default_pose)
            self.pose["gripper"] = 0.0
            created_buses.append(self)

        @property
        def is_calibrated(self):
            return is_calibrated

        def connect(self):
            if second_connect_error is not None and any(
                bus.is_connected for bus in created_buses if bus is not self
            ):
                raise second_connect_error
            self.is_connected = True

        def disconnect(self, disable_torque=True):
            self.disconnect_calls.append(disable_torque)
            if disable_torque:
                self.disable_torque(None)
            self.is_connected = False

        def enable_torque(self, motors=None):
            self.enable_torque_calls.append(motors)
            assert motors != "gripper" and (motors is None or "gripper" not in motors)
            self.torque_enabled = True

        def disable_torque(self, motors=None):
            self.disable_torque_calls.append(motors)
            assert motors != "gripper" and (motors is None or "gripper" not in motors)
            self.torque_enabled = False

        def sync_read(self, data_name, motors=None):
            self.sync_read_calls.append(
                (data_name, list(motors) if motors is not None else list(self.motors))
            )
            target_names = list(motors) if motors is not None else list(self.motors)
            assert "gripper" not in target_names
            return {name: self.pose[name] for name in target_names}

        def sync_write(self, data_name, values, *, normalize=True):
            self.sync_write_calls.append((data_name, dict(values), normalize))
            assert "gripper" not in values
            for name, val in values.items():
                self.pose[name] = float(val)

        def read(self, data_name, motor, *, normalize=True):
            self.read_calls.append((data_name, motor, normalize))
            assert motor != "gripper"
            if data_name == "Operating_Mode":
                return 0
            return self.pose[motor]

        def write(self, data_name, motor, value, *, normalize=True):
            self.write_calls.append((data_name, motor, value, normalize))
            assert motor != "gripper"

        def configure_motors(self):
            pass

        def write_calibration(self, cal):
            pass

    monkeypatch.setattr("lerobot.motors.feetech.FeetechMotorsBus", MockFeetechMotorsBus)
    # Verify real Motor / MotorNormMode types remain importable cleanly alongside mock.
    assert isinstance(Motor(1, "sts3215", MotorNormMode.DEGREES), Motor)
    return cal_dir, created_buses


def test_urdf_zero_differs_from_calibration_zero_and_measurements_mode_recovers_offsets(tmp_path):
    pytest.importorskip("mujoco")
    robot = _fake_robot(tmp_path, current={"right": np.full(7, 42.0), "left": np.full(7, -33.0)})
    config = load_config(CONFIG)
    signs = {"right": [1, -1, 1, -1, 1, -1, 1], "left": [-1, 1, -1, 1, -1, 1, -1]}
    offsets = {
        "right": [12.0, -18.0, 25.0, -40.0, 7.5, -14.0, 31.0],
        "left": [-22.0, 15.0, -9.0, 35.0, -11.5, 19.0, -27.0],
    }
    measurements_file = tmp_path / "measurements.yaml"
    entries = {}
    for side in ("right", "left"):
        for idx in range(7):
            entries[f"{side}_joint{idx}"] = _measurement_entry(
                signs[side][idx],
                offsets[side][idx],
                model_ref=90.0 if idx == 3 else (15.0 if idx % 2 else 0.0),
                multi_angle=(idx % 3 == 0),
                basis="CAD基準と90度スコヤ" if idx == 3 else "CADゲージ実測",
            )
    measurements_file.write_text(yaml.safe_dump({"joints": entries}, sort_keys=False), encoding="utf-8")

    output = tmp_path / "verified-measurements.yaml"
    report = verify_dataset_mapping_jointwise(
        None,
        robot,
        output,
        config=config,
        measurements=measurements_file,
    )
    assert report["status"] == "ok"
    assert not robot.right_bus.enable_torque_calls and not robot.left_bus.enable_torque_calls
    document = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert document["verified"] is True
    for side in ("right", "left"):
        assert document["sides"][side]["sign"] == signs[side]
        np.testing.assert_allclose(document["sides"][side]["offset_deg"], offsets[side], atol=1e-9)
        assert (
            document["sides"][side]["joint_verification"]["joint0"]["method"] == METHOD_PER_JOINT_MULTI_ANGLE
        )
        assert (
            document["sides"][side]["joint_verification"]["joint3"]["method"] == METHOD_PER_JOINT_MULTI_ANGLE
        )
        assert document["sides"][side]["joint_verification"]["joint1"]["method"] == METHOD_PER_JOINT_REFERENCE
        # Verify URDF/Model Zero maps cleanly to non-zero Calibration offsets, distinct from current pose.
        episode = _fake_episode([{"right": np.zeros(7), "left": np.zeros(7)}])
        ctx = build_pose_context(
            episode,
            document,
            robot,
            config,
            current={"right": np.full(7, 42.0), "left": np.full(7, -33.0)},
        )
        np.testing.assert_allclose(ctx["model_zero_deg"][side], np.zeros(7))
        np.testing.assert_allclose(ctx["calibration_zero"][side]["motor_deg"], np.zeros(7))
        np.testing.assert_allclose(
            ctx["dataset_initial_pose"][side]["mapped_motor_deg"], offsets[side], atol=1e-9
        )
        assert not np.allclose(
            ctx["dataset_initial_pose"][side]["mapped_motor_deg"],
            ctx["robot_current_pose"][side]["motor_deg"],
        )


def test_partial_mapping_and_unmeasured_joint_reference_never_guessed(tmp_path):
    pytest.importorskip("mujoco")
    robot = _fake_robot(tmp_path, current={"right": np.full(7, 55.0), "left": np.full(7, -45.0)})
    config = load_config(CONFIG)
    episode = _fake_episode(
        [{"right": np.zeros(7), "left": np.zeros(7)}, {"right": np.ones(7), "left": np.ones(7)}]
    )

    partial_measurements = {
        "joints": {
            "right_joint3": _measurement_entry(-1, 22.5, model_ref=90.0, basis="90度治具"),
            "right_joint4": {"unmeasured": True, "reason": "治具未製作のため未測定"},
            "right_joint5": {
                "direction_test": {
                    "baseline_motor_deg": 10.0,
                    "positive_motor_deg": 25.0,
                    "negative_motor_deg": -5.0,
                },
                "reason": "符号のみ確認・絶対ゼロ基準未測定",
            },
        }
    }
    partial_path = tmp_path / "partial.yaml"
    with pytest.raises(ValueError, match="failed"):
        verify_dataset_mapping_jointwise(
            episode,
            robot,
            tmp_path / "should-not-write.yaml",
            config=config,
            measurements=partial_measurements,
            allow_partial=False,
        )

    report = verify_dataset_mapping_jointwise(
        episode,
        robot,
        partial_path,
        config=config,
        measurements=partial_measurements,
        allow_partial=True,
    )
    assert report["status"] == "failed"
    partial_doc = yaml.safe_load(partial_path.read_text(encoding="utf-8"))
    assert partial_doc["verified"] is False
    # right_joint3 is verified; unmeasured joints retain offset_deg=null (NEVER guessed from encoder current!).
    r_rec = partial_doc["sides"]["right"]["joint_verification"]
    assert r_rec["joint3"]["verified"] is True
    assert r_rec["joint3"]["sign"] == -1
    assert r_rec["joint3"]["offset_deg"] == pytest.approx(22.5)
    assert r_rec["joint4"]["verified"] is False
    assert r_rec["joint4"]["method"] == METHOD_UNMEASURED
    assert r_rec["joint4"]["offset_deg"] is None
    assert r_rec["joint5"]["verified"] is False
    assert r_rec["joint5"]["sign"] == 1
    assert r_rec["joint5"]["offset_deg"] is None

    # Partial mapping can be inspected cleanly without crashing, and is strictly refused for replay.
    inspected = inspect_dataset_mapping(episode, robot, config, mapping_path=partial_path)
    assert inspected["mapping_verified"] is False
    assert len(inspected["inspection"]["verified_joints"]) == 1
    assert len(inspected["inspection"]["unverified_joints"]) == 13
    with pytest.raises(ValueError, match="not verified"):
        execute_joint_episode(
            episode,
            yaml.safe_load(partial_path.read_text(encoding="utf-8")),
            config,
            robot,
            tmp_path / "partial-replay.json",
        )
    # Hand-tampering verified=True on a partial document must be rejected too.
    tampered = yaml.safe_load(partial_path.read_text(encoding="utf-8"))
    tampered["verified"] = True
    with pytest.raises(ValueError, match="unverified|null|offset"):
        validate_mapping_document(tampered, require_verified=True)

    # Incremental merge via existing_mapping finishes remaining joints without losing right_joint3.
    remaining = {"joints": {}}
    for side in ("right", "left"):
        for idx in range(7):
            key = f"{side}_joint{idx}"
            if key != "right_joint3":
                remaining["joints"][key] = _measurement_entry(
                    1, float(idx - 3), model_ref=0.0, basis="CAD水平基準"
                )
    completed_path = tmp_path / "completed.yaml"
    complete_report = verify_dataset_mapping_jointwise(
        episode,
        robot,
        completed_path,
        config=config,
        existing_mapping=partial_path,
        measurements=remaining,
        allow_partial=False,
    )
    assert complete_report["status"] == "ok"
    completed_doc = yaml.safe_load(completed_path.read_text(encoding="utf-8"))
    assert completed_doc["verified"] is True
    assert completed_doc["sides"]["right"]["joint_verification"]["joint3"]["sign"] == -1
    assert completed_doc["sides"]["right"]["joint_verification"]["joint3"]["offset_deg"] == pytest.approx(
        22.5
    )


def test_invalid_mapping_and_calibration_hash_mismatch_rejected(tmp_path):
    pytest.importorskip("mujoco")
    robot = _fake_robot(tmp_path)
    cal_hash = file_sha256(robot.calibration_fpath)

    # Bad sign value (not +-1)
    bad_sign = _mapping_document("robot-test", cal_hash)
    bad_sign["sides"]["right"]["sign"][2] = 2
    with pytest.raises(ValueError, match="mapping sign"):
        validate_mapping_document(bad_sign)

    # Bad model_joint assignment
    bad_model_joint = _mapping_document("robot-test", cal_hash)
    bad_model_joint["sides"]["right"]["joint_verification"]["joint0"]["model_joint"] = "joint0_L"
    with pytest.raises(ValueError, match="model_joint"):
        validate_mapping_for_robot(bad_model_joint, robot)

    # Bad calibration SHA-256
    bad_hash_doc = _mapping_document("robot-test", cal_hash)
    cal_data = json.loads(robot.calibration_fpath.read_text(encoding="utf-8"))
    cal_data["right_joint0"]["range_min"] = 100
    robot.calibration_fpath.write_text(json.dumps(cal_data), encoding="utf-8")
    with pytest.raises(ValueError, match="calibration hash differs"):
        validate_mapping_for_robot(bad_hash_doc, robot)


def test_allow_initial_move_succeeds_from_different_pose_and_respects_start_separation(tmp_path):
    pytest.importorskip("mujoco")
    config = load_config(CONFIG)
    episode = _fake_episode(
        [
            {"right": np.zeros(7), "left": np.zeros(7)},
            {"right": np.array([4.0, -3.0, 2.0, 5.0, 0.0, 0.0, 0.0]), "left": np.zeros(7)},
        ]
    )
    signs = np.array([1, -1, 1, -1, 1, 1, 1])
    offsets = np.array([5.0, -10.0, 3.0, 8.0, 0.0, 0.0, 0.0])
    # Robot starts away from both URDF zero and mapped dataset start.
    robot_current = {
        "right": np.array([9.0, -5.0, 3.0, 14.0, 0.0, 0.0, 0.0]),
        "left": np.full(7, -6.0),
    }
    robot = _fake_robot(tmp_path, current=robot_current)
    document = _mapping_document(
        "robot-test", file_sha256(robot.calibration_fpath), signs=signs, offsets=offsets
    )

    report_path = tmp_path / "replay-with-initial-move.json"
    report = execute_joint_episode(
        episode,
        document,
        config,
        robot,
        report_path,
        allow_initial_move=True,
        realtime=False,
    )
    assert report["status"] == "complete"
    assert report["gripper_commanded"] is False
    assert report["initial_transition"]["required"] is True
    assert report["initial_transition"]["applied"] is True
    assert report["initial_transition"]["final_start_error_deg"] <= 2.0
    assert report["pose_context"]["robot_current_pose"]["right"]["motor_deg"] == pytest.approx(
        robot_current["right"].tolist()
    )
    assert report["pose_context"]["dataset_initial_pose"]["right"]["mapped_motor_deg"] == pytest.approx(
        offsets.tolist()
    )
    # Robot moved to mapped frame 1 (s * q_model + offset).
    expected_final_right = signs * episode.frames[-1].joints["right"] + offsets
    actual_final_right = np.array([robot.current[f"right_joint{i}.pos"] for i in range(7)])
    np.testing.assert_allclose(actual_final_right, expected_final_right, atol=1e-6)
    assert robot.right_bus.disconnect_disable_torque is False
    assert robot.left_bus.disconnect_disable_torque is False


def test_execute_refuses_intermediate_initial_transition_collision(tmp_path):
    pytest.importorskip("mujoco")
    config = load_config(CONFIG)
    # Endpoints individually have 0 collision, while linear midpoint [0, 0, 0, 90, ...] collides (>0).
    start_model = np.array([-70.0, 0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
    target_model = np.array([70.0, 0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
    episode = _fake_episode(
        [
            {"right": target_model, "left": np.zeros(7)},
            {"right": target_model + np.array([-1.0, 0, 0, 0, 0, 0, 0]), "left": np.zeros(7)},
        ]
    )
    robot = _fake_robot(tmp_path, current={"right": start_model, "left": np.zeros(7)})
    document = _mapping_document("robot-test", file_sha256(robot.calibration_fpath))
    report_path = tmp_path / "collision-report.json"

    with pytest.raises(RuntimeError, match="collision|not safe"):
        execute_joint_episode(
            episode,
            document,
            config,
            robot,
            report_path,
            allow_initial_move=True,
            realtime=False,
        )
    assert not robot.right_bus.enable_torque_calls
    assert not robot.left_bus.enable_torque_calls
    assert not robot.right_bus.torque_enabled and not robot.left_bus.torque_enabled
    # Only passive hold-at-current packet allowed; never moves toward colliding target.
    for cmd in robot.raw_commands:
        for i in range(7):
            assert cmd[f"right_joint{i}.pos"] == pytest.approx(start_model[i])
            assert cmd[f"left_joint{i}.pos"] == pytest.approx(0.0)
    saved = json.loads(report_path.read_text(encoding="utf-8"))
    assert saved["status"] == "aborted"
    assert saved["initial_transition"]["safe"] is False
    assert len(saved["initial_transition"]["collision_frames"]) > 0


def test_execute_and_transition_reject_joint_limit_violations(tmp_path):
    pytest.importorskip("mujoco")
    config = load_config(CONFIG)
    episode = _fake_episode(
        [
            {"right": np.zeros(7), "left": np.zeros(7)},
            {"right": np.full(7, 2.0), "left": np.zeros(7)},
        ]
    )
    # Within wide calibration range (-180..180), but outside configured joint_limits_deg!
    current_right = np.array([40.0, 0, 0, 0, 0, 0, 0])
    robot = _fake_robot(tmp_path, current={"right": current_right, "left": np.zeros(7)})
    document = _mapping_document("robot-test", file_sha256(robot.calibration_fpath))
    document["sides"]["right"]["joint_limits_deg"] = {
        "min": [-20.0] * 7,
        "max": [20.0] * 7,
    }
    report_path = tmp_path / "limit-report.json"
    with pytest.raises(RuntimeError, match="limit|not safe"):
        execute_joint_episode(
            episode,
            document,
            config,
            robot,
            report_path,
            allow_initial_move=True,
            realtime=False,
        )
    assert not robot.right_bus.enable_torque_calls
    assert not robot.right_bus.torque_enabled
    for cmd in robot.raw_commands:
        for i in range(7):
            assert cmd[f"right_joint{i}.pos"] == pytest.approx(current_right[i])
            assert cmd[f"left_joint{i}.pos"] == pytest.approx(0.0)
    saved = json.loads(report_path.read_text(encoding="utf-8"))
    assert saved["start_pose"]["limit_violations"]


def test_large_tracking_error_triggers_hold_and_aborts_replay(tmp_path):
    pytest.importorskip("mujoco")
    config = load_config(CONFIG)
    # Case A: stalled during trajectory playback.
    episode = _fake_episode(
        [
            {"right": np.zeros(7), "left": np.zeros(7)},
            {"right": np.full(7, 10.0), "left": np.full(7, 10.0)},
        ]
    )
    robot_traj = _fake_robot(tmp_path / "traj", stalled=True)
    doc_traj = _mapping_document("robot-test", file_sha256(robot_traj.calibration_fpath))
    report_traj = tmp_path / "stalled-traj.json"
    with pytest.raises(RuntimeError, match="HOLD"):
        execute_joint_episode(episode, doc_traj, config, robot_traj, report_traj, realtime=False)
    saved_traj = json.loads(report_traj.read_text(encoding="utf-8"))
    assert saved_traj["status"] == "aborted"
    assert saved_traj["hold_sent"] is True
    assert robot_traj.right_bus.disconnect_disable_torque is False

    # Case B: stalled during initial transition.
    robot_init = _fake_robot(
        tmp_path / "init",
        stalled=True,
        current={"right": np.full(7, -8.0), "left": np.zeros(7)},
    )
    doc_init = _mapping_document("robot-test", file_sha256(robot_init.calibration_fpath))
    report_init = tmp_path / "stalled-init.json"
    with pytest.raises(RuntimeError, match="HOLD"):
        execute_joint_episode(
            episode,
            doc_init,
            config,
            robot_init,
            report_init,
            allow_initial_move=True,
            realtime=False,
        )
    saved_init = json.loads(report_init.read_text(encoding="utf-8"))
    assert saved_init["status"] == "aborted"
    assert saved_init["hold_sent"] is True
    assert robot_init.right_bus.disconnect_disable_torque is False


@pytest.mark.parametrize("initial_torque", [True, False])
def test_read_only_commands_preserve_torque_on_mock_motor_bus_and_never_touch_gripper(
    tmp_path, monkeypatch, initial_torque
):
    pytest.importorskip("mujoco")
    cal_dir, created_buses = _install_mock_feetech_bus(
        monkeypatch,
        tmp_path,
        robot_id="robot-mock",
        initial_torque_enabled=initial_torque,
    )
    config = load_config(CONFIG)
    episode = _fake_episode(
        [
            {"right": np.zeros(7), "left": np.zeros(7)},
            {"right": np.full(7, 5.0), "left": np.full(7, 5.0)},
        ]
    )
    mapping_doc = _mapping_document("robot-mock", file_sha256(cal_dir / "robot-mock.json"))
    mapping_path = tmp_path / "mock-mapping.yaml"
    mapping_path.write_text(yaml.safe_dump(mapping_doc, sort_keys=False), encoding="utf-8")

    # Test inspect_dataset_mapping and replay_joint_dataset --check-start with mocked load_joint_episode.
    monkeypatch.setattr(
        "telegrip.mocopi.dataset.hardware_dataset.load_joint_episode", lambda path, ep=0: episode
    )

    # 1. inspect_dataset_mapping --check-start using real DualScorpionFollower via arm_only_follower
    robot = arm_only_follower("/dev/mock-left", "/dev/mock-right", "robot-mock", 0.5, str(cal_dir))
    assert "gripper" not in robot.left_bus.motors and "gripper" not in robot.right_bus.motors
    inspect_res = inspect_dataset_mapping(episode, robot, config, mapping_path=mapping_path, check_start=True)
    assert inspect_res["robot_current_motor_deg"] is not None
    assert robot.right_bus.torque_enabled is initial_torque
    assert robot.left_bus.torque_enabled is initial_torque
    assert not robot.right_bus.enable_torque_calls and not robot.left_bus.enable_torque_calls
    assert not robot.right_bus.disable_torque_calls and not robot.left_bus.disable_torque_calls
    assert robot.right_bus.disconnect_calls == [False] and robot.left_bus.disconnect_calls == [False]

    # 2. replay_joint_dataset --check-start
    monkeypatch.setattr("telegrip.mocopi.dataset.hardware_dataset.arm_only_follower", lambda *a, **k: robot)
    replay_res = replay_joint_dataset(
        config,
        tmp_path,
        mapping_path=mapping_path,
        robot_id="robot-mock",
        calibration_dir=str(cal_dir),
        left_port="/dev/mock-left",
        right_port="/dev/mock-right",
        check_start=True,
    )
    assert replay_res["start_pose"]["within_tolerance"] is False
    assert replay_res["gripper_commanded"] is False
    assert robot.right_bus.torque_enabled is initial_torque
    assert robot.left_bus.torque_enabled is initial_torque
    assert not robot.right_bus.enable_torque_calls and not robot.left_bus.enable_torque_calls
    assert not robot.right_bus.disable_torque_calls and not robot.left_bus.disable_torque_calls
    assert robot.right_bus.disconnect_calls[-1] is False and robot.left_bus.disconnect_calls[-1] is False

    # 3. Post-connect exception preserves torque state (e.g., calibration mismatch).
    cal_dir_bad, _ = _install_mock_feetech_bus(
        monkeypatch,
        tmp_path / "bad_cal",
        robot_id="robot-mock",
        initial_torque_enabled=initial_torque,
        is_calibrated=False,
    )
    bad_robot = arm_only_follower("/dev/mock-left", "/dev/mock-right", "robot-mock", 0.5, str(cal_dir_bad))
    with pytest.raises(ValueError, match="recalibrate"):
        inspect_dataset_mapping(episode, bad_robot, config, check_start=True)
    assert bad_robot.right_bus.torque_enabled is initial_torque
    assert bad_robot.left_bus.torque_enabled is initial_torque
    assert not bad_robot.right_bus.enable_torque_calls and not bad_robot.left_bus.enable_torque_calls
    assert not bad_robot.right_bus.disable_torque_calls and not bad_robot.left_bus.disable_torque_calls
    assert bad_robot.right_bus.disconnect_calls == [False] and bad_robot.left_bus.disconnect_calls == [False]

    # 4. Partial connection failure disconnects already-connected bus without changing torque.
    cal_dir_partial, partial_buses = _install_mock_feetech_bus(
        monkeypatch,
        tmp_path / "partial_conn",
        robot_id="robot-mock",
        initial_torque_enabled=initial_torque,
        second_connect_error=OSError("mock second port down"),
    )
    partial_robot = arm_only_follower(
        "/dev/mock-left", "/dev/mock-right", "robot-mock", 0.5, str(cal_dir_partial)
    )
    with pytest.raises(OSError, match="mock second port down"):
        inspect_dataset_mapping(episode, partial_robot, config, check_start=True)
    assert all(not bus.is_connected for bus in partial_buses)
    assert any(bus.disconnect_calls == [False] for bus in partial_buses)
    assert all(bus.torque_enabled is initial_torque for bus in partial_buses)
    assert all(not bus.disable_torque_calls for bus in partial_buses)


def test_declining_torque_disable_aborts_verification_without_touching_torque(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    robot = _fake_robot(tmp_path, initial_torque_enabled=True)
    monkeypatch.setattr("builtins.input", lambda prompt="": "n")
    with pytest.raises(ValueError, match="Torque disable was not confirmed"):
        verify_dataset_mapping_jointwise(
            None,
            robot,
            tmp_path / "aborted.yaml",
            config=load_config(CONFIG),
            allow_partial=True,
        )
    assert robot.right_bus.torque_enabled is True
    assert robot.left_bus.torque_enabled is True
    assert not robot.right_bus.disable_torque_calls and not robot.left_bus.disable_torque_calls
    assert robot.right_bus.disconnect_disable_torque is False
    assert robot.left_bus.disconnect_disable_torque is False


def _run_cli(monkeypatch, args):
    monkeypatch.setattr(sys, "argv", ["telegrip.mocopi.dataset", *args])
    return mocopi_dataset_cli()


def test_cli_workflows_and_dataset_immutability(tmp_path, monkeypatch, capsys):
    pytest.importorskip("mujoco")
    pytest.importorskip("datasets")
    dataset = tmp_path / "dataset"
    _make_joint_dataset(dataset, file_sha256(full_model_path(load_config(CONFIG)["robot"])))
    before_fp = dataset_fingerprint(dataset, manifest_name="mocopi_joint_dataset.json")
    cal_dir, created_buses = _install_mock_feetech_bus(
        monkeypatch,
        tmp_path,
        robot_id="robot-mock",
        initial_torque_enabled=True,
    )

    # 1. CLI rejects --reference-frame in joint-wise mode.
    with pytest.raises(SystemExit) as exc_info:
        _run_cli(
            monkeypatch,
            [
                "--config",
                str(CONFIG),
                "verify-dataset-mapping",
                "--mode",
                "joint-wise",
                "--reference-frame",
                "0",
                "--left-port",
                "/dev/mock-left",
                "--right-port",
                "/dev/mock-right",
                "--robot-id",
                "robot-mock",
                "--calibration-dir",
                str(cal_dir),
                "--output",
                str(tmp_path / "nope.yaml"),
            ],
        )
    assert exc_info.value.code == 1
    assert "--reference-frame" in capsys.readouterr().err

    # 2. CLI verify-dataset-mapping --measurements --allow-partial without dataset.
    meas_path = tmp_path / "meas.yaml"
    meas_path.write_text(
        yaml.safe_dump(
            {"joints": {"right_joint3": _measurement_entry(-1, 15.0, model_ref=90.0)}}, sort_keys=False
        ),
        encoding="utf-8",
    )
    partial_out = tmp_path / "cli-partial.yaml"
    _run_cli(
        monkeypatch,
        [
            "--config",
            str(CONFIG),
            "verify-dataset-mapping",
            "--robot-id",
            "robot-mock",
            "--calibration-dir",
            str(cal_dir),
            "--measurements",
            str(meas_path),
            "--allow-partial",
            "--output",
            str(partial_out),
        ],
    )
    partial_doc = yaml.safe_load(partial_out.read_text(encoding="utf-8"))
    assert partial_doc["verified"] is False

    # 3. CLI inspect-dataset-mapping --check-start on the current-model dataset preserves torque and outputs JSON.
    inspect_out = tmp_path / "cli-inspect.json"
    _run_cli(
        monkeypatch,
        [
            "--config",
            str(CONFIG),
            "inspect-dataset-mapping",
            "--dataset",
            str(dataset),
            "--mapping",
            str(partial_out),
            "--robot-id",
            "robot-mock",
            "--calibration-dir",
            str(cal_dir),
            "--left-port",
            "/dev/mock-left",
            "--right-port",
            "/dev/mock-right",
            "--check-start",
            "--output",
            str(inspect_out),
        ],
    )
    inspect_data = json.loads(inspect_out.read_text(encoding="utf-8"))
    assert inspect_data["mapping_verified"] is False
    assert inspect_data["robot_current_motor_deg"] is not None

    # 4. Complete mapping via --measurements -> CLI replay-dataset --check-start preserves torque.
    full_meas = {
        "joints": {
            f"{side}_joint{idx}": _measurement_entry(1, float(idx - 2), model_ref=0.0)
            for side in ("right", "left")
            for idx in range(7)
        }
    }
    full_meas_path = tmp_path / "full-meas.yaml"
    full_meas_path.write_text(yaml.safe_dump(full_meas, sort_keys=False), encoding="utf-8")
    verified_out = tmp_path / "cli-verified.yaml"
    _run_cli(
        monkeypatch,
        [
            "--config",
            str(CONFIG),
            "verify-dataset-mapping",
            "--robot-id",
            "robot-mock",
            "--calibration-dir",
            str(cal_dir),
            "--measurements",
            str(full_meas_path),
            "--output",
            str(verified_out),
        ],
    )
    _run_cli(
        monkeypatch,
        [
            "--config",
            str(CONFIG),
            "replay-dataset",
            "--dataset",
            str(dataset),
            "--mapping",
            str(verified_out),
            "--robot-id",
            "robot-mock",
            "--calibration-dir",
            str(cal_dir),
            "--left-port",
            "/dev/mock-left",
            "--right-port",
            "/dev/mock-right",
            "--check-start",
        ],
    )
    assert created_buses
    for bus in created_buses:
        assert bus.torque_enabled is True
        assert not bus.enable_torque_calls
        assert not bus.disable_torque_calls
        if bus.disconnect_calls:
            assert all(flag is False for flag in bus.disconnect_calls)

    # 5. Dataset directory files & hashes remain strictly unmodified.
    after_fp = dataset_fingerprint(dataset, manifest_name="mocopi_joint_dataset.json")
    assert before_fp == after_fp


def test_unidirectional_desk_safe_verification_for_joint5_at_limit(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    # Simulate physical 45cm desk robot: left_joint5 sits at +96.53 deg (max +98.55 deg, margin only +2.02 deg).
    # Unidirectional lift (- direction only) determines sign=-1 and offset_deg without MID or pushing into desk.
    direct = verify_joint_direction_unidirectional(
        96.53,
        66.53,
        expected_model_direction="+",
        side="left",
        joint="joint5",
        min_test_deg=10.0,
    )
    assert direct["sign_verified"] is True
    assert direct["sign"] == -1
    assert direct["direction_mode"] == "unidirectional"

    rec = fit_joint_from_unidirectional_reference(
        0.0,
        15.0,
        96.53,
        66.53,
        expected_model_direction="+",
        side="left",
        joint="joint5",
        motor_id=6,
        measurement_basis="机上の静止位置(+96.53°)から上へ持ち上げて水平一直線(q=0°)を計測",
    )
    assert rec["verified"] is True
    assert rec["sign"] == -1
    assert rec["offset_deg"] == pytest.approx(15.0)
    assert rec["method"] == METHOD_PER_JOINT_UNIDIRECTIONAL

    # Verify both --measurements unidirectional YAML and interactive CLI joint-wise '-'/'+' prompt work.
    robot = _fake_robot(tmp_path)
    config = load_config(CONFIG)
    meas = {
        "joints": {
            "left_joint5": {
                "direction_test": {
                    "mode": "unidirectional",
                    "start_motor_deg": 96.53,
                    "moved_motor_deg": 56.53,
                    "expected_model_direction": "+",
                },
                "direct_drive_confirmed": True,
                "measurement_basis": "手首を上方向へ片側1回持ち上げ+一直線0度基準",
                "model_reference_deg": 0.0,
                "motor_reference_deg": 20.0,
            }
        }
    }
    partial_out = tmp_path / "uni-partial.yaml"
    verify_dataset_mapping_jointwise(
        None,
        robot,
        partial_out,
        config=config,
        measurements=meas,
        allow_partial=True,
    )
    partial_doc = yaml.safe_load(partial_out.read_text(encoding="utf-8"))
    rec_yaml = partial_doc["sides"]["left"]["joint_verification"]["joint5"]
    assert rec_yaml["verified"] is True
    assert rec_yaml["sign"] == -1
    assert rec_yaml["offset_deg"] == pytest.approx(20.0)
    assert rec_yaml["verification_method"] == METHOD_PER_JOINT_UNIDIRECTIONAL

    # Interactive single-joint unidirectional flow using '+' without ever moving to MID.
    base_rest = {
        f"{s}_joint{i}.pos": (96.53 if (s == "left" and i == 5) else 0.0)
        for s in ("right", "left")
        for i in range(7)
    }
    lifted = dict(base_rest)
    lifted["left_joint5.pos"] = 60.53  # moved -36 deg away from limit in CAD '+' direction => sign=-1
    inline_ref = dict(base_rest)
    inline_ref["left_joint5.pos"] = 18.0
    live_robot = _fake_robot(tmp_path / "live", measurements=[base_rest, lifted, inline_ref])

    def _uni_input(prompt=""):
        if "トルクを解除" in prompt or "直結" in prompt:
            return "y"
        if "方向テスト方法" in prompt:
            return "+"
        if "既知のモデル角度" in prompt:
            return "0"
        if "測定根拠" in prompt:
            return "机上から持ち上げて一直線0度"
        return ""

    monkeypatch.setattr("builtins.input", _uni_input)
    live_out = tmp_path / "live-uni.yaml"
    verify_dataset_mapping_jointwise(
        None,
        live_robot,
        live_out,
        config=config,
        joints=["left_joint5"],
        arm="left",
        allow_partial=True,
    )
    live_doc = yaml.safe_load(live_out.read_text(encoding="utf-8"))
    assert live_doc["sides"]["left"]["joint_verification"]["joint5"]["verified"] is True
    assert live_doc["sides"]["left"]["joint_verification"]["joint5"]["sign"] == -1
    assert live_doc["sides"]["left"]["joint_verification"]["joint5"]["offset_deg"] == pytest.approx(18.0)


def test_lift_first_and_auto_initial_transition_clears_table_obstacle(tmp_path):
    pytest.importorskip("mujoco")
    from telegrip.mocopi.full_mujoco import FullMujocoSimulation

    config = load_config(CONFIG)
    obs = [
        {
            "name": "cup_block",
            # Same right-shoulder-relative location with the 300 mm crossbar.
            "center_m": [0.195, 0.16, 0.02],
            "half_size_m": [0.06, 0.06, 0.05],
            "rotation_deg": [0.0, 0.0, 0.0],
        }
    ]
    start = {"right": np.array([0.0, 60.0, 0.0, 0.0, 0.0, -40.0, 0.0]), "left": np.zeros(7)}
    target = {"right": np.array([0.0, -40.0, 0.0, 0.0, 0.0, 0.0, 0.0]), "left": np.zeros(7)}
    episode = _fake_episode(
        [
            target,
            {"right": target["right"] + np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]), "left": np.zeros(7)},
        ]
    )
    episode.metadata["obstacles"] = obs
    robot = _fake_robot(tmp_path, current=start)
    document = _mapping_document("robot-test", file_sha256(robot.calibration_fpath))

    sim = FullMujocoSimulation(config["robot"], gui=False, obstacles=obs)
    try:
        linear_eval = evaluate_initial_transition(
            start,
            episode,
            document,
            robot,
            max_step_deg=0.5,
            max_speed_deg_s=5.0,
            control_hz=20.0,
            simulation=sim,
            strategy="linear",
        )
        assert linear_eval["safe"] is False
        assert linear_eval["collision_frames"]

        auto_eval = evaluate_initial_transition(
            start,
            episode,
            document,
            robot,
            max_step_deg=0.5,
            max_speed_deg_s=5.0,
            control_hz=20.0,
            simulation=sim,
            strategy="auto",
        )
        assert auto_eval["strategy"] == "lift_first"
        assert auto_eval["safe"] is True
        assert not auto_eval["collision_frames"]
        assert len(auto_eval["stages"]) == 2
        assert auto_eval["stages"][0]["name"] == "lift_distal_pitch"
    finally:
        sim.close()

    # Execution with linear strategy refuses before enabling torque.
    refuse_report = tmp_path / "linear-refuse.json"
    with pytest.raises(RuntimeError, match="not safe"):
        execute_joint_episode(
            episode,
            document,
            config,
            robot,
            refuse_report,
            allow_initial_move=True,
            initial_move_strategy="linear",
            realtime=False,
        )
    assert not robot.right_bus.enable_torque_calls

    # Execution with auto/lift-first completes safely and tracks stages properly.
    ok_report = tmp_path / "lift-first-ok.json"
    rep = execute_joint_episode(
        episode,
        document,
        config,
        robot,
        ok_report,
        allow_initial_move=True,
        initial_move_strategy="auto",
        realtime=False,
    )
    assert rep["status"] == "complete"
    assert rep["initial_transition"]["strategy"] == "lift_first"
    assert rep["initial_transition"]["applied"] is True


def test_export_ft_dataset_maps_to_16dim_follower_and_validates_replay_report(tmp_path, monkeypatch):
    pytest.importorskip("datasets")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    config = load_config(CONFIG)
    model_hash = file_sha256(full_model_path(config["robot"]))
    robot = _fake_robot(tmp_path)
    cal_hash = file_sha256(robot.calibration_fpath)
    signs = np.array([1, -1, 1, -1, 1, -1, 1])
    offsets = np.array([10.0, -20.0, 5.0, -90.0, 15.0, 30.0, -5.0])
    document = _mapping_document("robot-test", cal_hash, signs=signs, offsets=offsets)
    mapping_path = tmp_path / "verified-mapping.yaml"
    mapping_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    src_root = tmp_path / "src_joint_ds"
    src_meta, q_frames = _make_joint_dataset(src_root, model_hash)
    before_fp = dataset_fingerprint(src_root, "mocopi_joint_dataset.json")

    cal_dir = tmp_path / "cal_dir"
    cal_dir.mkdir()
    (cal_dir / "robot-test.json").write_text(
        robot.calibration_fpath.read_text(encoding="utf-8"), encoding="utf-8"
    )

    # 1. Refuses without replay report unless --allow-without-replay-report.
    with pytest.raises(ValueError, match="replay verification report"):
        export_ft_dataset(
            src_root,
            mapping_path,
            tmp_path / "ft-blocked",
            "local/ft-blocked",
            config=config,
            calibration_dir=str(cal_dir),
        )

    aborted_report = tmp_path / "replay-aborted.json"
    aborted_report.write_text(
        json.dumps(
            {
                "schema": "dual_scorpion.mocopi.hardware_dataset_replay.v1",
                "status": "aborted",
                "gripper_commanded": False,
                "dataset_sha256": before_fp,
                "mujoco_model_sha256": model_hash,
                "calibration_sha256": cal_hash,
                "robot_id": "robot-test",
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="status='complete'"):
        export_ft_dataset(
            src_root,
            mapping_path,
            tmp_path / "ft-aborted",
            "local/ft-aborted",
            config=config,
            calibration_dir=str(cal_dir),
            replay_report=aborted_report,
        )

    # 2. Succeeds with a matching completed replay report -> 16-dim follower motor space + aliased image key.
    report_path = tmp_path / "replay-ok.json"
    report_path.write_text(
        json.dumps(
            {
                "schema": "dual_scorpion.mocopi.hardware_dataset_replay.v1",
                "status": "complete",
                "gripper_commanded": False,
                "dataset_sha256": before_fp,
                "mujoco_model_sha256": model_hash,
                "calibration_sha256": cal_hash,
                "robot_id": "robot-test",
                "frames": [{"plan_frame": i} for i in range(4)],
            }
        ),
        encoding="utf-8",
    )
    ft_out = tmp_path / "ft-16dim"
    _run_cli(
        monkeypatch,
        [
            "--config",
            str(CONFIG),
            "export-ft-dataset",
            "--dataset",
            str(src_root),
            "--mapping",
            str(mapping_path),
            "--output",
            str(ft_out),
            "--repo-id",
            "local/ft-16dim",
            "--calibration-dir",
            str(cal_dir),
            "--replay-report",
            str(report_path),
            "--image-key",
            "observation.images.top",
        ],
    )
    manifest = json.loads((ft_out / "meta" / "mocopi_ft_dataset.json").read_text(encoding="utf-8"))
    assert manifest["schema"] == FT_DATASET_SCHEMA
    assert manifest["hardware_replay_verified"] is True
    assert manifest["state_action_dim"] == 16
    assert manifest["include_gripper"] is True
    assert manifest["gripper_synthesized_constant"] is True
    assert manifest["camera_mapping"] == {
        "source_key": "observation.images.head",
        "exported_key": "observation.images.top",
    }

    ft_ds = LeRobotDataset("local/ft-16dim", root=ft_out, video_backend="pyav")
    assert len(ft_ds) == 3
    row0 = ft_ds[0]
    assert row0["observation.state"].shape == (16,)
    assert row0["action"].shape == (16,)
    assert row0["observation.images.top"].shape == (3, 48, 64)
    # Verify exact mapping q_motor = sign * q_model + offset and 0.0 gripper padding.
    expected_r0 = signs * q_frames[0][:7] + offsets
    expected_l0 = signs * q_frames[0][7:] + offsets
    np.testing.assert_allclose(row0["observation.state"][:7].numpy(), expected_r0, atol=1e-4)
    assert row0["observation.state"][7].item() == pytest.approx(0.0)
    np.testing.assert_allclose(row0["observation.state"][8:15].numpy(), expected_l0, atol=1e-4)
    assert row0["observation.state"][15].item() == pytest.approx(0.0)

    # 3. Also test 14-dim --arm-only export with explicit --allow-without-replay-report.
    ft_arm_out = tmp_path / "ft-14dim"
    manifest_14 = export_ft_dataset(
        src_root,
        mapping_path,
        ft_arm_out,
        "local/ft-14dim",
        config=config,
        calibration_dir=str(cal_dir),
        allow_without_replay_report=True,
        include_gripper=False,
    )
    assert manifest_14["state_action_dim"] == 14
    assert manifest_14["hardware_replay_verified"] is False
    ft_ds14 = LeRobotDataset("local/ft-14dim", root=ft_arm_out, video_backend="pyav")
    row1 = ft_ds14[1]
    assert row1["observation.state"].shape == (14,)
    np.testing.assert_allclose(
        row1["observation.state"][:7].numpy(), signs * q_frames[1][:7] + offsets, atol=1e-4
    )
    np.testing.assert_allclose(
        row1["observation.state"][7:].numpy(), signs * q_frames[1][7:] + offsets, atol=1e-4
    )

    # 4. Also test omitting --mapping (auto calibration midpoint mapping) on export-ft-dataset.
    ft_auto_out = tmp_path / "ft-auto-cal"
    _run_cli(
        monkeypatch,
        [
            "--config",
            str(CONFIG),
            "export-ft-dataset",
            "--dataset",
            str(src_root),
            "--output",
            str(ft_auto_out),
            "--repo-id",
            "local/ft-auto-cal",
            "--robot-id",
            "robot-test",
            "--calibration-dir",
            str(cal_dir),
            "--allow-without-replay-report",
        ],
    )
    auto_manifest = json.loads((ft_auto_out / "meta" / "mocopi_ft_dataset.json").read_text(encoding="utf-8"))
    assert auto_manifest["mapping_path"] is None
    assert auto_manifest["mapping_verification_method"] == METHOD_CALIBRATION_MIDPOINT
    assert auto_manifest["state_action_dim"] == 16

    # 5. Source dataset strictly untouched.
    assert dataset_fingerprint(src_root, "mocopi_joint_dataset.json") == before_fp


def test_calibration_midpoint_mapping_and_optional_mapping_cli(tmp_path, monkeypatch):
    """Verify calibration-based mapping builds without manual CAD posing and works when --mapping is omitted."""
    config = load_config(CONFIG)
    cal_dir = tmp_path / "cal"
    cal_dir.mkdir(parents=True, exist_ok=True)
    cal_path = cal_dir / "robot-cal.json"
    cal_data = {
        f"{side}_joint{i}": {
            "id": i + 1,
            "drive_mode": 1 if (side == "right" and i == 2) else 0,
            "homing_offset": 0,
            "range_min": 0,
            "range_max": 4095,
        }
        for side in ("right", "left")
        for i in range(7)
    }
    cal_path.write_text(json.dumps(cal_data), encoding="utf-8")
    robot = arm_only_follower("/dev/not-used-left", "/dev/not-used-right", "robot-cal", 0.5, cal_dir)
    doc, report = build_calibration_mapping(robot, config=config)
    assert doc["verified"] is True
    assert doc["verification_method"] == METHOD_CALIBRATION_MIDPOINT
    assert report["status"] == "ok"
    assert report["manual_cad_pose_required"] is False
    expected_right = list(DEFAULT_FULL_CAD_SIGNS_DRIVE_MODE_0["right"])
    expected_right[2] *= -1  # drive_mode == 1 flips sign
    assert doc["sides"]["right"]["sign"] == expected_right
    assert doc["sides"]["left"]["sign"] == list(DEFAULT_FULL_CAD_SIGNS_DRIVE_MODE_0["left"])
    assert doc["sides"]["right"]["offset_deg"] == [0.0] * 7
    assert doc["sides"]["left"]["offset_deg"] == [0.0] * 7

    # CLI verify-dataset-mapping --mode calibration --overwrite
    out_yaml = tmp_path / "cal-mapping.yaml"
    out_yaml.write_text("stale: true\n", encoding="utf-8")
    _run_cli(
        monkeypatch,
        [
            "--config",
            str(CONFIG),
            "verify-dataset-mapping",
            "--mode",
            "calibration",
            "--overwrite",
            "--robot-id",
            "robot-cal",
            "--calibration-dir",
            str(cal_dir),
            "--output",
            str(out_yaml),
        ],
    )
    saved = yaml.safe_load(out_yaml.read_text(encoding="utf-8"))
    assert saved["verified"] is True
    assert saved["verification_method"] == METHOD_CALIBRATION_MIDPOINT
