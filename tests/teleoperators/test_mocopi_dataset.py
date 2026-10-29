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


def test_jittery_mocopi_dropout_preserves_episode_grid_boundaries(tmp_path):
    pytest.importorskip("cv2")
    dataset_type = pytest.importorskip("lerobot.datasets.lerobot_dataset").LeRobotDataset
    from telegrip.mocopi.dataset.convert import convert_sessions
    from telegrip.mocopi.dataset.evaluate import session_metrics

    cfg = load_config(CONFIG)
    cfg["camera"].update(width=96, height=64, fps=20)
    raw = tmp_path / "jitter-dropout"
    capture_session(cfg, raw, 1.2, "long jitter check", fake=True)
    motion = rows(raw / "mocopi.jsonl")
    motion = [row for index, row in enumerate(motion) if not 10 <= index < 14]
    for index, row in enumerate(motion):
        if index % 4 == 1:
            row["timestamp_s"] += 0.012
        elif index % 4 == 3:
            row["timestamp_s"] -= 0.012
    (raw / "mocopi.jsonl").write_text(
        "".join(json.dumps(row, allow_nan=False) + "\n" for row in motion), encoding="utf-8"
    )
    metadata = json.loads((raw / "session.json").read_text(encoding="utf-8"))
    metadata["mocopi_frames"] = len(motion)
    (raw / "session.json").write_text(json.dumps(metadata), encoding="utf-8")

    quality = session_metrics(raw, fps=20, max_sync_s=0.04)
    assert quality["unsynchronized_samples"] > 0
    assert quality["usable_segments"] >= 2
    output = tmp_path / "jitter-dataset"
    manifest = convert_sessions([raw], output, "local/mocopi-jitter", fps=20, max_sync_s=0.04)
    dataset = dataset_type("local/mocopi-jitter", root=output, video_backend="pyav")
    assert dataset.num_episodes == quality["usable_segments"] == len(manifest["episodes"])

    episode_times = []
    for episode, entry in enumerate(manifest["episodes"]):
        start, end = (
            int(dataset.meta.episodes[episode]["dataset_from_index"]),
            int(dataset.meta.episodes[episode]["dataset_to_index"]),
        )
        times = np.array(
            [
                dataset.hf_dataset[index]["observation.capture.grid_time_s"].item()
                for index in range(start, end)
            ]
        )
        np.testing.assert_allclose(np.diff(times), 1 / dataset.fps, atol=1e-7)
        np.testing.assert_allclose(times[0], entry["first_grid_time_s"], atol=1e-7)
        episode_times.append(times)
    assert any(
        following[0] - previous[-1] > 3 / dataset.fps
        for previous, following in zip(episode_times, episode_times[1:], strict=False)
    )


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
    import shutil
    from copy import deepcopy

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

    second = tmp_path / "raw-second"
    shutil.copytree(session, second)
    with pytest.raises(ValueError, match="reference-id"):
        convert_sessions([session, second], tmp_path / "unverified", "local/unverified", fps=20)
    for folder in (session, second):
        metadata = json.loads((folder / "session.json").read_text())
        metadata["mocopi"]["reference_id"] = "same-recording-calibration"
        (folder / "session.json").write_text(json.dumps(metadata))
    joined = convert_sessions([session, second], tmp_path / "joined", "local/joined", fps=20)
    assert joined["coordinate_reference_policy"] == "operator_declared_matching_reference_id"
    assert len(joined["sessions"]) == 2

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
    feasibility = simulation["feasibility"]
    assert feasibility["ik_failures"] == 0 and "hardware safety" in feasibility["scope"]
    # The verdict must follow its own stated criteria; synthetic motion is not
    # required to pass them, and the numbers are never rounded to guarantee it.
    assert feasibility["within_preview_tolerance"] is (
        feasibility["max_position_error_m"] <= feasibility["position_tolerance_m"]
        and feasibility["max_joint_step_observed_deg"] <= feasibility["max_joint_step_limit_deg"] + 1e-6
    )
    assert simulation["workspace_clamp_measured"] is True
    assert set(simulation["workspace_clamped_frames"]) == {"left", "right"}
    assert (
        simulation["tracking"]["right"]["max_joint_step_deg"]["max"]
        <= cfg["robot"]["max_joint_step_deg"] + 1e-6
    )

    from telegrip.mocopi.dataset.joint_dataset import JOINT_NAMES, export_joint_dataset

    slowed, _ = replay_sim(cfg, output, gui=False, realtime=False, slowdown=2)
    plan_file = tmp_path / "plan.json"
    plan_file.write_text(json.dumps(slowed))
    joint_root = tmp_path / "kinematic-joints"
    joint_meta = export_joint_dataset(
        cfg, output, [plan_file], joint_root, "local/mocopi-joints", max_sim_error_m=0.1
    )
    assert joint_meta["robot_state_source"] == "kinematic_simulation_not_real_encoders"
    assert joint_meta["capture_source"] == "fake" and not joint_meta["gripper_recorded"]
    assert joint_meta["urdf_sha256"] == slowed["urdf_sha256"]
    joint_data = dataset_type("local/mocopi-joints", root=joint_root, video_backend="pyav")
    assert joint_data.meta.robot_type == "dual_scorpion_kinematic"
    assert len(joint_data) == len(slowed["frames"]) - 1
    assert joint_data.meta.features["action"]["names"] == JOINT_NAMES
    assert not any("gripper" in name for name in JOINT_NAMES)
    for index in (0, 1, len(joint_data) - 1):
        current, following = slowed["frames"][index : index + 2]
        row = joint_data[index]
        expected_state = np.concatenate([current["arms"][side]["joint_deg"] for side in ("right", "left")])
        expected_action = np.concatenate([following["arms"][side]["joint_deg"] for side in ("right", "left")])
        np.testing.assert_allclose(row["observation.state"], expected_state, atol=1e-5)
        np.testing.assert_allclose(row["action"], expected_action, atol=1e-5)
        np.testing.assert_array_equal(row["observation.sim.frame"], [index, current["source_frame"]])
        np.testing.assert_allclose(
            row["observation.mocopi.state"], dataset[current["source_frame"]]["observation.state"]
        )
    assert (
        float((joint_data[0]["observation.images.head"] - dataset[0]["observation.images.head"]).abs().mean())
        < 0.05
    )
    with pytest.raises(ValueError, match="appears more than once"):
        export_joint_dataset(
            cfg,
            output,
            [plan_file, plan_file],
            tmp_path / "duplicate",
            "local/duplicate",
            max_sim_error_m=0.1,
        )
    altered_plan = json.loads(plan_file.read_text())
    altered_plan["frames"][1]["source_frame"] = 999
    bad_plan = tmp_path / "bad-plan.json"
    bad_plan.write_text(json.dumps(altered_plan))
    with pytest.raises(ValueError, match="source frame mapping"):
        export_joint_dataset(
            cfg, output, [bad_plan], tmp_path / "bad-frames", "local/bad-frames", max_sim_error_m=0.1
        )
    stats_file = output / "meta" / "stats.json"
    previous_stats = stats_file.read_text()
    stats_file.write_text(previous_stats + " ")
    with pytest.raises(ValueError, match="does not match dataset content"):
        export_joint_dataset(cfg, output, [plan_file], tmp_path / "changed-source", "local/changed-source")
    stats_file.write_text(previous_stats)

    joined_root = tmp_path / "joined"
    joined_plans = []
    for episode in (0, 1):
        joined_plan, _ = replay_sim(cfg, joined_root, episode=episode, gui=False, realtime=False, slowdown=2)
        path = tmp_path / f"joined-plan-{episode}.json"
        path.write_text(json.dumps(joined_plan))
        joined_plans.append(path)
    combined = export_joint_dataset(
        cfg,
        joined_root,
        joined_plans,
        tmp_path / "combined-joints",
        "local/combined-joints",
        max_sim_error_m=0.1,
    )
    combined_data = dataset_type(
        "local/combined-joints", root=tmp_path / "combined-joints", video_backend="pyav"
    )
    assert combined_data.num_episodes == 2
    assert [episode["source_episode"] for episode in combined["episodes"]] == [0, 1]
    for episode in range(2):
        first = int(combined_data.meta.episodes[episode]["dataset_from_index"])
        np.testing.assert_array_equal(combined_data[first]["observation.sim.frame"], [0, 0])

    from telegrip.mocopi.dataset.real import replay_real, validate_plan
    from telegrip.mocopi.full_mujoco import FullMujocoSimulation

    # The normal synthetic motion is deliberately fast enough to exercise IK
    # failures under the strict per-update bounds. A stationary source provides
    # a clean, valid full-CAD plan for testing downstream validation/export.
    stationary_raw = tmp_path / "raw-stationary"
    capture_session(cfg, stationary_raw, 0.35, "stationary check", fake=True)
    motion_path = stationary_raw / "mocopi.jsonl"
    motion = [json.loads(line) for line in motion_path.read_text().splitlines() if line]
    stationary_bones = motion[0]["bones"]
    for row in motion[1:]:
        row["bones"] = stationary_bones
    motion_path.write_text(
        "".join(json.dumps(row, allow_nan=False) + "\n" for row in motion), encoding="utf-8"
    )
    stationary_output = tmp_path / "stationary-dataset"
    convert_sessions([stationary_raw], stationary_output, "local/mocopi-stationary", fps=20)
    full_plan, full_report = replay_sim(
        cfg, stationary_output, gui=False, realtime=False, slowdown=2, sim_model="full"
    )
    assert full_plan["model_kind"] == "full_mujoco" and not full_plan["hardware_replay_allowed"]
    assert full_plan["start_pose"] == cfg["robot"]["start_pose"] == "work"
    assert full_report["start_pose"] == "work"
    assert full_plan["urdf_sha256"] is None and full_report["joint_limit_source"] == "unmeasured"
    assert full_report["tracking"]["left"]["joint_limit_margin_deg"]["max"] is None
    assert full_report["feasibility"]["ik_failures"] == len(full_report["ik_failures"])
    assert full_report["collision_check"]["measured"] is True
    assert full_report["collision_check"]["frames_with_collisions"] == 0
    assert full_report["collision_check"]["max_penetration_m"] == 0.0
    assert full_report["feasibility"]["modeled_collision_frames"] == 0
    assert all(frame["collision"]["count"] == 0 for frame in full_plan["frames"])
    assert isinstance(full_report["feasibility"]["within_preview_tolerance"], bool)
    assert full_report["workspace_clamp_measured"] is False
    assert full_report["workspace_clamped_frames"] is None
    assert not full_plan["failures"]
    full_file = tmp_path / "full-plan.json"
    full_file.write_text(json.dumps(full_plan))
    preview = replay_real(cfg, full_file, allow_synthetic=True, max_sim_error_m=0.1)
    assert preview["mode"] == "validation_only" and not preview["hardware_replay_allowed"]
    with pytest.raises(ValueError, match="simulation-only"):
        replay_real(cfg, full_file, execute=True, allow_synthetic=True, max_sim_error_m=0.1)
    collision_plan = json.loads(full_file.read_text())
    collision_plan["frames"][0]["collision"]["count"] = 1
    collision_file = tmp_path / "collision-plan.json"
    collision_file.write_text(json.dumps(collision_plan))
    with pytest.raises(ValueError, match="modeled collisions"):
        replay_real(cfg, collision_file, allow_synthetic=True, max_sim_error_m=0.1)
    full_joints = export_joint_dataset(
        cfg,
        stationary_output,
        [full_file],
        tmp_path / "full-joints",
        "local/full-joints",
        max_sim_error_m=0.1,
    )
    assert full_joints["model_kind"] == "full_mujoco" and not full_joints["hardware_replay_allowed"]
    assert full_joints["start_pose"] == "work"
    assert full_joints["episodes"][0]["start_pose"] == "work"
    assert full_joints["obstacles"] == [] and full_joints["obstacle_count"] == 0
    assert full_joints["obstacles_sha256"] == full_plan["obstacles_sha256"]

    obstacle_file = tmp_path / "obstacles.yaml"
    obstacle_file.write_text(
        "\n".join(
            [
                "schema: dual_scorpion.mocopi.obstacles.v1",
                "obstacles:",
                "  - name: test-box",
                "    center_m: [0.0, 0.0, 0.30]",
                "    half_size_m: [0.50, 0.50, 0.50]",
                "    rotation_deg: [0.0, 0.0, 0.0]",
                "",
            ]
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="full MuJoCo model"):
        replay_sim(cfg, output, gui=False, realtime=False, sim_model="legacy", obstacles=obstacle_file)
    blocked_plan, blocked_report = replay_sim(
        cfg,
        stationary_output,
        gui=False,
        realtime=False,
        slowdown=2,
        sim_model="full",
        obstacles=obstacle_file,
    )
    assert blocked_plan["obstacle_count"] == 1
    assert blocked_plan["obstacles"][0]["name"] == "test-box"
    assert blocked_report["collision_check"]["obstacle_names"] == ["test-box"]
    assert blocked_report["collision_check"]["frames_with_collisions"] > 0
    assert blocked_report["collision_check"]["contact_count"] > 0
    assert blocked_report["collision_check"]["max_penetration_m"] > 0
    assert blocked_report["feasibility"]["modeled_collision_frames"] > 0
    blocked_file = tmp_path / "blocked-plan.json"
    blocked_file.write_text(json.dumps(blocked_plan))
    with pytest.raises(ValueError, match="modeled collisions"):
        replay_real(cfg, blocked_file, allow_synthetic=True, max_sim_error_m=0.1)
    with pytest.raises(ValueError, match="modeled collisions"):
        export_joint_dataset(
            cfg,
            stationary_output,
            [blocked_file],
            tmp_path / "blocked-joints",
            "local/blocked-joints",
            max_sim_error_m=0.1,
        )
    changed_scene = json.loads(blocked_file.read_text())
    changed_scene["obstacles_sha256"] = "0" * 64
    changed_file = tmp_path / "changed-obstacle-plan.json"
    changed_file.write_text(json.dumps(changed_scene))
    with pytest.raises(ValueError, match="obstacle scene differs"):
        replay_real(cfg, changed_file, allow_synthetic=True, max_sim_error_m=0.1)

    backwards_plan, backwards_report = replay_sim(
        cfg,
        stationary_output,
        gui=False,
        realtime=False,
        slowdown=2,
        sim_model="full",
        start_pose="backwards",
    )
    assert backwards_plan["start_pose"] == backwards_report["start_pose"] == "backwards"
    assert backwards_plan["model_kind"] == "full_mujoco"
    assert backwards_report["collision_check"]["measured"] is True
    assert backwards_report["collision_check"]["frames_with_collisions"] == 0
    assert len(backwards_plan["frames"]) == len(full_plan["frames"])
    # The two experiments intentionally start from different robot poses. Their
    # saved start_pose and numeric reports make the dependency reproducible.
    final_left = np.linalg.norm(
        np.asarray(full_plan["frames"][-1]["arms"]["left"]["joint_deg"])
        - np.asarray(backwards_plan["frames"][-1]["arms"]["left"]["joint_deg"])
    )
    assert final_left > 0
    from telegrip.mocopi.dataset.verify import verify_dataset

    verified = verify_dataset(tmp_path / "full-joints", sample_frames=2)
    assert verified["status"] == "ok" and verified["dataset_kind"] == "joints"
    tampered = tmp_path / "tampered-joints"
    shutil.copytree(tmp_path / "full-joints", tampered)
    tampered_meta = tampered / "meta" / "mocopi_joint_dataset.json"
    document = json.loads(tampered_meta.read_text())
    document["hardware_replay_allowed"] = True
    tampered_meta.write_text(json.dumps(document))
    tampered_report = verify_dataset(tampered, sample_frames=1)
    assert tampered_report["status"] == "failed"
    assert "preview_model_simulation_only" in tampered_report["errors"]
    full_data = dataset_type("local/full-joints", root=tmp_path / "full-joints", video_backend="pyav")
    assert len(full_data) == len(full_plan["frames"]) - 1
    np.testing.assert_allclose(
        full_data[0]["observation.state"],
        np.concatenate([full_plan["frames"][0]["arms"][side]["joint_deg"] for side in ("right", "left")]),
        atol=1e-5,
    )
    alternate_xml = tmp_path / "changed-model.xml"
    alternate_xml.write_bytes(Path(cfg["robot"]["mujoco_full_model"]).read_bytes() + b"\n")
    changed_cfg = deepcopy(cfg)
    changed_cfg["robot"]["mujoco_full_model"] = str(alternate_xml)
    full_simulation = FullMujocoSimulation(cfg["robot"], gui=False)
    try:
        with pytest.raises(ValueError, match="MuJoCo model differs"):
            validate_plan(
                full_plan, changed_cfg, full_simulation, allow_synthetic=True, allow_preview_model=True
            )
        with pytest.raises(ValueError, match="simulation-only"):
            validate_plan(full_plan, cfg, full_simulation, allow_synthetic=True)
    finally:
        full_simulation.close()

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
    import hashlib
    import os
    import subprocess
    from copy import deepcopy
    from types import SimpleNamespace

    pytest.importorskip("pybullet")
    from scipy.spatial.transform import Rotation
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
            "slowdown": 1,
            "expected_frames": 4,
            "urdf_sha256": model_fingerprint(cfg),
            "joint_names": list(JOINTS),
            "gripper_included": False,
            "frames": [],
            "failures": [],
        }
        for index in range(4):
            frame = {"time_s": index / 30, "source_frame": index, "arms": {}}
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
        result = validate_plan(plan, cfg, simulation, allow_synthetic=True)
        assert result["frames"] == 4
        assert result["simulated_orientation_error_deg"]["left"]["max"] < 1e-5
        rotated = deepcopy(plan)
        rotated_arm = rotated["frames"][1]["arms"]["left"]
        rotated_pose = rotated_arm["target_pose"] = list(rotated_arm["target_pose"])
        rotated_pose[3:] = (
            (Rotation.from_euler("x", 35, degrees=True) * Rotation.from_quat(rotated_pose[3:]))
            .as_quat()
            .tolist()
        )
        assert validate_plan(rotated, cfg, simulation, allow_synthetic=True)[
            "simulated_orientation_error_deg"
        ]["left"]["max"] == pytest.approx(35)
        with pytest.raises(ValueError, match="orientation error"):
            validate_plan(rotated, cfg, simulation, allow_synthetic=True, max_sim_orientation_error_deg=20)
        validate_plan(rotated, cfg, simulation, allow_synthetic=True, max_sim_orientation_error_deg=40)
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

        rejected_robot = FakeRobot()
        with pytest.raises(ValueError, match="orientation error"):
            execute_plan(
                rotated,
                cfg,
                simulation,
                rejected_robot,
                tmp_path / "rejected.json",
                realtime=False,
                allow_synthetic=True,
                max_sim_orientation_error_deg=20,
            )
        assert not rejected_robot.left_bus.is_connected and not rejected_robot.right_bus.is_connected
        assert not rejected_robot.commands and not (tmp_path / "rejected.json").exists()
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
        from telegrip.mocopi.dataset.compare import compare_real

        plan_file = tmp_path / "plan.json"
        plan_file.write_text(json.dumps(plan))
        dataset_type = pytest.importorskip("lerobot.datasets.lerobot_dataset").LeRobotDataset
        from telegrip.mocopi.dataset.real_dataset import export_real_dataset

        timed = deepcopy(report)
        for index, sample in enumerate(timed["frames"]):
            sample["elapsed_s"] = index * 0.04
        timed_file = tmp_path / "timed-feedback.json"
        timed_file.write_text(json.dumps(timed))
        feedback_root = tmp_path / "feedback-dataset"
        feedback_meta = export_real_dataset([(plan_file, timed_file)], feedback_root, "local/feedback")
        feedback = dataset_type("local/feedback", root=feedback_root)
        assert len(feedback) == 3 and feedback.meta.fps == 30
        assert feedback_meta["human_capture_source"] == "fake" and not feedback_meta["images_included"]
        assert (
            feedback_meta["episodes"][0]["report_sha256"]
            == hashlib.sha256(timed_file.read_bytes()).hexdigest()
        )
        assert feedback_meta["episodes"][0]["mean_read_interval_s"] == pytest.approx(0.04)
        assert feedback_meta["episodes"][0]["observed_read_fps"] == pytest.approx(25)
        for index in range(len(feedback)):
            row, current, following = feedback[index], timed["frames"][index], timed["frames"][index + 1]
            expected = np.concatenate(
                [current["arms"][side]["measured_joint_deg"] for side in ("right", "left")]
            )
            action = np.concatenate([following["arms"][side]["sent_joint_deg"] for side in ("right", "left")])
            np.testing.assert_allclose(row["observation.state"], expected, atol=1e-5)
            np.testing.assert_allclose(row["action"], action, atol=1e-5)
            np.testing.assert_allclose(
                row["observation.execution.time"], [index * 0.04, (index + 1) * 0.04, index / 30]
            )
            np.testing.assert_array_equal(row["observation.execution.frame"], [index, index + 1, index])
        assert not any("images" in key or "gripper" in key for key in feedback.meta.features)
        invalid_timing = deepcopy(timed)
        invalid_timing["frames"][1]["elapsed_s"] = invalid_timing["frames"][0]["elapsed_s"]
        invalid_file = tmp_path / "invalid-timing.json"
        invalid_file.write_text(json.dumps(invalid_timing))
        with pytest.raises(ValueError, match="increasing"):
            export_real_dataset([(plan_file, invalid_file)], tmp_path / "bad-timing", "local/bad-timing")
        assert not (tmp_path / "bad-timing").exists()
        partial = deepcopy(timed)
        partial["status"], partial["frames"] = "aborted", partial["frames"][:3]
        partial_file = tmp_path / "partial-feedback.json"
        partial_file.write_text(json.dumps(partial))
        partial_meta = export_real_dataset(
            [(plan_file, timed_file), (plan_file, partial_file)], tmp_path / "partial", "local/partial"
        )
        assert len(partial_meta["episodes"]) == 2
        assert partial_meta["episodes"][1]["report_status"] == "aborted"
        assert partial_meta["episodes"][1]["frames"] == 2
        assert dataset_type("local/partial", root=tmp_path / "partial").num_episodes == 2
        mismatched = deepcopy(timed)
        mismatched["plan_sha256"] = "0" * 64
        mismatch_file = tmp_path / "mismatched.json"
        mismatch_file.write_text(json.dumps(mismatched))
        with pytest.raises(ValueError, match="does not match"):
            export_real_dataset([(plan_file, mismatch_file)], tmp_path / "mismatch", "local/mismatch")
        cli = subprocess.run(
            [
                sys.executable,
                "-m",
                "telegrip.mocopi.dataset",
                "convert-real",
                "--run",
                str(plan_file),
                str(timed_file),
                "--output",
                str(tmp_path / "feedback-cli"),
                "--repo-id",
                "local/feedback-cli",
            ],
            capture_output=True,
            text=True,
            check=False,
            env={
                **os.environ,
                "PYTHONPATH": os.pathsep.join(
                    [
                        str(Path(__file__).resolve().parents[2] / "telegrip_teleoperation"),
                        os.environ.get("PYTHONPATH", ""),
                    ]
                ),
            },
        )
        assert cli.returncode == 0, cli.stderr
        assert json.loads(cli.stdout)["episodes"][0]["frames"] == 3
        reference = tmp_path / "physical-measurements.csv"
        lines = ["frame,side,x_m,y_m,z_m\n"]
        for side in ("left", "right"):
            joint = np.r_[plan["frames"][-1]["arms"][side]["joint_deg"], 0.0]
            position, _ = simulation.arms[side].solver.fk_solver.compute(joint)
            position = position + [0.01, 0, 0]
            lines.append(f"3,{side},{position[0]},{position[1]},{position[2]}\n")
        reference.write_text("".join(lines))
        comparison = compare_real(cfg, plan_file, tmp_path / "real-report.json", reference)
        assert comparison["measured_frames"] == 4
        for side in ("left", "right"):
            assert comparison["tracking"][side]["tcp_sim_vs_encoder_fk_m"]["max"] < 1e-7
            assert comparison["external_reference"]["measurements"][side]["count"] == 1
            assert comparison["external_reference"]["measurements"][side]["fk_model_error_m"][
                "mean"
            ] == pytest.approx(0.01)
        reference.write_text("frame,side,x_m,y_m,z_m\n100,left,0,0,0\n")
        with pytest.raises(ValueError, match="not present"):
            compare_real(cfg, plan_file, tmp_path / "real-report.json", reference)
        reference.write_text("frame,side,x_m,y_m,z_m\n3,left,nan,0,0\n")
        with pytest.raises(ValueError, match="finite metres"):
            compare_real(cfg, plan_file, tmp_path / "real-report.json", reference)
        altered = deepcopy(plan)
        altered["frames"][0]["arms"]["left"]["joint_deg"][0] += 0.1
        plan_file.write_text(json.dumps(altered))
        with pytest.raises(ValueError, match="does not match"):
            compare_real(cfg, plan_file, tmp_path / "real-report.json")
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


def test_head_camera_stream_keeps_image_and_acquisition_timestamp():
    import time

    pytest.importorskip("cv2")
    from telegrip.mocopi.dataset.image_stream import HeadFrameClient, HeadFrameServer

    server = HeadFrameServer(0)
    client = None
    try:
        client = HeadFrameClient(server.port)
        assert server.connected.wait(2)
        image = np.zeros((40, 80, 3), dtype=np.uint8)
        image[:, :, 2] = 230
        stamp = time.monotonic()
        server.publish(stamp, image)
        received_stamp, received = client.read()
        assert received_stamp == pytest.approx(stamp, abs=1e-9)
        assert received.shape == image.shape and received[:, :, 2].mean() > 200
        assert client.last_jpeg.startswith(b"\xff\xd8")
        assert client.last_frame_id == 0
    finally:
        if client:
            client.close()
        server.close()


def test_live_local_image_and_slam_protocols_feed_synchronized_capture(tmp_path):
    import socket
    import threading
    import time

    from scipy.spatial.transform import Rotation

    pytest.importorskip("cv2")
    from telegrip.mocopi.calibration import HeadCalibration
    from telegrip.mocopi.dataset.capture import FakeMocopi
    from telegrip.mocopi.dataset.evaluate import session_metrics
    from telegrip.mocopi.dataset.image_stream import HeadFrameClient, HeadFrameServer
    from telegrip.mocopi.geometry import pose
    from telegrip.mocopi.slam import SlamReceiver
    from telegrip.mocopi.sources import fake_pair

    cfg = load_config(CONFIG)
    cfg["camera"].update(width=96, height=64, fps=20)
    mocopi = FakeMocopi(20)
    slam = SlamReceiver(0)
    slam_port = slam.socket.getsockname()[1]
    server = HeadFrameServer(0)
    client = HeadFrameClient(server.port)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop = threading.Event()
    producer_errors = []
    assert server.connected.wait(2)

    def produce():
        try:
            while not stop.is_set():
                stamp = time.monotonic()
                _, sample = fake_pair(stamp - mocopi.start, stamp)
                udp.sendto(
                    json.dumps(
                        {
                            "schema": 1,
                            "convention": "T_S_C_ros",
                            "timestamp": stamp,
                            "position": sample.camera[:3, 3].tolist(),
                            "quaternion": Rotation.from_matrix(sample.camera[:3, :3]).as_quat().tolist(),
                            "valid": True,
                            "session": "fake",
                            "intrinsics_id": "fake",
                        }
                    ).encode(),
                    ("127.0.0.1", slam_port),
                )
                server.publish(stamp, np.full((64, 96, 3), 100, dtype=np.uint8))
                stop.wait(0.05)
        except Exception as exc:
            producer_errors.append(exc)

    thread = threading.Thread(target=produce, daemon=True)
    thread.start()
    extrinsic = pose([0.035, -0.015, 0.06], Rotation.from_rotvec([0.2, -0.1, 0.15]).as_quat())
    calibration = HeadCalibration(1.7, pose(), extrinsic, {}, "fake", "fake")
    root = tmp_path / "streamed"
    try:
        capture_session(
            cfg,
            root,
            0.45,
            "stream test",
            fake=True,
            head_source="slam",
            camera=client,
            receiver=mocopi,
            slam_receiver=slam,
            calibration=calibration,
        )
        report = session_metrics(root, fps=20)
        assert report["usable_dataset_frames"] >= 4
        assert report["camera_to_slam_abs_offset_ms"]["max"] < 0.001
        assert report["camera_stream_missing_frames"] >= 0
        assert not producer_errors
    finally:
        stop.set()
        thread.join(timeout=2)
        udp.close()
        server.close()


def test_slam_head_corrects_dataset_but_keeps_raw_mocopi_and_detects_loss(tmp_path):
    pytest.importorskip("cv2")
    dataset_type = pytest.importorskip("lerobot.datasets.lerobot_dataset").LeRobotDataset
    from telegrip.mocopi.dataset.convert import convert_sessions
    from telegrip.mocopi.dataset.evaluate import session_metrics
    from telegrip.mocopi.dataset.session import Session, aligned_segments
    from telegrip.mocopi.geometry import inverse, validate_pose

    cfg = load_config(CONFIG)
    cfg["camera"].update(width=96, height=64, fps=20)
    root = tmp_path / "slam"
    metadata = capture_session(cfg, root, 0.65, "move cup", fake=True, head_source="slam")
    assert metadata["head_pose_source"] == "slam" and metadata["slam_frames"] > 0
    # Add a known global head displacement to every SLAM camera pose. The
    # corrected head and wrists must shift, while the raw mocopi data remain.
    slam_path = root / "slam.jsonl"
    slam = rows(slam_path)
    for row in slam:
        row["camera_pose"][0][3] += 0.05 / 1.7
    slam_path.write_text("".join(json.dumps(row) + "\n" for row in slam))
    session = Session(root)
    segments, _ = aligned_segments(session, 20, 0.08)
    assert segments and len(segments[0]) >= 2
    first = segments[0][0]
    raw_head = validate_pose(session.motion[first["motion"]]["bones"]["10"])
    expected_head = session.calibration.head(session.slam[first["slam"]]["camera_pose"])
    world_from_mocopi = expected_head @ inverse(raw_head)
    expected_hand = world_from_mocopi @ validate_pose(session.motion[first["motion"]]["bones"]["14"])
    output = tmp_path / "slam-dataset"
    manifest = convert_sessions([root], output, "local/slam-test", fps=20)
    dataset = dataset_type("local/slam-test", root=output, video_backend="pyav")
    actual = dataset[0]
    np.testing.assert_allclose(actual["observation.state"][:3], expected_head[:3, 3], atol=1e-6)
    np.testing.assert_allclose(actual["observation.state"][21:24], expected_hand[:3, 3], atol=1e-6)
    np.testing.assert_allclose(actual["observation.mocopi.raw_state"][:3], raw_head[:3, 3], atol=1e-6)
    assert np.linalg.norm(actual["observation.state"][:3] - actual["observation.mocopi.raw_state"][:3]) > 0.03
    assert "observation.slam.camera_pose" in dataset.meta.features
    assert (
        manifest["head_pose_source"] == "slam" and manifest["sessions"][0]["head_calibration"]["scale"] == 1.7
    )
    report = session_metrics(root, fps=20)
    assert report["slam_frames"] == metadata["slam_frames"]
    assert report["camera_frames_without_valid_slam"] == 0
    assert report["hands_coordinate_frame"].startswith("calibrated_world")
    # An explicitly invalid SLAM pose is never treated as a valid aligned frame.
    lost_index = segments[0][len(segments[0]) // 2]["slam"]
    session.slam[lost_index]["valid"] = False
    _, quality = aligned_segments(session, 20, 0.08)
    assert quality["unsynchronized_samples"] >= 1
    session.calibration.world_from_map[0, 3] += 1.0
    with pytest.raises(ValueError, match="alignment inconsistent"):
        aligned_segments(session, 20, 0.08)


def test_slam_map_mismatch_aborts_recording_before_camera_capture(tmp_path):
    from telegrip.mocopi.calibration import HeadCalibration
    from telegrip.mocopi.geometry import pose

    calibration = HeadCalibration(1.7, pose(), pose(), {}, "wrong-map", "fake")
    root = tmp_path / "map-mismatch"
    with pytest.raises(RuntimeError, match="map or intrinsics changed"):
        capture_session(
            load_config(CONFIG), root, 0.1, "test", fake=True, head_source="slam", calibration=calibration
        )
    meta = json.loads((root / "session.json").read_text())
    assert meta["status"] == "aborted" and meta["camera_frames"] == 0


def test_metrics_svg_renders_missing_values_and_provenance(tmp_path):
    from telegrip.mocopi.dataset.report_plot import export_metrics_svg

    first = tmp_path / "quality.json"
    first.write_text(
        json.dumps({"schema": "dual_scorpion.mocopi.quality.v1", "camera_fps": 19.8, "task": "first"})
    )
    second = tmp_path / "simulation.json"
    second.write_text(
        json.dumps(
            {
                "schema": "dual_scorpion.mocopi.simulation_metrics.v1",
                "tracking": {"left": {"position_m": {"max": 0.021}}},
                "task": "second",
            }
        )
    )
    plot = tmp_path / "metrics.svg"
    summary = export_metrics_svg([first, second], ["camera_fps", "tracking.left.position_m.max"], plot)
    assert summary["schema"] == "dual_scorpion.mocopi.metrics_plot.v1"
    assert summary["reports"] == 2 and summary["output"] == str(plot)
    assert summary["missing_values"] == {
        "camera_fps": 1,
        "tracking.left.position_m.max": 1,
    }
    text = plot.read_text(encoding="utf-8")
    assert text.startswith('<?xml version="1.0" encoding="UTF-8"?>')
    assert "camera_fps" in text and "tracking.left.position_m.max" in text
    assert "19.8" in text and "0.021" in text and "missing" in text
    assert "Dual Scorpion mocopi metrics comparison" in text

    with pytest.raises(FileExistsError):
        export_metrics_svg([first], ["camera_fps"], plot)
    with pytest.raises(ValueError, match="duplicates"):
        export_metrics_svg([first], ["camera_fps", "camera_fps"], tmp_path / "duplicate.svg")
    with pytest.raises(ValueError, match="no numeric value"):
        export_metrics_svg([first], ["not.a.metric"], tmp_path / "empty.svg")


def test_metrics_csv_combines_quality_and_simulation_reports(tmp_path):
    import csv

    pytest.importorskip("cv2")
    pytest.importorskip("mujoco")
    pytest.importorskip("pybullet")
    pytest.importorskip("lerobot.datasets.lerobot_dataset")
    from telegrip.mocopi.dataset.convert import convert_sessions
    from telegrip.mocopi.dataset.evaluate import session_metrics
    from telegrip.mocopi.dataset.replay import replay_sim
    from telegrip.mocopi.dataset.report_csv import export_metrics_csv

    cfg = load_config(CONFIG)
    cfg["camera"].update(width=96, height=64, fps=20)
    session = tmp_path / "raw"
    capture_session(cfg, session, 0.5, "csv check", fake=True)
    quality = session_metrics(session, fps=20)
    quality_file = tmp_path / "quality.json"
    quality_file.write_text(json.dumps(quality))
    output = tmp_path / "dataset"
    convert_sessions([session], output, "local/csv-check", fps=20)
    _, simulation = replay_sim(cfg, output, gui=False, realtime=False, sim_model="full")
    sim_file = tmp_path / "sim.json"
    sim_file.write_text(json.dumps(simulation))

    table = tmp_path / "metrics.csv"
    summary = export_metrics_csv([quality_file, sim_file], table)
    assert summary["schema"] == "dual_scorpion.mocopi.metrics_table.v1"
    assert summary["reports"] == 2 and summary["output"] == str(table)
    parsed = list(csv.DictReader(table.open(newline="", encoding="utf-8")))
    assert [row["report"] for row in parsed] == [str(quality_file), str(sim_file)]
    assert [row["schema"] for row in parsed] == [quality["schema"], simulation["schema"]]
    # Quality-only and simulation-only columns share one table; missing cells stay empty.
    assert float(parsed[0]["camera_fps"]) == quality["camera_fps"]
    assert int(parsed[0]["usable_dataset_frames"]) == quality["usable_dataset_frames"]
    assert parsed[0]["tracking.left.position_m.max"] == ""
    assert parsed[1]["camera_fps"] == ""
    assert (
        float(parsed[1]["tracking.right.position_m.max"])
        == simulation["tracking"]["right"]["position_m"]["max"]
    )
    assert int(parsed[1]["ik_failures.count"]) == len(simulation["ik_failures"])
    assert parsed[1]["model_kind"] == "full_mujoco" and parsed[1]["task"] == "csv check"
    # Unmeasured CAD joint limits stay empty instead of a fabricated margin.
    assert simulation["tracking"]["left"]["joint_limit_margin_deg"]["max"] is None
    assert parsed[1]["tracking.left.joint_limit_margin_deg.max"] == ""
    assert parsed[0]["episode"] == "" and parsed[1]["episode"] == "0"

    with pytest.raises(FileExistsError):
        export_metrics_csv([quality_file], table)
    with pytest.raises(ValueError, match="more than once"):
        export_metrics_csv([quality_file, quality_file], tmp_path / "duplicate.csv")
    other = tmp_path / "other.json"
    other.write_text(json.dumps({"schema": "some.other.report"}))
    with pytest.raises(ValueError, match="Not a mocopi evaluation report"):
        export_metrics_csv([other], tmp_path / "other.csv")


def test_verify_dataset_reads_human_dataset_and_rejects_wrong_claims(tmp_path):
    pytest.importorskip("cv2")
    pytest.importorskip("lerobot.datasets.lerobot_dataset")
    from telegrip.mocopi.dataset.convert import convert_sessions
    from telegrip.mocopi.dataset.verify import verify_dataset

    cfg = load_config(CONFIG)
    cfg["camera"].update(width=96, height=64, fps=20)
    session = tmp_path / "raw"
    capture_session(cfg, session, 0.5, "verify check", fake=True)
    output = tmp_path / "dataset"
    convert_sessions([session], output, "local/verify-check", fps=20)

    report = verify_dataset(output, sample_frames=4)
    assert report["schema"] == "dual_scorpion.mocopi.verify.v1"
    assert report["status"] == "ok" and not report["errors"]
    assert report["dataset_kind"] == "human" and report["frames"] >= 4
    assert report["sampled_pairs"] >= 1
    assert report["capture_grid_error_s"] <= 1e-5
    assert report["camera_mocopi_error_s"] <= report["max_sync_s"]
    checks = {item["check"]: item["ok"] for item in report["checks"]}
    assert checks["gripper_not_recorded"] and checks["no_gripper_feature"]
    assert checks["raw_mocopi_state_present"] and checks["action_is_next_measured_state"]
    assert checks["head_video_decodable"] and checks["timestamps_are_episode_grid"]
    assert checks["capture_features_present"] and checks["capture_times_finite"]
    assert checks["capture_grid_matches_episode_timestamp"]
    assert checks["camera_mocopi_sync_within_manifest"]
    assert checks["capture_frame_ids_nondecreasing"]

    import shutil

    tight_sync = tmp_path / "tight-sync"
    shutil.copytree(output, tight_sync)
    tight_meta_path = tight_sync / "meta" / "mocopi_dataset.json"
    tight_meta = json.loads(tight_meta_path.read_text(encoding="utf-8"))
    tight_meta["max_sync_s"] = 1e-9
    tight_meta_path.write_text(json.dumps(tight_meta), encoding="utf-8")
    tight_report = verify_dataset(tight_sync, sample_frames=1)
    assert tight_report["status"] == "failed"
    assert "camera_mocopi_sync_within_manifest" in tight_report["errors"]

    with pytest.raises(ValueError, match="sample_frames"):
        verify_dataset(output, sample_frames=0)
    with pytest.raises(ValueError, match="Not a mocopi dataset"):
        verify_dataset(tmp_path)

    import shutil

    tampered = tmp_path / "tampered"
    shutil.copytree(output, tampered)
    metadata = tampered / "meta" / "mocopi_dataset.json"
    document = json.loads(metadata.read_text())
    document["gripper_recorded"] = True
    metadata.write_text(json.dumps(document))
    tampered_report = verify_dataset(tampered, sample_frames=1)
    assert tampered_report["status"] == "failed" and "gripper_not_recorded" in tampered_report["errors"]
