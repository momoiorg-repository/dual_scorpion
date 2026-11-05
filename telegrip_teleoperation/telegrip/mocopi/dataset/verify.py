"""Read a finalized mocopi dataset and check the claims its metadata makes.

This is a read-only control step between dataset creation and training. It
reloads the LeRobot data through the same public API a trainer uses, decodes the
recorded head-camera video, and verifies that ``action`` really is the next
frame's pose/joint state rather than a fabricated robot command.
"""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..obstacles import obstacles_sha256

HUMAN_SCHEMA = "dual_scorpion.mocopi.lerobot.v1"
JOINT_SCHEMA = "dual_scorpion.mocopi.kinematic_joint_dataset.v1"
# Position of the left/right hand block inside the 49-value human state.
HAND_SLICES = {"left": slice(21, 28), "right": slice(42, 49)}
CAPTURE_GRID_TOLERANCE_S = 1e-5


def _sample_positions(span, count):
    """Evenly spaced positions in 0..span-1, including both ends."""
    if span <= 0:
        return []
    if span == 1 or count == 1:
        return [0]
    return sorted({round(step * (span - 1) / (count - 1)) for step in range(count)})


def verify_dataset(dataset_path, sample_frames=8):
    """Load a finalized dataset and return a report of structural and semantic checks."""
    if not isinstance(sample_frames, int) or isinstance(sample_frames, bool) or not 1 <= sample_frames <= 200:
        raise ValueError("sample_frames must be an integer between 1 and 200")
    root = Path(dataset_path).resolve()
    human_meta, joint_meta = (
        root / "meta" / "mocopi_dataset.json",
        root / "meta" / "mocopi_joint_dataset.json",
    )
    if human_meta.is_file():
        kind = "human"
        manifest = json.loads(human_meta.read_text(encoding="utf-8"))
        expected = {"schema": HUMAN_SCHEMA, "state": 49, "action": 14}
    elif joint_meta.is_file():
        kind = "joints"
        manifest = json.loads(joint_meta.read_text(encoding="utf-8"))
        expected = {"schema": JOINT_SCHEMA, "state": 14, "action": 14}
    else:
        raise ValueError("Not a mocopi dataset: metadata not found under meta/")

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    checks = []

    def check(name, ok, detail=""):
        checks.append({"check": name, "ok": bool(ok), "detail": str(detail)})

    check("manifest_schema", manifest.get("schema") == expected["schema"], manifest.get("schema"))
    check("gripper_not_recorded", manifest.get("gripper_recorded") is False)
    if kind == "joints":
        check("joint_unit_degrees", manifest.get("joint_unit") == "degree", manifest.get("joint_unit"))
        check(
            "robot_dataset_role",
            manifest.get("dataset_role") == "robot_imitation_kinematic",
            manifest.get("dataset_role"),
        )
        check(
            "kinematic_state_labelled",
            manifest.get("robot_state_source") == "kinematic_simulation_not_real_encoders",
            manifest.get("robot_state_source"),
        )
        check(
            "human_actions_kept_distinct",
            isinstance(manifest.get("human_action_semantics"), str)
            and "next measured" in manifest["human_action_semantics"],
            manifest.get("human_action_semantics"),
        )
        check(
            "gripper_not_commanded",
            manifest.get("gripper_commanded") is False and manifest.get("gripper_fixed_zero") is True,
            manifest.get("gripper_commanded"),
        )
        check(
            "tcp_metadata_present",
            isinstance(manifest.get("tcp"), dict)
            and manifest["tcp"].get("frame") == "l_link_8/r_link_8 origin",
            manifest.get("tcp"),
        )
        validation = manifest.get("validation", {})
        check(
            "quality_status_reported",
            validation.get("status") in ("ok", "ok_with_orientation_warnings"),
            validation.get("status"),
        )
        check(
            "ik_failures_not_hidden",
            isinstance(validation.get("ik_failure_frames"), list)
            and not validation["ik_failure_frames"],
            validation.get("ik_failure_frames"),
        )
        check(
            "collision_failures_not_hidden",
            isinstance(validation.get("collision_frames"), list)
            and not validation["collision_frames"],
            validation.get("collision_frames"),
        )
        retarget = manifest.get("retarget_validation", {})
        check("retarget_validation_present", isinstance(retarget, dict) and bool(retarget))
        check(
            "tcp_alignment_limitation_reported",
            retarget.get("verification", {}).get("tcp_physical_alignment") == "not_externally_measured",
            retarget.get("verification", {}).get("tcp_physical_alignment"),
        )
        # A dataset from the unmeasured CAD preview must never claim hardware replay.
        if manifest.get("model_kind") == "full_mujoco":
            check(
                "preview_model_simulation_only",
                manifest.get("hardware_replay_allowed") is False,
                manifest.get("hardware_replay_allowed"),
            )
            check(
                "obstacle_scene_present",
                isinstance(manifest.get("obstacles"), list),
                manifest.get("obstacles"),
            )
            check(
                "obstacle_scene_hash_matches",
                manifest.get("obstacles_sha256") == obstacles_sha256(manifest.get("obstacles", [])),
                manifest.get("obstacles_sha256"),
            )
            check(
                "obstacle_count_matches_scene",
                manifest.get("obstacle_count") == len(manifest.get("obstacles", [])),
                manifest.get("obstacle_count"),
            )

    dataset = LeRobotDataset(manifest["repo_id"], root=root, video_backend="pyav")
    features = dataset.meta.features
    check("dataset_not_empty", len(dataset) >= 1 and dataset.num_episodes >= 1, f"{len(dataset)} frames")
    check(
        "episode_count_matches_manifest",
        len(manifest.get("episodes", [])) == dataset.num_episodes,
        f"{dataset.num_episodes} episodes",
    )
    check("features_contain_head_image", "observation.images.head" in features)
    check(
        "state_shape_matches_schema",
        tuple(features.get("observation.state", {}).get("shape", ())) == (expected["state"],),
        features.get("observation.state", {}).get("shape"),
    )
    check(
        "action_shape_matches_schema",
        tuple(features.get("action", {}).get("shape", ())) == (expected["action"],),
        features.get("action", {}).get("shape"),
    )
    check("no_gripper_feature", not any("gripper" in name.lower() for name in features))
    if kind == "human":
        check("raw_mocopi_state_present", "observation.mocopi.raw_state" in features)
    else:
        check("sim_frame_present", "observation.sim.frame" in features)
    if kind == "joints":
        for name in (
            "observation.robot.tcp_pose",
            "observation.robot.target_pose",
            "observation.robot.error",
            "observation.robot.status",
            "observation.robot.collision",
            "observation.robot.warning",
            "observation.robot.plan_time_s",
        ):
            check(f"{name}_present", name in features)
        check(
            "capture_features_present",
            "observation.capture.camera_time_s" in features
            and "observation.capture.mocopi_time_s" in features
            and "observation.capture.grid_time_s" in features,
        )

    bounds = [
        (
            int(dataset.meta.episodes[episode]["dataset_from_index"]),
            int(dataset.meta.episodes[episode]["dataset_to_index"]),
        )
        for episode in range(dataset.num_episodes)
    ]
    sampled = sorted(
        {
            start + position
            for start, end in bounds
            for position in _sample_positions(end - start, sample_frames)
        }
    )
    pairs = []
    for start, end in bounds:
        span = end - start
        pairs.extend(
            (start + position, start + position + 1)
            for position in _sample_positions(span - 1, sample_frames)
        )
    image_shape = tuple(features["observation.images.head"]["shape"])
    mismatches, image_shapes, grid_error = 0.0, set(), 0.0
    robot_error_mismatch = robot_status_invalid = robot_collision_nonzero = 0
    for index in sampled:
        row = dataset[index]
        state = np.asarray(row["observation.state"], dtype=float)
        action = np.asarray(row["action"], dtype=float)
        if not (np.isfinite(state).all() and np.isfinite(action).all()):
            mismatches = float("inf")
        if kind == "joints":
            for name in (
                "observation.robot.tcp_pose",
                "observation.robot.target_pose",
                "observation.robot.error",
                "observation.robot.status",
                "observation.robot.collision",
                "observation.robot.warning",
                "observation.robot.plan_time_s",
            ):
                if not np.isfinite(np.asarray(row[name], dtype=float)).all():
                    mismatches = float("inf")
            target = np.asarray(row["observation.robot.target_pose"], dtype=float).reshape(2, 7)
            actual = np.asarray(row["observation.robot.tcp_pose"], dtype=float).reshape(2, 7)
            recorded_error = np.asarray(row["observation.robot.error"], dtype=float)
            expected_error = []
            for pose_index in range(2):
                expected_error.extend(
                    (
                        np.linalg.norm(target[pose_index, :3] - actual[pose_index, :3]),
                        np.rad2deg(
                            Rotation.from_matrix(
                                Rotation.from_quat(target[pose_index, 3:]).as_matrix().T
                                @ Rotation.from_quat(actual[pose_index, 3:]).as_matrix()
                            ).magnitude()
                        ),
                    )
                )
            robot_error_mismatch = max(
                robot_error_mismatch, float(np.max(np.abs(recorded_error - np.asarray(expected_error))))
            )
            status = np.asarray(row["observation.robot.status"], dtype=float)
            if (
                status.shape != (6,)
                or not np.allclose(status[[0, 3]], 1.0, atol=1e-6)
                or not np.allclose(status[[1, 2, 4, 5]], 0.0, atol=1e-6)
            ):
                robot_status_invalid += 1
            collision = np.asarray(row["observation.robot.collision"], dtype=float)
            if collision.shape != (2,) or collision[0] != 0:
                robot_collision_nonzero += 1
        image = row["observation.images.head"]
        image_shapes.add(tuple(image.shape))
        grid_error = max(
            grid_error,
            abs(float(row["timestamp"]) - int(row["frame_index"]) / float(dataset.fps)),
        )
    check("sampled_values_finite", mismatches == 0.0)
    check("head_video_decodable", image_shapes == {image_shape}, sorted(image_shapes))
    check("timestamps_are_episode_grid", grid_error <= 1e-6, f"max grid error {grid_error:g} s")

    difference = 0.0
    for index, following in pairs:
        current, next_row = dataset[index], dataset[following]
        state = np.asarray(next_row["observation.state"], dtype=float)
        if kind == "human":
            expected_action = np.concatenate([state[HAND_SLICES[side]] for side in ("left", "right")])
        else:
            expected_action = state
        difference = max(
            difference, float(np.max(np.abs(np.asarray(current["action"], dtype=float) - expected_action)))
        )
    check(
        "action_is_next_measured_state" if kind == "human" else "action_is_next_planned_state",
        difference <= 1e-5,
        f"max abs difference {difference:g}",
    )
    if kind == "joints":
        check("robot_tcp_error_matches_pose", robot_error_mismatch <= 1e-4, robot_error_mismatch)
        check("robot_ik_status_valid", robot_status_invalid == 0, robot_status_invalid)
        check("robot_collision_count_zero", robot_collision_nonzero == 0, robot_collision_nonzero)

    capture_grid_error_s = camera_mocopi_error_s = None
    if kind == "human":
        capture_features = {
            "camera": "observation.capture.camera_time_s",
            "mocopi": "observation.capture.mocopi_time_s",
            "grid": "observation.capture.grid_time_s",
            "camera_id": "observation.capture.camera_frame_id",
            "mocopi_id": "observation.capture.mocopi_frame_id",
        }
        check(
            "capture_features_present",
            all(name in features for name in capture_features.values()),
            sorted(set(capture_features.values()) - set(features)),
        )
        if all(name in features for name in capture_features.values()):
            max_sync_s = manifest.get("max_sync_s")
            check(
                "max_sync_s_positive",
                isinstance(max_sync_s, (int, float))
                and not isinstance(max_sync_s, bool)
                and np.isfinite(max_sync_s)
                and max_sync_s > 0,
                max_sync_s,
            )
            capture_grid_error_s = camera_mocopi_error_s = 0.0
            finite, frame_ids_ordered = True, True
            for start, end in bounds:
                episode = dataset.hf_dataset[start:end]
                camera = np.asarray(episode[capture_features["camera"]], dtype=float).reshape(-1)
                mocopi = np.asarray(episode[capture_features["mocopi"]], dtype=float).reshape(-1)
                grid = np.asarray(episode[capture_features["grid"]], dtype=float).reshape(-1)
                timestamp = np.asarray(episode["timestamp"], dtype=float).reshape(-1)
                camera_id = np.asarray(episode[capture_features["camera_id"]], dtype=np.int64).reshape(-1)
                mocopi_id = np.asarray(episode[capture_features["mocopi_id"]], dtype=np.int64).reshape(-1)
                if not (
                    np.isfinite(camera).all()
                    and np.isfinite(mocopi).all()
                    and np.isfinite(grid).all()
                    and np.isfinite(timestamp).all()
                ):
                    finite = False
                    continue
                capture_grid_error_s = max(
                    capture_grid_error_s,
                    float(np.max(np.abs((grid - grid[0]) - (timestamp - timestamp[0])))),
                )
                camera_mocopi_error_s = max(camera_mocopi_error_s, float(np.max(np.abs(camera - mocopi))))
                frame_ids_ordered = frame_ids_ordered and np.all(np.diff(camera_id) >= 0)
                frame_ids_ordered = frame_ids_ordered and np.all(np.diff(mocopi_id) >= 0)
            check("capture_times_finite", finite)
            check(
                "capture_grid_matches_episode_timestamp",
                finite and capture_grid_error_s <= CAPTURE_GRID_TOLERANCE_S,
                f"max grid error {capture_grid_error_s:g} s",
            )
            check(
                "camera_mocopi_sync_within_manifest",
                finite
                and isinstance(max_sync_s, (int, float))
                and not isinstance(max_sync_s, bool)
                and camera_mocopi_error_s <= float(max_sync_s),
                f"max camera/mocopi error {camera_mocopi_error_s:g} s, limit {max_sync_s}",
            )
            check("capture_frame_ids_nondecreasing", finite and frame_ids_ordered)

    status = "failed" if any(not item["ok"] for item in checks) else "ok"
    if (
        status == "ok"
        and kind == "joints"
        and manifest.get("validation", {}).get("status") == "ok_with_orientation_warnings"
    ):
        status = "warning"
    return {
        "schema": "dual_scorpion.mocopi.verify.v1",
        "dataset": str(root),
        "dataset_kind": kind,
        "episodes": dataset.num_episodes,
        "frames": len(dataset),
        "sampled_frames": len(sampled),
        "sampled_pairs": len(pairs),
        "max_sync_s": manifest.get("max_sync_s") if kind == "human" else None,
        "capture_grid_error_s": capture_grid_error_s,
        "camera_mocopi_error_s": camera_mocopi_error_s,
        "status": status,
        "quality_status": manifest.get("validation", {}).get("status") if kind == "joints" else None,
        "quality_warnings": manifest.get("validation", {}).get("orientation_warning_ranges")
        if kind == "joints"
        else None,
        "checks": checks,
        "errors": [item["check"] for item in checks if not item["ok"]],
        "note": "Read-only structural and semantic checks; not a measurement of mocopi or robot accuracy",
    }
