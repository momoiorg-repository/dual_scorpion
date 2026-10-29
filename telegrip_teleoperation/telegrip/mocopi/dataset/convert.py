"""Convert timed human demonstrations to the repository's LeRobot v3 format."""

import hashlib
import json
from pathlib import Path

import numpy as np

from .session import (
    ACTION_NAMES,
    POSE_BONES,
    STATE_NAMES,
    Session,
    aligned_segments,
    fused_pose_vectors,
    pose_vectors,
)


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def convert_sessions(
    inputs,
    output,
    repo_id,
    *,
    fps=30,
    max_sync_s=0.08,
    allow_mixed_sources=False,
    allow_unverified_reference=False,
):
    """Create one dataset; episodes split at missing/unsynchronized intervals.

    `action` is the next measured human hand pose, not a robot joint command.
    The final sample of each contiguous segment has no next pose and is omitted.
    """
    import cv2

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    if not inputs or not repo_id.strip():
        raise ValueError("At least one session and a nonempty repo_id are required")
    root = Path(output)
    if root.exists():
        raise FileExistsError(f"Dataset output already exists: {root}")
    sessions = [Session(path) for path in inputs]
    sources = {session.meta["source"] for session in sessions}
    if len(sources) > 1 and not allow_mixed_sources:
        raise ValueError("Refusing to mix fake and real sessions without --allow-mixed-sources")
    head_sources = {session.meta["head_pose_source"] for session in sessions}
    if len(head_sources) != 1:
        raise ValueError("Cannot mix mocopi and SLAM head coordinate frames in one dataset")
    head_source = head_sources.pop()
    if len(sessions) > 1 and not allow_unverified_reference:
        references = {session.meta["mocopi"].get("reference_id") for session in sessions}
        if None in references or len(references) != 1:
            raise ValueError(
                "Multiple sessions require the same --reference-id, or --allow-unverified-reference"
            )
        if (
            head_source == "slam"
            and len({session.meta["head_calibration_sha256"] for session in sessions}) != 1
        ):
            raise ValueError(
                "SLAM sessions use different head calibrations; review references before combining"
            )
    sizes = {(int(s.meta["camera"]["height"]), int(s.meta["camera"]["width"])) for s in sessions}
    if len(sizes) != 1:
        raise ValueError("All sessions must have the same camera resolution")
    height, width = sizes.pop()
    if min(height, width) <= 0 or height % 2 or width % 2:
        raise ValueError("Video width/height must be positive even numbers")
    plans = [(session, *aligned_segments(session, fps, max_sync_s)) for session in sessions]
    if not any(len(segment) >= 2 for _, segments, _ in plans for segment in segments):
        raise ValueError("No synchronized segments with at least two samples")
    features = {
        "observation.images.head": {
            "dtype": "video",
            "shape": (3, height, width),
            "names": ["channels", "height", "width"],
        },
        "observation.state": {"dtype": "float32", "shape": (len(STATE_NAMES),), "names": STATE_NAMES},
        "observation.mocopi.raw_state": {
            "dtype": "float32",
            "shape": (len(STATE_NAMES),),
            "names": STATE_NAMES,
        },
        "action": {"dtype": "float32", "shape": (len(ACTION_NAMES),), "names": ACTION_NAMES},
        "observation.capture.camera_time_s": {"dtype": "float64", "shape": (1,), "names": None},
        "observation.capture.mocopi_time_s": {"dtype": "float64", "shape": (1,), "names": None},
        "observation.capture.grid_time_s": {"dtype": "float64", "shape": (1,), "names": None},
        "observation.capture.mocopi_frame_id": {"dtype": "int64", "shape": (1,), "names": None},
        "observation.capture.camera_frame_id": {"dtype": "int64", "shape": (1,), "names": None},
    }
    if head_source == "slam":
        features["observation.slam.camera_pose"] = {
            "dtype": "float32",
            "shape": (7,),
            "names": ["x_map", "y_map", "z_map", "qx", "qy", "qz", "qw"],
        }
        features["observation.capture.slam_time_s"] = {"dtype": "float64", "shape": (1,), "names": None}
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=features,
        root=root,
        robot_type="human_mocopi",
        use_videos=True,
        vcodec="h264",
        image_writer_threads=2,
    )
    manifest = {
        "schema": "dual_scorpion.mocopi.lerobot.v1",
        "repo_id": repo_id,
        "fps": fps,
        "max_sync_s": max_sync_s,
        "coordinate_frame": (
            "calibrated_world_x_forward_y_left_z_up"
            if head_source == "slam"
            else "mocopi_global_x_forward_y_left_z_up"
        ),
        "raw_mocopi_coordinate_frame": "mocopi_global_x_forward_y_left_z_up",
        "head_pose_source": head_source,
        "slam_camera_translation_unit": "unscaled_slam_map_units" if head_source == "slam" else None,
        "coordinate_reference_policy": (
            "single_session"
            if len(sessions) == 1
            else "unverified_override"
            if allow_unverified_reference
            else "operator_declared_matching_reference_id"
        ),
        "position_unit": "metre",
        "quaternion_order": "xyzw",
        "state_bones": [name for name, _ in POSE_BONES],
        "action_semantics": "next measured human left/right hand pose; not robot joint actions",
        "gripper_recorded": False,
        "robot_ik_at_capture": False,
        "timestamp_semantics": "LeRobot timestamp is episode grid time; capture times are relative to each raw session",
        "episodes": [],
        "sessions": [],
    }
    try:
        for session, segments, quality in plans:
            session_id = len(manifest["sessions"])
            manifest["sessions"].append(
                {
                    "session_name": session.root.name,
                    "source": session.meta["source"],
                    "head_pose_source": session.meta["head_pose_source"],
                    "task": session.meta["task"],
                    "camera": session.meta["camera"],
                    "mocopi": session.meta["mocopi"],
                    "camera_index_sha256": _sha256(session.root / "camera.jsonl"),
                    "motion_sha256": _sha256(session.root / "mocopi.jsonl"),
                    **quality,
                }
            )
            if session.calibration:
                calibration = session.calibration
                manifest["sessions"][-1]["head_calibration"] = {
                    "scale": calibration.scale,
                    "world_from_map": calibration.world_from_map.tolist(),
                    "camera_from_head": calibration.camera_from_head.tolist(),
                    "quality": calibration.quality,
                    "session": calibration.session,
                    "intrinsics_id": calibration.intrinsics_id,
                    "sha256": session.meta["head_calibration_sha256"],
                }
                manifest["sessions"][-1]["slam_sha256"] = _sha256(session.root / "slam.jsonl")
            if session.intrinsics:
                manifest["sessions"][-1]["original_camera_intrinsics"] = session.intrinsics
            for segment in segments:
                if len(segment) < 2:
                    continue
                previous, raw_previous = {}, {}
                vectors = [fused_pose_vectors(session, item, previous) for item in segment]
                raw_vectors = [pose_vectors(session.motion[item["motion"]], raw_previous) for item in segment]
                for index, item in enumerate(segment[:-1]):
                    image = cv2.imread(str(session.root / session.camera[item["camera"]]["file"]))
                    if image is None or image.shape != (height, width, 3):
                        raise ValueError(f"Invalid camera frame {item['camera']} in {session.root}")
                    state = np.concatenate([vectors[index][name] for name, _ in POSE_BONES])
                    action = np.concatenate(
                        [vectors[index + 1][f"{side}_hand"] for side in ("left", "right")]
                    )
                    data = {
                        "task": session.meta["task"],
                        "observation.images.head": cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
                        "observation.state": state,
                        "observation.mocopi.raw_state": np.concatenate(
                            [raw_vectors[index][name] for name, _ in POSE_BONES]
                        ),
                        "action": action,
                        "observation.capture.camera_time_s": np.array(
                            [session.camera_times[item["camera"]]], dtype=np.float64
                        ),
                        "observation.capture.mocopi_time_s": np.array(
                            [session.motion_times[item["motion"]]], dtype=np.float64
                        ),
                        "observation.capture.grid_time_s": np.array([item["grid_s"]], dtype=np.float64),
                        "observation.capture.mocopi_frame_id": np.array(
                            [session.motion[item["motion"]]["frame_id"]], dtype=np.int64
                        ),
                        "observation.capture.camera_frame_id": np.array(
                            [session.camera[item["camera"]].get("frame_id", item["camera"])], dtype=np.int64
                        ),
                    }
                    if session.slam is not None:
                        from scipy.spatial.transform import Rotation

                        camera_pose = np.asarray(session.slam[item["slam"]]["camera_pose"])
                        data["observation.slam.camera_pose"] = np.r_[
                            camera_pose[:3, 3], Rotation.from_matrix(camera_pose[:3, :3]).as_quat()
                        ].astype(np.float32)
                        data["observation.capture.slam_time_s"] = np.array(
                            [session.slam_times[item["slam"]]], dtype=np.float64
                        )
                    dataset.add_frame(data)
                dataset.save_episode()
                manifest["episodes"].append(
                    {
                        "episode_index": len(manifest["episodes"]),
                        "source_session": session_id,
                        "first_grid_time_s": segment[0]["grid_s"],
                        "frames": len(segment) - 1,
                        "duplicate_camera_frames": sum(
                            a["camera"] == b["camera"] for a, b in zip(segment, segment[1:], strict=False)
                        ),
                        "duplicate_mocopi_frames": sum(
                            a["motion"] == b["motion"] for a, b in zip(segment, segment[1:], strict=False)
                        ),
                    }
                )
    finally:
        dataset.finalize()
    (root / "meta" / "mocopi_dataset.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return manifest
