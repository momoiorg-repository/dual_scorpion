"""Convert timed human demonstrations to the repository's LeRobot v3 format."""

import hashlib
import json
from pathlib import Path

import numpy as np

from .session import ACTION_NAMES, POSE_BONES, STATE_NAMES, Session, aligned_segments, pose_vectors


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def convert_sessions(inputs, output, repo_id, *, fps=30, max_sync_s=0.08, allow_mixed_sources=False):
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
        "action": {"dtype": "float32", "shape": (len(ACTION_NAMES),), "names": ACTION_NAMES},
        "observation.capture.camera_time_s": {"dtype": "float64", "shape": (1,), "names": None},
        "observation.capture.mocopi_time_s": {"dtype": "float64", "shape": (1,), "names": None},
        "observation.capture.grid_time_s": {"dtype": "float64", "shape": (1,), "names": None},
        "observation.capture.mocopi_frame_id": {"dtype": "int64", "shape": (1,), "names": None},
    }
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
        "coordinate_frame": "mocopi_global_x_forward_y_left_z_up",
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
            for segment in segments:
                if len(segment) < 2:
                    continue
                previous = {}
                vectors = [pose_vectors(session.motion[item["motion"]], previous) for item in segment]
                for index, item in enumerate(segment[:-1]):
                    image = cv2.imread(str(session.root / session.camera[item["camera"]]["file"]))
                    if image is None or image.shape != (height, width, 3):
                        raise ValueError(f"Invalid camera frame {item['camera']} in {session.root}")
                    state = np.concatenate([vectors[index][name] for name, _ in POSE_BONES])
                    action = np.concatenate(
                        [vectors[index + 1][f"{side}_hand"] for side in ("left", "right")]
                    )
                    dataset.add_frame(
                        {
                            "task": session.meta["task"],
                            "observation.images.head": cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
                            "observation.state": state,
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
                        }
                    )
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
