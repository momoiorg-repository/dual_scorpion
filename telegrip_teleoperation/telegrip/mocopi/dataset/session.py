"""Validated raw sessions and explicit nearest-sample alignment."""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..geometry import validate_pose
from .capture import REQUIRED_BONES, SCHEMA

# Order is part of the dataset schema and must not depend on a robot URDF.
POSE_BONES = (
    ("head", 10),
    ("left_shoulder", 12),
    ("left_elbow", 13),
    ("left_hand", 14),
    ("right_shoulder", 16),
    ("right_elbow", 17),
    ("right_hand", 18),
)
COMPONENTS = ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")
STATE_NAMES = [f"{name}.{component}" for name, _ in POSE_BONES for component in COMPONENTS]
ACTION_NAMES = [f"{side}_hand.{component}" for side in ("left", "right") for component in COMPONENTS]


def _read_rows(path):
    with path.open(encoding="utf-8") as file:
        rows = [json.loads(line) for line in file if line.strip()]
    if not rows:
        raise ValueError(f"No records in {path}")
    stamps = np.array([row["timestamp_s"] for row in rows], dtype=float)
    if not np.isfinite(stamps).all() or np.any(np.diff(stamps) <= 0):
        raise ValueError(f"Timestamps must be finite and strictly increasing: {path}")
    return rows, stamps


class Session:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.meta = json.loads((self.root / "session.json").read_text(encoding="utf-8"))
        if self.meta.get("schema") != SCHEMA or self.meta.get("status") != "complete":
            raise ValueError(f"Incomplete or unsupported recording: {self.root}")
        self.camera, self.camera_times = _read_rows(self.root / "camera.jsonl")
        self.motion, self.motion_times = _read_rows(self.root / "mocopi.jsonl")
        if self.meta.get("camera_frames") != len(self.camera) or self.meta.get("mocopi_frames") != len(
            self.motion
        ):
            raise ValueError(f"Recording counts do not match files: {self.root}")
        for row in self.camera:
            image = (self.root / row["file"]).resolve()
            if not image.is_relative_to(self.root / "images") or not image.is_file():
                raise ValueError(f"Missing or invalid image path in {self.root}")
        for row in self.motion:
            if not {str(bone) for bone in REQUIRED_BONES} <= row["bones"].keys():
                raise ValueError(f"Motion frame missing required bones in {self.root}")
            for _, bone in POSE_BONES:
                validate_pose(row["bones"][str(bone)])


def nearest_index(stamps, value):
    index = int(np.searchsorted(stamps, value))
    if index == 0:
        return 0
    if index == len(stamps):
        return len(stamps) - 1
    return index - 1 if value - stamps[index - 1] <= stamps[index] - value else index


def aligned_segments(session, fps, max_sync_s):
    """Fixed-rate samples, split at gaps rather than silently compressing time."""
    if not isinstance(fps, int) or not 0 < fps <= 120 or not np.isfinite(max_sync_s) or max_sync_s <= 0:
        raise ValueError("fps must be 1..120 and max_sync_s must be positive")
    start, end = session.camera_times[0], session.camera_times[-1]
    count = int(np.floor((end - start) * fps + 1e-9)) + 1
    segments, current, rejected = [], [], 0
    for i in range(count):
        grid = start + i / fps
        ci = nearest_index(session.camera_times, grid)
        mi = nearest_index(session.motion_times, grid)
        camera_t, mocopi_t = session.camera_times[ci], session.motion_times[mi]
        if max(abs(camera_t - grid), abs(mocopi_t - grid), abs(camera_t - mocopi_t)) > max_sync_s:
            if current:
                segments.append(current)
                current = []
            rejected += 1
            continue
        current.append({"grid_s": float(grid), "camera": ci, "motion": mi})
    if current:
        segments.append(current)
    return segments, {"grid_samples": count, "unsynchronized_samples": rejected}


def pose_vectors(row, previous=None):
    """xyz + xyzw for each bone; preserve quaternion sign continuity in an episode."""
    previous = {} if previous is None else previous
    poses = {}
    for name, bone in POSE_BONES:
        frame = validate_pose(row["bones"][str(bone)])
        quaternion = Rotation.from_matrix(frame[:3, :3]).as_quat()
        if name in previous and np.dot(quaternion, previous[name]) < 0:
            quaternion = -quaternion
        previous[name] = quaternion
        poses[name] = np.r_[frame[:3, 3], quaternion].astype(np.float32)
    return poses
