"""Validated raw sessions and explicit nearest-sample alignment."""

import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..calibration import HeadCalibration
from ..camera import load_intrinsics
from ..geometry import inverse, validate_pose
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
        if self.meta.get("source") not in ("real", "fake"):
            raise ValueError("Recording must declare real or fake source")
        mocopi = self.meta.get("mocopi", {})
        if (
            mocopi.get("coordinate_axes") != "x_forward_y_left_z_up"
            or mocopi.get("position_unit") != "metre"
            or mocopi.get("bone_space") != "mocopi_global"
        ):
            raise ValueError("Unsupported mocopi coordinate convention")
        self.intrinsics = None
        if self.meta["camera"].get("intrinsics_sha256"):
            intrinsic_path = self.root / "intrinsics.yaml"
            if (
                hashlib.sha256(intrinsic_path.read_bytes()).hexdigest()
                != self.meta["camera"]["intrinsics_sha256"]
            ):
                raise ValueError("Recorded camera intrinsics changed")
            self.intrinsics, _, _ = load_intrinsics(
                intrinsic_path, [self.meta["camera"]["width"], self.meta["camera"]["height"]]
            )
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
        if self.meta["camera"].get("frame_id_source") == "bridge_sequence":
            frame_ids = [row["frame_id"] for row in self.camera]
            if any(not isinstance(value, int) or value < 0 for value in frame_ids) or np.any(
                np.diff(frame_ids) <= 0
            ):
                raise ValueError("Head camera bridge frame IDs must increase")
        for row in self.motion:
            if not {str(bone) for bone in REQUIRED_BONES} <= row["bones"].keys():
                raise ValueError(f"Motion frame missing required bones in {self.root}")
            for _, bone in POSE_BONES:
                validate_pose(row["bones"][str(bone)])
        self.slam, self.slam_times, self.calibration = None, None, None
        if self.meta.get("head_pose_source") == "slam":
            calibration_path = self.root / "head_calibration.yaml"
            digest = hashlib.sha256(calibration_path.read_bytes()).hexdigest()
            if digest != self.meta.get("head_calibration_sha256"):
                raise ValueError("Recorded head calibration changed or is missing")
            self.calibration = HeadCalibration.load(calibration_path)
            self.slam, self.slam_times = _read_rows(self.root / "slam.jsonl")
            if len(self.slam) != self.meta.get("slam_frames"):
                raise ValueError("SLAM recording counts do not match")
            for row in self.slam:
                validate_pose(row["camera_pose"])
                if not isinstance(row["valid"], bool):
                    raise ValueError("Invalid SLAM tracking flag")
        elif self.meta.get("head_pose_source") != "mocopi":
            raise ValueError("Unknown head pose source")


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
    alignment_checked = False
    for i in range(count):
        grid = start + i / fps
        ci = nearest_index(session.camera_times, grid)
        mi = nearest_index(session.motion_times, grid)
        camera_t, mocopi_t = session.camera_times[ci], session.motion_times[mi]
        aligned = max(abs(camera_t - grid), abs(mocopi_t - grid), abs(camera_t - mocopi_t)) <= max_sync_s
        item = {"grid_s": float(grid), "camera": ci, "motion": mi}
        if getattr(session, "slam", None) is not None:
            si = nearest_index(session.slam_times, grid)
            slam_t = session.slam_times[si]
            aligned = (
                aligned
                and session.slam[si]["valid"]
                and max(abs(slam_t - grid), abs(slam_t - camera_t), abs(slam_t - mocopi_t)) <= max_sync_s
            )
            item["slam"] = si
            if aligned and not alignment_checked:
                mocopi_head = validate_pose(session.motion[mi]["bones"]["10"])
                head = session.calibration.head(session.slam[si]["camera_pose"])
                difference = inverse(mocopi_head) @ head
                position_limit = float(session.meta.get("startup_head_error_m", 0.15))
                rotation_limit = float(session.meta.get("startup_head_error_rad", 0.35))
                if (
                    not np.isfinite([position_limit, rotation_limit]).all()
                    or min(position_limit, rotation_limit) <= 0
                    or np.linalg.norm(difference[:3, 3]) > position_limit
                    or Rotation.from_matrix(difference[:3, :3]).magnitude() > rotation_limit
                ):
                    raise ValueError("Saved head/SLAM alignment inconsistent; recalibrate")
                alignment_checked = True
        if not aligned:
            if current:
                segments.append(current)
                current = []
            rejected += 1
            continue
        current.append(item)
    if current:
        segments.append(current)
    return segments, {"grid_samples": count, "unsynchronized_samples": rejected}


def pose_vectors(row, previous=None, world_from_mocopi=None):
    """xyz + xyzw for each bone; preserve quaternion sign continuity in an episode."""
    previous = {} if previous is None else previous
    poses = {}
    for name, bone in POSE_BONES:
        frame = validate_pose(row["bones"][str(bone)])
        if world_from_mocopi is not None:
            frame = validate_pose(world_from_mocopi @ frame)
        quaternion = Rotation.from_matrix(frame[:3, :3]).as_quat()
        if name in previous and np.dot(quaternion, previous[name]) < 0:
            quaternion = -quaternion
        previous[name] = quaternion
        poses[name] = np.r_[frame[:3, 3], quaternion].astype(np.float32)
    return poses


def fused_pose_vectors(session, item, previous=None):
    row = session.motion[item["motion"]]
    if session.slam is None:
        return pose_vectors(row, previous)
    head = session.calibration.head(session.slam[item["slam"]]["camera_pose"])
    world_from_mocopi = head @ inverse(row["bones"]["10"])
    return pose_vectors(row, previous, world_from_mocopi)
