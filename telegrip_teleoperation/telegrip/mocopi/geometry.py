"""Right-handed SE(3); T_A_B maps B vectors to A. Positions are metres."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

# Sony native: X left, Y up, Z forward -> X forward, Y left, Z up.
SONY_TO_WORLD = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def validate_pose(value):
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError("SE(3) must be a finite 4x4 matrix")
    r = t[:3, :3]
    if not np.allclose(t[3], [0, 0, 0, 1]) or not np.allclose(r.T @ r, np.eye(3), atol=1e-5):
        raise ValueError("Invalid homogeneous transform")
    if not np.isclose(np.linalg.det(r), 1.0, atol=1e-5):
        raise ValueError("Rotation must belong to SO(3)")
    return t.copy()


def pose(position=(0.0, 0.0, 0.0), quaternion=(0.0, 0.0, 0.0, 1.0)):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError("Invalid xyzw quaternion")
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(q).as_matrix()
    t[:3, 3] = position
    return validate_pose(t)


def inverse(t):
    t = validate_pose(t)
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def sony_pose(values):
    t = pose(values[4:7], values[:4])
    b = np.eye(4)
    b[:3, :3] = SONY_TO_WORLD
    return b @ t @ b.T


@dataclass
class MocopiSample:
    timestamp: float
    bones: dict[int, np.ndarray]
    frame_id: int = -1
    valid: bool = True


@dataclass
class SlamSample:
    timestamp: float
    camera: np.ndarray
    valid: bool = True
    session: str = ""
    intrinsics_id: str = ""


def world_hands(head_world, sample):
    relative_head = inverse(sample.bones[10])
    return {
        side: head_world @ relative_head @ sample.bones[bone] for side, bone in (("left", 14), ("right", 18))
    }


def world_arm_landmarks(head_world, sample):
    """Virtual bone origins, not physical ANKLE sensor positions.

    Upper-arm origins (12/16) are shoulders; lower-arm origins (13/17)
    are elbows in Sony's hierarchy. Preserve head-relative fusion for all.
    These poses also drive optional arm-posture IK. Minimal older replay
    samples may lack these bones and retain hand-only retargeting.
    """
    world_from_mocopi = head_world @ inverse(sample.bones[10])
    bones = {"left_shoulder": 12, "left_elbow": 13, "right_shoulder": 16, "right_elbow": 17}
    return {
        name: world_from_mocopi @ sample.bones[bone] for name, bone in bones.items() if bone in sample.bones
    }
