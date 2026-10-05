"""Anchor relative human motion to each robot's initial TCP independently."""

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import inverse, validate_pose, world_arm_landmarks


def alignment_rotation(head, sample, robot_from_operator, mode="body"):
    """Align the operator's horizontal forward/left/up axes at each neutral capture."""
    frame = np.eye(4)
    frame[:3, :3] = robot_from_operator
    robot_from_operator = validate_pose(frame)[:3, :3]
    if mode == "world":
        return robot_from_operator, "configured world axes"
    if mode != "body":
        raise ValueError("retarget.alignment_mode must be body or world")
    landmarks = world_arm_landmarks(head, sample)
    if "left_shoulder" in landmarks and "right_shoulder" in landmarks:
        left = landmarks["left_shoulder"][:3, 3] - landmarks["right_shoulder"][:3, 3]
        left[2] = 0
        if np.isfinite(left).all() and np.linalg.norm(left) > 0.05:
            left /= np.linalg.norm(left)
            forward = np.cross(left, [0, 0, 1])
            source = "shoulders"
        else:
            raise ValueError("Cannot determine body heading from shoulders; stand upright and retry Enter")
    elif 0 in sample.bones:
        root = validate_pose(head @ inverse(sample.bones[10]) @ sample.bones[0])
        forward = root[:3, 0].copy()
        forward[2] = 0
        if np.linalg.norm(forward) < 0.1:
            raise ValueError("Cannot determine body heading from root; use --alignment world")
        forward /= np.linalg.norm(forward)
        left = np.cross([0, 0, 1], forward)
        source = "root (shoulder bones unavailable)"
    else:
        raise ValueError("Body alignment requires shoulders or root; use --alignment world for older replays")
    world_from_operator = np.column_stack([forward, left, [0, 0, 1]])
    heading = np.rad2deg(np.arctan2(forward[1], forward[0]))
    return robot_from_operator @ world_from_operator.T, f"{source}, heading={heading:.1f} deg"


class Retarget:
    def __init__(
        self,
        human,
        robot,
        translation_scale=0.8,
        rotation=None,
        orientation_scale=1.0,
        orientation_enabled=True,
        max_translation_m=0.35,
        arm_posture=None,
    ):
        if not 0 < translation_scale <= 10 or not 0 <= orientation_scale <= 2 or max_translation_m <= 0:
            raise ValueError("Invalid retarget gain/workspace")
        self.human = validate_pose(human)
        self.robot = validate_pose(robot)
        self.scale = translation_scale
        self.orientation_scale = orientation_scale
        self.orientation_enabled = orientation_enabled
        self.limit = max_translation_m
        self.arm_posture = arm_posture
        frame = np.eye(4)
        frame[:3, :3] = np.eye(3) if rotation is None else rotation
        self.rotation = validate_pose(frame)[:3, :3]

    def target(self, human):
        human = validate_pose(human)
        offset = self.scale * self.rotation @ (human[:3, 3] - self.human[:3, 3])
        distance = np.linalg.norm(offset)
        if distance > self.limit:
            offset *= self.limit / distance
        target = self.robot.copy()
        target[:3, 3] += offset
        if self.orientation_enabled:
            delta = human[:3, :3] @ self.human[:3, :3].T
            mapped = self.rotation @ delta @ self.rotation.T
            scaled = Rotation.from_rotvec(Rotation.from_matrix(mapped).as_rotvec() * self.orientation_scale)
            target[:3, :3] = scaled.as_matrix() @ self.robot[:3, :3]
        return target

    def posture_target(self, landmarks, side):
        if self.arm_posture is None:
            return None
        return self.arm_posture.target(landmarks[side + "_shoulder"], landmarks[side + "_elbow"])
