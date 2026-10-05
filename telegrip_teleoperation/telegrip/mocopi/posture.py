"""Soft upper-arm guidance in the existing robot's TCP null space.

The hand remains the primary task. Human elbow/upper-arm observations select
the redundant arm posture; they cannot force an unreachable elbow or replace
the existing TCP IK, joint limits, or per-command speed limit.
"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import validate_pose


@dataclass
class ArmPostureTarget:
    elbow: np.ndarray
    upper_rotation: np.ndarray

    def validate(self):
        elbow = np.asarray(self.elbow, dtype=float)
        if elbow.shape != (3,) or not np.isfinite(elbow).all():
            raise ValueError("Invalid elbow target")
        frame = np.eye(4)
        frame[:3, :3] = self.upper_rotation
        validate_pose(frame)
        return self


def robot_arm_state(solver, joints):
    """Use the existing FK and URDF joint frames, never CAD mesh centroids."""
    import pybullet as p

    points = solver.fk_solver.compute_joint_positions(joints)
    # joint2's child is the upper-arm link; joint4's origin is the elbow
    # landmark already used by Telegrip's robot skeleton visualization.
    state = p.getLinkState(
        solver.robot_id,
        solver.joint_indices[2],
        computeForwardKinematics=True,
        physicsClientId=solver.physics_client,
    )
    return points[0].copy(), points[4].copy(), Rotation.from_quat(state[5]).as_matrix()


class ArmPostureRetarget:
    """Anchor relative limb motion without requiring identical starting poses."""

    def __init__(self, shoulder, elbow, robot_state, rotation, gain=0.8):
        shoulder, elbow = validate_pose(shoulder), validate_pose(elbow)
        if not np.isfinite(gain) or not 0 < gain <= 1:
            raise ValueError("arm_posture_gain must be in (0, 1]")
        self.human_vector = elbow[:3, 3] - shoulder[:3, 3]
        length = np.linalg.norm(self.human_vector)
        if length < 0.05:
            raise ValueError("Upper-arm landmarks too close: check mocopi calibration")
        robot_shoulder, self.robot_elbow, self.robot_upper_rotation = robot_state
        self.scale = gain * np.linalg.norm(self.robot_elbow - robot_shoulder) / length
        self.human_upper_rotation = shoulder[:3, :3]
        self.rotation = rotation
        self.gain = gain

    def target(self, shoulder, elbow):
        shoulder, elbow = validate_pose(shoulder), validate_pose(elbow)
        vector = elbow[:3, 3] - shoulder[:3, 3]
        if np.linalg.norm(vector) < 0.05:
            raise ValueError("Upper-arm landmarks too close: HOLD")
        elbow_target = self.robot_elbow + self.scale * self.rotation @ (vector - self.human_vector)
        delta = shoulder[:3, :3] @ self.human_upper_rotation.T
        mapped = self.rotation @ delta @ self.rotation.T
        scaled = Rotation.from_rotvec(Rotation.from_matrix(mapped).as_rotvec() * self.gain)
        return ArmPostureTarget(elbow_target, scaled.as_matrix() @ self.robot_upper_rotation).validate()


class ArmPostureIK:
    """Refine a slew-limited TCP command using local redundant joint motion."""

    ORIENTATION_LENGTH = 0.06  # metres/radian in the secondary posture objective

    def __init__(self, solver, lower, upper, max_step_deg):
        self.solver = solver
        self.lower, self.upper = np.asarray(lower)[:7], np.asarray(upper)[:7]
        self.max_step = max_step_deg

    def state(self, body, gripper):
        joints = np.r_[body, gripper]
        tcp, quat = self.solver.fk_solver.compute(joints)
        _, elbow, upper = robot_arm_state(self.solver, joints)
        return tcp, Rotation.from_quat(quat).as_matrix(), elbow, upper

    def error(self, state, target, tcp_rotation):
        return np.r_[
            target.elbow - state[2],
            self.ORIENTATION_LENGTH * Rotation.from_matrix(target.upper_rotation @ state[3].T).as_rotvec(),
            0.02 * Rotation.from_matrix(tcp_rotation @ state[1].T).as_rotvec(),
        ]

    def refine(self, candidate, previous, target, tcp_target):
        target.validate()
        base = self.state(candidate, previous[7])
        tcp_rotation = tcp_target[:3, :3]
        error = self.error(base, target, tcp_rotation)
        # Finite differences deliberately reuse the established FK, including
        # mirrored axes and the TCP offset. No independent robot model here.
        epsilon = 1e-3  # radians; above Bullet link-frame float32 noise
        tcp_jacobian, posture_jacobian = np.zeros((3, 7)), np.zeros((9, 7))
        for index in range(7):
            shifted = candidate.copy()
            shifted[index] += np.rad2deg(epsilon)
            state = self.state(shifted, previous[7])
            tcp_jacobian[:, index] = (state[0] - base[0]) / epsilon
            posture_jacobian[:, index] = (
                np.r_[
                    state[2] - base[2],
                    self.ORIENTATION_LENGTH * Rotation.from_matrix(state[3] @ base[3].T).as_rotvec(),
                    0.02 * Rotation.from_matrix(state[1] @ base[1].T).as_rotvec(),
                ]
                / epsilon
            )
        _, singular, vh = np.linalg.svd(tcp_jacobian, full_matrices=True)
        rank = np.count_nonzero(singular > max(singular[0] * 1e-3, 1e-6))
        null = vh[rank:].T
        projected = posture_jacobian @ null
        # Damped least squares avoids excessive secondary commands near a
        # singular posture. The result still passes the original slew/limits.
        step = null @ np.linalg.solve(
            projected.T @ projected + np.eye(null.shape[1]) * 1e-4,
            projected.T @ error,
        )
        delta = np.rad2deg(step)
        delta *= min(1.0, self.max_step / max(np.max(np.abs(delta)), 1e-9))
        result, state = candidate.copy(), base
        mode = "settled" if np.linalg.norm(error) < 0.001 else "limited"
        for fraction in (1.0, 0.5, 0.25, 0.125):
            trial = np.clip(
                candidate + fraction * delta,
                np.maximum(self.lower, previous[:7] - self.max_step),
                np.minimum(self.upper, previous[:7] + self.max_step),
            )
            trial_state = self.state(trial, previous[7])
            # Clipping can leave the null space. Reject such steps by FK;
            # preserve the primary position command within 0.5 mm. Wrist
            # orientation is a softer objective than elbow/upper-arm posture.
            if (
                np.linalg.norm(trial_state[0] - base[0]) <= 0.0005
                and np.linalg.norm(self.error(trial_state, target, tcp_rotation))
                < np.linalg.norm(error) - 1e-8
            ):
                result, state, mode = trial, trial_state, "active"
                break
        return result, {
            "mode": mode,
            "elbow_error_m": float(np.linalg.norm(target.elbow - state[2])),
            "upper_arm_error_deg": float(
                np.rad2deg(Rotation.from_matrix(target.upper_rotation @ state[3].T).magnitude())
            ),
            "wrist_error_deg": float(np.rad2deg(Rotation.from_matrix(tcp_rotation @ state[1].T).magnitude())),
        }
