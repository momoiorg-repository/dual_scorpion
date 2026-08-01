"""
Kinematics utilities for the dual_scorpion arms.
Contains forward and full-pose inverse kinematics solvers using PyBullet.
"""

import numpy as np
import pybullet as p
from typing import Optional, Tuple
import logging
from scipy.spatial.transform import Rotation as R

from ..config import (
    NUM_JOINTS, NUM_IK_JOINTS, IK_MAX_JOINT_STEP_DEG,
    OPERATOR_TO_ROBOT_YAW_DEG, IK_POSITION_TOLERANCE_M, IK_WORKSPACE_RADIUS_M,
    IK_WORKSPACE_SOFT_START_M, IK_ORIENTATION_SLEW_DEG,
)

logger = logging.getLogger(__name__)

class ForwardKinematics:
    """Forward kinematics solver using PyBullet."""
    
    def __init__(self, physics_client, robot_id: int, joint_indices: list, end_effector_link_index: int):
        self.physics_client = physics_client
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.end_effector_link_index = end_effector_link_index
    
    def compute(self, joint_angles_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics for given joint angles.
        
        Args:
            joint_angles_deg: Joint angles in degrees
            
        Returns:
            Tuple of (position, quaternion) of end effector
        """
        if self.physics_client is None or self.robot_id is None:
            # No silent fallback: an SO-100-era default TCP here used to mask a
            # dead kinematics engine and drive the Scorpion arms to a bogus pose.
            raise RuntimeError("Forward kinematics called before PyBullet was initialized")
        
        # Keep the gripper at neutral for FK so the TCP pose (link7) is
        # independent of the gripper opening, which the IK target ignores.
        fk_state_angles = joint_angles_deg.copy()
        fk_state_angles[7] = 0.0
        
        # Set joint positions
        joint_angles_rad = np.deg2rad(fk_state_angles)
        for i in range(NUM_JOINTS):
            if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                p.resetJointState(self.robot_id, self.joint_indices[i], joint_angles_rad[i])
        
        # Get end effector position and orientation
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index)
        position = np.array(link_state[0])
        quaternion = np.array(link_state[1])
        
        return position, quaternion

    def compute_joint_positions(self, joint_angles_deg: np.ndarray) -> np.ndarray:
        """World positions of every body-joint frame plus the TCP.

        Returns an (8, 3) array: joint0..joint6 frame origins followed by the
        TCP, in robot base/world coordinates. Used to stream a skeleton view
        of the arm to the VR headset.
        """
        if self.physics_client is None or self.robot_id is None:
            raise RuntimeError("Forward kinematics called before PyBullet was initialized")

        fk_state_angles = joint_angles_deg.copy()
        fk_state_angles[7] = 0.0
        joint_angles_rad = np.deg2rad(fk_state_angles)
        for i in range(NUM_JOINTS):
            if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                p.resetJointState(self.robot_id, self.joint_indices[i], joint_angles_rad[i])

        points = []
        for i in range(NUM_IK_JOINTS):
            # Index 4 is the URDF link frame origin, which coincides with the
            # joint frame - the visually meaningful "elbow" point.
            state = p.getLinkState(self.robot_id, self.joint_indices[i],
                                   computeForwardKinematics=True)
            points.append(state[4])
        tcp_state = p.getLinkState(self.robot_id, self.end_effector_link_index,
                                   computeForwardKinematics=True)
        points.append(tcp_state[4])
        return np.array(points)


class IKSolver:
    """Position-priority full-pose IK for one seven-DOF dual-Scorpion arm.

    All seven body joints participate in the six-dimensional TCP target
    (position + orientation). The remaining redundant DOF is resolved by
    PyBullet's null-space solver using the current commanded pose as its rest
    pose, which favors continuous local motion instead of configuration flips.

    Position takes priority over orientation: the full-pose solution is
    verified with FK, and if the TCP misses the target position by more than
    ``position_tolerance_m`` the solve is repeated position-only. This stops
    the solver from sacrificing metres of position error to honour an
    unreachable wrist orientation (the failure mode seen in the Jul 16
    telemetry, where joints pinned at their limits while the TCP lagged the
    target by 5-70 cm).

    Targets are also clamped to a reachable-workspace sphere around the
    shoulder (joint0 origin) before solving, so impossible targets never drive
    joints into their limits.
    """

    def __init__(self, physics_client, robot_id: int, joint_indices: list,
                 end_effector_link_index: int, joint_limits_min_deg: np.ndarray,
                 joint_limits_max_deg: np.ndarray, arm_name: str = "",
                 position_tolerance_m: float = IK_POSITION_TOLERANCE_M,
                 workspace_radius_m: float = IK_WORKSPACE_RADIUS_M,
                 workspace_soft_start_m: float = IK_WORKSPACE_SOFT_START_M,
                 orientation_slew_deg: float = IK_ORIENTATION_SLEW_DEG,
                 max_joint_step_deg: float = IK_MAX_JOINT_STEP_DEG,
                 locked_joints_deg: Optional[dict[int, float]] = None):
        self.physics_client = physics_client
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.end_effector_link_index = end_effector_link_index
        self.joint_limits_min_deg = joint_limits_min_deg
        self.joint_limits_max_deg = joint_limits_max_deg
        self.arm_name = arm_name
        self.position_tolerance_m = float(position_tolerance_m)
        self.workspace_radius_m = float(workspace_radius_m)
        self.workspace_soft_start_m = float(np.clip(
            workspace_soft_start_m, 0.0,
            max(0.0, self.workspace_radius_m - 1e-3)))
        self.orientation_slew_deg = float(orientation_slew_deg)
        self.max_joint_step_deg = float(max_joint_step_deg)
        self.locked_joints_deg = {
            int(index): float(angle)
            for index, angle in (locked_joints_deg or {}).items()
        }

        # PyBullet expects one limit/range/rest value for every movable DOF,
        # including the gripper branch. The TCP is attached to link7 before the
        # gripper, so that final DOF has no influence on the solution but must
        # still be represented to keep the null-space arrays well-formed.
        self.ik_lower_limits = np.deg2rad(joint_limits_min_deg[:NUM_JOINTS])
        self.ik_upper_limits = np.deg2rad(joint_limits_max_deg[:NUM_JOINTS])
        self.ik_ranges = self.ik_upper_limits - self.ik_lower_limits

        self.fk_solver = ForwardKinematics(physics_client, robot_id, joint_indices, end_effector_link_index)

        # Shoulder pivot (joint0 frame origin) in world coordinates, used as the
        # centre of the reachable-workspace sphere. The URDF link frame of
        # joint0's child link coincides with the joint frame, and the joint sits
        # on the fixed crossbar so its origin never moves.
        link_state = p.getLinkState(robot_id, joint_indices[0], computeForwardKinematics=True)
        self.workspace_center = np.array(link_state[4])

        # Diagnostics from the most recent solve() call (for telemetry).
        self.last_solve_info: dict = {}

        # Orientation authority adapts continuously from 0..1 according to
        # position feasibility, avoiding binary full-pose/position-only
        # chatter and delayed wrist catch-up.
        self._orientation_weight = 1.0

    def reset_mode_state(self):
        """Restore full orientation authority at a fresh grip engage.

        A new grip starts from a fresh measured origin, so reduced authority
        from the previous reach should not carry into it.
        """
        self._orientation_weight = 1.0

    def _clamp_to_workspace(self, target_position: np.ndarray) -> Tuple[np.ndarray, bool]:
        """Soft-compress targets near reach; hard-limit only as an asymptote."""
        offset = target_position - self.workspace_center
        dist = np.linalg.norm(offset)
        soft = self.workspace_soft_start_m
        radius = self.workspace_radius_m
        if dist > soft and dist > 1e-9 and radius > soft:
            span = radius - soft
            compressed_dist = soft + span * (1.0 - np.exp(-(dist - soft) / span))
            return self.workspace_center + offset * (compressed_dist / dist), True
        return target_position, False

    def _solve_once(self, target_position: np.ndarray,
                    target_orientation_quat: Optional[np.ndarray],
                    current_angles_rad: np.ndarray) -> np.ndarray:
        """One raw PyBullet IK solve, seeded/rested at the current pose."""
        current_angles_rad = np.asarray(current_angles_rad, dtype=float).copy()
        for index, angle_deg in self.locked_joints_deg.items():
            if 0 <= index < NUM_IK_JOINTS:
                current_angles_rad[index] = np.deg2rad(angle_deg)
        for i in range(NUM_JOINTS):
            if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                p.resetJointState(self.robot_id, self.joint_indices[i], current_angles_rad[i])

        joint_damping = [0.08] * NUM_JOINTS
        for index in self.locked_joints_deg:
            if 0 <= index < NUM_IK_JOINTS:
                # PyBullet does not expose a true per-joint lock in its IK
                # API. Heavy damping keeps the DOF at its locked rest pose;
                # the exact angle is enforced on the returned solution.
                joint_damping[index] = 1e6

        kwargs = {
            "bodyUniqueId": self.robot_id,
            "endEffectorLinkIndex": self.end_effector_link_index,
            "targetPosition": np.asarray(target_position, dtype=float).tolist(),
            "lowerLimits": self.ik_lower_limits.tolist(),
            "upperLimits": self.ik_upper_limits.tolist(),
            "jointRanges": self.ik_ranges.tolist(),
            "restPoses": current_angles_rad.tolist(),
            "jointDamping": joint_damping,
            "solver": p.IK_DLS,
            "maxNumIterations": 100,
            "residualThreshold": 1e-5,
        }
        if target_orientation_quat is not None:
            quat = np.asarray(target_orientation_quat, dtype=float)
            norm = np.linalg.norm(quat)
            if norm > 1e-9:
                kwargs["targetOrientation"] = (quat / norm).tolist()

        raw_solution = p.calculateInverseKinematics(**kwargs)
        if len(raw_solution) < NUM_IK_JOINTS:
            raise RuntimeError(
                f"PyBullet returned {len(raw_solution)} joints; expected at least {NUM_IK_JOINTS}"
            )

        solution_deg = np.rad2deg(np.asarray(raw_solution[:NUM_IK_JOINTS]))
        for index, angle_deg in self.locked_joints_deg.items():
            if 0 <= index < NUM_IK_JOINTS:
                solution_deg[index] = angle_deg
        return np.clip(
            solution_deg,
            self.joint_limits_min_deg[:NUM_IK_JOINTS],
            self.joint_limits_max_deg[:NUM_IK_JOINTS],
        )

    def _position_residual(self, solution_deg: np.ndarray, target_position: np.ndarray) -> float:
        """FK-verify a candidate solution's TCP position error in metres."""
        fk_angles = np.concatenate([solution_deg, [0.0]])
        achieved_position, _ = self.fk_solver.compute(fk_angles)
        return float(np.linalg.norm(achieved_position - target_position))

    def _slew_limit_orientation(self, target_quat: np.ndarray,
                                current_angles_deg: np.ndarray) -> Tuple[np.ndarray, bool, float]:
        """Rate-limit the orientation target relative to the CURRENT TCP orientation.

        Whenever the requested orientation is far from where the wrist actually
        is (position-priority dropped it for a while, or the operator turned the
        controller quickly), feeding the raw target to IK makes the solver
        demand the whole reorientation in one tick - the joints then wind at
        their slew cap for seconds, which reads as the arm 'rotating the wrong
        way' (Jul 17 telemetry: 74 deg orientation error unwound j4/j6 by ~50 deg
        while the shoulder swung to 130 deg). Instead, the effective target only
        advances ``orientation_slew_deg`` per solve along the geodesic, so
        orientation changes stay continuous and full-pose <-> position-only
        transitions cannot jump.

        Returns (effective_quat, was_limited, orientation_error_deg).
        """
        fk_angles = np.concatenate([current_angles_deg[:NUM_IK_JOINTS], [0.0]])
        _, current_quat = self.fk_solver.compute(fk_angles)

        current = R.from_quat(np.asarray(current_quat, dtype=float))
        target = R.from_quat(np.asarray(target_quat, dtype=float))
        delta = target * current.inv()
        error_deg = float(np.rad2deg(delta.magnitude()))

        if error_deg <= self.orientation_slew_deg or error_deg < 1e-6:
            return np.asarray(target_quat, dtype=float), False, error_deg

        fraction = self.orientation_slew_deg / error_deg
        limited = R.from_rotvec(delta.as_rotvec() * fraction) * current
        return limited.as_quat(), True, error_deg

    def _weighted_orientation(self, target_quat: np.ndarray,
                              current_angles_deg: np.ndarray,
                              weight: float) -> Optional[np.ndarray]:
        """Blend current TCP orientation toward target without binary on/off."""
        if weight <= 1e-3:
            return None
        fk_angles = np.concatenate([current_angles_deg[:NUM_IK_JOINTS], [0.0]])
        _, current_quat = self.fk_solver.compute(fk_angles)
        current = R.from_quat(current_quat)
        target = R.from_quat(target_quat)
        delta = target * current.inv()
        return (R.from_rotvec(delta.as_rotvec() * np.clip(weight, 0.0, 1.0))
                * current).as_quat()

    def solve(self, target_position: np.ndarray, target_orientation_quat: Optional[np.ndarray],
              current_angles_deg: np.ndarray,
              seed_angles_deg: Optional[np.ndarray] = None) -> np.ndarray:
        """Solve a continuous position-priority full-pose target for all seven body joints."""
        if self.physics_client is None or self.robot_id is None:
            return current_angles_deg[:NUM_IK_JOINTS]

        requested_position = np.asarray(target_position, dtype=float)
        target, was_clamped = self._clamp_to_workspace(requested_position)
        # Rest/branch selection follows measured state, while output slew is
        # applied from the last command. Keeping these separate preserves 50 Hz
        # command progression without pretending a lagging arm reached it.
        seed_angles = (np.asarray(seed_angles_deg, dtype=float)
                       if seed_angles_deg is not None
                       else np.asarray(current_angles_deg, dtype=float))
        current_angles_rad = np.deg2rad(seed_angles[:NUM_JOINTS])

        try:
            orientation_limited = False
            orientation_error_deg = 0.0
            current_position_residual = self._position_residual(
                seed_angles[:NUM_IK_JOINTS], target)
            if target_orientation_quat is not None and self.orientation_slew_deg > 0:
                target_orientation_quat, orientation_limited, orientation_error_deg = \
                    self._slew_limit_orientation(target_orientation_quat, seed_angles)

            # Continuously adapt orientation authority instead of switching it
            # fully off for 25 ticks and then abruptly back on. A feasible
            # solution slowly restores weight; position misses immediately
            # reduce it in proportion to the miss.
            orientation_requested = target_orientation_quat is not None
            weighted_quat = (
                self._weighted_orientation(
                    target_orientation_quat, seed_angles,
                    self._orientation_weight)
                if orientation_requested else None
            )
            solution_deg = self._solve_once(
                target, weighted_quat, current_angles_rad)
            residual_full = self._position_residual(solution_deg, target)
            residual_final = residual_full
            used_position_only = orientation_requested and weighted_quat is None

            if orientation_requested and residual_full > self.position_tolerance_m:
                ratio = self.position_tolerance_m / max(
                    residual_full, self.position_tolerance_m)
                reduced_weight = max(0.0, self._orientation_weight * ratio)
                reduced_quat = self._weighted_orientation(
                    target_orientation_quat, seed_angles, reduced_weight)
                reduced_deg = self._solve_once(
                    target, reduced_quat, current_angles_rad)
                reduced_residual = self._position_residual(reduced_deg, target)
                if reduced_residual < residual_final:
                    solution_deg = reduced_deg
                    residual_final = reduced_residual
                    self._orientation_weight = reduced_weight
                    used_position_only = reduced_quat is None

            # Absolute position priority remains the final fallback, but the
            # retained low orientation weight makes re-entry gradual instead
            # of a frozen-half-second / sudden-retry cycle.
            if orientation_requested and residual_final > self.position_tolerance_m:
                position_only_deg = self._solve_once(target, None, current_angles_rad)
                residual_position_only = self._position_residual(position_only_deg, target)
                if residual_position_only < residual_final:
                    solution_deg = position_only_deg
                    residual_final = residual_position_only
                    used_position_only = True
                    self._orientation_weight = 0.0

            # Do not restore orientation authority based only on a distant raw
            # IK endpoint. Wait until the measured/seeded TCP is also close to
            # the target, otherwise the outgoing slew-limited command has not
            # reached the position-feasible neighborhood yet.
            near_target = current_position_residual <= self.position_tolerance_m * 2.0
            if (orientation_requested
                    and residual_final <= self.position_tolerance_m
                    and near_target):
                self._orientation_weight = min(
                    1.0, self._orientation_weight + 0.04)
            elif not orientation_requested:
                self._orientation_weight = 1.0
            # A current-pose rest seed strongly discourages branch flips, and
            # this hard slew limit is the final physical safety boundary.
            current_body = current_angles_deg[:NUM_IK_JOINTS]
            delta = np.clip(
                solution_deg - current_body,
                -self.max_joint_step_deg,
                self.max_joint_step_deg,
            )

            self.last_solve_info = {
                "target_requested": requested_position.tolist(),
                "target_clamped": target.tolist(),
                "workspace_clamped": was_clamped,
                "residual_full_pose_m": residual_full,
                "residual_final_m": residual_final,
                "used_position_only": used_position_only,
                "orientation_slew_limited": orientation_limited,
                "orientation_error_deg": orientation_error_deg,
                "orientation_weight": self._orientation_weight,
                "current_position_residual_m": current_position_residual,
            }
            return current_body + delta
        except Exception as exc:
            logger.warning("%s position-priority IK failed: %s", self.arm_name, exc)
            self.last_solve_info = {"error": str(exc)}
            return current_angles_deg[:NUM_IK_JOINTS]


def vr_to_robot_rotation(headset_yaw_deg: float = 0.0,
                         operator_to_robot_yaw_deg: float = OPERATOR_TO_ROBOT_YAW_DEG,
                         mirror_lateral: bool = False) -> np.ndarray:
    """Build the calibrated linear map from VR world coordinates to robot base coordinates.

    The map is composed of these stages (applied right to left):

    1. ``Ry(-headset_yaw)`` - rotates VR-world vectors into the operator's
       facing frame. ``headset_yaw_deg`` is the headset's yaw captured when the
       grip engaged, so "forward" is wherever the operator was looking at that
       moment, independent of where the Quest room/floor frame was recentred.
       (WebXR world axes: X=right, Y=up, Z=back.)
    2. A fixed axis permutation from the VR operator frame to a robot-style
       (forward, left, up) basis: forward=-Z, left=-X, up=+Y.
    3. Optional lateral mirror - negates the operator's left/right axis while
       leaving forward and up untouched. Needed when the operator faces the
       arms head-on: forward must flip (handled by the yaw below) but
       left/right must NOT, and no pure yaw can do one without the other.
    4. ``Rz(operator_to_robot_yaw_deg)`` - a single bench-calibrated yaw that
       maps "operator forward" onto the Scorpion crossbar mount.

    The same matrix is used for position deltas and (as a similarity
    transform) orientation deltas, so pushing and twisting the controller stay
    consistent in one frame. Note: with the mirror enabled the matrix is a
    reflection (det -1), which is why orientation conjugation is done with
    plain matrices rather than scipy Rotation objects.
    """
    theta = np.deg2rad(headset_yaw_deg)
    ry_neg_headset = np.array([
        [np.cos(theta), 0.0, -np.sin(theta)],
        [0.0, 1.0, 0.0],
        [np.sin(theta), 0.0, np.cos(theta)],
    ])
    vr_to_flu = np.array([
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ])
    # Reflection of the operator's lateral (left/right) axis in the
    # forward-left-up frame; identity when mirroring is disabled.
    mirror = np.diag([1.0, -1.0, 1.0]) if mirror_lateral else np.eye(3)
    phi = np.deg2rad(operator_to_robot_yaw_deg)
    rz_operator = np.array([
        [np.cos(phi), -np.sin(phi), 0.0],
        [np.sin(phi), np.cos(phi), 0.0],
        [0.0, 0.0, 1.0],
    ])
    return rz_operator @ mirror @ vr_to_flu @ ry_neg_headset


def compute_relative_position(current_vr_pos: dict, origin_vr_pos: dict, scale: float = 1.0,
                              headset_yaw_deg: float = 0.0,
                              operator_to_robot_yaw_deg: float = OPERATOR_TO_ROBOT_YAW_DEG,
                              mirror_lateral: bool = False) -> np.ndarray:
    """Map the VR controller's displacement since grip into a robot-frame delta."""
    delta_vr = np.array([
        current_vr_pos['x'] - origin_vr_pos['x'],
        current_vr_pos['y'] - origin_vr_pos['y'],
        current_vr_pos['z'] - origin_vr_pos['z'],
    ])
    rotation = vr_to_robot_rotation(headset_yaw_deg, operator_to_robot_yaw_deg, mirror_lateral)
    return (rotation @ delta_vr) * scale


def compute_relative_orientation(current_quat: np.ndarray, origin_quat: np.ndarray,
                                 headset_yaw_deg: float = 0.0,
                                 operator_to_robot_yaw_deg: float = OPERATOR_TO_ROBOT_YAW_DEG,
                                 mirror_lateral: bool = False,
                                 rotation_scale: float = 1.0) -> np.ndarray:
    """Map a world-frame VR orientation delta into the robot base frame.

    The delta is conjugated by the SAME calibrated linear map used for position
    in :func:`vr_to_robot_rotation` (C R C^T). This keeps translation and
    rotation controls in one consistent frame, so twisting the controller
    twists the TCP the same way pushing it moves the TCP - regardless of which
    way the operator was facing when grip engaged.

    ``rotation_scale`` scales the delta's rotation angle about its own axis
    (rotation analogue of vr_to_robot_scale): 0.5 means the TCP twists half
    as far as the controller. Applied to the total accumulated delta, so it
    stays consistent when the controller returns to its origin orientation.

    The conjugation is done with plain matrices because C may be a reflection
    (det -1) when mirror_lateral is on; C R C^T is still a proper rotation
    (det +1) either way, so the quaternion conversion below is always valid.
    """
    current = R.from_quat(np.asarray(current_quat, dtype=float))
    origin = R.from_quat(np.asarray(origin_quat, dtype=float))
    delta = current * origin.inv()
    if rotation_scale != 1.0:
        delta = R.from_rotvec(delta.as_rotvec() * float(rotation_scale))
    vr_delta = delta.as_matrix()

    frame = vr_to_robot_rotation(headset_yaw_deg, operator_to_robot_yaw_deg, mirror_lateral)
    robot_delta = frame @ vr_delta @ frame.T
    return R.from_matrix(robot_delta).as_quat()