"""
Robot interface module for the dual_scorpion teleoperation system.
Provides a clean wrapper around robot devices with safety checks and convenience methods.
"""

import numpy as np
import time
import logging
import os
import sys
import contextlib
from typing import Optional, Dict, Tuple

# Reuse the Dual Scorpion follower implementation already shipped by this
# repository. Telegrip adds only Cartesian/VR control and does not duplicate
# motor setup, calibration, recording, or policy functionality.
from lerobot.robots.dual_scorpion_follower import (
    DualScorpionFollower,
    DualScorpionFollowerConfig,
)

from ..config import (
    TelegripConfig, NUM_JOINTS, NUM_IK_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE,
    GRIPPER_INDEX, URDF_TO_INTERNAL_NAME_MAP
)
from .kinematics import ForwardKinematics, IKSolver

logger = logging.getLogger(__name__)


@contextlib.contextmanager
def suppress_stdout_stderr():
    """Context manager to suppress stdout and stderr output at the file descriptor level."""
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()

    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)

    try:
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        yield
    finally:
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


class RobotInterface:
    """High-level interface for dual_scorpion robot control with safety features."""

    def __init__(self, config: TelegripConfig, telemetry=None):
        self.config = config
        self.telemetry = telemetry
        self.robot = None  # Single DualScorpionFollower managing both arms
        self.is_connected = False
        self.is_engaged = False

        # Individual arm connection status
        self.left_arm_connected = False
        self.right_arm_connected = False

        # Joint state (8 joints per arm: joint0-joint6 + gripper)
        self.left_arm_angles = np.zeros(NUM_JOINTS)
        self.right_arm_angles = np.zeros(NUM_JOINTS)

        # Joint limits per arm (will be set by visualizer; the arms are mirrored)
        self.joint_limits_min_deg = {arm: np.full(NUM_JOINTS, -180.0) for arm in ('left', 'right')}
        self.joint_limits_max_deg = {arm: np.full(NUM_JOINTS, 180.0) for arm in ('left', 'right')}

        # Kinematics solvers (will be set after PyBullet setup)
        self.fk_solvers = {'left': None, 'right': None}
        self.ik_solvers = {'left': None, 'right': None}

        # Control timing
        self.last_send_time = 0
        self.last_actual_telemetry_time = 0

        # Latest measured motor angles, refreshed by the periodic sample in
        # send_command (no extra serial reads). Used by the VR skeleton view.
        self.last_measured_angles = {'left': None, 'right': None}
        self.last_measured_time = {'left': None, 'right': None}
        self.last_lag_clamped = {'left': False, 'right': False}
        self.last_measurement_ok = False

        # Error tracking
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3
        self.max_general_errors = 8

    def connect(self) -> bool:
        """Connect to robot hardware."""
        if self.is_connected:
            logger.info("Robot interface already connected")
            return True

        if not self.config.enable_robot:
            logger.info("Robot interface disabled in config")
            self.is_connected = True
            return True

        should_suppress = (self.config.log_level == "warning" or
                           self.config.log_level == "critical" or
                           self.config.log_level == "error")

        try:
            robot_config = DualScorpionFollowerConfig(
                right_arm_port=self.config.follower_ports["right"],
                left_arm_port=self.config.follower_ports["left"],
                use_degrees=True,
            )

            if not should_suppress:
                logger.info(f"Connecting to DualScorpionFollower "
                            f"(right={robot_config.right_arm_port}, left={robot_config.left_arm_port})...")

            try:
                if should_suppress:
                    with suppress_stdout_stderr():
                        self.robot = DualScorpionFollower(robot_config)
                        self.robot.connect(calibrate=False)
                else:
                    self.robot = DualScorpionFollower(robot_config)
                    self.robot.connect(calibrate=False)

                self.left_arm_connected = True
                self.right_arm_connected = True
                logger.info("Connected to DualScorpionFollower (both arms)")

            except Exception as e:
                logger.error(f"DualScorpionFollower connection failed: {e}")
                self.left_arm_connected = False
                self.right_arm_connected = False
                self.robot = None

            self.is_connected = self.left_arm_connected or self.right_arm_connected

            if self.is_connected:
                self._read_initial_state()
                logger.info(f"Robot interface connected: Left={self.left_arm_connected}, Right={self.right_arm_connected}")
            else:
                logger.error("Failed to connect robot")

            return self.is_connected

        except Exception as e:
            logger.error(f"Robot connection failed with exception: {e}")
            self.is_connected = False
            return False

    @staticmethod
    def _observation_angles(observation: Dict, arm: str) -> np.ndarray:
        """Extract one complete, finite 7-DOF + gripper observation."""
        keys = [f"{arm}_joint{i}.pos" for i in range(7)]
        keys.append(f"{arm}_gripper.pos")
        missing = [key for key in keys if key not in observation]
        if missing:
            raise ValueError(f"Incomplete {arm} motor observation; missing {missing}")
        values = np.asarray([observation[key] for key in keys], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError(f"Non-finite {arm} motor observation: {values}")
        return values

    def _read_initial_state(self):
        """Read initial joint state from robot."""
        try:
            if self.robot is None:
                return
            observation = self.robot.get_observation()
            if observation:
                self.left_arm_angles = self._observation_angles(observation, "left")
                logger.info(f"Left arm initial state: {self.left_arm_angles.round(1)}")

                self.right_arm_angles = self._observation_angles(observation, "right")
                logger.info(f"Right arm initial state: {self.right_arm_angles.round(1)}")
                now = time.monotonic()
                self.last_measured_angles['left'] = self.left_arm_angles.copy()
                self.last_measured_angles['right'] = self.right_arm_angles.copy()
                self.last_measured_time['left'] = now
                self.last_measured_time['right'] = now
                self.last_measurement_ok = True

        except Exception as e:
            logger.error(f"Error reading initial state: {e}")

    def setup_kinematics(self, physics_client, robot_ids: Dict, joint_indices: Dict,
                         end_effector_link_indices: Dict, joint_limits_min_deg: Dict,
                         joint_limits_max_deg: Dict):
        """Setup kinematics solvers using PyBullet components for both arms.

        joint_limits_min_deg/joint_limits_max_deg are per-arm dicts
        ({'left': array, 'right': array}) since the arms are mirrored.
        """
        self.joint_limits_min_deg = {arm: lim.copy() for arm, lim in joint_limits_min_deg.items()}
        self.joint_limits_max_deg = {arm: lim.copy() for arm, lim in joint_limits_max_deg.items()}

        for arm in ['left', 'right']:
            locked_joints_deg = {
                JOINT_NAMES.index(name): float(angle)
                for name, angle in self.config.locked_joints.get(arm, {}).items()
                if name in JOINT_NAMES[:NUM_IK_JOINTS]
            }
            self.fk_solvers[arm] = ForwardKinematics(
                physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm]
            )

            self.ik_solvers[arm] = IKSolver(
                physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm],
                joint_limits_min_deg[arm], joint_limits_max_deg[arm], arm_name=arm,
                position_tolerance_m=self.config.ik_position_tolerance_m,
                workspace_radius_m=self.config.ik_workspace_radius_m,
                workspace_soft_start_m=self.config.ik_workspace_soft_start_m,
                orientation_slew_deg=self.config.ik_orientation_slew_deg,
                max_joint_step_deg=self.config.ik_max_joint_step_deg,
                locked_joints_deg=locked_joints_deg,
            )
            if locked_joints_deg:
                logger.info("%s IK locked joints: %s", arm, {
                    JOINT_NAMES[index]: angle
                    for index, angle in locked_joints_deg.items()
                })

        logger.info("Kinematics solvers initialized for both arms")

    def get_current_end_effector_position(self, arm: str) -> np.ndarray:
        """Get current end effector position for specified arm."""
        if arm == "left":
            angles = self.left_arm_angles
        elif arm == "right":
            angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")

        if self.fk_solvers[arm] is None:
            # Fail loudly: returning a made-up TCP here (the old SO-100 home
            # point) would silently command the Scorpion arms to a bogus pose.
            raise RuntimeError(f"FK solver for {arm} arm not initialized (setup_kinematics not run)")
        position, _ = self.fk_solvers[arm].compute(angles)
        return position

    def get_current_end_effector_orientation(self, arm: str) -> np.ndarray:
        """Get current end effector orientation quaternion [x, y, z, w]."""
        if arm == "left":
            angles = self.left_arm_angles
        elif arm == "right":
            angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")

        if self.fk_solvers[arm] is None:
            raise RuntimeError(f"FK solver for {arm} arm not initialized (setup_kinematics not run)")
        _, quaternion = self.fk_solvers[arm].compute(angles)
        return quaternion

    def solve_ik(self, arm: str, target_position: np.ndarray,
                 target_orientation: Optional[np.ndarray] = None) -> np.ndarray:
        """Solve full-pose inverse kinematics for the specified arm.

        Returns commanded angles for all NUM_IK_JOINTS body joints (degrees).
        """
        if arm == "left":
            commanded_angles = self.left_arm_angles
        elif arm == "right":
            commanded_angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")

        # Measured state selects the IK branch/rest pose, while the last
        # command remains the base for the 50 Hz joint-slew step.
        current_angles = commanded_angles.copy()
        seed_angles = current_angles.copy()
        measured = self.last_measured_angles.get(arm)
        measured_time = self.last_measured_time.get(arm)
        sample_hz = max(0.0, float(self.config.telemetry_actual_sample_hz))
        max_age = max(0.1, 3.0 / sample_hz) if sample_hz > 0 else 0.1
        if (measured is not None and measured_time is not None
                and time.monotonic() - measured_time <= max_age):
            seed_angles[:NUM_IK_JOINTS] = measured[:NUM_IK_JOINTS]

        if self.ik_solvers[arm]:
            return self.ik_solvers[arm].solve(
                target_position, target_orientation, current_angles, seed_angles)
        else:
            return current_angles[:NUM_IK_JOINTS]

    def get_last_ik_info(self, arm: str) -> Dict:
        """Diagnostics from the most recent IK solve for the given arm."""
        solver = self.ik_solvers.get(arm)
        return dict(solver.last_solve_info) if solver is not None else {}

    def sync_commanded_to_actual(self, arm: str) -> bool:
        """Overwrite the commanded body-joint state with measured motor angles.

        Used at grip engage so the control origin (FK of the commanded state)
        matches the physical arm even if the motors lagged previous commands.
        The commanded gripper opening is preserved (it is latched by the
        trigger, not tracked). Returns True if measured angles were applied.
        """
        if not self.is_connected or self.robot is None:
            return False

        try:
            measured = self._observation_angles(
                self.robot.get_observation(), arm)
        except Exception as exc:
            logger.warning("Could not sync %s command to measured state: %s",
                           arm, exc)
            return False
        commanded = self.left_arm_angles if arm == "left" else self.right_arm_angles
        commanded[:NUM_IK_JOINTS] = measured[:NUM_IK_JOINTS]
        self.last_measured_angles[arm] = measured.copy()
        self.last_measured_time[arm] = time.monotonic()
        self.last_measurement_ok = True

        # Fresh grip, fresh origin: restore full orientation authority rather
        # than carrying reduced authority from the previous reach.
        solver = self.ik_solvers.get(arm)
        if solver is not None:
            solver.reset_mode_state()
        return True

    def clamp_joint_angles(self, joint_angles: np.ndarray, arm: str) -> np.ndarray:
        """Clamp joint angles to the arm's limits with wrap-around for the base joint."""
        processed_angles = joint_angles.copy()
        limits_min = self.joint_limits_min_deg[arm]
        limits_max = self.joint_limits_max_deg[arm]

        base_idx = 0
        base_angle = processed_angles[base_idx]
        if base_angle < limits_min[base_idx] or base_angle > limits_max[base_idx]:
            for offset in [-360.0, 360.0]:
                wrapped_angle = base_angle + offset
                if limits_min[base_idx] <= wrapped_angle <= limits_max[base_idx]:
                    logger.debug(f"Wrapped joint0 from {base_angle:.1f} to {wrapped_angle:.1f}")
                    processed_angles[base_idx] = wrapped_angle
                    break

        return np.clip(processed_angles, limits_min, limits_max)

    def update_arm_angles(self, arm: str, ik_angles: np.ndarray):
        """Update all seven body joints for the specified arm from full-pose IK.

        The gripper (index GRIPPER_INDEX) is controlled independently via
        set_gripper and is deliberately preserved here.
        """
        if arm == "left":
            target_angles = self.left_arm_angles
        elif arm == "right":
            target_angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")

        # Every body joint (0-6) now comes from the full-pose IK solution.
        target_angles[:NUM_IK_JOINTS] = ik_angles[:NUM_IK_JOINTS]

        # Preserve the independently-controlled gripper opening.
        preserved_gripper = np.clip(
            target_angles[GRIPPER_INDEX], GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)

        # Apply joint limits to all body joints (base joint may wrap).
        clamped_angles = self.clamp_joint_angles(target_angles, arm)
        clamped_angles[GRIPPER_INDEX] = preserved_gripper

        if arm == "left":
            self.left_arm_angles = clamped_angles
        else:
            self.right_arm_angles = clamped_angles

    def engage(self) -> bool:
        """Engage robot motors (start sending commands)."""
        if not self.is_connected:
            logger.warning("Cannot engage robot: not connected")
            return False

        self.is_engaged = True
        logger.info("Robot motors ENGAGED - commands will be sent")

        # Drive to the calibrated zero pose on engage so teleop always starts
        # from a known, repeatable configuration instead of holding whatever
        # random pose the arms powered on in. Can be disabled via config.
        if getattr(self.config, "home_on_engage", True):
            self.move_to_zero_pose()

        return True

    def _interpolate_move(self, target_left: np.ndarray, target_right: np.ndarray,
                          duration: float = 2.0, hz: int = 50) -> bool:
        """Smoothly interpolate both arms from their current measured angles to
        the given per-joint targets (8 values each: joint0-joint6 + gripper).

        Enables torque first (so it also works right after a disengage), glides
        over ``duration`` seconds, and commits the targets as the current
        commanded state so the idle control loop keeps holding them.
        """
        if not self.is_connected or self.robot is None:
            return False

        try:
            # Best-effort torque enable so the move also works right after a
            # previous disengage (which disables torque).
            try:
                self.robot.right_bus.enable_torque()
                self.robot.left_bus.enable_torque()
            except Exception as e:
                logger.debug(f"enable_torque before move skipped: {e}")

            start_left = self.get_actual_arm_angles("left").astype(float)
            start_right = self.get_actual_arm_angles("right").astype(float)
            target_left = np.asarray(target_left, dtype=float)
            target_right = np.asarray(target_right, dtype=float)

            steps = max(1, int(duration * hz))
            for s in range(1, steps + 1):
                alpha = s / steps
                cur_left = start_left + (target_left - start_left) * alpha
                cur_right = start_right + (target_right - start_right) * alpha

                action_dict = {}
                for i in range(7):  # joint0-joint6
                    action_dict[f"left_joint{i}.pos"] = float(cur_left[i])
                    action_dict[f"right_joint{i}.pos"] = float(cur_right[i])
                action_dict["left_gripper.pos"] = float(cur_left[GRIPPER_INDEX])
                action_dict["right_gripper.pos"] = float(cur_right[GRIPPER_INDEX])

                self.robot.send_action(action_dict)
                time.sleep(1.0 / hz)

            # Commit targets as the current commanded state so the idle control
            # loop holds them (it continuously resends these angles).
            self.left_arm_angles = target_left.copy()
            self.right_arm_angles = target_right.copy()
            self.last_send_time = time.monotonic()
            return True
        except Exception as e:
            logger.error(f"Error during interpolated move: {e}")
            return False

    def move_to_zero_pose(self, duration: float = 2.5, hz: int = 50) -> bool:
        """Smoothly drive both arms to the configured start ("home") pose.

        The pose comes from config ``robot.home_pose_deg`` (default: URDF zero
        with joint2 tucked back so the arms protrude less at rest). The gripper
        opening is preserved so we don't slam it on startup. After the glide, a
        short settle loop nulls each servo's steady-state droop so both arms
        PHYSICALLY sit at the target (see _settle_at_targets).
        """
        if not self.is_connected or self.robot is None:
            return False

        start_left = self.get_actual_arm_angles("left").astype(float)
        start_right = self.get_actual_arm_angles("right").astype(float)

        home = getattr(self.config, "home_pose_deg", None) or {}
        target_left = np.zeros(NUM_JOINTS)
        target_right = np.zeros(NUM_JOINTS)
        target_left[:NUM_IK_JOINTS] = np.asarray(
            home.get("left", [0.0] * NUM_IK_JOINTS), dtype=float)[:NUM_IK_JOINTS]
        target_right[:NUM_IK_JOINTS] = np.asarray(
            home.get("right", [0.0] * NUM_IK_JOINTS), dtype=float)[:NUM_IK_JOINTS]
        target_left = self.clamp_joint_angles(target_left, "left")
        target_right = self.clamp_joint_angles(target_right, "right")
        # Preserve current gripper opening (don't force it on startup).
        target_left[GRIPPER_INDEX] = start_left[GRIPPER_INDEX]
        target_right[GRIPPER_INDEX] = start_right[GRIPPER_INDEX]

        logger.info(f"Homing arms to start pose over {duration:.1f}s "
                    f"(left {target_left[:NUM_IK_JOINTS].round(0)}, "
                    f"right {target_right[:NUM_IK_JOINTS].round(0)})...")
        ok = self._interpolate_move(target_left, target_right, duration, hz)
        if ok:
            self._settle_at_targets(target_left, target_right)
            logger.info("Arms homed to start pose")
        return ok

    def _settle_at_targets(self, target_left: np.ndarray, target_right: np.ndarray,
                           iterations: int = 8, tol_deg: float = 0.8,
                           max_trim_deg: float = 8.0, wait_s: float = 0.15) -> None:
        """Null steady-state servo droop so the arms physically reach the targets.

        The Feetech servos hold position with proportional control, so under
        gravity load they settle a few degrees short of the commanded angle.
        The droop is asymmetric between the mirrored arms (telemetry Jul 16-17:
        left joint0 sat at -3.9 deg vs right -0.3 deg with both commanded to
        zero), which made the left arm visibly hang lower at the start pose.

        This loop integrates the measured error into a per-joint command trim
        until the MEASURED pose matches the target: cmd = target + trim. The
        trim is bounded so an obstructed joint cannot wind the command up, and
        the gripper is excluded (it is allowed to stall on an object). The
        trimmed command is committed as the held state; at grip engage,
        sync_commanded_to_actual re-anchors the control origin to the measured
        (= true zero) pose, so teleop starts from the right place.
        """
        if not self.is_connected or self.robot is None:
            return

        trim_left = np.zeros(NUM_JOINTS)
        trim_right = np.zeros(NUM_JOINTS)
        try:
            for _ in range(iterations):
                err_left = target_left - self.get_actual_arm_angles("left").astype(float)
                err_right = target_right - self.get_actual_arm_angles("right").astype(float)
                err_left[GRIPPER_INDEX] = 0.0
                err_right[GRIPPER_INDEX] = 0.0

                worst = max(np.abs(err_left[:NUM_IK_JOINTS]).max(),
                            np.abs(err_right[:NUM_IK_JOINTS]).max())
                if worst <= tol_deg:
                    break

                trim_left = np.clip(trim_left + err_left, -max_trim_deg, max_trim_deg)
                trim_right = np.clip(trim_right + err_right, -max_trim_deg, max_trim_deg)

                cmd_left = self.clamp_joint_angles(target_left + trim_left, "left")
                cmd_right = self.clamp_joint_angles(target_right + trim_right, "right")

                action = {}
                for i in range(NUM_IK_JOINTS):
                    action[f"left_joint{i}.pos"] = float(cmd_left[i])
                    action[f"right_joint{i}.pos"] = float(cmd_right[i])
                action["left_gripper.pos"] = float(target_left[GRIPPER_INDEX])
                action["right_gripper.pos"] = float(target_right[GRIPPER_INDEX])
                self.robot.send_action(action)
                time.sleep(wait_s)

            # Commit the trimmed command as the held state so the idle loop
            # keeps the arms level (resending the raw target would let the
            # droop right back in).
            self.left_arm_angles = self.clamp_joint_angles(target_left + trim_left, "left")
            self.right_arm_angles = self.clamp_joint_angles(target_right + trim_right, "right")
            self.left_arm_angles[GRIPPER_INDEX] = target_left[GRIPPER_INDEX]
            self.right_arm_angles[GRIPPER_INDEX] = target_right[GRIPPER_INDEX]

            final_left = self.get_actual_arm_angles("left")[:NUM_IK_JOINTS]
            final_right = self.get_actual_arm_angles("right")[:NUM_IK_JOINTS]
            logger.info(
                "Settled at start pose: residual left %s right %s (trim left %s right %s)",
                (target_left[:NUM_IK_JOINTS] - final_left).round(1),
                (target_right[:NUM_IK_JOINTS] - final_right).round(1),
                trim_left[:NUM_IK_JOINTS].round(1),
                trim_right[:NUM_IK_JOINTS].round(1),
            )
        except Exception as e:
            logger.warning(f"Zero-pose settle loop failed (arms may sit slightly low): {e}")

    def move_to_park_pose(self, duration: float = 2.5) -> bool:
        """Smoothly drive both arms to the hang-down rest pose, keeping torque on.

        Same target as the shutdown park (park_pose.json captured with
        read_joints.py --save, falling back to folding the park_joints), but
        the motors keep holding afterwards - used for the VR "park" combo,
        not for shutdown.
        """
        if not self.is_connected or self.robot is None:
            return False

        start_left = self.get_actual_arm_angles("left").astype(float)
        start_right = self.get_actual_arm_angles("right").astype(float)

        park_pose = self._load_park_pose()
        if park_pose is not None:
            target_left = park_pose["left"].copy()
            target_right = park_pose["right"].copy()
        else:
            target_left = start_left.copy()
            target_right = start_right.copy()
            for j in getattr(self.config, "park_joints", (3,)):
                target_left[int(j)] = 0.0
                target_right[int(j)] = 0.0

        # Preserve the current gripper opening.
        target_left[GRIPPER_INDEX] = start_left[GRIPPER_INDEX]
        target_right[GRIPPER_INDEX] = start_right[GRIPPER_INDEX]

        logger.info(f"Moving arms to rest (park) pose over {duration:.1f}s...")
        ok = self._interpolate_move(target_left, target_right, duration=duration)
        if ok:
            logger.info("Arms at rest pose (torque still on)")
        return ok

    def disengage(self) -> bool:
        """Disengage robot motors gracefully.

        Instead of cutting torque wherever the arms happen to be (which makes
        them flop/drop), first drive both arms smoothly to the captured dead-hang
        pose, let them settle, and only then release torque. The park move and
        release are isolated so a failed move cannot prevent torque release.
        """
        if not self.is_connected:
            logger.info("Robot already disconnected")
            return True

        try:
            self._park_arms()
        except Exception as e:
            logger.error(f"Error parking arms before release: {e}")

        released = False
        try:
            released = self.disable_torque()
        except Exception as e:
            logger.error(f"Error disabling torque: {e}")

        self.is_engaged = False
        logger.info(f"Robot motors DISENGAGED - parked and released (motors released: {bool(released)})")
        return True

    # Optional captured dead-hang/rest pose stored beside config.yaml.
    PARK_POSE_FILE = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        "park_pose.json",
    )

    def _load_park_pose(self):
        """Load the captured dead-hang park pose from disk, or None if unavailable.

        Returns a dict {'left': ndarray[8], 'right': ndarray[8]} of joint angles
        (deg) recorded with ``read_joints.py --save`` while the arms were held in
        the desired rest pose.
        """
        try:
            import json
            with open(self.PARK_POSE_FILE, "r") as f:
                data = json.load(f)
            left = np.asarray(data["left"], dtype=float)
            right = np.asarray(data["right"], dtype=float)
            if left.shape[0] == NUM_JOINTS and right.shape[0] == NUM_JOINTS:
                return {"left": left, "right": right}
            logger.warning(f"Park pose file has wrong shape; ignoring: {self.PARK_POSE_FILE}")
        except FileNotFoundError:
            pass
        except Exception as e:
            logger.warning(f"Could not load park pose ({self.PARK_POSE_FILE}): {e}")
        return None

    def _park_arms(self, duration: float = 2.0, settle: float = 0.4) -> None:
        """Smoothly drive the arms to the captured dead-hang rest pose, then settle.

        On shutdown we glide both arms to the pose recorded in park_pose.json
        (captured with ``read_joints.py --save`` while the arms hung in their
        natural rest position). Releasing torque from that dead-hang pose means
        the arms barely move. If no park pose has been captured, we fall back to
        folding the configured ``park_joints`` (default joint3) to zero. The
        current gripper opening is always preserved so we don't slam it closed.
        Controlled by the ``park_on_disengage`` config flag.
        """
        if not getattr(self.config, "park_on_disengage", True):
            return
        if not self.is_connected or self.robot is None:
            return

        # Shutdown runs only after the control-loop task has stopped, so no
        # legitimate motor transaction can still be active here. Recover a
        # Feetech SDK lock left stale by an earlier interrupted transaction.
        self._clear_stale_bus_lock(getattr(self.robot, "right_bus", None))
        self._clear_stale_bus_lock(getattr(self.robot, "left_bus", None))

        start_left = self.get_actual_arm_angles("left").astype(float)
        start_right = self.get_actual_arm_angles("right").astype(float)

        park_pose = self._load_park_pose()
        if park_pose is not None:
            target_left = park_pose["left"].copy()
            target_right = park_pose["right"].copy()
            logger.info("🅿️  Parking arms to captured dead-hang pose before release...")
        else:
            # Fallback: hold every joint in place and just fold the park joints.
            target_left = start_left.copy()
            target_right = start_right.copy()
            park_joints = getattr(self.config, "park_joints", (3,))
            for j in park_joints:
                target_left[int(j)] = 0.0
                target_right[int(j)] = 0.0
            logger.info(f"🅿️  Parking arms (folding joint(s) {list(park_joints)} down) before release...")

        # Preserve the current gripper opening so parking never slams the gripper.
        target_left[GRIPPER_INDEX] = start_left[GRIPPER_INDEX]
        target_right[GRIPPER_INDEX] = start_right[GRIPPER_INDEX]

        self._interpolate_move(target_left, target_right, duration=duration)
        # Hold briefly so the motors fully settle before torque is removed.
        if settle > 0:
            time.sleep(settle)

    def refresh_measured_state(self, trace_ids: Optional[Dict[str, str]] = None) -> Dict[str, bool]:
        """Refresh cached motor positions and clamp commands before transmission.

        The read is rate-limited by ``telemetry_actual_sample_hz``. Crucially,
        anti-windup runs here, before IK and before the next Goal_Position
        write, rather than after an excessive command has already reached the
        servos.
        """
        now = time.monotonic()
        if self.robot is None or not self.is_connected or not self.is_engaged:
            self.last_lag_clamped = {'left': False, 'right': False}
            return dict(self.last_lag_clamped)

        sample_hz = max(0.0, float(self.config.telemetry_actual_sample_hz))
        sample_interval = 1.0 / sample_hz if sample_hz > 0 else float("inf")
        if now - self.last_actual_telemetry_time < sample_interval:
            return {'left': False, 'right': False}

        # Rate-limit failed reads too; otherwise a disconnected bus is hammered
        # on every 50 Hz control tick.
        self.last_actual_telemetry_time = now
        self.last_lag_clamped = {'left': False, 'right': False}

        sample_started_ns = time.monotonic_ns()
        try:
            observation = self.robot.get_observation()
            left_actual = self._observation_angles(observation, "left")
            right_actual = self._observation_angles(observation, "right")
            sample_finished_ns = time.monotonic_ns()

            self.last_measured_angles['left'] = left_actual.copy()
            self.last_measured_angles['right'] = right_actual.copy()
            self.last_measured_time['left'] = now
            self.last_measured_time['right'] = now
            self.last_measurement_ok = True

            # Record true lag before pulling commanded state back.
            left_error = self.left_arm_angles - left_actual
            right_error = self.right_arm_angles - right_actual
            self.last_lag_clamped = {
                'left': self._apply_command_lag_limit("left", left_actual),
                'right': self._apply_command_lag_limit("right", right_actual),
            }

            if self.telemetry:
                self.telemetry.record(
                    "motor_state",
                    trace_ids=trace_ids or {},
                    left_actual_joints=left_actual,
                    right_actual_joints=right_actual,
                    left_error_deg=left_error,
                    right_error_deg=right_error,
                    windup_clamped=dict(self.last_lag_clamped),
                    serial_read_ms=(sample_finished_ns - sample_started_ns) / 1e6,
                    command_write_to_state_sample_ms=None,
                )
        except Exception as exc:
            self.last_measurement_ok = False
            if self.telemetry:
                self.telemetry.record("motor_state_error", error=str(exc))
        return dict(self.last_lag_clamped)

    def send_command(self, trace_ids: Optional[Dict[str, str]] = None) -> bool:
        """Send current joint angles to robot using dictionary format."""
        if not self.is_connected or not self.is_engaged:
            return False

        current_time = time.monotonic()
        # Permit small scheduler jitter around the fixed 50 Hz deadline;
        # requiring a perfect >=20.000 ms interval can accidentally skip every
        # second write and collapse the effective command rate to 25 Hz.
        if current_time - self.last_send_time < self.config.send_interval * 0.8:
            return True

        try:
            if self.robot is None:
                return False

            # Normally refreshed at the beginning of the control tick. This
            # fallback also protects direct callers of send_command().
            self.refresh_measured_state(trace_ids)
            if not self.last_measurement_ok:
                logger.warning("Suppressing motor write: no complete fresh measurement")
                return False

            # Build combined action dict for both arms
            action_dict = {}

            for i in range(7):  # joint0-joint6
                action_dict[f"left_joint{i}.pos"] = float(self.left_arm_angles[i])
            action_dict["left_gripper.pos"] = float(self.left_arm_angles[GRIPPER_INDEX])

            for i in range(7):  # joint0-joint6
                action_dict[f"right_joint{i}.pos"] = float(self.right_arm_angles[i])
            action_dict["right_gripper.pos"] = float(self.right_arm_angles[GRIPPER_INDEX])

            try:
                write_started_ns = time.monotonic_ns()
                self.robot.send_action(action_dict)
                write_finished_ns = time.monotonic_ns()
            except Exception as e:
                logger.error(f"Error sending robot command: {e}")
                self.general_errors += 1
                if self.general_errors > self.max_general_errors:
                    self.is_connected = False
                    logger.error("Robot interface disconnected due to repeated errors")
                return False

            self.last_send_time = current_time
            if self.telemetry:
                self.telemetry.record(
                    "motor_command",
                    trace_ids=trace_ids or {},
                    left_commanded_joints=self.left_arm_angles,
                    right_commanded_joints=self.right_arm_angles,
                    serial_write_ms=(write_finished_ns - write_started_ns) / 1e6,
                )

            return True

        except Exception as e:
            logger.error(f"Error sending robot command: {e}")
            self.general_errors += 1
            if self.general_errors > self.max_general_errors:
                self.is_connected = False
                logger.error("Robot interface disconnected due to repeated errors")
            return False

    def _apply_command_lag_limit(self, arm: str, measured_angles: np.ndarray) -> bool:
        """Anti-windup: keep commanded body joints within a band of measured.

        If a motor stalls, hits something, or simply can't keep up, the
        commanded state must not keep marching away from the physical arm -
        that severs the link between the IK seed and reality, and the arm
        then holds a huge position error until the next grip re-engage (Jul 17
        telemetry: left joint0 froze 46 deg from its command for 5+ seconds).
        Clamping commanded to measured +/- command_lag_limit_deg keeps torque
        bounded and lets the IK (seeded from the commanded state) recover
        smoothly. The gripper is excluded: it is expected to stall when
        squeezing an object. Returns True if any joint was clamped.
        """
        band = float(getattr(self.config, "command_lag_limit_deg", 0.0))
        if band <= 0:
            return False

        commanded = self.left_arm_angles if arm == "left" else self.right_arm_angles
        body_cmd = commanded[:NUM_IK_JOINTS]
        body_meas = measured_angles[:NUM_IK_JOINTS]
        clamped = np.clip(body_cmd, body_meas - band, body_meas + band)
        changed = bool(np.any(np.abs(clamped - body_cmd) > 1e-9))
        if changed:
            commanded[:NUM_IK_JOINTS] = clamped
        return changed

    def set_gripper(self, arm: str, closed: bool):
        """Set gripper state for specified arm."""
        angle = GRIPPER_CLOSED_ANGLE if closed else GRIPPER_OPEN_ANGLE

        if arm == "left":
            self.left_arm_angles[GRIPPER_INDEX] = angle
        elif arm == "right":
            self.right_arm_angles[GRIPPER_INDEX] = angle
        else:
            raise ValueError(f"Invalid arm: {arm}")

    def get_arm_angles(self, arm: str) -> np.ndarray:
        """Get current joint angles for specified arm."""
        if arm == "left":
            return self.left_arm_angles.copy()
        elif arm == "right":
            return self.right_arm_angles.copy()
        else:
            raise ValueError(f"Invalid arm: {arm}")

    def get_arm_angles_for_visualization(self, arm: str) -> np.ndarray:
        """Get current joint angles for specified arm, for PyBullet visualization."""
        return self.get_arm_angles(arm)

    def get_actual_arm_angles(self, arm: str) -> np.ndarray:
        """Get actual joint angles from robot hardware (not commanded angles)."""
        try:
            if self.robot is not None and self.is_connected:
                observation = self.robot.get_observation()
                if observation:
                    return self._observation_angles(observation, arm)
        except Exception as e:
            logger.debug(f"Error reading actual arm angles for {arm}: {e}")

        return self.get_arm_angles(arm)

    def get_skeleton_state(self) -> Optional[Dict]:
        """Snapshot both arms for the VR skeleton view.

        Per arm: world positions of the joint chain for the COMMANDED pose and
        the last MEASURED pose, per-joint limit proximity (0 = mid-range,
        1 = at a limit), and per-joint command-vs-measured error (deg).
        Uses cached measured angles - no serial traffic.
        """
        if self.fk_solvers['left'] is None or self.fk_solvers['right'] is None:
            return None

        state = {}
        for arm in ('left', 'right'):
            commanded = self.left_arm_angles if arm == 'left' else self.right_arm_angles
            measured = self.last_measured_angles[arm]
            measured_time = self.last_measured_time[arm]
            # More than three expected sample intervals without a read
            # means feedback is stale. Retain commanded geometry only as a
            # placeholder, but explicitly mark it invalid so VR never paints
            # a disconnected/stale arm as healthy.
            sample_hz = max(0.0, float(self.config.telemetry_actual_sample_hz))
            max_age = max(0.6, 3.0 / sample_hz) if sample_hz > 0 else 0.6
            measured_valid = (
                measured is not None
                and measured_time is not None
                and time.monotonic() - measured_time <= max_age
            )
            if not measured_valid:
                measured = commanded

            fk = self.fk_solvers[arm]
            try:
                cmd_points = fk.compute_joint_positions(commanded)
                meas_points = fk.compute_joint_positions(measured)
            except RuntimeError as exc:
                logger.debug("Skeleton FK unavailable for %s arm: %s", arm, exc)
                state[arm] = None
                continue

            lo = self.joint_limits_min_deg[arm][:NUM_IK_JOINTS]
            hi = self.joint_limits_max_deg[arm][:NUM_IK_JOINTS]
            mid = (lo + hi) / 2.0
            half = np.maximum((hi - lo) / 2.0, 1e-6)
            limit_frac = np.clip(np.abs(measured[:NUM_IK_JOINTS] - mid) / half, 0.0, 1.0)

            solver = self.ik_solvers.get(arm)
            state[arm] = {
                'commanded': np.round(cmd_points, 4).tolist(),
                'measured': np.round(meas_points, 4).tolist(),
                'limit_frac': np.round(limit_frac, 3).tolist(),
                'error_deg': np.round(
                    commanded[:NUM_IK_JOINTS] - measured[:NUM_IK_JOINTS], 1).tolist(),
                'measured_valid': measured_valid,
                'workspace_center': (np.round(solver.workspace_center, 4).tolist()
                                     if solver is not None else None),
            }
        if not any(state.get(arm) is not None for arm in ('left', 'right')):
            return None
        state['workspace_radius'] = float(self.config.ik_workspace_radius_m)
        return state

    def disable_torque(self, arm: str = None, retries: int = 4, retry_delay: float = 0.1):
        """Disable torque (release/limp) on robot joints, robustly.

        Each arm is released independently so a serial hiccup on one arm can
        NEVER prevent the other arm from being released (previously a single
        failure left the other arm powered). Writes are retried, and if the
        bulk disable keeps failing we fall back to releasing each motor
        individually. This is what guarantees the arms go limp on Ctrl+C.

        Args:
            arm: 'left', 'right', or None for both arms
        """
        if self.robot is None:
            return

        buses = []
        if arm is None or arm == "right":
            buses.append(("RIGHT", getattr(self.robot, "right_bus", None)))
        if arm is None or arm == "left":
            buses.append(("LEFT", getattr(self.robot, "left_bus", None)))

        all_released = True
        for name, bus in buses:
            if bus is None:
                continue
            logger.info(f"Disabling torque on {name} arm...")
            if self._release_bus_torque(bus, retries, retry_delay):
                logger.info(f"{name} arm torque disabled (motors released)")
            else:
                all_released = False
                logger.error(f"Could NOT fully release {name} arm torque after {retries} retries")

        return all_released

    def _release_bus_torque(self, bus, retries: int, retry_delay: float) -> bool:
        """Best-effort torque release for one arm's bus. Returns True on success."""
        self._clear_stale_bus_lock(bus)

        # Try the bulk disable first (fast path).
        for attempt in range(retries):
            try:
                bus.disable_torque()
                return True
            except Exception as e:
                logger.warning(f"Bulk torque disable failed (attempt {attempt + 1}/{retries}): {e}")
                if "Port is in use" in str(e):
                    self._clear_stale_bus_lock(bus)
                time.sleep(retry_delay)

        # Fall back to per-motor release so one unresponsive motor doesn't
        # block releasing the rest of the arm.
        all_ok = True
        for motor_name in ("joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"):
            released = False
            for attempt in range(retries):
                try:
                    bus.write("Torque_Enable", motor_name, 0)
                    released = True
                    break
                except Exception:
                    time.sleep(retry_delay)
            if not released:
                all_ok = False
                logger.error(f"Failed to release motor '{motor_name}'")
        return all_ok

    @staticmethod
    def _clear_stale_bus_lock(bus) -> None:
        """Clear an orphaned Feetech SDK port lock during shutdown.

        The SDK sets ``PortHandler.is_using`` while a packet is in flight. If a
        previous process was interrupted inside that transaction, the flag can
        remain true and reject every later safety write. Callers use this only
        after the control task has stopped, when no real transaction is active.
        """
        if bus is None:
            return
        port_handler = getattr(bus, "port_handler", None)
        if port_handler is not None and getattr(port_handler, "is_using", False):
            logger.warning("Clearing stale Feetech 'Port is in use' lock for safe shutdown")
            port_handler.is_using = False

    def disconnect(self):
        """Disconnect from robot hardware."""
        if not self.is_connected:
            return

        logger.info("Disconnecting from robot...")

        # If we're still engaged at disconnect time (e.g. disconnect called
        # without a prior disengage), park the arms in the safe hang-down pose
        # first so releasing torque below doesn't drop them.
        if self.is_engaged:
            try:
                self._park_arms()
            except Exception as e:
                logger.error(f"Error parking arms on disconnect: {e}")

        # Always release the motors before tearing down the connection, so the
        # arms reliably go limp even if the lower-level disconnect fails.
        try:
            self.disable_torque()
        except Exception as e:
            logger.error(f"Error releasing torque on disconnect: {e}")

        self.is_engaged = False

        if self.robot:
            try:
                self.robot.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting robot: {e}")
            self.robot = None

        self.is_connected = False
        self.is_engaged = False
        self.left_arm_connected = False
        self.right_arm_connected = False
        logger.info("Robot disconnected")

    def get_arm_connection_status(self, arm: str) -> bool:
        """Get live bus status; a surviving device symlink is not link health."""
        if self.robot is None or not self.is_connected:
            return False
        if arm == "left":
            return bool(self.left_arm_connected and self.robot.left_bus.is_connected)
        if arm == "right":
            return bool(self.right_arm_connected and self.robot.right_bus.is_connected)
        return False

    def update_arm_connection_status(self):
        """Update individual arm status from each serial bus."""
        if self.robot is not None and self.is_connected:
            self.left_arm_connected = bool(self.robot.left_bus.is_connected)
            self.right_arm_connected = bool(self.robot.right_bus.is_connected)

    @property
    def status(self) -> Dict:
        """Get robot status information."""
        return {
            "connected": self.is_connected,
            "left_arm_connected": self.left_arm_connected,
            "right_arm_connected": self.right_arm_connected,
            "left_arm_angles": self.left_arm_angles.tolist(),
            "right_arm_angles": self.right_arm_angles.tolist(),
            "joint_limits_min": {arm: lim.tolist() for arm, lim in self.joint_limits_min_deg.items()},
            "joint_limits_max": {arm: lim.tolist() for arm, lim in self.joint_limits_max_deg.items()},
        }
