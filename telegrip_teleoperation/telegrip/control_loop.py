"""
Main control loop for the teleoperation system.
Consumes control goals from the command queue and executes them via the robot interface.
"""

import asyncio
import numpy as np
import logging
import time
import queue  # Add import for thread-safe queue
from typing import Dict, Optional

from scipy.spatial.transform import Rotation as R

from .config import TelegripConfig, NUM_JOINTS, GRIPPER_INDEX
from .core.robot_interface import RobotInterface
# PyBulletVisualizer will be imported on demand
from .inputs.base import ControlGoal, ControlMode
# WebKeyboardHandler will be imported on demand to avoid circular imports

logger = logging.getLogger(__name__)


class ArmState:
    """State tracking for a single robot arm."""
    
    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None  # For visualization
        self.origin_position = None  # Robot position when grip was activated
        # TCP orientation captured when grip engaged, and the current absolute
        # commanded TCP orientation. Both are [x, y, z, w] quaternions.
        self.origin_orientation_quat = None
        self.target_orientation_quat = None
        self.latest_relative_position = np.zeros(3)
        self.latest_relative_orientation_quat = None
        self.trace_id = None
        self.input_received_monotonic_ns = None
        
    def reset(self):
        """Reset arm state to idle."""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None
        self.origin_position = None
        self.origin_orientation_quat = None
        self.target_orientation_quat = None
        self.latest_relative_position = np.zeros(3)
        self.latest_relative_orientation_quat = None
        self.trace_id = None
        self.input_received_monotonic_ns = None


class ControlLoop:
    """Main control loop that processes command queue and controls robot."""
    
    def __init__(
        self,
        command_queue: asyncio.Queue,
        config: TelegripConfig,
        control_commands_queue: Optional[queue.Queue] = None,
        telemetry=None,
    ):
        self.command_queue = command_queue
        self.control_commands_queue = control_commands_queue
        self.config = config
        self.telemetry = telemetry
        
        # Components
        self.robot_interface = None
        self.visualizer = None
        self.web_keyboard_handler = None  # Reference to web-based keyboard handler
        self.vr_server = None  # Set by main; used for the skeleton broadcast

        # VR skeleton view broadcast timing
        self.skeleton_interval = 0.1  # 10 Hz
        self._last_skeleton_time = 0.0
        
        # Arm states
        self.left_arm = ArmState("left")
        self.right_arm = ArmState("right")
        
        # Control timing
        self.last_log_time = 0
        self.log_interval = 1.0  # Log status every second
        
        # Debug flags
        self._queue_debug_logged = False
        self._process_debug_logged = False
        
        self.is_running = False
    
    def setup(self) -> bool:
        """Setup robot interface and visualizer."""
        success = True
        setup_errors = []
        
        # Setup robot interface
        try:
            self.robot_interface = RobotInterface(self.config, self.telemetry)
            if not self.robot_interface.connect():
                error_msg = "Robot interface failed to connect"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                if self.config.enable_robot:
                    success = False
        except Exception as e:
            error_msg = f"Robot interface setup failed with exception: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            if self.config.enable_robot:
                success = False
        
        # Setup mandatory PyBullet kinematics. The GUI is optional, but the
        # DIRECT-mode physics client is required because it supplies FK/IK.
        if not self.config.enable_pybullet:
            error_msg = "PyBullet kinematics cannot be disabled for Cartesian arm control"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            success = False
        else:
            try:
                # Import PyBulletVisualizer on demand
                from .core.visualizer import PyBulletVisualizer
                
                self.visualizer = PyBulletVisualizer(
                    self.config.get_absolute_urdf_paths(),
                    use_gui=self.config.enable_pybullet_gui,
                    log_level=self.config.log_level
                )
                if not self.visualizer.setup():
                    error_msg = "Required PyBullet kinematics engine failed to initialize"
                    logger.error(error_msg)
                    setup_errors.append(error_msg)
                    self.visualizer = None
                    success = False
                else:
                    # Connect kinematics to robot interface
                    joint_limits_min, joint_limits_max = self.visualizer.get_joint_limits
                    self.robot_interface.setup_kinematics(
                        self.visualizer.physics_client,
                        self.visualizer.robot_ids,  # Pass both robot instances
                        self.visualizer.joint_indices,  # Pass both joint index mappings
                        self.visualizer.end_effector_link_indices,  # Pass both end effector indices
                        joint_limits_min,
                        joint_limits_max
                    )
            except Exception as e:
                error_msg = f"Required PyBullet kinematics engine failed with exception: {e}"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                self.visualizer = None
                success = False
        
        # Report all setup issues
        if setup_errors:
            logger.error("Setup failed with the following errors:")
            for i, error in enumerate(setup_errors, 1):
                logger.error(f"  {i}. {error}")
        
        # Set robot interface on web keyboard handler so it can get current positions
        if self.web_keyboard_handler and self.robot_interface:
            self.web_keyboard_handler.set_robot_interface(self.robot_interface)
            logger.info("Set robot interface on web keyboard handler")

        return success
    
    async def start(self):
        """Start the control loop."""
        if not self.setup():
            logger.error("Control loop setup failed")
            return
        
        self.is_running = True
        logger.info("Control loop started")
        
        # Initialize arm states with current robot positions
        self._initialize_arm_states()
        
        # Main control loop
        next_tick = time.monotonic()
        while self.is_running:
            try:
                # Process command queue
                await self._process_commands()
                
                # Update robot (with error resilience)
                self._update_robot_safely()
                
                # Update visualization
                if self.visualizer and self.visualizer.gui_active:
                    self._update_visualization()
                
                # Stream the arm skeleton to VR clients
                self._broadcast_skeleton()

                # Periodic logging
                self._periodic_logging()
                
                # Fixed-deadline scheduling: processing time counts toward the
                # period. Sleeping a full interval after work made the nominal
                # 50 Hz loop run at only ~37-45 Hz under IK/serial load.
                next_tick += self.config.send_interval
                delay = next_tick - time.monotonic()
                if delay > 0:
                    await asyncio.sleep(delay)
                else:
                    # Do not burst through missed ticks; resume from now.
                    next_tick = time.monotonic()
                
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                next_tick = time.monotonic()
                await asyncio.sleep(0.1)
        
        logger.info("Control loop stopped")
    
    def _broadcast_skeleton(self):
        """Push the current arm skeleton (commanded + measured) to VR clients."""
        if self.vr_server is None or not self.vr_server.skeleton_clients:
            return
        now = time.monotonic()
        if now - self._last_skeleton_time < self.skeleton_interval:
            return
        self._last_skeleton_time = now
        try:
            if self.robot_interface is None:
                return
            state = self.robot_interface.get_skeleton_state()
            if state is None:
                return
            state['type'] = 'robot_skeleton'
            self.vr_server.broadcast(state)
        except Exception as e:
            logger.debug(f"Skeleton broadcast failed: {e}")

    async def stop(self):
        """Stop the control loop."""
        self.is_running = False

        # Cleanup - disengage robot first (returns to home and disables torque)
        if self.robot_interface:
            if self.robot_interface.is_engaged:
                logger.info("🛑 Disengaging robot before shutdown...")
                self.robot_interface.disengage()
            self.robot_interface.disconnect()

        if self.visualizer:
            self.visualizer.disconnect()
    
    def _initialize_arm_states(self):
        """Initialize arm states with current robot positions."""
        if self.robot_interface:
            # Get current end effector positions
            left_pos = self.robot_interface.get_current_end_effector_position("left")
            right_pos = self.robot_interface.get_current_end_effector_position("right")
            
            # Initialize target positions to current positions (ensure deep copies)
            self.left_arm.target_position = left_pos.copy()
            self.left_arm.goal_position = left_pos.copy()
            self.right_arm.target_position = right_pos.copy()
            self.right_arm.goal_position = right_pos.copy()

            # Seed the commanded TCP orientation from the current pose so that
            # the first IK solve does not request an orientation change.
            for arm_name, arm_state in (("left", self.left_arm), ("right", self.right_arm)):
                orientation = self.robot_interface.get_current_end_effector_orientation(arm_name)
                arm_state.origin_orientation_quat = orientation.copy()
                arm_state.target_orientation_quat = orientation.copy()

            logger.info(f"Initialized left arm at position: {left_pos.round(3)}")
            logger.info(f"Initialized right arm at position: {right_pos.round(3)}")

    def _capture_origin(self, arm: str, arm_state: "ArmState") -> bool:
        """Snapshot the current TCP pose as the origin for relative control."""
        # Align the commanded state with the measured motor angles first, so
        # the origin describes the PHYSICAL arm. Otherwise, if the motors
        # lagged earlier commands, every subsequent relative target inherits
        # that offset and the arm starts from the wrong place.
        try:
            if not self.robot_interface.sync_commanded_to_actual(arm):
                logger.warning("Refusing %s origin capture without measured state", arm)
                return False
        except Exception as e:
            logger.warning(f"Could not sync {arm} commanded state to measured angles: {e}")
            return False

        current_position = self.robot_interface.get_current_end_effector_position(arm)
        current_orientation = self.robot_interface.get_current_end_effector_orientation(arm)

        arm_state.target_position = current_position.copy()
        arm_state.goal_position = current_position.copy()
        arm_state.origin_position = current_position.copy()
        arm_state.origin_orientation_quat = current_orientation.copy()
        # Begin each grip in position-only mode. VR orientation is enabled only
        # after the controller exceeds its configured intentional-rotation
        # deadband, preventing redundant IK joints from twisting during a
        # straight translation such as lifting the hand.
        arm_state.target_orientation_quat = None
        arm_state.latest_relative_position = np.zeros(3)
        arm_state.latest_relative_orientation_quat = None
        return True
    
    async def _process_commands(self):
        """Process commands from the command queue."""
        try:
            # Drain the queue, retaining only the newest high-rate VR motion
            # goal per arm within each uninterrupted motion segment. Mode,
            # release, reset and gripper goals remain ordered and lossless.
            pending = []
            latest_vr_index = {}
            while not self.command_queue.empty():
                goal = self.command_queue.get_nowait()
                metadata = goal.metadata or {}
                if metadata.get("source") == "vr_grip":
                    old_index = latest_vr_index.get(goal.arm)
                    if old_index is not None:
                        pending[old_index] = None
                    latest_vr_index[goal.arm] = len(pending)
                else:
                    latest_vr_index.pop(goal.arm, None)
                pending.append(goal)

            for goal in pending:
                if goal is None:
                    continue
                await self._execute_goal(goal)
        except Exception as e:
            logger.error(f"Error processing commands: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
    
    async def _handle_command(self, command):
        """Handle individual commands."""
        action = command.get('action', '')
        logger.info(f"🔌 Processing control command: {action}")
        
        if action == 'enable_keyboard':
            if self.web_keyboard_handler:
                await self.web_keyboard_handler.start()
                logger.info("🎮 Keyboard control ENABLED via API")
        elif action == 'disable_keyboard':
            if self.web_keyboard_handler:
                await self.web_keyboard_handler.stop()
                logger.info("🎮 Keyboard control DISABLED via API")
        elif action == 'web_keypress':
            # Handle individual keypress events from web interface
            key = command.get('key')
            event = command.get('event')  # 'press' or 'release'

            if self.web_keyboard_handler and self.web_keyboard_handler.is_enabled:
                logger.debug(f"🌐 Processing web keypress: {key}_{event}")
                if event == 'press':
                    self.web_keyboard_handler.on_key_press(key)
                elif event == 'release':
                    self.web_keyboard_handler.on_key_release(key)
            else:
                logger.warning("🎮 Web keyboard handler not enabled")
        elif action == 'robot_connect':
            logger.info("🔌 Processing robot_connect command")
            if self.robot_interface and self.robot_interface.is_connected:
                logger.info(f"🔌 Robot interface available and connected: {self.robot_interface.is_connected}")
                success = self.robot_interface.engage()
                if success:
                    logger.info("🔌 Robot motors ENGAGED via API")
                    # No need to sync keyboard targets - unified system handles this automatically
                else:
                    logger.error("❌ Failed to engage robot motors")
            else:
                logger.warning(f"Cannot engage robot: interface={self.robot_interface is not None}, connected={self.robot_interface.is_connected if self.robot_interface else False}")
        elif action == 'robot_disconnect':
            logger.info("🔌 Processing robot_disconnect command")
            if self.robot_interface:
                logger.info(f"🔌 Robot interface available")
                success = self.robot_interface.disengage()
                if success:
                    logger.info("🔌 Robot motors DISENGAGED via API")
                    # Reset arm states to IDLE when robot is disengaged
                    self.left_arm.reset()
                    self.right_arm.reset()
                    logger.info("🔓 Both arms: Position control DEACTIVATED after robot disconnect")
                    
                    # Hide visualization markers
                    if self.visualizer:
                        for arm in ["left", "right"]:
                            self.visualizer.hide_marker(f"{arm}_goal")
                            self.visualizer.hide_frame(f"{arm}_goal_frame")
                            self.visualizer.hide_marker(f"{arm}_target")
                            self.visualizer.hide_frame(f"{arm}_target_frame")
                else:
                    logger.error("❌ Failed to disengage robot motors")
            else:
                logger.warning("Cannot disengage robot: no robot interface")
        else:
            logger.warning(f"Unknown command: {action}")

    async def _execute_goal(self, goal: ControlGoal):
        """Execute a control goal."""
        arm_state = self.left_arm if goal.arm == "left" else self.right_arm
        dequeued_ns = time.monotonic_ns()
        metadata = goal.metadata or {}
        input_received_ns = metadata.get("input_received_monotonic_ns")
        trace_id = metadata.get("trace_id")
        if self.telemetry:
            self.telemetry.record(
                "control_goal_dequeued",
                trace_id=trace_id,
                arm=goal.arm,
                source=metadata.get("source"),
                mode=goal.mode.value if goal.mode is not None else None,
                target_position=goal.target_position,
                wrist_roll_deg=goal.wrist_roll_deg,
                wrist_flex_deg=goal.wrist_flex_deg,
                gripper_closed=goal.gripper_closed,
                input_to_dequeue_ms=(
                    (dequeued_ns - input_received_ns) / 1e6
                    if input_received_ns is not None else None
                ),
            )
        
        # Handle preset-pose combos from the VR controllers (A+B = home to
        # zero pose, X+Y = park to rest pose). Applies to BOTH arms: position
        # control is dropped so nothing fights the move, then the arms glide
        # to the preset and hold it.
        preset_pose = metadata.get("preset_pose")
        if preset_pose in ("home", "park"):
            if (self.robot_interface and self.robot_interface.is_connected
                    and self.robot_interface.is_engaged):
                self.left_arm.reset()
                self.right_arm.reset()
                if self.visualizer:
                    for arm in ("left", "right"):
                        self.visualizer.hide_marker(f"{arm}_goal")
                        self.visualizer.hide_frame(f"{arm}_goal_frame")
                        self.visualizer.hide_marker(f"{arm}_target")
                        self.visualizer.hide_frame(f"{arm}_target_frame")
                if preset_pose == "home":
                    logger.info("🏠 VR combo: homing both arms to zero pose")
                    self.robot_interface.move_to_zero_pose()
                else:
                    logger.info("🛌 VR combo: parking both arms to rest pose")
                    self.robot_interface.move_to_park_pose()
            else:
                logger.warning(f"Ignoring preset pose '{preset_pose}': robot not engaged")
            return

        # Handle special reset signal from keyboard idle timeout
        if (goal.metadata and goal.metadata.get("reset_target_to_current", False)):
            if self.robot_interface and arm_state.mode == ControlMode.POSITION_CONTROL:
                if self._capture_origin(goal.arm, arm_state):
                    logger.info(f"🔄 {goal.arm.upper()} arm: Target pose reset to current robot pose (idle timeout)")
            return
        
        # Handle mode changes (only if mode is specified)
        if goal.mode is not None and goal.mode != arm_state.mode:
            if goal.mode == ControlMode.POSITION_CONTROL:
                # Activate position control - always reset target to current pose
                if (not self.robot_interface
                        or not self._capture_origin(goal.arm, arm_state)):
                    arm_state.reset()
                    logger.warning("Could not activate %s arm without fresh motor feedback",
                                   goal.arm.upper())
                    return
                arm_state.mode = ControlMode.POSITION_CONTROL
                
                logger.info(f"🔒 {goal.arm.upper()} arm: Position control ACTIVATED (target reset to current pose)")
                
            elif goal.mode == ControlMode.IDLE:
                # Deactivate position control
                arm_state.reset()
                
                # Hide visualization markers
                if self.visualizer:
                    self.visualizer.hide_marker(f"{goal.arm}_goal")
                    self.visualizer.hide_frame(f"{goal.arm}_goal_frame")
                
                logger.info(f"🔓 {goal.arm.upper()} arm: Position control DEACTIVATED")
        
        # Handle position control - both VR and keyboard now work the same way (absolute offset from origin)
        if goal.target_position is not None and arm_state.mode == ControlMode.POSITION_CONTROL:
            arm_state.trace_id = trace_id
            arm_state.input_received_monotonic_ns = input_received_ns
            if goal.metadata and goal.metadata.get("relative_position", False):
                # Both VR and keyboard send absolute offset from robot origin position
                arm_state.latest_relative_position = np.asarray(
                    goal.target_position, dtype=float).copy()
                if arm_state.origin_position is not None:
                    arm_state.target_position = arm_state.origin_position + goal.target_position
                    arm_state.goal_position = arm_state.target_position.copy()
                else:
                    # No origin set yet, use current position as base
                    if self.robot_interface:
                        current_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                        arm_state.target_position = current_position + goal.target_position
                        arm_state.goal_position = arm_state.target_position.copy()
            else:
                # Absolute position (legacy - should not be used anymore)
                arm_state.target_position = goal.target_position.copy()
                arm_state.goal_position = goal.target_position.copy()

            self._update_target_orientation(arm_state, goal)
        
        # Handle gripper control (independent of mode)
        if goal.gripper_closed is not None and self.robot_interface:
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed)

    def _update_target_orientation(self, arm_state: "ArmState", goal: ControlGoal):
        """Compose the commanded absolute TCP orientation from an input goal.

        VR provides a relative rotation quaternion (controller motion since
        grip). Keyboard provides incremental roll/flex offsets, which are
        applied about the robot Z and X axes so both input methods drive the
        same orientation target consumed by full-pose IK.
        """
        if arm_state.origin_orientation_quat is None:
            return

        origin = R.from_quat(arm_state.origin_orientation_quat)

        if goal.target_orientation_quat is not None:
            relative = R.from_quat(goal.target_orientation_quat)
            arm_state.latest_relative_orientation_quat = np.asarray(
                goal.target_orientation_quat, dtype=float).copy()
            arm_state.target_orientation_quat = (relative * origin).as_quat()
            return

        if goal.wrist_roll_deg is not None or goal.wrist_flex_deg is not None:
            roll = np.deg2rad(goal.wrist_roll_deg or 0.0)
            flex = np.deg2rad(goal.wrist_flex_deg or 0.0)
            # World-frame increments: roll about Z (up), flex about X (forward).
            relative = R.from_euler("xyz", [flex, 0.0, roll])
            arm_state.target_orientation_quat = (relative * origin).as_quat()
    
    def _update_robot_safely(self):
        """Update robot with current control goals (with error handling)."""
        if not self.robot_interface:
            return
        
        try:
            self._update_robot()
        except Exception as e:
            logger.error(f"Error updating robot: {e}")
            # Don't shutdown, just continue - robot interface will handle connection issues

    def _reanchor_after_lag_clamp(self, arm: str, arm_state: ArmState):
        """Discard unreachable Cartesian windup while preserving controller continuity."""
        if arm_state.mode != ControlMode.POSITION_CONTROL:
            return

        reachable_position = self.robot_interface.get_current_end_effector_position(arm)
        relative_position = np.asarray(
            arm_state.latest_relative_position, dtype=float)
        arm_state.origin_position = reachable_position - relative_position
        arm_state.target_position = reachable_position.copy()
        arm_state.goal_position = reachable_position.copy()

        reachable_orientation = self.robot_interface.get_current_end_effector_orientation(arm)
        relative_quat = arm_state.latest_relative_orientation_quat
        if relative_quat is not None:
            relative = R.from_quat(relative_quat)
            current = R.from_quat(reachable_orientation)
            arm_state.origin_orientation_quat = (relative.inv() * current).as_quat()
        else:
            arm_state.origin_orientation_quat = reachable_orientation.copy()
        arm_state.target_orientation_quat = reachable_orientation.copy()
        logger.debug("%s Cartesian origin re-anchored after motor lag clamp", arm)
    
    def _update_robot(self):
        """Update robot with current control goals."""
        if not self.robot_interface:
            return

        trace_ids = {
            "left": self.left_arm.trace_id,
            "right": self.right_arm.trace_id,
        }
        lag_clamped = self.robot_interface.refresh_measured_state(trace_ids)
        if lag_clamped.get("left"):
            self._reanchor_after_lag_clamp("left", self.left_arm)
        if lag_clamped.get("right"):
            self._reanchor_after_lag_clamp("right", self.right_arm)
        
        # Update left arm (only if connected)
        if (self.left_arm.mode == ControlMode.POSITION_CONTROL and 
            self.left_arm.target_position is not None and
            self.robot_interface.get_arm_connection_status("left")):
            
            # Solve full-pose IK for all seven body joints
            ik_started_ns = time.monotonic_ns()
            ik_solution = self.robot_interface.solve_ik(
                "left", self.left_arm.target_position, self.left_arm.target_orientation_quat)
            ik_finished_ns = time.monotonic_ns()
            
            # Update robot angles
            self.robot_interface.update_arm_angles("left", ik_solution)
            if self.telemetry:
                self.telemetry.record(
                    "ik_result",
                    trace_id=self.left_arm.trace_id,
                    arm="left",
                    target_position=self.left_arm.target_position,
                    ik_solution=ik_solution,
                    commanded_joints=self.robot_interface.get_arm_angles("left"),
                    ik_ms=(ik_finished_ns - ik_started_ns) / 1e6,
                    input_to_ik_ms=(
                        (ik_finished_ns - self.left_arm.input_received_monotonic_ns) / 1e6
                        if self.left_arm.input_received_monotonic_ns is not None else None
                    ),
                    solve_info=self.robot_interface.get_last_ik_info("left"),
                )

        # Update right arm (only if connected)
        if (self.right_arm.mode == ControlMode.POSITION_CONTROL and 
            self.right_arm.target_position is not None and
            self.robot_interface.get_arm_connection_status("right")):
            
            # Solve full-pose IK for all seven body joints
            ik_started_ns = time.monotonic_ns()
            ik_solution = self.robot_interface.solve_ik(
                "right", self.right_arm.target_position, self.right_arm.target_orientation_quat)
            ik_finished_ns = time.monotonic_ns()
            
            # Update robot angles
            self.robot_interface.update_arm_angles("right", ik_solution)
            if self.telemetry:
                self.telemetry.record(
                    "ik_result",
                    trace_id=self.right_arm.trace_id,
                    arm="right",
                    target_position=self.right_arm.target_position,
                    ik_solution=ik_solution,
                    commanded_joints=self.robot_interface.get_arm_angles("right"),
                    ik_ms=(ik_finished_ns - ik_started_ns) / 1e6,
                    input_to_ik_ms=(
                        (ik_finished_ns - self.right_arm.input_received_monotonic_ns) / 1e6
                        if self.right_arm.input_received_monotonic_ns is not None else None
                    ),
                    solve_info=self.robot_interface.get_last_ik_info("right"),
                )

        # Send commands to robot
        if self.robot_interface.is_connected and self.robot_interface.is_engaged:
            self.robot_interface.send_command(trace_ids)
    
    def _update_visualization(self):
        """Update PyBullet visualization."""
        if not self.visualizer:
            return
        
        # Update robot poses for both arms using ACTUAL angles from robot hardware
        left_angles = self.robot_interface.get_actual_arm_angles("left")
        right_angles = self.robot_interface.get_actual_arm_angles("right")
        
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')
        
        # Update visualization markers
        if self.left_arm.mode == ControlMode.POSITION_CONTROL:
            if self.left_arm.target_position is not None:
                # Show current end effector position
                current_pos = self.robot_interface.get_current_end_effector_position("left")
                self.visualizer.update_marker_position("left_target", current_pos)
                self.visualizer.update_coordinate_frame("left_target_frame", current_pos)
            
            if self.left_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("left_goal", self.left_arm.goal_position)
                self.visualizer.update_coordinate_frame("left_goal_frame", self.left_arm.goal_position)
        else:
            # Hide markers when not in position control
            self.visualizer.hide_marker("left_target")
            self.visualizer.hide_marker("left_goal")
            self.visualizer.hide_frame("left_target_frame")
            self.visualizer.hide_frame("left_goal_frame")
        
        if self.right_arm.mode == ControlMode.POSITION_CONTROL:
            if self.right_arm.target_position is not None:
                # Show current end effector position
                current_pos = self.robot_interface.get_current_end_effector_position("right")
                self.visualizer.update_marker_position("right_target", current_pos)
                self.visualizer.update_coordinate_frame("right_target_frame", current_pos)
            
            if self.right_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("right_goal", self.right_arm.goal_position)
                self.visualizer.update_coordinate_frame("right_goal_frame", self.right_arm.goal_position)
        else:
            # Hide markers when not in position control
            self.visualizer.hide_marker("right_target")
            self.visualizer.hide_marker("right_goal")
            self.visualizer.hide_frame("right_target_frame")
            self.visualizer.hide_frame("right_goal_frame")
        
        # Step simulation
        self.visualizer.step_simulation()
    
    def _periodic_logging(self):
        """Log status information periodically."""
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            
            active_arms = []
            if self.left_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("LEFT")
            if self.right_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("RIGHT")
            
            if active_arms and self.robot_interface:
                left_angles = self.robot_interface.get_arm_angles("left")
                right_angles = self.robot_interface.get_arm_angles("right")
                logger.info(f"🤖 Active control: {', '.join(active_arms)} | Left: {left_angles.round(1)} | Right: {right_angles.round(1)}")
    
    @property
    def status(self) -> Dict:
        """Get current control loop status."""
        return {
            "running": self.is_running,
            "left_arm_mode": self.left_arm.mode.value,
            "right_arm_mode": self.right_arm.mode.value,
            "robot_connected": self.robot_interface.is_connected if self.robot_interface else False,
            "left_arm_connected": self.robot_interface.get_arm_connection_status("left") if self.robot_interface else False,
            "right_arm_connected": self.robot_interface.get_arm_connection_status("right") if self.robot_interface else False,
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
        } 
