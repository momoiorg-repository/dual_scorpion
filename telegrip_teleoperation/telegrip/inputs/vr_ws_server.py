"""
VR WebSocket server for receiving controller data from web browsers.
Adapted from the original vr_robot_teleop.py script.
"""

import asyncio
import json
import ssl
import websockets
import numpy as np
import math
import logging
import time
from typing import Dict, Optional, Set
from scipy.spatial.transform import Rotation as R

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TelegripConfig
from ..core.kinematics import compute_relative_orientation, compute_relative_position

logger = logging.getLogger(__name__)


class VRControllerState:
    """State tracking for a VR controller."""
    
    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        self.trigger_active = False
        
        # Position tracking for relative movement
        self.origin_position = None
        
        # Quaternion-based rotation tracking (more stable than Euler)
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None  # Accumulated rotation as quaternion

        # Headset yaw (deg) snapshotted when grip engaged. Controller deltas
        # are expressed relative to where the operator was facing at that
        # moment, so "forward" never depends on the Quest room recentre.
        self.origin_headset_yaw_deg = 0.0

        # Position tracking
        self.current_position = None
    
    def reset_grip(self):
        """Reset grip state but preserve trigger state."""
        self.grip_active = False
        self.origin_position = None
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None
        self.origin_headset_yaw_deg = 0.0


class VRWebSocketServer(BaseInputProvider):
    """WebSocket server for VR controller input."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig, telemetry=None):
        super().__init__(command_queue)
        self.config = config
        self.telemetry = telemetry
        self.clients: Set = set()
        self.skeleton_clients: Set = set()
        self._skeleton_send_tasks = {}
        self.server = None
        
        # Controller states
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")
        
        # Robot state tracking (for relative position calculation)
        self.left_arm_origin_position = None
        self.right_arm_origin_position = None

    def broadcast(self, payload: dict):
        """Send one latest skeleton frame per subscribed client, without backlog."""
        if not self.skeleton_clients:
            return
        message = json.dumps(payload)
        for client in list(self.skeleton_clients):
            previous = self._skeleton_send_tasks.get(client)
            if previous is not None and not previous.done():
                continue
            try:
                task = asyncio.create_task(client.send(message))
                self._skeleton_send_tasks[client] = task
                task.add_done_callback(
                    lambda completed, c=client:
                    self._finish_skeleton_send(c, completed))
            except Exception:
                pass  # Disconnects are handled by the client handler.

    def _finish_skeleton_send(self, client, task):
        self._skeleton_send_tasks.pop(client, None)
        try:
            task.result()
        except Exception:
            self.skeleton_clients.discard(client)

    def _get_local_ip(self) -> str:
        """Get the local IP address of this machine."""
        import socket
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            try:
                return socket.gethostbyname(socket.gethostname())
            except Exception:
                return "localhost"

    def setup_ssl(self) -> Optional[ssl.SSLContext]:
        """Setup SSL context for WebSocket server."""
        # Automatically generate SSL certificates if they don't exist
        if not self.config.ssl_files_exist:
            logger.info("SSL certificates not found for WebSocket server, attempting to generate them...")
            if not self.config.ensure_ssl_certificates():
                logger.error("Failed to generate SSL certificates for WebSocket server")
                return None
        
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        try:
            # Get absolute paths for SSL certificates
            cert_path, key_path = self.config.get_absolute_ssl_paths()
            ssl_context.load_cert_chain(certfile=cert_path, keyfile=key_path)
            logger.info("SSL certificate and key loaded successfully for WebSocket server")
            return ssl_context
        except ssl.SSLError as e:
            logger.error(f"Error loading SSL cert/key: {e}")
            return None
    
    async def start(self):
        """Start the WebSocket server."""
        if not self.config.enable_vr:
            logger.info("VR WebSocket server disabled in configuration")
            return
        
        ssl_context = self.setup_ssl()
        if ssl_context is None:
            logger.error("Failed to setup SSL for WebSocket server")
            return
        
        host = self.config.host_ip
        port = self.config.websocket_port
        self._browser_warning_shown = False

        try:
            self.server = await websockets.serve(
                self.websocket_handler,
                host,
                port,
                ssl=ssl_context,
                process_request=self._process_request
            )
            self.is_running = True
            host_display = self._get_local_ip() if host == "0.0.0.0" else host
            logger.info(f"VR WebSocket server running on wss://{host_display}:{port}")
        except Exception as e:
            logger.error(f"Failed to start WebSocket server: {e}")

    async def _process_request(self, connection, request):
        """Process incoming requests and detect browser visits to the WebSocket port."""
        # Check if this looks like a browser request (not a proper WebSocket upgrade)
        # In newer websockets versions, request.headers is a Headers object
        headers = request.headers
        connection_header = headers.get("Connection", "")
        upgrade_header = headers.get("Upgrade", "")

        # Proper WebSocket requests have "Upgrade" in Connection header and "websocket" in Upgrade header
        is_websocket_request = (
            "upgrade" in connection_header.lower() and
            "websocket" in upgrade_header.lower()
        )

        if not is_websocket_request:
            # Only show warning once to avoid spam
            if not self._browser_warning_shown:
                self._browser_warning_shown = True
                host_display = self._get_local_ip() if self.config.host_ip == "0.0.0.0" else self.config.host_ip
                print(f"\n⚠️  Someone is trying to open port {self.config.websocket_port} in a browser.")
                print(f"   This port is for VR WebSocket connections only.")
                print(f"   The web UI is at: https://{host_display}:{self.config.https_port}\n")

        # Return None to let websockets library handle the request normally
        # (it will reject non-WebSocket requests with 426 Upgrade Required)
        return None

    async def stop(self):
        """Stop the WebSocket server."""
        self.is_running = False

        # Close all active client connections to unblock websocket_handler
        for client in list(self.clients):
            try:
                await client.close()
            except Exception:
                pass

        if self.server:
            self.server.close()
            await self.server.wait_closed()
            logger.info("VR WebSocket server stopped")
    
    async def websocket_handler(self, websocket, path=None):
        """Handle WebSocket connections from VR controllers."""
        client_address = websocket.remote_address
        logger.info(f"VR client connected: {client_address}")
        self.clients.add(websocket)
        if self.config.vr_shadow_enabled_by_default:
            self.skeleton_clients.add(websocket)
        await websocket.send(json.dumps({
            "type": "vr_ui_config",
            "shadow_enabled_by_default":
                self.config.vr_shadow_enabled_by_default,
            "controller_axes_enabled_by_default":
                self.config.vr_controller_axes_enabled_by_default,
        }))
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get('type') == 'skeleton_subscription':
                        if data.get('enabled'):
                            self.skeleton_clients.add(websocket)
                        else:
                            self.skeleton_clients.discard(websocket)
                        continue
                    await self.process_controller_data(data)
                except json.JSONDecodeError:
                    logger.warning(f"Received non-JSON message: {message}")
                except Exception as e:
                    logger.error(f"Error processing VR data: {e}")
        
        except websockets.exceptions.ConnectionClosedOK:
            logger.info(f"VR client {client_address} disconnected normally")
        except websockets.exceptions.ConnectionClosedError as e:
            logger.warning(f"VR client {client_address} disconnected with error: {e}")
        except Exception as e:
            logger.error(f"Unexpected error with VR client {client_address}: {e}")
        finally:
            self.clients.discard(websocket)
            self.skeleton_clients.discard(websocket)
            self._skeleton_send_tasks.pop(websocket, None)
            # Handle grip releases when client disconnects
            await self.handle_grip_release('left')
            await self.handle_grip_release('right')
            logger.info(f"VR client {client_address} cleanup complete")
    
    async def process_controller_data(self, data: Dict):
        """Process incoming VR controller data."""

        # Handle preset-pose commands (button combos held on the controllers):
        # 'home' -> zero pose, 'park' -> hang-down rest pose. Grips are force-
        # released first so position control stops before the arms move; if the
        # operator keeps squeezing, the next packet re-engages with a fresh
        # origin at the new pose (no jump).
        pose = data.get('pose')
        if pose in ('home', 'park'):
            logger.info(f"🕹️  VR combo requested preset pose: {pose}")
            await self.handle_grip_release('left')
            await self.handle_grip_release('right')
            goal = ControlGoal(
                arm='left',  # placeholder; the action applies to both arms
                metadata={"source": "vr_pose_combo", "preset_pose": pose},
            )
            await self.send_goal(goal)
            return
        
        # Handle new dual controller format
        if 'leftController' in data and 'rightController' in data:
            left_data = data['leftController']
            right_data = data['rightController']
            headset_yaw_deg = float(data.get('headsetYawDeg', 0.0) or 0.0)
            
            # Process left controller
            await self.process_trigger_state('left', left_data)
            if left_data.get('position') and left_data.get('gripActive', False):
                await self.process_single_controller('left', left_data, headset_yaw_deg)
            elif not left_data.get('gripActive', False) and self.left_controller.grip_active:
                await self.handle_grip_release('left')
            
            # Process right controller
            await self.process_trigger_state('right', right_data)
            if right_data.get('position') and right_data.get('gripActive', False):
                await self.process_single_controller('right', right_data, headset_yaw_deg)
            elif not right_data.get('gripActive', False) and self.right_controller.grip_active:
                await self.handle_grip_release('right')
                
            return
        
        # Handle legacy single controller format
        hand = data.get('hand')
        
        # Handle explicit release messages
        if data.get('gripReleased'):
            await self.handle_grip_release(hand)
            return
        
        if data.get('triggerReleased'):
            await self.handle_trigger_release(hand)
            return
            
        # Process single controller data
        if hand:
            await self.process_trigger_state(hand, data)
        if hand and data.get('position') and data.get('gripActive', False):
            headset_yaw_deg = float(data.get('headsetYawDeg', 0.0) or 0.0)
            await self.process_single_controller(hand, data, headset_yaw_deg)
    
    async def process_trigger_state(self, hand: str, data: Dict):
        """Apply trigger/gripper changes even when positional tracking is absent."""
        controller = self.left_controller if hand == 'left' else self.right_controller
        trigger_active = data.get('trigger', 0) > 0.5
        if trigger_active == controller.trigger_active:
            return
        controller.trigger_active = trigger_active
        await self.send_goal(ControlGoal(
            arm=hand,
            gripper_closed=trigger_active,
            metadata={"source": "vr_trigger"},
        ))
        logger.info(f"🤏 {hand.upper()} gripper "
                    f"{'CLOSED' if trigger_active else 'OPENED'}")

    async def process_single_controller(self, hand: str, data: Dict,
                                        headset_yaw_deg: float = 0.0):
        """Process tracked movement data for a single controller."""
        input_received_ns = time.monotonic_ns()
        trace_id = f"{hand}-{input_received_ns}"
        position = data.get('position', {})
        rotation = data.get('rotation', {})
        quaternion = data.get('quaternion', {})  # Get quaternion data directly
        grip_active = data.get('gripActive', False)
        trigger = data.get('trigger', 0)

        if self.telemetry:
            self.telemetry.record(
                "vr_controller",
                trace_id=trace_id,
                hand=hand,
                position=position,
                rotation=rotation,
                quaternion=quaternion,
                grip_active=grip_active,
                trigger=trigger,
                headset_yaw_deg=headset_yaw_deg,
                input_received_monotonic_ns=input_received_ns,
            )
        
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        # Handle grip button for arm movement control
        if grip_active:
            if not controller.grip_active:
                # Grip just activated - set origin and reset target position
                controller.grip_active = True
                controller.origin_position = position.copy()

                # Freeze the operator's facing direction for this grip: all
                # controller deltas are interpreted relative to where the
                # operator was looking when they squeezed the grip.
                controller.origin_headset_yaw_deg = headset_yaw_deg
                
                # Use quaternion data directly if available, otherwise fall back to Euler conversion
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    controller.origin_quaternion = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                else:
                    # Fallback to Euler angle conversion
                    controller.origin_quaternion = self.euler_to_quaternion(rotation) if rotation else None
                
                controller.accumulated_rotation_quat = controller.origin_quaternion
                
                logger.info(f"🔒 {hand.upper()} grip activated - controlling {hand} arm (target reset to current position)")
            
            # Compute target position
            if controller.origin_position:
                relative_delta = compute_relative_position(
                    position, 
                    controller.origin_position, 
                    self.config.vr_to_robot_scale,
                    headset_yaw_deg=controller.origin_headset_yaw_deg,
                    operator_to_robot_yaw_deg=self.config.operator_to_robot_yaw_deg,
                    mirror_lateral=self.config.mirror_lateral[hand],
                )
                
                # Full relative orientation quaternion in the same calibrated frame.
                relative_orientation_quat = None
                if (
                    self.config.vr_orientation_enabled[hand]
                    and controller.origin_quaternion is not None
                ):
                    # Update quaternion-based rotation tracking
                    if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                        # Use quaternion data directly
                        current_quat = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                        self.update_quaternion_rotation_direct(controller, current_quat)
                    else:
                        # Fallback to Euler angle conversion
                        self.update_quaternion_rotation(controller, rotation)
                    
                    controller_delta = (
                        R.from_quat(controller.accumulated_rotation_quat)
                        * R.from_quat(controller.origin_quaternion).inv()
                    )
                    controller_rotation_deg = np.rad2deg(
                        np.linalg.norm(controller_delta.as_rotvec())
                    )
                    if (
                        controller_rotation_deg
                        >= self.config.vr_orientation_deadband_deg
                    ):
                        relative_orientation_quat = compute_relative_orientation(
                            controller.accumulated_rotation_quat,
                            controller.origin_quaternion,
                            headset_yaw_deg=controller.origin_headset_yaw_deg,
                            operator_to_robot_yaw_deg=self.config.operator_to_robot_yaw_deg,
                            mirror_lateral=self.config.mirror_lateral[hand],
                            rotation_scale=self.config.vr_rotation_scale[hand],
                        )
                
                # Create position control goal
                # Note: We send relative position here, the control loop will handle
                # adding it to the robot's current position
                goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=relative_delta,  # Relative position delta
                    target_orientation_quat=relative_orientation_quat,
                    metadata={
                        "source": "vr_grip",
                        "relative_position": True,
                        "relative_orientation": True,
                        "origin_position": controller.origin_position.copy(),
                        "trace_id": trace_id,
                        "input_received_monotonic_ns": input_received_ns,
                    }
                )
                if self.telemetry:
                    self.telemetry.record(
                        "control_goal_created",
                        trace_id=trace_id,
                        arm=hand,
                        source="vr_grip",
                        relative_position=relative_delta,
                        relative_orientation_quat=relative_orientation_quat,
                        origin_headset_yaw_deg=controller.origin_headset_yaw_deg,
                    )
                await self.send_goal(goal)
    
    async def handle_grip_release(self, hand: str):
        """Handle grip release for a controller."""
        if hand == 'left':
            controller = self.left_controller
        elif hand == 'right':
            controller = self.right_controller
        else:
            return
        
        if controller.grip_active:
            controller.reset_grip()
            
            # Send idle goal to stop arm control
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                metadata={"source": "vr_grip_release"}
            )
            await self.send_goal(goal)
            
            logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")
    
    async def handle_trigger_release(self, hand: str):
        """Handle trigger release for a controller."""
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        if controller.trigger_active:
            controller.trigger_active = False
            
            # Releasing the trigger opens the gripper.
            goal = ControlGoal(
                arm=hand,
                gripper_closed=False,
                metadata={"source": "vr_trigger_release"}
            )
            await self.send_goal(goal)
            
            logger.info(f"🤏 {hand.upper()} gripper OPENED (trigger released)")
    
    def euler_to_quaternion(self, euler_deg: Dict[str, float]) -> np.ndarray:
        """Convert Euler angles in degrees to quaternion [x, y, z, w]."""
        euler_rad = [math.radians(euler_deg['x']), math.radians(euler_deg['y']), math.radians(euler_deg['z'])]
        rotation = R.from_euler('xyz', euler_rad)
        return rotation.as_quat()
    
    def update_quaternion_rotation(self, controller: VRControllerState, current_euler: dict):
        """Update quaternion-based rotation tracking."""
        if not current_euler:
            return
        
        # Convert current Euler to quaternion
        current_quat = self.euler_to_quaternion(current_euler)
        
        # Store current quaternion for accumulated rotation calculation
        controller.accumulated_rotation_quat = current_quat
    
    def update_quaternion_rotation_direct(self, controller: VRControllerState, current_quat: np.ndarray):
        """Update quaternion-based rotation tracking using quaternion data directly."""
        if current_quat is None:
            return
        
        # Store current quaternion for accumulated rotation calculation
        controller.accumulated_rotation_quat = current_quat
