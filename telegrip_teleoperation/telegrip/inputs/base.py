"""
Base classes and data structures for input providers.
"""

import asyncio
import logging
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Literal, Dict, Any
from enum import Enum

logger = logging.getLogger(__name__)

class ControlMode(Enum):
    """Control modes for the teleoperation system."""
    POSITION_CONTROL = "position"
    IDLE = "idle"

@dataclass
class ControlGoal:
    """High-level control goal message sent from input providers."""
    arm: Literal["left", "right"]
    mode: Optional[ControlMode] = None            # Control mode (None = no mode change)
    target_position: Optional[np.ndarray] = None  # 3D position in robot coordinates
    target_orientation_quat: Optional[np.ndarray] = None  # Relative/absolute [x, y, z, w]
    # Legacy keyboard fields. The control loop converts these two rotations
    # into a relative quaternion before invoking full-pose IK.
    wrist_roll_deg: Optional[float] = None
    wrist_flex_deg: Optional[float] = None
    gripper_closed: Optional[bool] = None         # Gripper state (None = no change)
    
    # Additional data for debugging/monitoring
    metadata: Optional[Dict[str, Any]] = None

class BaseInputProvider(ABC):
    """Abstract base class for input providers."""
    
    def __init__(self, command_queue: asyncio.Queue):
        self.command_queue = command_queue
        self.is_running = False
    
    @abstractmethod
    async def start(self):
        """Start the input provider."""
        pass
    
    @abstractmethod
    async def stop(self):
        """Stop the input provider."""
        pass
    
    async def send_goal(self, goal: ControlGoal):
        """Send a control goal to the command queue."""
        try:
            await self.command_queue.put(goal)
        except Exception as e:
            logger.warning("Could not enqueue control goal for %s: %s", goal.arm, e)