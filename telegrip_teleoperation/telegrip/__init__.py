"""
TeleGrip - dual_scorpion teleoperation system.
"""

from .core.robot_interface import RobotInterface
from .core.visualizer import PyBulletVisualizer as Visualizer
from .control_loop import ControlLoop
from .config import TelegripConfig, load_config

__version__ = "0.2.0"
__all__ = ["RobotInterface", "Visualizer", "ControlLoop", "TelegripConfig", "load_config"] 