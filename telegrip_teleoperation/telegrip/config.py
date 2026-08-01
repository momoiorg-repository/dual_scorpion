"""
Configuration module for the unified teleoperation system.
Loads configuration from config.yaml file with fallback to default values.
"""

import os
import copy
import yaml
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import numpy as np
from pathlib import Path
import logging
from .utils import get_absolute_path, get_project_root

logger = logging.getLogger(__name__)

# Portable defaults only. Set stable /dev/serial/by-id paths in config.yaml or
# pass --left-port/--right-port for the host running the arms.
_RIGHT_PORT = "/dev/ttyACM0"
_LEFT_PORT = "/dev/ttyACM1"

# Default configuration values (fallback if YAML file doesn't exist)
DEFAULT_CONFIG = {
    "network": {
        "https_port": 8443,
        "websocket_port": 8442,
        "host_ip": "0.0.0.0"
    },
    "ssl": {
        "certfile": "cert.pem",
        "keyfile": "key.pem"
    },
    "robot": {
        "left_arm": {
            "name": "Left Arm",
            "port": _LEFT_PORT,
            "enabled": True
        },
        "right_arm": {
            "name": "Right Arm",
            "port": _RIGHT_PORT,
            "enabled": True
        },
        # Slightly amplify controller translation so the compact robot
        # workspace can be covered without a full human-arm reach.
        "vr_to_robot_scale": 1.10,
        # Per-arm controller-to-TCP rotation gain.
        "vr_rotation_scale": {"left": 0.5, "right": 0.5},
        # Both controllers drive full-pose IK across all seven body joints.
        "vr_orientation_enabled": {"left": True, "right": True},
        # No body joints are fixed by default.
        "locked_joints": {"left": {}, "right": {}},
        # Keep ordinary hand translation position-only. Controller orientation
        # engages only after an intentional rotation beyond this deadband.
        "vr_orientation_deadband_deg": 8.0,
        "send_interval": 0.02,  # 50 Hz control/send rate (was 0.05 = 20 Hz)
        # Anti-windup: commanded joint angles may never run more than this far
        # ahead of the measured motor angles. If a motor stalls or lags, the
        # commanded state is pulled back so it stays anchored to the physical
        # arm instead of running away (Jul 17: left arm commanded drifted 76
        # deg from the stalled motors and stayed stuck there).
        "command_lag_limit_deg": 8.0,
        # Which start ("home") pose preset the arms glide to on engage and on
        # the A+B VR combo. See HOME_POSE_PRESETS; override per-launch with
        # the --start-pose CLI flag (./start.sh --backwards).
        "start_pose": "zero",
    },
    "control": {
        # Bench-calibrated yaw mapping "operator forward" (headset gaze at grip
        # time) onto the robot base frame. -90 deg = operator-forward is robot
        # -Y, the direction the arms extend outward (embodiment-style teleop:
        # the operator's forward matches the arms' reach direction).
        "operator_to_robot_yaw_deg": -90.0,
        # Mechanical left/right mirroring belongs in the two URDFs. Both
        # controllers map through the same proper robot-world rotation.
        "mirror_lateral": {"left": False, "right": False},
        "keyboard": {
            "enabled": True,
            "pos_step": 0.01,
            "angle_step": 5.0,
            "gripper_step": 10.0
        },
        "vr": {
            "enabled": True,
            "shadow_enabled_by_default": True,
            "controller_axes_enabled_by_default": True,
        },
        "pybullet": {
            "enabled": True
        }
    },
    "paths": {
        "urdf_left_path": "URDF/dual_scorpion/dual_scorpion_left.urdf",
        "urdf_right_path": "URDF/dual_scorpion/dual_scorpion_right.urdf"
    },
    "gripper": {
        "open_angle": 0.0,
        "closed_angle": 45.0,
    },
    "ik": {
        # At 50 Hz this caps commanded joint motion at 25 deg/s. The previous
        # 4 deg/tick allowed 200 deg/s command ramps, far beyond the unloaded
        # sweep behavior and able to drive the supply into voltage sag.
        "max_joint_step_deg": 0.5,
        # If the full-pose solution misses the target position by more than
        # this, re-solve position-only (position priority over orientation).
        "position_tolerance_m": 0.02,
        # Reachable-workspace sphere radius around each arm's shoulder
        # (joint0). Targets outside are projected onto the sphere. Absolute
        # max reach is 0.546 m; stop short of the straight-arm singularity.
        # Temporary low-power reach envelope: the right arm dropped while
        # holding near full extension on the current 5 A supply.
        "workspace_radius_m": 0.52,
        "workspace_soft_start_m": 0.48,
        # Max change (deg) of the ORIENTATION TARGET per solve, measured from
        # the current TCP orientation. Keeps orientation demands continuous so
        # a large accumulated error can never unwind the wrist in one burst.
        "orientation_slew_deg": 6.0
    },
    "telemetry": {
        "enabled": True,
        "log_dir": "logs/sessions",
        "actual_sample_hz": 20.0
    }
}

def load_config(config_path: str = "config.yaml") -> dict:
    """Load configuration from YAML file with fallback to defaults."""
    # The YAML merge is recursive; isolate nested defaults from mutation.
    config = copy.deepcopy(DEFAULT_CONFIG)
    
    # Try to load from project root first (package installation directory)
    package_config_path = get_absolute_path(config_path)
    
    # Check if config exists in package directory
    if package_config_path.exists():
        config_file_to_use = package_config_path
        logger.info(f"Loading config from package directory: {config_file_to_use}")
    # Fallback to current working directory (for user-provided configs)
    elif os.path.exists(config_path):
        config_file_to_use = Path(config_path)
        logger.info(f"Loading config from current directory: {config_file_to_use}")
    else:
        logger.info(f"Config file {config_path} not found in package directory ({package_config_path}) or current directory, using defaults")
        return config
    
    try:
        with open(config_file_to_use, 'r') as f:
            yaml_config = yaml.safe_load(f)
            if yaml_config:
                # Deep merge yaml config into default config
                _deep_merge(config, yaml_config)
    except Exception as e:
        logger.warning(f"Could not load config from {config_file_to_use}: {e}")
        logger.info("Using default configuration")
    
    return config

def save_config(config: dict, config_path: str = "config.yaml"):
    """Save configuration to YAML file in project root."""
    # Always save to project root directory
    abs_config_path = get_absolute_path(config_path)
    try:
        with open(abs_config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, indent=2)
        return True
    except Exception as e:
        logger.error(f"Error saving config to {abs_config_path}: {e}")
        return False

def _deep_merge(base: dict, update: dict):
    """Deep merge update dict into base dict."""
    for key, value in update.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value

# Load configuration
_config_data = load_config()

# Extract values for backward compatibility
HTTPS_PORT = _config_data["network"]["https_port"]
WEBSOCKET_PORT = _config_data["network"]["websocket_port"]
HOST_IP = _config_data["network"]["host_ip"]

CERTFILE = _config_data["ssl"]["certfile"]
KEYFILE = _config_data["ssl"]["keyfile"]

VR_TO_ROBOT_SCALE = _config_data["robot"]["vr_to_robot_scale"]

def _normalize_per_arm_float(value) -> Dict[str, float]:
    """Accept a legacy scalar or a per-arm {left, right} mapping."""
    if isinstance(value, dict):
        return {
            "left": float(value.get("left", 1.0)),
            "right": float(value.get("right", 1.0)),
        }
    scalar = float(value)
    return {"left": scalar, "right": scalar}


def _normalize_per_arm_bool(value) -> Dict[str, bool]:
    """Accept a legacy bool or a per-arm {left, right} mapping."""
    if isinstance(value, dict):
        return {
            "left": bool(value.get("left", True)),
            "right": bool(value.get("right", True)),
        }
    enabled = bool(value)
    return {"left": enabled, "right": enabled}


VR_ROTATION_SCALE = _normalize_per_arm_float(_config_data["robot"].get(
    "vr_rotation_scale", DEFAULT_CONFIG["robot"]["vr_rotation_scale"]))
VR_ORIENTATION_ENABLED = _normalize_per_arm_bool(_config_data["robot"].get(
    "vr_orientation_enabled",
    DEFAULT_CONFIG["robot"]["vr_orientation_enabled"]))
LOCKED_JOINTS = _config_data["robot"].get(
    "locked_joints", DEFAULT_CONFIG["robot"]["locked_joints"])
VR_ORIENTATION_DEADBAND_DEG = _config_data["robot"].get(
    "vr_orientation_deadband_deg",
    DEFAULT_CONFIG["robot"]["vr_orientation_deadband_deg"])
SEND_INTERVAL = _config_data["robot"]["send_interval"]

POS_STEP = _config_data["control"]["keyboard"]["pos_step"]
ANGLE_STEP = _config_data["control"]["keyboard"]["angle_step"]
GRIPPER_STEP = _config_data["control"]["keyboard"]["gripper_step"]

URDF_LEFT_PATH = _config_data["paths"].get("urdf_left_path", DEFAULT_CONFIG["paths"]["urdf_left_path"])
URDF_RIGHT_PATH = _config_data["paths"].get("urdf_right_path", DEFAULT_CONFIG["paths"]["urdf_right_path"])

GRIPPER_OPEN_ANGLE = _config_data["gripper"]["open_angle"]
GRIPPER_CLOSED_ANGLE = _config_data["gripper"]["closed_angle"]

# IK Configuration
IK_MAX_JOINT_STEP_DEG = _config_data["ik"]["max_joint_step_deg"]
IK_POSITION_TOLERANCE_M = _config_data["ik"].get(
    "position_tolerance_m", DEFAULT_CONFIG["ik"]["position_tolerance_m"])
IK_WORKSPACE_RADIUS_M = _config_data["ik"].get(
    "workspace_radius_m", DEFAULT_CONFIG["ik"]["workspace_radius_m"])
IK_WORKSPACE_SOFT_START_M = _config_data["ik"].get(
    "workspace_soft_start_m", DEFAULT_CONFIG["ik"]["workspace_soft_start_m"])
IK_ORIENTATION_SLEW_DEG = _config_data["ik"].get(
    "orientation_slew_deg", DEFAULT_CONFIG["ik"]["orientation_slew_deg"])
COMMAND_LAG_LIMIT_DEG = _config_data["robot"].get(
    "command_lag_limit_deg", DEFAULT_CONFIG["robot"]["command_lag_limit_deg"])

# Named start ("home") pose presets (joint0-joint6, deg, per arm). Joint2 is
# mirrored between the arms (negative left, positive right); joint0 shares the
# same axis on both, so its sign is the same.
HOME_POSE_PRESETS = {
    # URDF zero: arms straight out at the calibrated 90-degree elbow pose.
    "zero": {
        "left": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "right": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
    # Folded backwards: joint2 tucked 80 deg and joint0 bent back 20 deg, so
    # the arms rest behind the shoulder line instead of sticking out.
    "backwards": {
        "left": [-20.0, 0.0, -80.0, 0.0, 0.0, 0.0, 0.0],
        "right": [-20.0, 0.0, 80.0, 0.0, 0.0, 0.0, 0.0],
    },
}
START_POSE = str(_config_data["robot"].get(
    "start_pose", DEFAULT_CONFIG["robot"]["start_pose"]))

# Operator frame calibration (see kinematics.vr_to_robot_rotation)
OPERATOR_TO_ROBOT_YAW_DEG = _config_data["control"].get(
    "operator_to_robot_yaw_deg", DEFAULT_CONFIG["control"]["operator_to_robot_yaw_deg"])

def _normalize_mirror_lateral(value) -> Dict[str, bool]:
    """Accept either a single bool (legacy) or a per-arm {left, right} dict."""
    if isinstance(value, dict):
        return {"left": bool(value.get("left", False)),
                "right": bool(value.get("right", False))}
    return {"left": bool(value), "right": bool(value)}

MIRROR_LATERAL = _normalize_mirror_lateral(_config_data["control"].get(
    "mirror_lateral", DEFAULT_CONFIG["control"]["mirror_lateral"]))
VR_SHADOW_ENABLED_BY_DEFAULT = bool(_config_data["control"]["vr"].get(
    "shadow_enabled_by_default",
    DEFAULT_CONFIG["control"]["vr"]["shadow_enabled_by_default"]))
VR_CONTROLLER_AXES_ENABLED_BY_DEFAULT = bool(_config_data["control"]["vr"].get(
    "controller_axes_enabled_by_default",
    DEFAULT_CONFIG["control"]["vr"]["controller_axes_enabled_by_default"]))

# --- Joint Configuration (dual_scorpion: 7-DOF body + gripper = 8 motors) ---
JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)
NUM_IK_JOINTS = 7  # Every body joint participates in full position + orientation IK.
GRIPPER_INDEX = 7  # Gripper is the 8th motor (index 7)

# URDF joint name mapping (DualScorpion URDF uses TeleGrip motor names directly)
URDF_TO_INTERNAL_NAME_MAP = {
    "joint0": "joint0",
    "joint1": "joint1",
    "joint2": "joint2",
    "joint3": "joint3",
    "joint4": "joint4",
    "joint5": "joint5",
    "joint6": "joint6",
    "gripper": "gripper",
}

# --- PyBullet Configuration ---
END_EFFECTOR_LINK_NAME = "tcp_link"

# --- Keyboard Control ---
POS_STEP = 0.01  # meters
ANGLE_STEP = 5.0 # degrees
GRIPPER_STEP = 10.0 # degrees

# --- Device Ports ---
DEFAULT_FOLLOWER_PORTS = {
    "left": _config_data["robot"]["left_arm"]["port"],
    "right": _config_data["robot"]["right_arm"]["port"]
}

@dataclass
class TelegripConfig:
    """Main configuration class for the teleoperation system."""
    
    # Network settings
    https_port: int = HTTPS_PORT
    websocket_port: int = WEBSOCKET_PORT
    host_ip: str = HOST_IP
    
    # SSL settings
    certfile: str = CERTFILE
    keyfile: str = KEYFILE
    
    # Robot settings
    vr_to_robot_scale: float = VR_TO_ROBOT_SCALE
    # Per-arm rotation analogue of vr_to_robot_scale.
    vr_rotation_scale: Dict[str, float] = None
    vr_orientation_enabled: Dict[str, bool] = None
    locked_joints: Dict[str, Dict[str, float]] = None
    vr_orientation_deadband_deg: float = VR_ORIENTATION_DEADBAND_DEG
    send_interval: float = SEND_INTERVAL
    
    # Device ports
    follower_ports: Dict[str, str] = None
    
    # Control flags
    enable_pybullet: bool = True
    enable_pybullet_gui: bool = True
    enable_robot: bool = True
    enable_vr: bool = True
    vr_shadow_enabled_by_default: bool = VR_SHADOW_ENABLED_BY_DEFAULT
    vr_controller_axes_enabled_by_default: bool = VR_CONTROLLER_AXES_ENABLED_BY_DEFAULT
    enable_keyboard: bool = True
    autoconnect: bool = False
    # If set, auto-connect (engage + home) waits for this file to exist before
    # moving the arms. Used to gate homing on the Cloudflare teleop link being
    # live on the gist, so the arms only "go up" once teleop is actually ready.
    wait_ready_file: Optional[str] = None
    # Max seconds to wait for wait_ready_file before giving up (and NOT homing).
    wait_ready_timeout: float = 180.0
    # On engage, smoothly drive both arms to the calibrated zero pose so teleop
    # always starts from a known configuration (instead of holding the random
    # power-on pose). Set False to keep the previous hold-in-place behavior.
    home_on_engage: bool = True
    # On disengage/shutdown (e.g. Ctrl+C), smoothly fold the arms down to the
    # front-facing rest pose and let them settle BEFORE cutting torque, so the
    # arms don't flop/drop. Set False to release immediately.
    park_on_disengage: bool = True
    # Which joint indices to drive to zero when parking on disengage. Default is
    # only joint3 (the forearm flex) so the arms simply lower to the front-facing
    # rest without any shoulder rotation - every other joint is left in place.
    park_joints: tuple = (3,)
    log_level: str = "warning"
    
    # Paths (per-arm URDFs; both carry the shared stand mount offsets in base_link)
    urdf_left_path: str = URDF_LEFT_PATH
    urdf_right_path: str = URDF_RIGHT_PATH
    webapp_dir: str = "webapp"
    
    # IK settings. Full-pose IK seeds from the current commanded pose and
    # limits per-tick joint change; see IK_MAX_JOINT_STEP_DEG.
    ik_max_joint_step_deg: float = IK_MAX_JOINT_STEP_DEG
    ik_position_tolerance_m: float = IK_POSITION_TOLERANCE_M
    ik_workspace_radius_m: float = IK_WORKSPACE_RADIUS_M
    ik_workspace_soft_start_m: float = IK_WORKSPACE_SOFT_START_M
    ik_orientation_slew_deg: float = IK_ORIENTATION_SLEW_DEG

    # Anti-windup: max lead (deg) of commanded over measured joint angles.
    command_lag_limit_deg: float = COMMAND_LAG_LIMIT_DEG

    # Start-pose preset name (see HOME_POSE_PRESETS) used by engage homing
    # and the A+B VR combo. home_pose_deg is resolved from it unless set
    # explicitly to a per-arm dict of joint0-joint6 angles (deg).
    start_pose: str = START_POSE
    home_pose_deg: Optional[Dict[str, list]] = None

    # Yaw (deg) mapping operator-forward onto the robot base frame.
    operator_to_robot_yaw_deg: float = OPERATOR_TO_ROBOT_YAW_DEG
    # Per-arm mirror of the operator's left/right axis ({'left': bool,
    # 'right': bool}); the arms are mirrored mechanical builds so one side
    # may need the flip while the other does not.
    mirror_lateral: Dict[str, bool] = None
    
    # Gripper settings
    gripper_open_angle: float = GRIPPER_OPEN_ANGLE
    gripper_closed_angle: float = GRIPPER_CLOSED_ANGLE
    
    # Keyboard control
    pos_step: float = POS_STEP
    angle_step: float = ANGLE_STEP
    gripper_step: float = GRIPPER_STEP

    # Session telemetry. Actual motor positions are sampled less frequently
    # than commands to limit serial-bus overhead.
    telemetry_enabled: bool = _config_data["telemetry"]["enabled"]
    telemetry_log_dir: str = _config_data["telemetry"]["log_dir"]
    telemetry_actual_sample_hz: float = _config_data["telemetry"]["actual_sample_hz"]
    
    def __post_init__(self):
        # PyBullet is the kinematics engine, not merely the optional 3D viewer.
        # The arms cannot perform Cartesian position control without it. Keep
        # the engine mandatory; enable_pybullet_gui independently controls
        # whether it opens a window or runs in low-overhead DIRECT mode.
        self.enable_pybullet = True

        # Initialize follower_ports if not set
        if self.follower_ports is None:
            self.follower_ports = {
                "left": _config_data["robot"]["left_arm"]["port"],
                "right": _config_data["robot"]["right_arm"]["port"]
            }

        if self.mirror_lateral is None:
            self.mirror_lateral = dict(MIRROR_LATERAL)
        else:
            self.mirror_lateral = _normalize_mirror_lateral(self.mirror_lateral)

        if self.vr_rotation_scale is None:
            self.vr_rotation_scale = dict(VR_ROTATION_SCALE)
        else:
            self.vr_rotation_scale = _normalize_per_arm_float(
                self.vr_rotation_scale)

        if self.vr_orientation_enabled is None:
            self.vr_orientation_enabled = dict(VR_ORIENTATION_ENABLED)
        else:
            self.vr_orientation_enabled = _normalize_per_arm_bool(
                self.vr_orientation_enabled)

        if self.locked_joints is None:
            self.locked_joints = {
                arm: dict(LOCKED_JOINTS.get(arm, {}))
                for arm in ("left", "right")
            }

        if self.home_pose_deg is None:
            self.set_start_pose(self.start_pose)

    def set_start_pose(self, preset: str):
        """Resolve home_pose_deg from a named preset (fall back to 'zero')."""
        if preset not in HOME_POSE_PRESETS:
            logger.warning(f"Unknown start pose preset '{preset}', using 'zero'")
            preset = "zero"
        self.start_pose = preset
        self.home_pose_deg = {
            arm: list(HOME_POSE_PRESETS[preset][arm]) for arm in ("left", "right")
        }
        
        # Ensure ports are not None
        if self.follower_ports["left"] is None:
            self.follower_ports["left"] = _LEFT_PORT
        if self.follower_ports["right"] is None:
            self.follower_ports["right"] = _RIGHT_PORT
    
    @property
    def ssl_files_exist(self) -> bool:
        """Check if SSL certificate files exist."""
        cert_path = get_absolute_path(self.certfile)
        key_path = get_absolute_path(self.keyfile)
        return cert_path.exists() and key_path.exists()
    
    def ensure_ssl_certificates(self) -> bool:
        """Ensure SSL certificates exist, generating them if necessary."""
        from .utils import ensure_ssl_certificates
        return ensure_ssl_certificates(self.certfile, self.keyfile)
    
    @property
    def urdf_exists(self) -> bool:
        """Check if both per-arm URDF files exist."""
        return (get_absolute_path(self.urdf_left_path).exists()
                and get_absolute_path(self.urdf_right_path).exists())
    
    @property
    def webapp_exists(self) -> bool:
        """Check if webapp directory exists."""
        webapp_path = get_absolute_path(self.webapp_dir)
        return webapp_path.exists()
    
    def get_absolute_urdf_paths(self) -> Dict[str, str]:
        """Get absolute paths to the per-arm URDF files."""
        return {
            "left": str(get_absolute_path(self.urdf_left_path)),
            "right": str(get_absolute_path(self.urdf_right_path)),
        }
    
    def get_absolute_ssl_paths(self) -> tuple:
        """Get absolute paths to SSL certificate files."""
        cert_path = str(get_absolute_path(self.certfile))
        key_path = str(get_absolute_path(self.keyfile))
        return cert_path, key_path

def get_config_data():
    """Get the current configuration data."""
    return _config_data.copy()

def update_config_data(new_config: dict):
    """Update the global configuration data."""
    global _config_data
    _config_data = new_config
    
    # Save to file
    save_config(_config_data)

# Global configuration instance
config = TelegripConfig() 
