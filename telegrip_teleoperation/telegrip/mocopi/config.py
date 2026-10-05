"""YAML style shared with the existing Telegrip add-on; all paths relative to config."""

from pathlib import Path

import numpy as np
import yaml


def camera_device(value):
    """Keep UVC paths as strings; pass numeric OpenCV indices as integers."""
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        raise ValueError("Camera must be a non-negative index or a device path")
    if isinstance(value, str):
        value = value.strip()
        if not value:
            raise ValueError("Camera device cannot be empty")
        if value.lstrip("+-").isdigit():
            value = int(value)
    if isinstance(value, int) and value < 0:
        raise ValueError("Camera index must be non-negative")
    return value


def load_config(path, *, camera=None):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    cfg["camera"]["device"] = camera_device(cfg["camera"]["device"] if camera is None else camera)
    cfg["mocopi"].setdefault("tracking_mode", "standard")
    if cfg["mocopi"]["tracking_mode"] not in ("standard", "upper_body"):
        raise ValueError("mocopi.tracking_mode must be standard or upper_body")
    cfg["retarget"].setdefault("alignment_mode", "world")
    if cfg["retarget"]["alignment_mode"] not in ("body", "world"):
        raise ValueError("retarget.alignment_mode must be body or world")
    cfg["retarget"].setdefault("arm_posture", "upper-arm")
    cfg["retarget"].setdefault("arm_posture_gain", 0.8)
    if cfg["retarget"]["arm_posture"] not in ("upper-arm", "hand-only"):
        raise ValueError("retarget.arm_posture must be upper-arm or hand-only")
    gain = cfg["retarget"]["arm_posture_gain"]
    if not np.isfinite(gain) or not 0 < gain <= 1:
        raise ValueError("arm_posture_gain must be in (0, 1]")
    cfg["robot"].setdefault("start_pose", "config")
    if cfg["robot"]["start_pose"] not in ("work", "backwards", "config"):
        raise ValueError("robot.start_pose must be work, backwards or config")
    for key in ("intrinsics", "stella_config", "head_calibration"):
        cfg["slam"][key] = str((path.parent / cfg["slam"][key]).resolve())
    cfg["robot"]["urdf"] = {
        side: str((path.parent / value).resolve()) for side, value in cfg["robot"]["urdf"].items()
    }
    if cfg["robot"].get("visual_urdf"):
        cfg["robot"]["visual_urdf"] = str((path.parent / cfg["robot"]["visual_urdf"]).resolve())
    limits = [
        (
            cfg["tracking"],
            [
                "max_pair_dt_s",
                "timeout_s",
                "max_head_step_m",
                "max_head_step_rad",
                "max_hand_step_m",
                "max_hand_step_rad",
                "hz",
            ],
        ),
        (
            cfg["calibration"],
            ["duration_s", "min_samples", "max_rmse_m", "max_rotation_deg", "max_condition"],
        ),
        (cfg["robot"], ["max_joint_step_deg", "position_tolerance_m"]),
        (cfg["camera"], ["width", "height", "fps"]),
    ]
    for group, keys in limits:
        for key in keys:
            if not np.isfinite(group[key]) or group[key] <= 0:
                raise ValueError(f"{key} must be positive and finite")
    if cfg["tracking"]["max_pair_dt_s"] >= cfg["tracking"]["timeout_s"]:
        raise ValueError("Synchronization tolerance must be shorter than tracking timeout")
    return cfg
