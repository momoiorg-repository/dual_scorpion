"""Data-quality metrics from recorded measurements; no ground-truth claims."""

import numpy as np

from .session import Session, aligned_segments, nearest_index


def _summary(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {"mean": None, "p95": None, "max": None}
    return {
        "mean": float(np.mean(values)),
        "p95": float(np.quantile(values, 0.95)),
        "max": float(np.max(values)),
    }


def session_metrics(path, fps=30, max_sync_s=0.08, workspace=None):
    session = Session(path)
    segments, grid = aligned_segments(session, fps, max_sync_s)
    offsets = np.array(
        [
            session.motion_times[nearest_index(session.motion_times, stamp)] - stamp
            for stamp in session.camera_times
        ]
    )
    report = {
        "schema": "dual_scorpion.mocopi.quality.v1",
        "source": session.meta["source"],
        "head_pose_source": session.meta["head_pose_source"],
        "camera_frames": len(session.camera),
        "mocopi_frames": len(session.motion),
        "camera_fps": float(
            (len(session.camera) - 1) / max(session.camera_times[-1] - session.camera_times[0], 1e-9)
        ),
        "mocopi_fps": float(
            (len(session.motion) - 1) / max(session.motion_times[-1] - session.motion_times[0], 1e-9)
        ),
        "camera_to_mocopi_abs_offset_ms": _summary(np.abs(offsets) * 1000),
        "camera_frames_outside_tolerance": int(np.count_nonzero(np.abs(offsets) > max_sync_s)),
        "max_sync_s": max_sync_s,
        "grid_fps": fps,
        **grid,
        "usable_segments": sum(len(segment) >= 2 for segment in segments),
        "usable_dataset_frames": sum(max(0, len(segment) - 1) for segment in segments),
        "mocopi_discarded_at_capture": session.meta.get("mocopi_discarded", 0),
        "timestamp_limit": "PC receive/read times; exposure, wireless latency and tracking accuracy not measured",
        "hands": {},
    }
    for side, bone in (("left", 14), ("right", 18)):
        positions = np.array([row["bones"][str(bone)][:3] for row in session.motion], dtype=float)[:, :, 3]
        distance = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        speed = distance / np.diff(session.motion_times)
        hand = {
            "bounds_min_m": positions.min(axis=0).tolist(),
            "bounds_max_m": positions.max(axis=0).tolist(),
            "path_length_m": float(distance.sum()),
            "speed_m_s": _summary(speed),
        }
        if workspace is not None:
            box = workspace[side]
            lower, upper = np.asarray(box["min_m"], dtype=float), np.asarray(box["max_m"], dtype=float)
            if (
                lower.shape != (3,)
                or upper.shape != (3,)
                or not np.isfinite([lower, upper]).all()
                or np.any(lower >= upper)
            ):
                raise ValueError(f"Invalid {side} workspace box")
            outside = np.any((positions < lower) | (positions > upper), axis=1)
            hand["outside_workspace_frames"] = int(outside.sum())
            hand["outside_workspace_fraction"] = float(outside.mean())
        report["hands"][side] = hand
    return report
