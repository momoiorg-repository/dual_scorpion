"""Source contracts, bounded timestamp matching, replay and deterministic fake motion."""

import json
from collections import deque
from pathlib import Path
from typing import Protocol

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import MocopiSample, SlamSample, inverse, pose, validate_pose


class CameraSource(Protocol):
    def read(self): ...  # (monotonic acquisition timestamp, BGR image)
    def close(self): ...


class GripperEncoderSource(Protocol):
    def read(self) -> tuple[float, float]: ...  # timestamp, aperture [0,1]
    def close(self): ...


class SampleBuffer:
    def __init__(self, maxlen=300):
        self.samples = deque(maxlen=maxlen)

    def add(self, sample):
        if not np.isfinite(sample.timestamp):
            raise ValueError("Nonfinite timestamp")
        if self.samples and sample.timestamp <= self.samples[-1].timestamp:
            return False
        self.samples.append(sample)
        return True

    def nearest(self, timestamp, tolerance):
        if not self.samples:
            return None
        sample = min(self.samples, key=lambda item: abs(item.timestamp - timestamp))
        return sample if sample.valid and abs(sample.timestamp - timestamp) <= tolerance else None


def fake_pair(t, timestamp):
    head = pose(
        [0.10 * np.sin(t * 0.8), 0.12 * np.sin(t * 0.6), 1.5 + 0.09 * np.sin(t * 1.1)],
        Rotation.from_rotvec(
            [0.25 * np.sin(t * 0.9), 0.3 * np.sin(t * 0.7), 0.2 * np.sin(t * 0.5)]
        ).as_quat(),
    )
    bones = {0: pose(), 10: head}
    for side, bone, sign in (("left", 14, 1), ("right", 18, -1)):
        # Different trajectories make swapped-arm wiring visible.
        wave = 0.025 * np.sin(t * 1.3) if side == "left" else 0.02 * np.sin(t * 0.9)
        relative = pose([0.25, sign * 0.28, -0.35 + wave])
        bones[bone] = head @ relative
        # Synthetic landmarks for diagnostics, not Sony sensor/body constants.
        shoulder, elbow = (12, 13) if side == "left" else (16, 17)
        bones[shoulder] = head @ pose([0.0, sign * 0.18, -0.2])
        bones[elbow] = head @ pose([0.12, sign * 0.3, -0.3 + wave / 2])
    extrinsic = pose([0.035, -0.015, 0.06], Rotation.from_rotvec([0.2, -0.1, 0.15]).as_quat())
    camera_metric = head @ inverse(extrinsic)
    camera = camera_metric.copy()
    camera[:3, 3] /= 1.7
    return MocopiSample(timestamp, bones), SlamSample(timestamp, camera, session="fake", intrinsics_id="fake")


class Replay:
    """JSONL pairs with relative timestamps, rebased onto a new monotonic epoch."""

    def __init__(self, path):
        self.rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
        if not self.rows:
            raise ValueError("Empty replay")
        times = [row["timestamp"] for row in self.rows]
        if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
            raise ValueError("Replay timestamps must increase")
        self.index = 0
        self.start = None
        self.origin = times[0]

    def poll(self, now):
        if self.start is None:
            self.start = now
        result = None
        while self.index < len(self.rows):
            row = self.rows[self.index]
            stamp = self.start + row["timestamp"] - self.origin
            if stamp > now:
                break
            mocopi = MocopiSample(
                stamp,
                {int(k): validate_pose(v) for k, v in row["bones"].items()},
                valid=row.get("valid", True),
            )
            slam = SlamSample(
                stamp,
                validate_pose(row["camera"]),
                valid=row.get("valid", True),
                session=row.get("session", "replay"),
                intrinsics_id=row.get("intrinsics_id", "replay"),
            )
            result = mocopi, slam
            self.index += 1
        return result


def record_pair(file, origin, mocopi, slam):
    file.write(
        json.dumps(
            {
                "timestamp": slam.timestamp - origin,
                "bones": {str(k): v.tolist() for k, v in mocopi.bones.items()},
                "camera": slam.camera.tolist(),
                "valid": mocopi.valid and slam.valid,
                "session": slam.session,
                "intrinsics_id": slam.intrinsics_id,
            }
        )
        + "\n"
    )
    file.flush()
