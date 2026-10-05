"""Synchronization and tracking safety independent from ROS/camera/hardware."""

import time

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import inverse, validate_pose, world_arm_landmarks, world_hands
from .receiver import MocopiReceiver
from .slam import SlamReceiver
from .sources import Replay, SampleBuffer, fake_pair


class Streams:
    def __init__(self, cfg, mode="live", replay=None):
        self.cfg = cfg
        self.mode = mode
        self.mocopi_buffer = SampleBuffer()
        self.pending = None
        self.latest_mocopi = None
        self.last_stamp = -np.inf
        self.start = time.monotonic()
        self.replay = Replay(replay) if replay else None
        self.mocopi = None
        self.slam = None
        try:
            if mode in ("live", "mocopi-only"):
                self.mocopi = MocopiReceiver(
                    cfg["mocopi"]["host"],
                    cfg["mocopi"]["port"],
                    cfg["mocopi"]["source_ip"],
                    cfg["mocopi"]["timestamp_offset_s"],
                )
            if mode == "live":
                self.slam = SlamReceiver(cfg["slam"]["pose_port"])
        except Exception:
            self.close()
            raise

    def poll(self):
        now = time.monotonic()
        if self.mode == "fake":
            return fake_pair(now - self.start, now)
        if self.replay:
            return self.replay.poll(now)
        for sample in self.mocopi.poll():
            self.mocopi_buffer.add(sample)
            self.latest_mocopi = sample
        if self.mode == "mocopi-only":
            sample = self.latest_mocopi
            if sample is not None and sample.timestamp > self.last_stamp:
                self.last_stamp = sample.timestamp
                return sample, None
            return None
        for sample in self.slam.poll():
            if sample.timestamp > self.last_stamp:
                self.pending = sample
        if self.pending:
            matched = self.mocopi_buffer.nearest(
                self.pending.timestamp, self.cfg["tracking"]["max_pair_dt_s"]
            )
            if matched:
                sample, self.pending = self.pending, None
                self.last_stamp = sample.timestamp
                return matched, sample
        return None

    def close(self):
        if self.mocopi:
            self.mocopi.close()
        if self.slam:
            self.slam.close()


class TrackingGate:
    def __init__(self, cfg, calibration=None):
        self.cfg = cfg
        self.calibration = calibration
        self.last_update = None
        self.last_head = None
        self.last_valid_target = None
        self.last_landmarks = {}
        self.tracking_valid = False
        self.reason = "waiting for synchronized streams"
        self.latched = False
        self.started = False

    def age(self, now):
        return np.inf if self.last_update is None else max(0.0, now - self.last_update)

    def accept(self, pair, now):
        try:
            m, s = pair
            stamp = m.timestamp if s is None else min(m.timestamp, s.timestamp)
            if not m.valid or (s is not None and not s.valid):
                raise ValueError("tracking invalid")
            if not -0.05 <= now - stamp <= self.cfg["timeout_s"]:
                raise ValueError("stale/future sample")
            if s is not None and abs(m.timestamp - s.timestamp) > self.cfg["max_pair_dt_s"]:
                raise ValueError("timestamp mismatch")
            if s is None:
                head = m.bones[10]
            else:
                if s.session != self.calibration.session or s.intrinsics_id != self.calibration.intrinsics_id:
                    raise ValueError("SLAM map/intrinsics changed: recalibrate")
                head = self.calibration.head(s.camera)
            if not self.started and s is not None:
                difference = inverse(m.bones[10]) @ head
                if (
                    np.linalg.norm(difference[:3, 3]) > self.cfg["startup_head_error_m"]
                    or Rotation.from_matrix(difference[:3, :3]).magnitude()
                    > self.cfg["startup_head_error_rad"]
                ):
                    raise ValueError("Saved map alignment inconsistent: recalibrate")
            if (
                self.last_head is not None
                and not self.latched
                and (
                    np.linalg.norm(head[:3, 3] - self.last_head[:3, 3]) > self.cfg["max_head_step_m"]
                    or Rotation.from_matrix(head[:3, :3] @ self.last_head[:3, :3].T).magnitude()
                    > self.cfg["max_head_step_rad"]
                )
            ):
                raise ValueError("Head pose discontinuity: HOLD")
            hands = world_hands(head, m)
            for target in hands.values():
                validate_pose(target)
            landmarks = world_arm_landmarks(head, m)
            for name, landmark in landmarks.items():
                validate_pose(landmark)
                if not self.latched and name in self.last_landmarks:
                    previous = self.last_landmarks[name]
                    if (
                        np.linalg.norm(landmark[:3, 3] - previous[:3, 3]) > self.cfg["max_hand_step_m"]
                        or Rotation.from_matrix(landmark[:3, :3] @ previous[:3, :3].T).magnitude()
                        > self.cfg["max_hand_step_rad"]
                    ):
                        raise ValueError(f"{name} pose discontinuity: HOLD")
            if not self.latched and self.last_landmarks.keys() - landmarks.keys():
                raise ValueError("Upper-arm bones disappeared: HOLD")
            if self.last_valid_target is not None and not self.latched:
                for side, target in hands.items():
                    previous = self.last_valid_target[side]
                    if (
                        np.linalg.norm(target[:3, 3] - previous[:3, 3]) > self.cfg["max_hand_step_m"]
                        or Rotation.from_matrix(target[:3, :3] @ previous[:3, :3].T).magnitude()
                        > self.cfg["max_hand_step_rad"]
                    ):
                        raise ValueError(f"{side} hand pose discontinuity: HOLD")
            self.last_head = head.copy()
            self.last_valid_target = hands
            self.last_landmarks = landmarks
            self.last_update = stamp
            self.started = True
            self.tracking_valid = not self.latched
            self.reason = "HOLD: press Enter to reanchor" if self.latched else "OK"
            return head, hands
        except (ValueError, KeyError, AttributeError) as exc:
            self.lose(str(exc))
            return None

    def lose(self, reason):
        self.tracking_valid = False
        self.reason = reason
        self.latched = self.started

    def tick(self, now):
        if self.age(now) > self.cfg["timeout_s"]:
            self.lose("tracking timeout: HOLD")
        return self.tracking_valid

    def rearm(self, now):
        if self.last_valid_target is None or self.age(now) > self.cfg["timeout_s"]:
            raise ValueError("Fresh synchronized tracking required before reanchor")
        self.latched = False
        self.tracking_valid = True
        self.reason = "OK"
