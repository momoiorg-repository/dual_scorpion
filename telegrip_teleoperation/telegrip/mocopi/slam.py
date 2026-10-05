"""Bounded localhost pose transport from external SLAM adapters."""

import json
import socket
import time

import numpy as np

from .geometry import SlamSample, pose


class SlamReceiver:
    def __init__(self, port=12352):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", port))
        self.socket.setblocking(False)
        self.invalid = 0

    def poll(self):
        samples = []
        for _ in range(200):
            try:
                data, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                row = json.loads(data)
                stamp = float(row["timestamp"])
                if not np.isfinite(stamp) or stamp > time.monotonic() + 0.05:
                    raise ValueError("Invalid monotonic timestamp")
                if row.get("schema") != 1 or row.get("convention") != "T_S_C_ros":
                    raise ValueError("Unsupported SLAM pose convention")
                samples.append(
                    SlamSample(
                        stamp,
                        pose(row["position"], row["quaternion"]),
                        bool(row["valid"]),
                        str(row["session"]),
                        str(row["intrinsics_id"]),
                    )
                )
            except (ValueError, KeyError, TypeError):
                self.invalid += 1
        return samples

    def close(self):
        self.socket.close()
