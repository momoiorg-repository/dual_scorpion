"""Sony 2026 official chunk format (see docs/MOCOPI_DESIGN.md for pinned sources).

Independent strict implementation, no Blender dependency. Standard 27-bone
hierarchy follows Sony's 3ds Max InitializeSkeletonChannels table. Non-root
translations use received skeleton offsets as in Sony Unity BufferAvatarPose.
"""

import logging
import socket
import struct
import time

from .geometry import MocopiSample, sony_pose

LOG = logging.getLogger(__name__)
PARENTS = (-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 7, 11, 12, 13, 7, 15, 16, 17, 0, 19, 20, 21, 0, 23, 24, 25)
REQUIRED = set(range(19))
CONTAINERS = {b"head", b"skdf", b"bons", b"bndt", b"fram", b"btrs", b"btdt", b"sndf"}


def chunks(data, depth=0):
    if depth > 8:
        raise ValueError("Excessively nested packet")
    offset = 0
    out = []
    while offset < len(data):
        if len(data) - offset < 8:
            raise ValueError("Truncated chunk header")
        size, tag = struct.unpack_from("<I4s", data, offset)
        end = offset + 8 + size
        if end > len(data):
            raise ValueError("Chunk exceeds datagram boundary")
        payload = data[offset + 8 : end]
        out.append((tag, chunks(payload, depth + 1) if tag in CONTAINERS else payload))
        offset = end
    return out


def flatten(tree, tag):
    for name, value in tree:
        if name == tag:
            yield value
        if isinstance(value, list):
            yield from flatten(value, tag)


def bone_transforms(tree, tag):
    out = {}
    for bone in flatten(tree, tag):
        fields = dict(bone)
        if b"bnid" not in fields or b"tran" not in fields:
            raise ValueError("Bone lacks bnid/tran")
        index = int.from_bytes(fields[b"bnid"], "little")
        raw = fields[b"tran"]
        if index not in range(27) or index in out or len(raw) != 28:
            raise ValueError("Invalid or duplicate bone")
        out[index] = sony_pose(struct.unpack("<7f", raw))
    return out


class MocopiDecoder:
    def __init__(self):
        self.rest = {}
        self.last_frame = None

    def decode(self, data, timestamp):
        tree = chunks(data)
        if not any(tag == b"head" for tag, _ in tree):
            raise ValueError("Not a Sony mocopi packet (missing head)")
        if list(flatten(tree, b"skdf")):
            rest = bone_transforms(tree, b"bndt")
            if not rest.keys() >= REQUIRED:
                raise ValueError("Skeleton missing torso/head/arm bones")
            # A newly received rest skeleton starts a new stream epoch.
            self.rest = rest
            self.last_frame = None
            return None
        if not list(flatten(tree, b"fram")):
            return None
        local = bone_transforms(tree, b"btdt")
        if not self.rest:
            raise ValueError("Waiting for skdf: restart SEND in mocopi app")
        if not local.keys() >= REQUIRED:
            raise ValueError("Incomplete motion frame")
        numbers = list(flatten(tree, b"fnum"))
        number = int.from_bytes(numbers[0], "little") if numbers else -1
        if number >= 0 and self.last_frame is not None and number <= self.last_frame:
            return None  # duplicates/reordered UDP; restart requires skdf
        global_bones = {}
        for index in sorted(local):
            t = local[index].copy()
            if index != 0:
                t[:3, 3] = self.rest[index][:3, 3]
            parent = PARENTS[index]
            if parent >= 0 and parent not in global_bones:
                raise ValueError("Incomplete bone hierarchy")
            global_bones[index] = global_bones[parent] @ t if parent >= 0 else t
        self.last_frame = number if number >= 0 else self.last_frame
        return MocopiSample(timestamp, global_bones, number)


class MocopiReceiver:
    def __init__(self, host="0.0.0.0", port=12351, source_ip=None, offset_s=0.0):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, port))
        self.socket.setblocking(False)
        self.decoder = MocopiDecoder()
        self.source_ip = source_ip
        self.offset_s = offset_s
        self.bad_packets = 0
        self.frames = 0
        self.started = time.monotonic()
        self.last_warning = 0.0

    def poll(self):
        out = []
        for _ in range(200):
            try:
                data, sender = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            if self.source_ip and sender[0] != self.source_ip:
                continue
            now = time.monotonic()
            try:
                sample = self.decoder.decode(data, now + self.offset_s)
                if sample:
                    out.append(sample)
                    self.frames += 1
            except (ValueError, KeyError, struct.error) as exc:
                self.bad_packets += 1
                if now - self.last_warning > 1.0:
                    LOG.warning("mocopi packet rejected: %s", exc)
                    self.last_warning = now
        return out

    @property
    def fps(self):
        return self.frames / max(time.monotonic() - self.started, 0.001)

    def close(self):
        self.socket.close()
