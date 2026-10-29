"""Bounded local image stream from the SLAM camera bridge to the recorder."""

import queue
import socket
import struct
import threading
import time
from contextlib import suppress

import numpy as np

HEADER = struct.Struct("<dQI")  # PC acquisition time, bridge frame id, JPEG payload bytes
MAX_IMAGE_BYTES = 8 * 1024 * 1024


class HeadFrameServer:
    """Single local subscriber; a slow recorder drops frames, not the SLAM loop."""

    def __init__(self, port):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", port))
        self.listener.listen(1)
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.connected = threading.Event()
        self.stop = threading.Event()
        self.queue = queue.Queue(maxsize=2)
        self.client = None
        self.dropped = 0
        self.frame_id = 0
        self.thread = threading.Thread(target=self._serve, name="head-frame-server", daemon=True)
        self.thread.start()

    def publish(self, timestamp, image):
        frame_id = self.frame_id
        self.frame_id += 1
        if not self.connected.is_set():
            return
        try:
            self.queue.put_nowait((timestamp, frame_id, image))
        except queue.Full:
            with suppress(queue.Empty):
                self.queue.get_nowait()
            self.dropped += 1
            with suppress(queue.Full):
                self.queue.put_nowait((timestamp, frame_id, image))

    def _serve(self):
        import cv2

        while not self.stop.is_set():
            if self.client is None:
                try:
                    client, _ = self.listener.accept()
                except (TimeoutError, OSError):
                    continue
                client.settimeout(0.5)
                self.client = client
                while not self.queue.empty():
                    with suppress(queue.Empty):
                        self.queue.get_nowait()
                self.connected.set()
            try:
                stamp, frame_id, image = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if not ok or encoded.nbytes > MAX_IMAGE_BYTES or not np.isfinite(stamp):
                    raise ValueError("Invalid camera frame")
                self.client.sendall(HEADER.pack(stamp, frame_id, encoded.nbytes) + encoded.tobytes())
            except (OSError, ValueError):
                self.connected.clear()
                with suppress(OSError):
                    self.client.close()
                self.client = None
        self.connected.clear()
        if self.client:
            with suppress(OSError):
                self.client.close()

    def close(self):
        self.stop.set()
        with suppress(OSError):
            self.listener.close()
        client = self.client
        if client:
            with suppress(OSError):
                client.shutdown(socket.SHUT_RDWR)
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError("Head frame stream did not stop")


class HeadFrameClient:
    def __init__(self, port):
        self.socket = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.socket.settimeout(3)
        self.last_jpeg = None
        self.last_frame_id = None

    def _read(self, size):
        chunks = bytearray()
        while len(chunks) < size:
            chunk = self.socket.recv(size - len(chunks))
            if not chunk:
                raise RuntimeError("Head camera stream disconnected")
            chunks.extend(chunk)
        return bytes(chunks)

    def read(self):
        import cv2

        stamp, frame_id, size = HEADER.unpack(self._read(HEADER.size))
        if not np.isfinite(stamp) or stamp > time.monotonic() + 0.05 or not 0 < size <= MAX_IMAGE_BYTES:
            raise ValueError("Invalid head camera stream header")
        self.last_jpeg = self._read(size)
        self.last_frame_id = frame_id
        image = cv2.imdecode(np.frombuffer(self.last_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Invalid JPEG from head camera stream")
        return stamp, image

    def close(self):
        self.socket.close()
