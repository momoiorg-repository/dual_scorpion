"""Record timestamped camera frames and mocopi poses without a robot or IK."""

import json
import threading
import time
from pathlib import Path

import numpy as np

from ..camera import UvcCamera
from ..geometry import validate_pose
from ..receiver import MocopiReceiver
from ..sources import fake_pair

REQUIRED_BONES = (0, 10, 12, 13, 14, 16, 17, 18)
SCHEMA = "dual_scorpion.mocopi.capture.v1"


def _json_line(file, row):
    file.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
    file.flush()


class FakeCamera:
    """Timed and visibly synthetic input for exercising the complete pipeline."""

    def __init__(self, config):
        self.width, self.height = int(config["width"]), int(config["height"])
        self.period = 1 / float(config["fps"])
        self.next_frame = time.monotonic()
        self.index = 0

    def read(self):
        import cv2

        time.sleep(max(0, self.next_frame - time.monotonic()))
        stamp = time.monotonic()
        self.next_frame = stamp + self.period
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[:, :, 0] = (self.index * 13) % 256
        cv2.putText(
            image,
            f"FAKE {self.index}",
            (10, min(45, self.height - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )
        self.index += 1
        return stamp, image

    def close(self):
        pass


class FakeMocopi:
    def __init__(self, fps):
        self.period = 1 / float(fps)
        self.start = self.next_sample = time.monotonic()
        self.index = 0

    def poll(self):
        stamp = time.monotonic()
        if stamp < self.next_sample:
            return []
        self.next_sample = stamp + self.period
        sample, _ = fake_pair(stamp - self.start, stamp)
        sample.frame_id = self.index
        self.index += 1
        return [sample]

    def close(self):
        pass


def capture_session(config, output, duration, task, *, fake=False, camera=None, receiver=None):
    """Write one immutable raw session. Each stream keeps its acquisition times.

    The camera timestamp is the PC's monotonic time just after its blocking
    read; mocopi timestamps are PC receive times, adjusted by config. These
    are not exposure times or measured sensor timestamps.
    """
    if not np.isfinite(duration) or duration <= 0 or not task.strip():
        raise ValueError("duration must be positive and task must not be empty")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "images").mkdir()
    config_camera = config["camera"]
    origin = time.monotonic()
    metadata = {
        "schema": SCHEMA,
        "status": "recording",
        "source": "fake" if fake else "real",
        "task": task.strip(),
        "camera": {
            "device": str(config_camera["device"]),
            "width": config_camera["width"],
            "height": config_camera["height"],
            "nominal_fps": config_camera["fps"],
            "timestamp": "pc_monotonic_after_camera_read",
            "image_format": "jpeg_bgr",
        },
        "mocopi": {
            "mode_declared": config["mocopi"]["tracking_mode"],
            "timestamp": "pc_monotonic_udp_receive_plus_offset",
            "timestamp_offset_s": config["mocopi"]["timestamp_offset_s"],
            "coordinate_axes": "x_forward_y_left_z_up",
            "position_unit": "metre",
            "quaternion_order": "xyzw",
            "bone_space": "mocopi_global",
        },
        "head_pose_source": "mocopi",
        "gripper_recorded": False,
        "robot_ik_at_capture": False,
    }
    metadata_path = output / "session.json"

    def save_metadata():
        temp = metadata_path.with_suffix(".json.tmp")
        temp.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temp.replace(metadata_path)

    save_metadata()
    stop = threading.Event()
    ready = threading.Event()
    errors = []
    counts = {"camera_frames": 0, "mocopi_frames": 0, "mocopi_discarded": 0}
    worker = None
    try:
        if receiver is None:
            receiver = (
                FakeMocopi(config_camera["fps"])
                if fake
                else MocopiReceiver(
                    config["mocopi"]["host"],
                    config["mocopi"]["port"],
                    config["mocopi"]["source_ip"],
                    config["mocopi"]["timestamp_offset_s"],
                )
            )
        if camera is None:
            camera = FakeCamera(config_camera) if fake else UvcCamera(config_camera)

        def receive():
            previous = -np.inf
            try:
                with (output / "mocopi.jsonl").open("w", encoding="utf-8") as file:
                    while not stop.is_set():
                        for sample in receiver.poll():
                            if (
                                not sample.valid
                                or not np.isfinite(sample.timestamp)
                                or sample.timestamp <= previous
                            ):
                                counts["mocopi_discarded"] += 1
                                continue
                            if any(bone not in sample.bones for bone in REQUIRED_BONES):
                                counts["mocopi_discarded"] += 1
                                continue
                            bones = {
                                str(k): validate_pose(v).tolist() for k, v in sample.bones.items() if k <= 18
                            }
                            _json_line(
                                file,
                                {
                                    "timestamp_s": sample.timestamp - origin,
                                    "frame_id": sample.frame_id,
                                    "bones": bones,
                                },
                            )
                            previous = sample.timestamp
                            counts["mocopi_frames"] += 1
                            ready.set()
                        stop.wait(0.003)
            except Exception as exc:
                errors.append(exc)
                ready.set()
                stop.set()

        worker = threading.Thread(target=receive, name="mocopi-capture", daemon=True)
        worker.start()
        if not fake:
            print("mocopiのSENDを一度停止→再開してください。受信後に撮影を開始します。", flush=True)
        if not ready.wait(15):
            raise RuntimeError("No mocopi frames. Restart SEND and check UDP port 12351")
        if errors:
            raise RuntimeError(f"mocopi receiver failed: {errors[0]}")
        print(f"Recording {duration:g} s to {output} (head camera + mocopi, no robot/gripper)", flush=True)
        import cv2

        start = time.monotonic()
        previous_camera = -np.inf
        with (output / "camera.jsonl").open("w", encoding="utf-8") as file:
            while time.monotonic() - start < duration and not stop.is_set():
                stamp, image = camera.read()
                if image.shape != (int(config_camera["height"]), int(config_camera["width"]), 3):
                    raise ValueError(f"Unexpected camera image shape: {image.shape}")
                if image.dtype != np.uint8 or not np.isfinite(stamp):
                    raise ValueError("Camera must provide a finite timestamp and uint8 BGR image")
                if stamp <= previous_camera:
                    raise ValueError("Camera timestamps must increase")
                name = f"images/{counts['camera_frames']:08d}.jpg"
                if not cv2.imwrite(str(output / name), image, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                    raise OSError(f"Failed to save camera frame {name}")
                _json_line(file, {"timestamp_s": stamp - origin, "file": name})
                previous_camera = stamp
                counts["camera_frames"] += 1
        # Retain a brief tail so the last camera frame can pair with a later UDP frame.
        stop.wait(min(0.1, config["tracking"]["max_pair_dt_s"]))
        if errors:
            raise RuntimeError(f"mocopi receiver failed: {errors[0]}")
        if not counts["camera_frames"] or not counts["mocopi_frames"]:
            raise RuntimeError("No camera or mocopi frames recorded")
        metadata["status"] = "complete"
    except BaseException:
        metadata["status"] = "aborted"
        raise
    finally:
        stop.set()
        if worker:
            worker.join(timeout=5)
            if worker.is_alive():
                metadata["status"] = "aborted"
                errors.append(RuntimeError("mocopi receiver did not stop"))
        if receiver:
            receiver.close()
        if camera:
            camera.close()
        metadata.update(counts)
        save_metadata()
    if errors:
        raise RuntimeError(str(errors[0]))
    return metadata
