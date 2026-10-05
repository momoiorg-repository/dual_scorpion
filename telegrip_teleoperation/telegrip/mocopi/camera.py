"""C270/UVC adapter around LeRobot's existing OpenCV camera abstraction."""

import hashlib
import json
import subprocess
import time
from pathlib import Path

import numpy as np
import yaml

from .calibration import save_yaml


def camera_list():
    from lerobot.cameras.opencv.camera_opencv import OpenCVCamera

    found = OpenCVCamera.find_cameras()
    for device in sorted(Path("/dev").glob("video*")):
        if not any(str(item["id"]) == str(device) for item in found):
            found.append({"id": str(device), "error": "cannot open (permissions/busy/non-capture device)"})
    for item in found:
        name_file = Path("/sys/class/video4linux") / Path(str(item["id"])).name / "name"
        if name_file.exists():
            item["name"] = name_file.read_text().strip()
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", str(item["id"]), "--list-formats-ext"],
                capture_output=True,
                text=True,
                timeout=3,
            )
            item["supported_modes"] = result.stdout.strip() or result.stderr.strip()
        except (FileNotFoundError, subprocess.TimeoutExpired):
            item["supported_modes"] = "Install v4l-utils for supported resolution/FPS enumeration"
    return found


class UvcCamera:
    def __init__(self, config):
        from lerobot.cameras.opencv.camera_opencv import OpenCVCamera
        from lerobot.cameras.opencv.configuration_opencv import ColorMode, OpenCVCameraConfig

        device = config["device"]
        self.camera = OpenCVCamera(
            OpenCVCameraConfig(
                index_or_path=Path(device) if isinstance(device, str) else device,
                width=config["width"],
                height=config["height"],
                fps=config["fps"],
                fourcc=config.get("fourcc"),
                color_mode=ColorMode.BGR,
            )
        )
        try:
            self.camera.connect()
        except Exception as exc:
            if self.camera.is_connected:
                self.camera.disconnect()
            available = camera_list()
            requested = f"/dev/video{device}" if isinstance(device, int) else str(Path(device).resolve())
            selected = next((item for item in available if str(item["id"]) == requested), {})
            alternatives = [
                item["id"]
                for item in available
                if selected.get("name") and item.get("name") == selected["name"] and "error" not in item
            ]
            hint = ""
            if alternatives:
                hint = (
                    f"\n同じカメラの映像取得用デバイス: {', '.join(map(str, alternatives))}"
                    f"\n--camera {alternatives[0]} を指定してください。"
                )
            raise RuntimeError(
                f"Cannot open {device}: {exc}{hint}\nAvailable cameras:\n" + json.dumps(available, indent=2)
            ) from exc

    def read(self):
        # Software acquisition timestamp, not exposure timestamp; latency must
        # be measured on the actual C270. Streams run on the same PC clock.
        image = self.camera.read()
        return time.monotonic(), image

    def close(self):
        if self.camera.is_connected:
            self.camera.disconnect()


def load_intrinsics(path, resolution=None):
    data = yaml.safe_load(Path(path).read_text())
    k = np.array([[data["fx"], 0.0, data["cx"]], [0.0, data["fy"], data["cy"]], [0.0, 0.0, 1.0]])
    d = np.array(data["distortion"], dtype=float)
    if (
        not np.isfinite(k).all()
        or not np.isfinite(d).all()
        or k[0, 0] <= 0
        or k[1, 1] <= 0
        or len(d) not in (4, 5, 8, 12, 14)
    ):
        raise ValueError("Invalid intrinsics")
    if resolution is not None and list(resolution) != data["resolution"]:
        raise ValueError(f"Intrinsics resolution {data['resolution']} != capture {resolution}; recalibrate")
    return data, k, d


def intrinsics_id(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]


def calibrate_images(images, board, square_m, output, max_rms=1.5):
    import cv2

    if min(board) < 3 or square_m <= 0:
        raise ValueError("Checkerboard must have >=3 inner corners/axis, square_m>0")
    object_grid = np.zeros((board[0] * board[1], 3), np.float32)
    object_grid[:, :2] = np.mgrid[: board[0], : board[1]].T.reshape(-1, 2) * square_m
    objects, points, size = [], [], None
    for image in images:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_size = gray.shape[::-1]
        if size is not None and size != current_size:
            raise ValueError("Calibration images have different resolutions")
        size = current_size
        ok, corners = cv2.findChessboardCorners(gray, tuple(board))
        if ok:
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            objects.append(object_grid.copy())
            points.append(corners)
    if len(points) < 12:
        raise ValueError(f"Need >=12 detected checkerboards; got {len(points)}")
    centres = np.array([p.mean(axis=0)[0] for p in points]) / np.array(size)
    if np.linalg.norm(np.ptp(centres, axis=0)) < 0.25:
        raise ValueError("Move the checkerboard to the edges as well as the centre")
    rms, k, d, _, _ = cv2.calibrateCamera(objects, points, size, None, None)
    if not np.isfinite(rms) or rms > max_rms:
        raise ValueError(f"Intrinsic calibration rejected, reprojection RMS={rms:.3f}px")
    data = {
        "fx": float(k[0, 0]),
        "fy": float(k[1, 1]),
        "cx": float(k[0, 2]),
        "cy": float(k[1, 2]),
        "distortion": d.ravel().tolist(),
        "resolution": list(size),
        "rms_px": float(rms),
        "board_inner_corners": list(board),
        "square_m": square_m,
        "images": len(points),
    }
    save_yaml(output, data)
    return data


def export_stella(intrinsics, fps, output):
    # Bridge undistorts images with the calibrated K: SLAM gets zero distortion.
    data, _, _ = load_intrinsics(intrinsics)
    width, height = data["resolution"]
    save_yaml(
        output,
        {
            "Camera": {
                "name": "C270 calibrated rectified",
                "setup": "monocular",
                "model": "perspective",
                "color_order": "BGR",
                "cols": width,
                "rows": height,
                "fps": float(fps),
                **{k: data[k] for k in ("fx", "fy", "cx", "cy")},
                "k1": 0.0,
                "k2": 0.0,
                "p1": 0.0,
                "p2": 0.0,
                "k3": 0.0,
            },
            "Feature": {
                "max_num_keypoints": 1600,
                "scale_factor": 1.2,
                "num_levels": 8,
                "ini_fast_threshold": 20,
                "min_fast_threshold": 7,
            },
        },
    )
