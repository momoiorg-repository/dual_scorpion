"""Joint metric Sim(3) and rigid camera-to-head calibration, with observability checks."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .geometry import validate_pose


def save_yaml(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def umeyama(source, target):
    source, target = np.asarray(source), np.asarray(target)
    a, b = source - source.mean(0), target - target.mean(0)
    if len(a) < 3 or np.sum(a * a) < 1e-9:
        raise ValueError("Translation excitation is insufficient")
    u, singular, vt = np.linalg.svd(b.T @ a / len(a))
    sign = np.ones(3)
    sign[-1] = np.linalg.det(u @ vt)
    r = u @ np.diag(sign) @ vt
    scale = np.sum(singular * sign) / np.mean(np.sum(a * a, axis=1))
    return scale, r, target.mean(0) - scale * r @ source.mean(0)


@dataclass
class HeadCalibration:
    scale: float
    world_from_map: np.ndarray
    camera_from_head: np.ndarray
    quality: dict
    session: str = ""
    intrinsics_id: str = ""

    def __post_init__(self):
        if not np.isfinite(self.scale) or self.scale <= 0:
            raise ValueError("Scale must be positive and finite")
        self.world_from_map = validate_pose(self.world_from_map)
        self.camera_from_head = validate_pose(self.camera_from_head)

    def head(self, camera):
        camera = validate_pose(camera)
        metric = camera.copy()
        metric[:3, 3] *= self.scale
        return self.world_from_map @ metric @ self.camera_from_head

    def save(self, path):
        save_yaml(
            path,
            {
                "schema": 1,
                "scale": float(self.scale),
                "world_from_map": self.world_from_map.tolist(),
                "camera_from_head": self.camera_from_head.tolist(),
                "quality": self.quality,
                "session": self.session,
                "intrinsics_id": self.intrinsics_id,
            },
        )

    @classmethod
    def load(cls, path):
        data = yaml.safe_load(Path(path).read_text())
        if data.pop("schema", None) != 1:
            raise ValueError("Unsupported calibration schema")
        return cls(**data)


def calibrate_head(pairs, min_samples=60, max_rmse_m=0.04, max_rotation_deg=8.0, max_condition=1e5):
    if len(pairs) < min_samples:
        raise ValueError(f"Need {min_samples} synchronized samples; got {len(pairs)}")
    sessions = {c.session for _, c in pairs}
    intrinsics = {c.intrinsics_id for _, c in pairs}
    if len(sessions) != 1 or len(intrinsics) != 1:
        raise ValueError("Map/intrinsics changed during calibration")
    h = np.stack([validate_pose(m.bones[10]) for m, _ in pairs])
    c = np.stack([validate_pose(s.camera) for _, s in pairs])
    if not all(m.valid and s.valid for m, s in pairs):
        raise ValueError("Tracking lost during calibration")
    if np.linalg.norm(np.ptp(h[:, :3, 3], axis=0)) < 0.08:
        raise ValueError("Headを上下左右へ8cm以上平行移動してください")
    rotations = Rotation.from_matrix(c[:, :3, :3])
    excitation = (rotations[0].inv() * rotations).as_rotvec()
    if np.linalg.svd(excitation - excitation.mean(0), compute_uv=False)[1] < 0.15:
        raise ValueError("Headを異なる2軸以上で回転してください")
    s0, a0, t0 = umeyama(c[:, :3, 3], h[:, :3, 3])
    if s0 <= 0:
        raise ValueError("Invalid initial scale")
    x0 = (
        Rotation.from_matrix(
            np.einsum("nij,jk,nkl->nil", c[:, :3, :3].transpose(0, 2, 1), a0.T, h[:, :3, :3])
        )
        .mean()
        .as_rotvec()
    )
    initial = np.r_[np.log(s0), Rotation.from_matrix(a0).as_rotvec(), t0, x0, [0.0, 0.0, 0.0]]

    def predict(values):
        scale = np.exp(values[0])
        a = Rotation.from_rotvec(values[1:4]).as_matrix()
        x = Rotation.from_rotvec(values[7:10]).as_matrix()
        rc = np.einsum("ij,njk->nik", a, c[:, :3, :3])
        rh = rc @ x
        ph = scale * (c[:, :3, 3] @ a.T) + values[4:7] + rc @ values[10:13]
        return rh, ph

    def residual(values):
        rh, ph = predict(values)
        angles = Rotation.from_matrix(rh.transpose(0, 2, 1) @ h[:, :3, :3]).as_rotvec()
        return np.r_[(ph - h[:, :3, 3]).ravel(), (0.2 * angles).ravel()]

    fit = least_squares(
        residual,
        initial,
        loss="soft_l1",
        f_scale=0.015,
        max_nfev=1200,
        bounds=(np.r_[np.log(0.001), np.full(12, -np.inf)], np.r_[np.log(1000.0), np.full(12, np.inf)]),
    )
    rh, ph = predict(fit.x)
    errors = np.linalg.norm(ph - h[:, :3, 3], axis=1)
    rmse = float(np.sqrt(np.mean(errors**2)))
    angle_errors = np.rad2deg(Rotation.from_matrix(rh.transpose(0, 2, 1) @ h[:, :3, :3]).magnitude())
    rot_rmse = float(np.sqrt(np.mean(angle_errors**2)))
    normalized_jac = fit.jac / np.maximum(np.linalg.norm(fit.jac, axis=0), 1e-12)
    sv = np.linalg.svd(normalized_jac, compute_uv=False)
    condition = float(sv[0] / max(sv[-1], 1e-15))
    if (
        not fit.success
        or rmse > max_rmse_m
        or np.percentile(errors, 95) > 2 * max_rmse_m
        or rot_rmse > max_rotation_deg
        or condition > max_condition
        or np.linalg.norm(fit.x[10:13]) > 0.3
    ):
        raise ValueError(
            f"Calibration rejected: RMSE={rmse:.4f}m rotation={rot_rmse:.2f}deg condition={condition:.1f}"
        )
    a, x = np.eye(4), np.eye(4)
    a[:3, :3] = Rotation.from_rotvec(fit.x[1:4]).as_matrix()
    a[:3, 3] = fit.x[4:7]
    x[:3, :3] = Rotation.from_rotvec(fit.x[7:10]).as_matrix()
    x[:3, 3] = fit.x[10:13]
    quality = {
        "samples": len(pairs),
        "rmse_m": rmse,
        "rotation_rmse_deg": rot_rmse,
        "condition": condition,
        "extrinsic_valid": True,
    }
    return HeadCalibration(float(np.exp(fit.x[0])), a, x, quality, sessions.pop(), intrinsics.pop())
