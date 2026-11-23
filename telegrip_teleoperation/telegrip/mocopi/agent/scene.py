"""Operator-measured table and support-stick geometry for real motion checks."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

SCENE_SCHEMA = "dual_scorpion.agent.scene.v1"
SUPPORT_STICK_LENGTH_M = 0.45


@dataclass(frozen=True)
class SceneObstacle:
    name: str
    role: str
    shape: str
    center_m: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]
    half_size_m: tuple[float, float, float] | None = None
    length_m: float | None = None
    radius_m: float | None = None


@dataclass(frozen=True)
class SceneGeometry:
    arm_collision_radius_m: float
    status: str
    coordinate_frame: str
    source: str
    measured_at: str
    collision_margin_m: float
    robot_to_scene: tuple[tuple[float, float, float, float], ...]
    obstacles: tuple[SceneObstacle, ...]
    required_obstacles: tuple[str, ...]
    sha256: str

    @property
    def execution_allowed(self) -> bool:
        return self.status == "measured" and bool(self.obstacles)

    @classmethod
    def from_file(cls, path: str | Path) -> SceneGeometry:
        scene_path = Path(path)
        document = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
        if not isinstance(document, dict):
            raise ValueError("scene geometry must be a YAML object")
        if document.get("schema") != SCENE_SCHEMA:
            raise ValueError(f"scene geometry must declare schema {SCENE_SCHEMA}")
        status = document.get("status")
        if status not in {"measured", "template_not_for_execution"}:
            raise ValueError("scene status must be measured or template_not_for_execution")
        coordinate_frame = _required_string(document, "coordinate_frame")
        source = _required_string(document, "source")
        measured_at = _required_string(document, "measured_at")
        margin = float(document.get("collision_margin_m"))
        if not np.isfinite(margin) or not 0 < margin <= 0.05:
            raise ValueError("scene collision_margin_m must be in (0, 0.05]")
        arm_radius = float(document.get("arm_collision_radius_m"))
        if not np.isfinite(arm_radius) or not 0 < arm_radius <= 0.05:
            raise ValueError("scene arm_collision_radius_m must be a measured positive value in (0, 0.05]")
        transform = _transform(document.get("robot_to_scene"))
        required = document.get("required_obstacles")
        if not isinstance(required, list) or not all(isinstance(item, str) for item in required):
            raise ValueError("scene required_obstacles must be a string array")
        rows = document.get("obstacles")
        if not isinstance(rows, list):
            raise ValueError("scene obstacles must be a list")
        obstacles = tuple(_obstacle_from_dict(row, transform) for row in rows)
        roles = {obstacle.role for obstacle in obstacles}
        missing = [role for role in required if role not in roles]
        if missing:
            raise ValueError(f"scene is missing required obstacle roles: {missing}")
        names = [obstacle.name for obstacle in obstacles]
        if len(names) != len(set(names)):
            raise ValueError("scene obstacle names must be unique")
        encoded = json.dumps(document, sort_keys=True, ensure_ascii=False, allow_nan=False).encode("utf-8")
        return cls(
            status=status,
            arm_collision_radius_m=arm_radius,
            coordinate_frame=coordinate_frame,
            source=source,
            measured_at=measured_at,
            collision_margin_m=margin,
            robot_to_scene=transform,
            obstacles=obstacles,
            required_obstacles=tuple(required),
            sha256=hashlib.sha256(encoded).hexdigest(),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": SCENE_SCHEMA,
            "status": self.status,
            "coordinate_frame": self.coordinate_frame,
            "source": self.source,
            "measured_at": self.measured_at,
            "collision_margin_m": self.collision_margin_m,
            "arm_collision_radius_m": self.arm_collision_radius_m,
            "robot_to_scene": [list(row) for row in self.robot_to_scene],
            "required_obstacles": list(self.required_obstacles),
            "obstacles": [
                {
                    "name": obstacle.name,
                    "role": obstacle.role,
                    "shape": obstacle.shape,
                    "center_m": list(obstacle.center_m),
                    "rotation_xyzw": list(obstacle.rotation_xyzw),
                    "half_size_m": None
                    if obstacle.half_size_m is None
                    else list(obstacle.half_size_m),
                    "length_m": obstacle.length_m,
                    "radius_m": obstacle.radius_m,
                }
                for obstacle in self.obstacles
            ],
            "sha256": self.sha256,
            "execution_allowed": self.execution_allowed,
        }


class SceneCollisionChecker:
    """Check the loaded PyBullet URDF arms against explicit measured scene bodies."""

    def __init__(self, simulation: Any, scene: SceneGeometry):
        import pybullet as p

        if not scene.execution_allowed:
            raise ValueError("measured scene geometry is required before collision checking")
        self.p = p
        self.simulation = simulation
        self.scene = scene
        self.body_ids: list[tuple[str, int]] = []
        self.proxy_bodies: list[dict[str, Any]] = []
        self._create_arm_proxies()
        for obstacle in scene.obstacles:
            if obstacle.shape == "box":
                shape = p.createCollisionShape(
                    p.GEOM_BOX,
                    halfExtents=np.asarray(obstacle.half_size_m, dtype=float),
                )
            elif obstacle.shape == "cylinder":
                shape = p.createCollisionShape(
                    p.GEOM_CYLINDER,
                    radius=float(obstacle.radius_m),
                    height=float(obstacle.length_m),
                )
            else:
                raise ValueError(f"unsupported obstacle shape {obstacle.shape!r}")
            body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=shape,
                basePosition=np.asarray(obstacle.center_m, dtype=float),
                baseOrientation=np.asarray(obstacle.rotation_xyzw, dtype=float),
            )
            self.body_ids.append((obstacle.name, int(body_id)))

    def close(self) -> None:
        for _, body_id in reversed(self.body_ids):
            self.p.removeBody(body_id)
        self.body_ids.clear()
        for proxy in reversed(self.proxy_bodies):
            self.p.removeBody(proxy["body_id"])
        self.proxy_bodies.clear()

    def check_pose(self, joints: dict[str, np.ndarray]) -> dict[str, Any]:
        contacts: list[dict[str, Any]] = []
        margin = self.scene.collision_margin_m
        for proxy in self.proxy_bodies:
            self._update_proxy(proxy, joints)
            for obstacle_name, obstacle_body_id in self.body_ids:
                for point in self.p.getClosestPoints(proxy["body_id"], obstacle_body_id, distance=margin):
                    distance = float(point[8])
                    if distance >= margin - 1e-9:
                        continue
                    contacts.append(
                        {
                            "arm": proxy["arm"],
                            "segment": proxy["segment"],
                            "obstacle": obstacle_name,
                            "distance_m": distance,
                            "penetrating": distance < 0.0,
                        }
                    )
        return {
            "configured": True,
            "checked": True,
            "margin_m": margin,
            "count": len(contacts),
            "clearance_violation": bool(contacts),
            "penetrating": any(item["penetrating"] for item in contacts),
            "min_distance_m": min((item["distance_m"] for item in contacts), default=None),
            "contacts": contacts,
        }

    def _create_arm_proxies(self) -> None:
        for side in ("left", "right"):
            fk = self.simulation.arms[side].solver.fk_solver
            points = np.asarray(fk.compute_joint_positions(np.zeros(8)), dtype=float)
            if points.shape != (8, 3):
                raise ValueError("URDF FK did not return eight joint-frame points")
            for segment in range(7):
                start, end = points[segment], points[segment + 1]
                length = float(np.linalg.norm(end - start))
                if length <= 1e-6:
                    continue
                shape = self.p.createCollisionShape(
                    self.p.GEOM_CAPSULE,
                    radius=self.scene.arm_collision_radius_m,
                    height=length,
                )
                body_id = self.p.createMultiBody(
                    baseMass=0,
                    baseCollisionShapeIndex=shape,
                    basePosition=(start + end) / 2.0,
                    baseOrientation=[0.0, 0.0, 0.0, 1.0],
                )
                self.proxy_bodies.append(
                    {
                        "arm": side,
                        "segment": segment,
                        "body_id": int(body_id),
                    }
                )

    def _update_proxy(self, proxy: dict[str, Any], joints: dict[str, np.ndarray]) -> None:
        fk = self.simulation.arms[proxy["arm"]].solver.fk_solver
        points = np.asarray(fk.compute_joint_positions(np.r_[joints[proxy["arm"]], 0.0]), dtype=float)
        start, end = points[proxy["segment"]], points[proxy["segment"] + 1]
        direction = end - start
        length = float(np.linalg.norm(direction))
        if length <= 1e-9:
            raise ValueError("URDF arm segment length collapsed")
        z_axis = np.array([0.0, 0.0, 1.0])
        unit = direction / length
        axis = np.cross(z_axis, unit)
        sin_angle = float(np.linalg.norm(axis))
        angle = float(np.arccos(np.clip(np.dot(z_axis, unit), -1.0, 1.0)))
        if sin_angle < 1e-9:
            quaternion = np.array([0.0, 0.0, 0.0, 1.0]) if angle < np.pi / 2 else np.array([1.0, 0.0, 0.0, 0.0])
        else:
            quaternion = Rotation.from_rotvec(axis / sin_angle * angle).as_quat()
        self.p.resetBasePositionAndOrientation(
            proxy["body_id"],
            ((start + end) / 2.0).tolist(),
            quaternion.tolist(),
        )


def _obstacle_from_dict(row: dict[str, Any], transform: np.ndarray) -> SceneObstacle:
    if not isinstance(row, dict):
        raise ValueError("each scene obstacle must be an object")
    name = _required_string(row, "name")
    role = _required_string(row, "role")
    shape = _required_string(row, "shape")
    if row.get("measurement_status") != "measured":
        raise ValueError(f"obstacle {name!r} must declare measurement_status: measured")
    center = _vector(row.get("center_m"), 3, f"obstacle {name!r} center_m")
    center = transform @ np.r_[center, 1.0]
    rotation = _quaternion(row.get("rotation_xyzw", [0.0, 0.0, 0.0, 1.0]))
    scene_rotation = Rotation.from_quat(rotation)
    robot_rotation = Rotation.from_matrix(transform[:3, :3]) * scene_rotation
    rotation_xyzw = robot_rotation.as_quat()
    if shape == "box":
        half_size = _vector(row.get("half_size_m"), 3, f"obstacle {name!r} half_size_m")
        if np.any(half_size <= 0):
            raise ValueError(f"obstacle {name!r} half_size_m values must be positive")
        return SceneObstacle(
            name=name,
            role=role,
            shape=shape,
            center_m=tuple(float(value) for value in center[:3]),
            rotation_xyzw=tuple(float(value) for value in rotation_xyzw),
            half_size_m=tuple(float(value) for value in half_size),
        )
    if shape == "cylinder":
        length = float(row.get("length_m"))
        radius = float(row.get("radius_m"))
        if not np.isfinite(length) or length <= 0 or not np.isfinite(radius) or radius <= 0:
            raise ValueError(f"obstacle {name!r} cylinder dimensions must be positive")
        if role == "support_stick" and abs(length - SUPPORT_STICK_LENGTH_M) > 1e-6:
            raise ValueError(f"support stick {name!r} must use the measured 0.45 m length")
        return SceneObstacle(
            name=name,
            role=role,
            shape=shape,
            center_m=tuple(float(value) for value in center[:3]),
            rotation_xyzw=tuple(float(value) for value in rotation_xyzw),
            length_m=length,
            radius_m=radius,
        )
    raise ValueError(f"obstacle {name!r} has unsupported shape {shape!r}")


def _transform(value: Any) -> np.ndarray:
    if not isinstance(value, dict):
        raise ValueError("scene robot_to_scene must be an object")
    position = _vector(value.get("position_m"), 3, "robot_to_scene.position_m")
    orientation = _quaternion(value.get("orientation_xyzw"))
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(orientation).as_matrix()
    result[:3, 3] = position
    return result


def _required_string(document: dict[str, Any], key: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return value


def _vector(value: Any, size: int, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {size} finite values")
    return result


def _quaternion(value: Any) -> np.ndarray:
    result = _vector(value, 4, "orientation_xyzw")
    if abs(np.linalg.norm(result) - 1.0) > 1e-5:
        raise ValueError("orientation_xyzw must be a unit quaternion")
    return result
