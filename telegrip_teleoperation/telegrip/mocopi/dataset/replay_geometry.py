"""Full-CAD collision and clearance checks for follower replay."""

import numpy as np
from scipy.spatial.transform import Rotation

from ..full_mujoco import SUFFIX, FullMujocoSimulation


def measured_scene_ready(scene):
    if scene is None or not scene.execution_allowed:
        return False
    return {("table", "box"), ("support_stick", "cylinder")} <= {
        (item.role, item.shape) for item in scene.obstacles
    }


class ReplayGeometry:
    """Evaluate both arms in the same headless MuJoCo model at every replay setpoint."""

    def __init__(self, simulation, scene=None):
        self.simulation = simulation
        self.scene = scene if measured_scene_ready(scene) else None
        grippers = self.scene.replay_gripper_poses if self.scene is not None else None
        self.gripper_model_deg = {side: grippers[side].model_deg if grippers else 0.0 for side in SUFFIX}
        model = simulation.model
        self.tcp_ids = {side: model.site(f"{side}/tcp_site").id for side in SUFFIX}
        self.joint_ids = {
            side: [model.joint(f"joint{i}_{suffix}").id for i in range(7)] for side, suffix in SUFFIX.items()
        }
        self.link_vertices = {}
        arm_geom_ids = []
        for suffix in SUFFIX.values():
            for index in range(1, 9):
                name = f"{suffix.lower()}_link_{index}"
                body_id = model.body(name).id
                geom_ids = np.flatnonzero((model.geom_bodyid == body_id) & (model.geom_contype != 0))
                vertices = []
                for geom_id in geom_ids:
                    if model.geom_type[geom_id] == simulation.mujoco.mjtGeom.mjGEOM_MESH:
                        mesh = model.geom_dataid[geom_id]
                        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                        rotation = Rotation.from_quat(model.geom_quat[geom_id], scalar_first=True)
                        vertices.append(
                            rotation.apply(model.mesh_vert[start : start + count]) + model.geom_pos[geom_id]
                        )
                    else:
                        raise ValueError(
                            f"Unsupported arm collision geometry: {model.geom(int(geom_id)).name}"
                        )
                if not vertices:
                    raise ValueError(f"No Full-CAD collision mesh for {name}")
                self.link_vertices[name] = (body_id, np.concatenate(vertices))
                arm_geom_ids.extend(geom_ids)
        self.arm_geom_ids = np.asarray(arm_geom_ids)
        self.obstacles = {
            name: (item.role, model.geom(f"obstacle/{name}/box").id)
            for name, item in _measured_obstacles(self.scene).items()
        }

    @classmethod
    def build(cls, config, obstacles, scene):
        measured = scene if measured_scene_ready(scene) else None
        boxes = list(obstacles)
        for name, item in _measured_obstacles(measured).items():
            boxes.append(
                {
                    "name": name,
                    "center_m": item.center_m,
                    "half_size_m": item.half_size_m
                    if item.shape == "box"
                    else [item.radius_m, item.radius_m, item.length_m / 2],
                    "rotation_deg": Rotation.from_quat(item.rotation_xyzw).as_euler("xyz", degrees=True),
                }
            )
        simulation = FullMujocoSimulation(config, gui=False, obstacles=boxes)
        try:
            return cls(simulation, measured)
        except BaseException:
            simulation.close()
            raise

    def check_pose(self, model_joints, *, include_heights=False):
        sim = self.simulation
        for side in SUFFIX:
            sim.data.qpos[sim.qpos_indices[side][:7]] = np.deg2rad(model_joints[side])
            sim.data.qpos[sim.qpos_indices[side][7]] = np.deg2rad(self.gripper_model_deg[side])
        sim.mujoco.mj_forward(sim.model, sim.data)
        collision = sim.collision_report(ignore_fixed_obstacle_contacts=True)
        categories = {"self": 0, "inter_arm": 0, "floor": 0, "obstacle": 0}
        for pair, count in collision["body_pairs"].items():
            first, second = pair.split("|", 1)
            if first.startswith("obstacle/") or second.startswith("obstacle/"):
                category = "obstacle"
            elif first == "world" or second == "world":
                category = "floor"
            elif first.startswith("l_link_") and second.startswith("r_link_"):
                category = "inter_arm"
            else:
                category = "self"
            categories[category] += count
        collision["categories"] = categories

        range_violations = []
        for side, ids in self.joint_ids.items():
            arm = sim.arms[side]
            declared = np.rad2deg(sim.model.jnt_range[ids])
            limited = sim.model.jnt_limited[ids].astype(bool)
            lower = np.where(limited, declared[:, 0], arm.lower[:7])
            upper = np.where(limited, declared[:, 1], arm.upper[:7])
            for index in np.flatnonzero(
                (model_joints[side] < lower - 1e-6) | (model_joints[side] > upper + 1e-6)
            ):
                range_violations.append(
                    {
                        "side": side,
                        "joint": f"joint{index}",
                        "limit_source": "mjcf_joint_range" if limited[index] else "full_cad_preview_range",
                        "value_deg": float(model_joints[side][index]),
                        "min_deg": float(lower[index]),
                        "max_deg": float(upper[index]),
                    }
                )
        clearances = {name: self._clearance(geom_id) for name, (_, geom_id) in self.obstacles.items()}
        report = {
            "collision": collision,
            "model_limit_violations": range_violations,
            "obstacle_clearance_m": clearances,
            "table_clearance_m": min(
                (clearances[name] for name, (role, _) in self.obstacles.items() if role == "table"),
                default=None,
            ),
            "clearance_violations": [
                name for name, clearance in clearances.items() if clearance <= self.scene.collision_margin_m
            ],
        }
        if include_heights:
            report["tcp_world_z_m"] = {
                side: float(sim.data.site_xpos[i, 2]) for side, i in self.tcp_ids.items()
            }
            report["link_min_world_z_m"] = self._link_heights()
        return report

    def _link_heights(self):
        data = self.simulation.data
        result = {}
        for name, (body_id, vertices) in self.link_vertices.items():
            local_to_world_z = data.xmat[body_id].reshape(3, 3)[2]
            result[name] = float((vertices @ local_to_world_z).min() + data.xpos[body_id, 2])
        return result

    def tcp_pose(self, side):
        data = self.simulation.data
        site = self.tcp_ids[side]
        rotation = Rotation.from_matrix(data.site_xmat[site].reshape(3, 3))
        return data.site_xpos[site].copy(), rotation.as_quat()

    def _clearance(self, obstacle_id):
        sim = self.simulation
        positions, radii = sim.data.geom_xpos, sim.model.geom_rbound
        # Bounding spheres give a lower bound, so pairs farther than the best
        # distance already found can be skipped. Distances above 1 m are capped.
        bounds = (
            np.linalg.norm(positions[self.arm_geom_ids] - positions[obstacle_id], axis=1)
            - radii[self.arm_geom_ids]
            - radii[obstacle_id]
        )
        clearance = 1.0
        for index in np.argsort(bounds):
            if bounds[index] >= clearance:
                break
            distance = sim.mujoco.mj_geomDistance(
                sim.model, sim.data, int(self.arm_geom_ids[index]), obstacle_id, max(clearance, 1e-6), None
            )
            if not np.isfinite(distance):
                raise ValueError("MuJoCo returned a non-finite obstacle distance")
            clearance = min(clearance, distance)
        return clearance


def _measured_obstacles(scene):
    return {f"measured-{item.name}": item for item in scene.obstacles} if scene is not None else {}
