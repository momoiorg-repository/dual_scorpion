"""Compare real motor feedback, simulated FK, and optional measured TCP points."""

import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..geometry import pose
from ..ik import DualSimulation
from .evaluate import _summary
from .real import SIDES, plan_hash, validate_plan


def compare_real(config, plan_path, report_path, reference_csv=None):
    plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
    report = json.loads(Path(report_path).read_text(encoding="utf-8"))
    if report.get("schema") != "dual_scorpion.mocopi.real_metrics.v1":
        raise ValueError("Unsupported real replay report")
    if report.get("plan_sha256") != plan_hash(plan) or report.get("urdf_sha256") != plan.get("urdf_sha256"):
        raise ValueError("Real report does not match this plan/URDF")
    samples = report.get("frames", [])
    if not samples:
        raise ValueError("Real report has no measured frames")
    indices = [sample["frame"] for sample in samples]
    if any(not isinstance(index, int) or index < 0 or index >= len(plan["frames"]) for index in indices):
        raise ValueError("Real report contains an invalid plan frame index")
    if any(b <= a for a, b in zip(indices, indices[1:], strict=False)):
        raise ValueError("Real report frame indices must increase")

    metrics = {
        side: {
            "joint_max_error_deg": [],
            "tcp_sim_vs_encoder_fk_m": [],
            "tcp_target_vs_encoder_fk_m": [],
            "orientation_sim_vs_encoder_fk_deg": [],
        }
        for side in SIDES
    }
    actual_positions = {}
    simulation = DualSimulation(config["robot"], gui=False)
    try:
        validate_plan(plan, config, simulation, allow_synthetic=True, max_sim_error_m=0.10)
        for sample in samples:
            index = sample["frame"]
            for side in SIDES:
                measured = np.asarray(sample["arms"][side]["measured_joint_deg"], dtype=float)
                planned = np.asarray(plan["frames"][index]["arms"][side]["joint_deg"], dtype=float)
                if measured.shape != (7,) or not np.isfinite(measured).all():
                    raise ValueError(f"Invalid measured joints at frame {index} ({side})")
                position, quaternion = simulation.arms[side].solver.fk_solver.compute(np.r_[measured, 0.0])
                actual = pose(position, quaternion)
                sim_vector = plan["frames"][index]["arms"][side]["simulated_pose"]
                target_vector = plan["frames"][index]["arms"][side]["target_pose"]
                simulated = pose(sim_vector[:3], sim_vector[3:])
                target = pose(target_vector[:3], target_vector[3:])
                actual_positions[(index, side)] = position
                metrics[side]["joint_max_error_deg"].append(float(np.max(np.abs(measured - planned))))
                metrics[side]["tcp_sim_vs_encoder_fk_m"].append(
                    float(np.linalg.norm(position - simulated[:3, 3]))
                )
                metrics[side]["tcp_target_vs_encoder_fk_m"].append(
                    float(np.linalg.norm(position - target[:3, 3]))
                )
                metrics[side]["orientation_sim_vs_encoder_fk_deg"].append(
                    float(np.rad2deg(Rotation.from_matrix(simulated[:3, :3] @ actual[:3, :3].T).magnitude()))
                )
    finally:
        simulation.close()

    result = {
        "schema": "dual_scorpion.mocopi.comparison.v1",
        "plan_sha256": plan_hash(plan),
        "urdf_sha256": plan["urdf_sha256"],
        "real_status": report["status"],
        "measured_frames": len(samples),
        "measured_fps": report.get("measured_fps"),
        "tracking": {
            side: {name: _summary(values) for name, values in metrics[side].items()} for side in SIDES
        },
        "external_reference": None,
        "limits": "Encoder FK and simulation share the URDF; external references are needed for independent TCP accuracy",
    }
    if reference_csv:
        references = {side: {"count": 0, "target_error_m": [], "fk_model_error_m": []} for side in SIDES}
        with Path(reference_csv).open(newline="", encoding="utf-8") as file:
            reader = csv.DictReader(file)
            required = {"frame", "side", "x_m", "y_m", "z_m"}
            if not required <= set(reader.fieldnames or []):
                raise ValueError("Reference CSV needs frame,side,x_m,y_m,z_m columns")
            for row in reader:
                index, side = int(row["frame"]), row["side"].strip().lower()
                if (index, side) not in actual_positions:
                    raise ValueError(f"Reference frame/side not present in the real report: {index}/{side}")
                measured_position = np.array([float(row[key]) for key in ("x_m", "y_m", "z_m")])
                if not np.isfinite(measured_position).all():
                    raise ValueError("Reference positions must be finite metres")
                target = np.asarray(plan["frames"][index]["arms"][side]["target_pose"][:3], dtype=float)
                entry = references[side]
                entry["count"] += 1
                entry["target_error_m"].append(float(np.linalg.norm(measured_position - target)))
                entry["fk_model_error_m"].append(
                    float(np.linalg.norm(measured_position - actual_positions[index, side]))
                )
        if not sum(references[side]["count"] for side in SIDES):
            raise ValueError("Reference CSV has no measurement rows")
        result["external_reference"] = {
            "coordinate_frame": "selected robot URDF base/world frame, metres",
            "measurements": {
                side: {
                    "count": values["count"],
                    "target_error_m": _summary(values["target_error_m"]),
                    "fk_model_error_m": _summary(values["fk_model_error_m"]),
                }
                for side, values in references.items()
            },
        }
    return result
