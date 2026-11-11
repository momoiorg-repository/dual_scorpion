"""End-to-end mocopi-to-robot dataset pipeline using the Full MuJoCo model."""

import json
from pathlib import Path

from .fingerprint import file_sha256
from .joint_dataset import export_joint_dataset
from .replay import replay_sim
from .verify import verify_dataset


def _write_new_json(path, value):
    path = Path(path)
    if path.exists():
        raise FileExistsError(f"Output already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as file:
        file.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def build_robot_dataset(
    config,
    dataset_path,
    output,
    repo_id,
    *,
    episodes=None,
    plan_dir=None,
    report_dir=None,
    preview_max_step_deg=None,
    ik_mode="default",
    arm_posture=None,
    orientation_tolerance_deg=None,
    slowdown=1,
    max_sim_error_m=0.025,
    max_sim_orientation_error_deg=None,
):
    """Run retargeting, Full-model IK, MuJoCo validation, and dataset export."""
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Robot dataset output already exists: {output}")
    plan_dir = Path(plan_dir or f"{output}-plans").resolve()
    report_dir = Path(report_dir or f"{output}-reports").resolve()
    if episodes is None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        manifest = json.loads(
            (Path(dataset_path) / "meta" / "mocopi_dataset.json").read_text(encoding="utf-8")
        )
        dataset = LeRobotDataset(manifest["repo_id"], root=Path(dataset_path), video_backend="pyav")
        episodes = list(range(dataset.num_episodes))
    if not episodes or any(not isinstance(value, int) or isinstance(value, bool) for value in episodes):
        raise ValueError("episodes must contain one or more integer episode indices")

    plans, plan_paths, report_paths, summaries = [], [], [], []
    try:
        for episode in episodes:
            plan_path = plan_dir / f"episode-{episode:03d}-plan.json"
            report_path = report_dir / f"episode-{episode:03d}-report.json"
            if plan_path.exists() or report_path.exists():
                raise FileExistsError(f"Plan/report already exists for episode {episode}")
            plan, report = replay_sim(
                config,
                dataset_path,
                episode,
                gui=False,
                realtime=False,
                show_camera=False,
                slowdown=slowdown,
                sim_model="full",
                preview_max_step_deg=preview_max_step_deg,
                ik_mode=ik_mode,
                arm_posture=arm_posture,
                orientation_tolerance_deg=orientation_tolerance_deg,
            )
            _write_new_json(plan_path, plan)
            _write_new_json(report_path, report)
            plans.append(plan)
            plan_paths.append(plan_path)
            report_paths.append(report_path)
            failures = plan.get("failures", [])
            collision = report.get("collision_check", {})
            summary = {
                "episode": episode,
                "plan_file": str(plan_path),
                "report_file": str(report_path),
                "plan_sha256": file_sha256(plan_path),
                "report_sha256": file_sha256(report_path),
                "ik_failures": failures,
                "collision_frames": collision.get("frames_with_collisions"),
                "max_position_error_m": report.get("feasibility", {}).get("max_position_error_m"),
                "max_orientation_error_deg": report.get("feasibility", {}).get("max_orientation_error_deg"),
                "within_preview_tolerance": report.get("feasibility", {}).get("within_preview_tolerance"),
            }
            summaries.append(summary)
            if failures or collision.get("frames_with_collisions"):
                raise RuntimeError(
                    f"Episode {episode} failed validation: "
                    f"ik_failures={len(failures)}, "
                    f"collision_frames={collision.get('frames_with_collisions')}"
                )

        exported = export_joint_dataset(
            config,
            dataset_path,
            plan_paths,
            output,
            repo_id,
            max_sim_error_m=max_sim_error_m,
            max_sim_orientation_error_deg=max_sim_orientation_error_deg,
            report_paths=report_paths,
        )
        _write_new_json(report_dir / "retarget-validation.json", exported["retarget_validation"])
        verification = verify_dataset(output, sample_frames=8)
        if verification["status"] not in ("ok", "warning"):
            raise RuntimeError(
                "Exported robot dataset failed verification: " + ", ".join(verification["errors"])
            )
        manifest_path = output / "meta" / "mocopi_joint_dataset.json"
        exported["pipeline"] = {
            "command": "build-robot",
            "source_dataset": str(Path(dataset_path).resolve()),
            "ik_mode": ik_mode,
            "arm_posture": arm_posture,
            "orientation_tolerance_deg": orientation_tolerance_deg,
            "episodes": summaries,
            "status": "ok_with_orientation_warnings"
            if exported.get("validation", {}).get("orientation_warning_count")
            else "ok",
        }
        exported["post_export_verification"] = verification
        exported["retarget_validation_file"] = str(report_dir / "retarget-validation.json")
        manifest_path.write_text(
            json.dumps(exported, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        return exported
    except Exception as exc:
        failure = {
            "schema": "dual_scorpion.mocopi.robot_pipeline.v1",
            "status": "failed",
            "reason": str(exc),
            "dataset": str(Path(dataset_path).resolve()),
            "ik_mode": ik_mode,
            "arm_posture": arm_posture,
            "orientation_tolerance_deg": orientation_tolerance_deg,
            "episodes": summaries,
        }
        _write_new_json(report_dir / "pipeline-failure.json", failure)
        raise
