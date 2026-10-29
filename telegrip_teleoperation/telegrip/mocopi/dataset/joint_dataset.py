"""Derive a distinctly labelled kinematic-joint dataset from reviewed plans."""

import json
from pathlib import Path

import numpy as np

from ..ik import DualSimulation
from .fingerprint import dataset_fingerprint, file_sha256
from .real import JOINTS, plan_hash, validate_plan

# The reference follower dataset uses right then left. Grippers are absent.
JOINT_SIDES = ("right", "left")
JOINT_NAMES = [f"{side}_{name}.pos" for side in JOINT_SIDES for name in JOINTS]


def _joint_vector(frame):
    return np.concatenate(
        [np.asarray(frame["arms"][side]["joint_deg"], dtype=np.float32) for side in JOINT_SIDES]
    )


def export_joint_dataset(
    config,
    dataset_path,
    plan_paths,
    output,
    repo_id,
    *,
    max_sim_error_m=0.025,
    max_sim_orientation_error_deg=None,
):
    """Create a LeRobot dataset of simulated joints and recorded human video.

    The robot state at frame t is the kinematic configuration in the plan;
    action is the next planned configuration. Neither is a motor reading.
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    source_root, output_root = Path(dataset_path).resolve(), Path(output).resolve()
    if not plan_paths or not repo_id.strip():
        raise ValueError("At least one plan and a nonempty repo_id are required")
    if output_root.exists():
        raise FileExistsError(f"Dataset output already exists: {output_root}")
    if output_root.is_relative_to(source_root) or source_root.is_relative_to(output_root):
        raise ValueError("Source and output datasets must be in separate directories")
    manifest_path = source_root / "meta" / "mocopi_dataset.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "dual_scorpion.mocopi.lerobot.v1" or manifest.get("gripper_recorded"):
        raise ValueError("Unsupported mocopi dataset schema")
    fingerprint = dataset_fingerprint(source_root)
    manifest_hash = file_sha256(manifest_path)
    source = LeRobotDataset(manifest["repo_id"], root=source_root, video_backend="pyav")
    plans = [json.loads(Path(path).read_text(encoding="utf-8")) for path in plan_paths]
    model_kinds = {plan.get("model_kind", "legacy_urdf") for plan in plans}
    if len(model_kinds) != 1:
        raise ValueError("All plans must use the same simulation model")
    full_model = model_kinds == {"full_mujoco"}
    episodes, sources, seen = [], set(), set()
    if full_model:
        from ..full_mujoco import FullMujocoSimulation

        simulation = FullMujocoSimulation(config["robot"], gui=False)
    else:
        simulation = DualSimulation(config["robot"], gui=False)
    try:
        for plan in plans:
            if (
                plan.get("dataset_content_sha256") != fingerprint
                or plan.get("dataset_manifest_sha256") != manifest_hash
            ):
                raise ValueError("Plan does not match dataset content; regenerate the plan from this dataset")
            episode = plan.get("episode")
            if (
                not isinstance(episode, int)
                or isinstance(episode, bool)
                or not 0 <= episode < source.num_episodes
            ):
                raise ValueError("Plan has an invalid source episode")
            if episode in seen:
                raise ValueError(f"Source episode {episode} appears more than once")
            seen.add(episode)
            entry = manifest["episodes"][episode]
            capture_source = manifest["sessions"][entry["source_session"]]["source"]
            if plan.get("source") != capture_source:
                raise ValueError("Plan capture source does not match the source dataset")
            sources.add(capture_source)
            metadata = source.meta.episodes[episode]
            start, end = int(metadata["dataset_from_index"]), int(metadata["dataset_to_index"])
            slowdown = plan.get("slowdown")
            if (
                not isinstance(slowdown, int)
                or isinstance(slowdown, bool)
                or not 1 <= slowdown <= 10
                or plan.get("fps") != source.fps
                or plan.get("expected_frames") != (end - start) * slowdown
            ):
                raise ValueError(f"Plan timing does not match source episode {episode}")
            validate_plan(
                plan,
                config,
                simulation,
                allow_synthetic=True,
                max_sim_error_m=max_sim_error_m,
                max_sim_orientation_error_deg=max_sim_orientation_error_deg,
                allow_preview_model=full_model,
            )
            if any(
                frame.get("source_frame") != index // slowdown for index, frame in enumerate(plan["frames"])
            ):
                raise ValueError(f"Plan source frame mapping is invalid for episode {episode}")
            episodes.append((plan, start, end))
    finally:
        simulation.close()
    if len(sources) != 1:
        raise ValueError("Refusing to mix synthetic and real human recordings in a joint dataset")

    image_shape = tuple(source.meta.features["observation.images.head"]["shape"])
    state_names = source.meta.features["observation.state"]["names"]
    features = {
        "observation.images.head": {
            "dtype": "video",
            "shape": image_shape,
            "names": ["channels", "height", "width"],
        },
        "observation.state": {"dtype": "float32", "shape": (len(JOINT_NAMES),), "names": JOINT_NAMES},
        "action": {"dtype": "float32", "shape": (len(JOINT_NAMES),), "names": JOINT_NAMES},
        "observation.mocopi.state": {"dtype": "float32", "shape": (len(state_names),), "names": state_names},
        "observation.mocopi.raw_state": {
            "dtype": "float32",
            "shape": (len(state_names),),
            "names": state_names,
        },
        "observation.sim.frame": {
            "dtype": "int64",
            "shape": (2,),
            "names": ["plan_frame", "human_source_frame"],
        },
    }
    destination = LeRobotDataset.create(
        repo_id=repo_id,
        fps=source.fps,
        features=features,
        root=output_root,
        robot_type="dual_scorpion_kinematic",
        use_videos=True,
        vcodec="h264",
        image_writer_threads=2,
    )
    result = {
        "schema": "dual_scorpion.mocopi.kinematic_joint_dataset.v1",
        "repo_id": repo_id,
        "source_dataset_sha256": fingerprint,
        "capture_source": next(iter(sources)),
        "robot_state_source": "kinematic_simulation_not_real_encoders",
        "joint_unit": "degree",
        "joint_names": JOINT_NAMES,
        "state_semantics": "planned kinematic configuration at this frame; no real motor observations",
        "action_semantics": "planned kinematic configuration at the next frame; no gripper",
        "image_semantics": "human head camera at the plan source frame, not a robot camera",
        "fps": source.fps,
        "gripper_recorded": False,
        "model_kind": plans[0].get("model_kind", "legacy_urdf"),
        "start_pose": plans[0].get("start_pose", "config"),
        "hardware_replay_allowed": not full_model,
        "mujoco_model_sha256": plans[0].get("mujoco_model_sha256"),
        "urdf_sha256": plans[0]["urdf_sha256"],
        "episodes": [],
    }
    try:
        for plan, start, _ in episodes:
            cached_index, cached = None, None
            for index, (frame, following) in enumerate(zip(plan["frames"], plan["frames"][1:], strict=False)):
                source_index = frame["source_frame"]
                if source_index != cached_index:
                    cached = source[start + source_index]
                    cached_index = source_index
                destination.add_frame(
                    {
                        "task": cached["task"],
                        "observation.images.head": cached["observation.images.head"].numpy(),
                        "observation.state": _joint_vector(frame),
                        "action": _joint_vector(following),
                        "observation.mocopi.state": cached["observation.state"].numpy(),
                        "observation.mocopi.raw_state": cached["observation.mocopi.raw_state"].numpy(),
                        "observation.sim.frame": np.array([index, source_index], dtype=np.int64),
                    }
                )
            destination.save_episode()
            result["episodes"].append(
                {
                    "source_episode": plan["episode"],
                    "start_pose": plan.get("start_pose", "config"),
                    "plan_sha256": plan_hash(plan),
                    "slowdown": plan["slowdown"],
                    "frames": len(plan["frames"]) - 1,
                }
            )
    finally:
        destination.finalize()
    if dataset_fingerprint(source_root) != fingerprint:
        raise ValueError("Source dataset changed during export; output is incomplete")
    (output_root / "meta" / "mocopi_joint_dataset.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return result
