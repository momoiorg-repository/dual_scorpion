"""Export recorded robot feedback for evaluation, retaining actual timing."""

import hashlib
import json
from pathlib import Path

import numpy as np

from .fingerprint import dataset_fingerprint
from .joint_dataset import JOINT_NAMES, JOINT_SIDES
from .real import JOINTS, plan_hash

HARDWARE_REPORT_SCHEMA = "dual_scorpion.mocopi.hardware_dataset_replay.v1"
HARDWARE_FEEDBACK_SCHEMA = "dual_scorpion.mocopi.hardware_replay_feedback_dataset.v1"


def _vector(sample, key):
    parts = [np.asarray(sample["arms"][side][key], dtype=np.float32) for side in JOINT_SIDES]
    if any(part.shape != (len(JOINTS),) or not np.isfinite(part).all() for part in parts):
        raise ValueError(f"Invalid {key} in robot feedback")
    return np.concatenate(parts)


def _hardware_vector(sample, key):
    parts = [np.asarray(sample["arms"][side][key], dtype=np.float32) for side in JOINT_SIDES]
    if any(part.shape != (len(JOINTS),) or not np.isfinite(part).all() for part in parts):
        raise ValueError(f"Invalid {key} in hardware replay feedback")
    return np.concatenate(parts)


def _hardware_dataset_frames(dataset, report):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    metadata_path = Path(report["dataset"]) / "meta" / "mocopi_joint_dataset.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("model_kind") != "full_mujoco":
        raise ValueError("Hardware replay feedback requires a full-CAD joint dataset")
    if report.get("mujoco_model_sha256") != metadata.get("mujoco_model_sha256"):
        raise ValueError("Hardware report model hash does not match the source dataset")
    root = Path(report["dataset"]).resolve()
    current_hash = dataset_fingerprint(root, "mocopi_joint_dataset.json")
    if report.get("dataset_sha256") != current_hash:
        raise ValueError("Hardware report source dataset hash does not match the current dataset")
    source = LeRobotDataset(metadata["repo_id"], root=root, video_backend="pyav")
    states = np.asarray(source.hf_dataset["observation.state"], dtype=float)
    actions = np.asarray(source.hf_dataset["action"], dtype=float)
    samples = []
    previous_frame = -1
    previous_elapsed = -1.0
    for sample in report.get("frames", []):
        plan_frame = sample.get("plan_frame")
        source_frame = sample.get("source_frame")
        elapsed = sample.get("elapsed_s")
        if (
            isinstance(plan_frame, bool)
            or not isinstance(plan_frame, int)
            or plan_frame != previous_frame + 1
            or isinstance(source_frame, bool)
            or not isinstance(source_frame, int)
            or source_frame < 0
            or isinstance(elapsed, bool)
            or not isinstance(elapsed, (int, float))
            or not np.isfinite(elapsed)
            or elapsed < 0
            or elapsed <= previous_elapsed
        ):
            raise ValueError(
                "Hardware replay frames must have contiguous plan frames and increasing elapsed times"
            )
        previous_frame, previous_elapsed = plan_frame, elapsed
        sample = dict(sample)
        sample["_planned"] = _hardware_vector(sample, "planned_model_joint_deg")
        sample["_mapped"] = _hardware_vector(sample, "mapped_motor_joint_deg")
        sample["_sent"] = _hardware_vector(sample, "sent_motor_joint_deg")
        sample["_measured"] = _hardware_vector(sample, "measured_motor_joint_deg")
        sample["_model_measured"] = _hardware_vector(sample, "measured_model_joint_deg")
        expected = states[plan_frame] if plan_frame < len(states) else actions[-1]
        if plan_frame > len(states):
            raise ValueError("Hardware report references an unavailable source plan frame")
        if not np.allclose(sample["_planned"], expected, atol=1e-4, rtol=0):
            raise ValueError(f"Hardware report planned joints do not match dataset plan frame {plan_frame}")
        samples.append(sample)
    if len(samples) < 2:
        raise ValueError("Hardware replay report needs at least two frames")
    episode = int(report.get("episode", 0))
    task = source.meta.episodes[episode]["tasks"][0]
    return samples, int(source.fps), task


def export_real_dataset(runs, output, repo_id):
    """Export replay reports; images remain in the separate human dataset.

    A sample is the encoder state read after command i, paired with command
    i+1. Elapsed read times are retained; LeRobot's timestamp is nominal.
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    if not runs or not repo_id.strip():
        raise ValueError("At least one plan/report pair and a nonempty repo_id are required")
    root = Path(output).resolve()
    if root.exists():
        raise FileExistsError(f"Dataset output already exists: {root}")
    validated, fps_values, model_hashes, sources = [], set(), [], set()
    for plan_path, report_path in runs:
        plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
        report_bytes = Path(report_path).read_bytes()
        report = json.loads(report_bytes)
        report_sha256 = hashlib.sha256(report_bytes).hexdigest()
        if (
            plan.get("schema") != "dual_scorpion.mocopi.robot_plan.v1"
            or plan.get("gripper_included") is not False
            or plan.get("joint_names") != list(JOINTS)
            or report.get("schema") != "dual_scorpion.mocopi.real_metrics.v1"
            or report.get("gripper_commanded") is not False
        ):
            raise ValueError("Unsupported plan/report or unexpected gripper data")
        if report.get("plan_sha256") != plan_hash(plan) or report.get("urdf_sha256") != plan.get(
            "urdf_sha256"
        ):
            raise ValueError("Real report does not match its plan/URDF")
        hashes = plan.get("urdf_sha256")
        if (
            not isinstance(hashes, dict)
            or set(hashes) != {"left", "right"}
            or any(
                not isinstance(value, str)
                or len(value) != 64
                or any(character not in "0123456789abcdef" for character in value)
                for value in hashes.values()
            )
        ):
            raise ValueError("Plan must identify both URDFs")
        if report.get("source") != plan.get("source") or plan.get("source") not in ("fake", "real"):
            raise ValueError("Real report capture source does not match its plan")
        frames, samples, fps = plan.get("frames"), report.get("frames"), plan.get("fps")
        slowdown = plan.get("slowdown")
        if (
            not isinstance(fps, int)
            or isinstance(fps, bool)
            or not 0 < fps <= 120
            or not isinstance(frames, list)
            or not isinstance(samples, list)
            or not isinstance(slowdown, int)
            or isinstance(slowdown, bool)
            or not 1 <= slowdown <= 10
            or len(samples) < 2
            or report.get("status") not in ("complete", "aborted")
            or report.get("fps") != fps
            or len(frames) != plan.get("expected_frames")
            or (report.get("status") == "complete" and len(samples) != len(frames))
        ):
            raise ValueError("Invalid replay length, rate or status")
        previous_elapsed = -1.0
        for expected_index, sample in enumerate(samples):
            if (
                isinstance(sample.get("frame"), bool)
                or sample.get("frame") != expected_index
                or expected_index >= len(frames)
            ):
                raise ValueError("Robot feedback must contain a contiguous prefix of plan frames")
            elapsed = sample.get("elapsed_s")
            if (
                isinstance(elapsed, bool)
                or not isinstance(elapsed, (int, float))
                or not np.isfinite(elapsed)
                or elapsed < 0
                or elapsed <= previous_elapsed
            ):
                raise ValueError("Robot feedback times must be finite, non-negative and increasing")
            previous_elapsed = elapsed
            plan_frame = frames[expected_index]
            plan_time = plan_frame.get("time_s")
            if (
                not isinstance(plan_time, (int, float))
                or not np.isfinite(plan_time)
                or abs(plan_time - expected_index / fps) > 1e-5
                or sample.get("time_s") != plan_time
            ):
                raise ValueError(f"Invalid plan/report timing at frame {expected_index}")
            source_frame = plan_frame.get("source_frame")
            if (
                not isinstance(source_frame, int)
                or isinstance(source_frame, bool)
                or source_frame != expected_index // slowdown
            ):
                raise ValueError(f"Invalid human source frame for plan frame {expected_index}")
            planned = np.concatenate(
                [np.asarray(plan_frame["arms"][side]["joint_deg"], dtype=np.float32) for side in JOINT_SIDES]
            )
            observed_plan = _vector(sample, "planned_joint_deg")
            if (
                planned.shape != (14,)
                or not np.isfinite(planned).all()
                or not np.allclose(observed_plan, planned, atol=1e-4, rtol=0)
            ):
                raise ValueError(f"Reported planned joints do not match plan frame {expected_index}")
            _vector(sample, "measured_joint_deg")
            _vector(sample, "sent_joint_deg")
        fps_values.add(fps)
        model_hashes.append(plan.get("urdf_sha256"))
        sources.add(plan["source"])
        validated.append((plan, report, report_sha256))
    if len(fps_values) != 1 or any(value != model_hashes[0] for value in model_hashes):
        raise ValueError("All replay reports must use the same nominal FPS and URDF")
    if len(sources) != 1:
        raise ValueError("Refusing to mix synthetic and real human source recordings")
    fps = fps_values.pop()
    features = {
        "observation.state": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "action": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "observation.planned.state": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "observation.planned.action": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "observation.execution.time": {
            "dtype": "float64",
            "shape": (3,),
            "names": ["elapsed_s", "next_elapsed_s", "plan_time_s"],
        },
        "observation.execution.frame": {
            "dtype": "int64",
            "shape": (3,),
            "names": ["plan_frame", "next_plan_frame", "human_source_frame"],
        },
    }
    destination = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=features,
        root=root,
        robot_type="dual_scorpion_replay_feedback",
        use_videos=False,
    )
    manifest = {
        "schema": "dual_scorpion.mocopi.real_feedback_dataset.v1",
        "repo_id": repo_id,
        "fps": fps,
        "joint_names": JOINT_NAMES,
        "joint_unit": "degree",
        "state_semantics": "measured joints in supplied replay report, after the current command",
        "action_semantics": "next joint command sent by the replay software, not the next encoder measurement",
        "time_semantics": "LeRobot timestamp is nominal; actual read times are in observation.execution.time",
        "images_included": False,
        "gripper_recorded": False,
        "human_capture_source": next(iter(sources)),
        "urdf_sha256": model_hashes[0],
        "episodes": [],
    }
    try:
        for plan, report, report_sha256 in validated:
            samples = report["frames"]
            for current, following in zip(samples, samples[1:], strict=False):
                frame_index = current["frame"]
                plan_frame = plan["frames"][frame_index]
                source_frame = plan_frame.get("source_frame")
                destination.add_frame(
                    {
                        "task": plan.get("task") or f"Replay of mocopi episode {plan.get('episode', 0)}",
                        "observation.state": _vector(current, "measured_joint_deg"),
                        "action": _vector(following, "sent_joint_deg"),
                        "observation.planned.state": _vector(current, "planned_joint_deg"),
                        "observation.planned.action": _vector(following, "planned_joint_deg"),
                        "observation.execution.time": np.array(
                            [current["elapsed_s"], following["elapsed_s"], plan_frame["time_s"]],
                            dtype=np.float64,
                        ),
                        "observation.execution.frame": np.array(
                            [frame_index, following["frame"], source_frame], dtype=np.int64
                        ),
                    }
                )
            destination.save_episode()
            intervals = np.diff([sample["elapsed_s"] for sample in samples])
            manifest["episodes"].append(
                {
                    "source_episode": plan.get("episode"),
                    "plan_sha256": plan_hash(plan),
                    "report_sha256": report_sha256,
                    "source_dataset_sha256": plan.get("dataset_content_sha256"),
                    "report_status": report["status"],
                    "failure": report.get("failure"),
                    "robot_id": report.get("robot_id"),
                    "slowdown": plan["slowdown"],
                    "frames": len(samples) - 1,
                    "mean_read_interval_s": float(np.mean(intervals)),
                    "max_read_interval_s": float(np.max(intervals)),
                    "observed_read_fps": float(1 / np.mean(intervals)),
                }
            )
    finally:
        destination.finalize()
    (root / "meta" / "mocopi_real_feedback_dataset.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return manifest


def export_hardware_replay_dataset(runs, output, repo_id):
    """Convert full-CAD hardware replay reports without collapsing their spaces.

    ``observation.state`` is the encoder measurement in calibrated motor
    degrees.  ``observation.model.measured_state`` is that measurement mapped
    back through the verified hardware mapping.  Planned model joints, mapped
    motor targets, and actually sent motor commands remain separate features.
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    if not runs or not repo_id.strip():
        raise ValueError("At least one dataset/report pair and a nonempty repo_id are required")
    root = Path(output).resolve()
    if root.exists():
        raise FileExistsError(f"Dataset output already exists: {root}")
    validated, fps_values, model_hashes, dataset_hashes = [], set(), [], []
    for dataset_path, report_path in runs:
        report_bytes = Path(report_path).read_bytes()
        report = json.loads(report_bytes)
        if (
            report.get("schema") != HARDWARE_REPORT_SCHEMA
            or report.get("gripper_commanded") is not False
            or report.get("status") not in ("complete", "aborted")
            or not isinstance(report.get("frames"), list)
        ):
            raise ValueError("Unsupported hardware replay report or unexpected gripper data")
        report["dataset"] = str(Path(dataset_path).resolve())
        samples, fps, task = _hardware_dataset_frames(dataset_path, report)
        if len(samples) < 2:
            raise ValueError("Hardware replay report needs at least two feedback frames")
        if not (
            isinstance(report.get("dataset_sha256"), str)
            and report["dataset_sha256"]
            and isinstance(report.get("mujoco_model_sha256"), str)
            and report["mujoco_model_sha256"]
        ):
            raise ValueError("Hardware replay report must identify its dataset and model")
        report_sha256 = hashlib.sha256(report_bytes).hexdigest()
        validated.append((report, samples, task, report_sha256))
        fps_values.add(fps)
        model_hashes.append(report["mujoco_model_sha256"])
        dataset_hashes.append(report["dataset_sha256"])
    if len(fps_values) != 1 or len(set(model_hashes)) != 1 or len(set(dataset_hashes)) != 1:
        raise ValueError("All hardware replay runs must use one dataset, model and FPS")
    fps = fps_values.pop()
    features = {
        "observation.state": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "action": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "observation.planned.state": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "observation.planned.action": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "observation.commanded.state": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "observation.motor.target_state": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "observation.model.measured_state": {"dtype": "float32", "shape": (14,), "names": JOINT_NAMES},
        "observation.execution.time": {
            "dtype": "float64",
            "shape": (4,),
            "names": ["elapsed_s", "next_elapsed_s", "plan_time_s", "next_plan_time_s"],
        },
        "observation.execution.frame": {
            "dtype": "int64",
            "shape": (4,),
            "names": ["plan_frame", "next_plan_frame", "source_frame", "next_source_frame"],
        },
        "observation.execution.control_steps": {
            "dtype": "int64",
            "shape": (2,),
            "names": ["current_steps", "next_steps"],
        },
    }
    destination = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=features,
        root=root,
        robot_type="dual_scorpion_replay_feedback_full_cad",
        use_videos=False,
    )
    manifest = {
        "schema": HARDWARE_FEEDBACK_SCHEMA,
        "source_report_schema": HARDWARE_REPORT_SCHEMA,
        "repo_id": repo_id,
        "fps": fps,
        "joint_names": JOINT_NAMES,
        "joint_unit": "degree",
        "state_semantics": "Measured encoder joints in calibrated motor degrees, after the command",
        "planned_semantics": "Joint angles calculated by the existing IK plan; full-MuJoCo model space",
        "commanded_semantics": "Motor command actually sent after the existing step clamp",
        "action_semantics": "Next commanded motor joints, not the next encoder measurement",
        "model_measured_semantics": "Measured motor joints mapped back to full-MuJoCo model space",
        "time_semantics": "LeRobot timestamp is nominal; actual command/read times are in observation.execution.time",
        "images_included": False,
        "gripper_recorded": False,
        "gripper_commanded": False,
        "model_kind": "full_mujoco",
        "mujoco_model_sha256": model_hashes[0],
        "episodes": [],
    }
    try:
        for report, samples, task, report_sha256 in validated:
            for current, following in zip(samples, samples[1:], strict=False):
                destination.add_frame(
                    {
                        "task": task,
                        "observation.state": current["_measured"],
                        "action": following["_sent"],
                        "observation.planned.state": current["_planned"],
                        "observation.planned.action": following["_planned"],
                        "observation.commanded.state": current["_sent"],
                        "observation.motor.target_state": current["_mapped"],
                        "observation.model.measured_state": current["_model_measured"],
                        "observation.execution.time": np.array(
                            [
                                current["elapsed_s"],
                                following["elapsed_s"],
                                current["source_time_s"],
                                following["source_time_s"],
                            ],
                            dtype=np.float64,
                        ),
                        "observation.execution.frame": np.array(
                            [
                                current["plan_frame"],
                                following["plan_frame"],
                                current["source_frame"],
                                following["source_frame"],
                            ],
                            dtype=np.int64,
                        ),
                        "observation.execution.control_steps": np.array(
                            [current["control_steps"], following["control_steps"]], dtype=np.int64
                        ),
                    }
                )
            destination.save_episode()
            intervals = np.diff([sample["elapsed_s"] for sample in samples])
            manifest["episodes"].append(
                {
                    "source_episode": report.get("episode"),
                    "dataset": report["dataset"],
                    "dataset_sha256": report["dataset_sha256"],
                    "report_sha256": report_sha256,
                    "report_status": report["status"],
                    "failure": report.get("failure"),
                    "robot_id": report.get("robot_id"),
                    "calibration_sha256": report.get("calibration_sha256"),
                    "mapping": report.get("mapping"),
                    "source_frames": report.get("source_frames"),
                    "feedback_frames": len(samples) - 1,
                    "mean_command_interval_s": float(np.mean(intervals)),
                    "max_command_interval_s": float(np.max(intervals)),
                    "observed_command_fps": float(1 / np.mean(intervals)),
                    "time_scale": report.get("time_scale"),
                    "control_steps": report.get("control_steps"),
                }
            )
    finally:
        destination.finalize()
    (root / "meta" / "mocopi_hardware_replay_feedback_dataset.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return manifest
