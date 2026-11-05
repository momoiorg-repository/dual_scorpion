"""Rerun viewer for recorded video and converted MOCOPI actions."""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .editor import load_dataset, validate_selection

SUMMARY_SCHEMA = "dual_scorpion.mocopi.rerun_view.v1"


def _numpy(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def _image_rgb(value):
    image = _numpy(value)
    if image.ndim == 3 and image.shape[0] in (3, 4):
        image = np.transpose(image, (1, 2, 0))
    if image.dtype != np.uint8:
        image = np.clip(image * 255, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(image[:, :, :3])


def _pose_map(values, names):
    state = _numpy(values).astype(float).reshape(-1, 7)
    return {
        name: {"position": state[index, :3], "quaternion": state[index, 3:7]}
        for index, name in enumerate(names)
    }


def _hand_action(values):
    action = _numpy(values).astype(float).reshape(2, 7)
    return {
        "left": {"position": action[0, :3], "quaternion": action[0, 3:7]},
        "right": {"position": action[1, :3], "quaternion": action[1, 3:7]},
    }


def _tcp_pair(values):
    poses = np.asarray(values, dtype=float).reshape(2, 7)
    return {
        "right": {"position": poses[0, :3], "quaternion": poses[0, 3:7]},
        "left": {"position": poses[1, :3], "quaternion": poses[1, 3:7]},
    }


def _pose_error(current, expected):
    position_m = float(np.linalg.norm(current["position"] - expected["position"]))
    relative = Rotation.from_quat(current["quaternion"]).inv() * Rotation.from_quat(
        expected["quaternion"]
    )
    return position_m, float(np.rad2deg(relative.magnitude()))


def _blueprint(rr):
    camera = rr.blueprint.Spatial2DView(origin="/camera", name="Recorded video")
    world = rr.blueprint.Spatial3DView(origin="/mocopi", name="Observation and next Action")
    plots = rr.blueprint.TimeSeriesView(origin="/plots", name="Action, timing, and selection")
    info = rr.blueprint.TextLogView(origin="/info/frame", name="Frame details")
    return rr.blueprint.Blueprint(
        rr.blueprint.Grid(camera, world, plots, info, grid_columns=2),
        auto_layout=False,
    )


def _log_pose_axes(recording, rr, prefix, hands):
    origins, vectors, colors, labels = [], [], [], []
    axis_colors = ((255, 80, 80), (80, 220, 120), (90, 150, 255))
    for side in ("left", "right"):
        pose = hands[side]
        matrix = Rotation.from_quat(pose["quaternion"]).as_matrix()
        for axis, color in zip(("x", "y", "z"), axis_colors, strict=True):
            origins.append(pose["position"])
            vectors.append(matrix[:, "xyz".index(axis)] * 0.07)
            colors.append(color)
            labels.append(f"{side}_{axis}")
    recording.log(
        prefix,
        rr.Arrows3D(
            origins=np.asarray(origins),
            vectors=np.asarray(vectors),
            colors=colors,
            labels=labels,
            show_labels=False,
            radii=[0.003] * len(labels),
        ),
    )


def _log_hands(recording, rr, path, hands, color, radius, label_prefix):
    positions = np.asarray([hands[side]["position"] for side in ("left", "right")])
    recording.log(
        path,
        rr.Points3D(
            positions,
            colors=[color, color],
            labels=[f"{label_prefix} left", f"{label_prefix} right"],
            show_labels=True,
            radii=[radius, radius],
        ),
    )


def log_dataset_to_rerun(
    dataset_path,
    *,
    episode=None,
    selection_path=None,
    save_path=None,
    spawn=True,
    max_frames=0,
):
    """Log video, observation poses, next actions, errors, and timing to Rerun."""
    import rerun as rr

    root = Path(dataset_path).resolve()
    dataset, manifest = load_dataset(root)
    if episode is not None and (
        not isinstance(episode, int) or isinstance(episode, bool) or not 0 <= episode < dataset.num_episodes
    ):
        raise ValueError(f"episode must be between 0 and {dataset.num_episodes - 1}")
    if not isinstance(max_frames, int) or max_frames < 0:
        raise ValueError("max_frames must be zero or a positive integer")
    if save_path is not None:
        save_path = Path(save_path)
        if save_path.exists():
            raise FileExistsError(f"Rerun output already exists: {save_path}")
        save_path.parent.mkdir(parents=True, exist_ok=True)
    selection = None
    selected_indices = set()
    if selection_path is not None:
        selection = validate_selection(dataset, root, json.loads(Path(selection_path).read_text()))
        for item in selection["selections"]:
            selected_indices.update(range(item["start_index"], item["end_index"]))

    blueprint = _blueprint(rr)
    recording = rr.RecordingStream("dual_scorpion_mocopi_dataset")
    if save_path is not None:
        recording.save(save_path, default_blueprint=blueprint)
    if spawn:
        recording.spawn(default_blueprint=blueprint)
    recording.send_blueprint(blueprint)

    names = manifest["state_bones"]
    episodes = [episode] if episode is not None else list(range(dataset.num_episodes))
    total_frames = sum(
        int(dataset.meta.episodes[index]["dataset_to_index"])
        - int(dataset.meta.episodes[index]["dataset_from_index"])
        for index in episodes
    )
    summary = {
        "dataset": str(root),
        "repo_id": dataset.repo_id,
        "fps": float(dataset.fps),
        "episodes": episodes,
        "frames": total_frames,
        "max_sync_s": manifest.get("max_sync_s"),
        "selection_file": None if selection_path is None else str(Path(selection_path).resolve()),
    }
    recording.log(
        "/info/summary",
        rr.TextDocument(
            "\n".join(
                [
                    f"dataset: {root}",
                    f"repo_id: {dataset.repo_id}",
                    f"fps: {dataset.fps:g}",
                    f"episodes: {episodes}",
                    f"frames: {total_frames}",
                    f"max_sync_s: {manifest.get('max_sync_s')}",
                    "",
                    "3D: gray=observation, red/cyan=converted action, green=next observed pose",
                    "Plots: current hand vs action, action error, capture timing, selection flag",
                ]
            )
        ),
        static=True,
    )
    recording.log("/mocopi", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    global_index = 0
    logged = 0
    max_camera_mocopi_ms = 0.0
    action_quality = {
        side: {"compared_frames": 0, "max_position_error_m": 0.0, "max_orientation_error_deg": 0.0}
        for side in ("left", "right")
    }
    for episode_index in episodes:
        episode_start = int(dataset.meta.episodes[episode_index]["dataset_from_index"])
        episode_end = int(dataset.meta.episodes[episode_index]["dataset_to_index"])
        for local_frame, index in enumerate(range(episode_start, episode_end)):
            if max_frames and logged >= max_frames:
                break
            row = dataset[index]
            current = _pose_map(row["observation.state"], names)
            action = _hand_action(row["action"])
            expected = None
            if index + 1 < episode_end:
                expected = _pose_map(dataset[index + 1]["observation.state"], names)

            recording.set_time("dataset_time", timestamp=global_index / float(dataset.fps))
            recording.set_time("episode_time", timestamp=float(row["timestamp"]))
            recording.set_time("frame", sequence=global_index)
            recording.set_time("episode", sequence=episode_index)
            recording.set_time("local_frame", sequence=local_frame)

            recording.log("/camera/image", rr.Image(_image_rgb(row["observation.images.head"])))

            point_names = ("head", "left_shoulder", "left_elbow", "left_hand", "right_shoulder", "right_elbow", "right_hand")
            observation_positions = np.asarray([current[name]["position"] for name in names])
            recording.log(
                "/mocopi/observation/points",
                rr.Points3D(
                    observation_positions,
                    colors=[(210, 210, 210)] * len(names),
                    labels=point_names,
                    show_labels=True,
                    radii=[0.006 if "hand" not in name else 0.009 for name in names],
                ),
            )
            observation_strips = [
                [current["head"]["position"], current["left_shoulder"]["position"], current["left_elbow"]["position"], current["left_hand"]["position"]],
                [current["head"]["position"], current["right_shoulder"]["position"], current["right_elbow"]["position"], current["right_hand"]["position"]],
                [current["left_shoulder"]["position"], current["head"]["position"], current["right_shoulder"]["position"]],
            ]
            recording.log(
                "/mocopi/observation/skeleton",
                rr.LineStrips3D(
                    observation_strips,
                    colors=[(180, 180, 180), (120, 160, 200), (220, 220, 220)],
                    radii=[0.003, 0.003, 0.002],
                ),
            )
            _log_hands(recording, rr, "/mocopi/action/points", action, (255, 80, 80), 0.012, "action")
            _log_pose_axes(recording, rr, "/mocopi/action/axes", action)

            delta_origins = np.asarray([current[f"{side}_hand"]["position"] for side in ("left", "right")])
            delta_vectors = np.asarray(
                [action[side]["position"] - current[f"{side}_hand"]["position"] for side in ("left", "right")]
            )
            recording.log(
                "/mocopi/action/delta",
                rr.Arrows3D(
                    origins=delta_origins,
                    vectors=delta_vectors,
                    colors=[(255, 80, 80), (80, 220, 220)],
                    labels=["left action delta", "right action delta"],
                    show_labels=False,
                    radii=[0.004, 0.004],
                ),
            )

            errors = {}
            if expected is not None:
                expected_hands = {"left": expected["left_hand"], "right": expected["right_hand"]}
                _log_hands(
                    recording,
                    rr,
                    "/mocopi/expected_next/points",
                    expected_hands,
                    (80, 220, 120),
                    0.008,
                    "next",
                )
                for side in ("left", "right"):
                    errors[side] = _pose_error(action[side], expected[f"{side}_hand"])
            else:
                recording.log("/mocopi/expected_next/points", rr.Clear(recursive=False))

            for side in ("left", "right"):
                for axis_index, axis in enumerate("xyz"):
                    recording.log(
                        f"/plots/hands/{side}/current/{axis}",
                        rr.Scalars([float(current[f'{side}_hand']['position'][axis_index])]),
                    )
                    recording.log(
                        f"/plots/hands/{side}/action/{axis}",
                        rr.Scalars([float(action[side]["position"][axis_index])]),
                    )
                if side in errors:
                    action_quality[side]["compared_frames"] += 1
                    action_quality[side]["max_position_error_m"] = max(
                        action_quality[side]["max_position_error_m"], errors[side][0]
                    )
                    action_quality[side]["max_orientation_error_deg"] = max(
                        action_quality[side]["max_orientation_error_deg"], errors[side][1]
                    )
                    recording.log(
                        f"/plots/action_error/{side}_position_m",
                        rr.Scalars([errors[side][0]]),
                    )
                    recording.log(
                        f"/plots/action_error/{side}_orientation_deg",
                        rr.Scalars([errors[side][1]]),
                    )

            camera_time = float(row["observation.capture.camera_time_s"])
            mocopi_time = float(row["observation.capture.mocopi_time_s"])
            grid_time = float(row["observation.capture.grid_time_s"])
            camera_mocopi_ms = abs(camera_time - mocopi_time) * 1000
            max_camera_mocopi_ms = max(max_camera_mocopi_ms, camera_mocopi_ms)
            recording.log(
                "/plots/timing/camera_mocopi_abs_ms",
                rr.Scalars([camera_mocopi_ms]),
            )
            recording.log(
                "/plots/timing/grid_camera_abs_ms",
                rr.Scalars([abs(camera_time - grid_time) * 1000]),
            )
            recording.log(
                "/plots/selection/included",
                rr.Scalars([1.0 if index in selected_indices else 0.0]),
            )

            lines = [
                f"episode {episode_index}, local frame {local_frame}, dataset index {index}",
                f"time {float(row['timestamp']):.4f}s",
                f"task: {row['task']}",
                f"camera frame id: {int(row['observation.capture.camera_frame_id'])}",
                f"mocopi frame id: {int(row['observation.capture.mocopi_frame_id'])}",
                f"grid/camera/mocopi: {grid_time:.4f} / {camera_time:.4f} / {mocopi_time:.4f}s",
            ]
            for side in ("left", "right"):
                position = action[side]["position"]
                text = f"{side} action xyz: {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f} m"
                if side in errors:
                    text += f"; position error {errors[side][0] * 1000:.3f} mm, orientation {errors[side][1]:.3f} deg"
                lines.append(text)
            lines.append("selected: " + ("yes" if index in selected_indices else "no"))
            recording.log("/info/frame", rr.TextLog("\n".join(lines)))

            logged += 1
            global_index += 1
        if max_frames and logged >= max_frames:
            break
    recording.flush()
    summary.update(
        {
            "schema": SUMMARY_SCHEMA,
            "logged_frames": logged,
            "selected_frames": len(selected_indices),
            "max_camera_mocopi_offset_ms": max_camera_mocopi_ms,
            "action_quality": action_quality,
            "rrd": None if save_path is None else str(save_path),
        }
    )
    return summary


def log_robot_dataset_to_rerun(
    dataset_path,
    *,
    episode=None,
    save_path=None,
    spawn=True,
    max_frames=0,
):
    """Log robot target/actual TCP trajectories and MuJoCo joint states to Rerun."""
    import rerun as rr

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(dataset_path).resolve()
    manifest = json.loads((root / "meta" / "mocopi_joint_dataset.json").read_text(encoding="utf-8"))
    dataset = LeRobotDataset(manifest["repo_id"], root=root, video_backend="pyav")
    if episode is not None and (
        not isinstance(episode, int) or isinstance(episode, bool) or not 0 <= episode < dataset.num_episodes
    ):
        raise ValueError(f"episode must be between 0 and {dataset.num_episodes - 1}")
    if save_path is not None:
        save_path = Path(save_path)
        if save_path.exists():
            raise FileExistsError(f"Rerun output already exists: {save_path}")
        save_path.parent.mkdir(parents=True, exist_ok=True)
    blueprint = _blueprint(rr)
    recording = rr.RecordingStream("dual_scorpion_robot_dataset")
    if save_path is not None:
        recording.save(save_path, default_blueprint=blueprint)
    if spawn:
        recording.spawn(default_blueprint=blueprint)
    recording.send_blueprint(blueprint)
    episodes = [episode] if episode is not None else list(range(dataset.num_episodes))
    recording.log(
        "/info/summary",
        rr.TextDocument(
            "\n".join(
                [
                    f"robot dataset: {root}",
                    f"repo_id: {dataset.repo_id}",
                    f"source: {manifest.get('source_dataset_repo_id')}",
                    f"fps: {dataset.fps:g}",
                    f"episodes: {episodes}",
                    f"quality status: {manifest.get('validation', {}).get('status')}",
                    "",
                    "3D: red/cyan=target TCP, green/yellow=MuJoCo actual TCP",
                    "Plots: target/actual TCP, joint angles, TCP error, capture timing",
                ]
            )
        ),
        static=True,
    )
    recording.log("/robot", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    global_index = 0
    logged = 0
    for episode_index in episodes:
        episode_start = int(dataset.meta.episodes[episode_index]["dataset_from_index"])
        episode_end = int(dataset.meta.episodes[episode_index]["dataset_to_index"])
        for local_frame, index in enumerate(range(episode_start, episode_end)):
            if max_frames and logged >= max_frames:
                break
            row = dataset[index]
            recording.set_time("dataset_time", timestamp=global_index / float(dataset.fps))
            recording.set_time("episode_time", timestamp=float(row["timestamp"]))
            recording.set_time("frame", sequence=global_index)
            recording.set_time("episode", sequence=episode_index)
            recording.set_time("local_frame", sequence=local_frame)
            recording.log("/camera/image", rr.Image(_image_rgb(row["observation.images.head"])))
            target = _tcp_pair(row["observation.robot.target_pose"])
            actual = _tcp_pair(row["observation.robot.tcp_pose"])
            _log_hands(recording, rr, "/robot/target/points", target, (255, 90, 90), 0.012, "target")
            _log_hands(recording, rr, "/robot/actual/points", actual, (90, 230, 150), 0.009, "actual")
            _log_pose_axes(recording, rr, "/robot/actual/axes", actual)
            for side, pose in target.items():
                recording.log(
                    f"/robot/target/{side}/delta",
                    rr.Arrows3D(
                        vectors=(pose["position"] - actual[side]["position"])[None, :],
                        origins=actual[side]["position"][None, :],
                        colors=[(255, 90, 90)],
                        radii=[0.004],
                    ),
                )
            joints = np.asarray(row["observation.state"], dtype=float)
            errors = np.asarray(row["observation.robot.error"], dtype=float)
            warnings = np.asarray(row["observation.robot.warning"], dtype=float)
            collisions = np.asarray(row["observation.robot.collision"], dtype=float)
            for joint_index, value in enumerate(joints):
                recording.log(
                    f"/plots/joints/{manifest['joint_names'][joint_index]}",
                    rr.Scalars([float(value)]),
                )
            for side_index, side in enumerate(("right", "left")):
                for axis_index, axis in enumerate("xyz"):
                    recording.log(
                        f"/plots/tcp/{side}/target_{axis}",
                        rr.Scalars([float(target[side]["position"][axis_index])]),
                    )
                    recording.log(
                        f"/plots/tcp/{side}/actual_{axis}",
                        rr.Scalars([float(actual[side]["position"][axis_index])]),
                    )
                recording.log(
                    f"/plots/error/{side}_position_m",
                    rr.Scalars([float(errors[side_index * 2])]),
                )
                recording.log(
                    f"/plots/error/{side}_orientation_deg",
                    rr.Scalars([float(errors[side_index * 2 + 1])]),
                )
                recording.log(
                    f"/plots/warning/{side}_orientation",
                    rr.Scalars([float(warnings[side_index])]),
                )
            recording.log("/plots/collision/count", rr.Scalars([float(collisions[0])]))
            camera_time = float(row["observation.capture.camera_time_s"])
            mocopi_time = float(row["observation.capture.mocopi_time_s"])
            grid_time = float(row["observation.capture.grid_time_s"])
            recording.log(
                "/plots/timing/camera_mocopi_abs_ms",
                rr.Scalars([abs(camera_time - mocopi_time) * 1000]),
            )
            recording.log(
                "/plots/timing/grid_camera_abs_ms",
                rr.Scalars([abs(camera_time - grid_time) * 1000]),
            )
            lines = [
                f"episode {episode_index}, local frame {local_frame}, dataset index {index}",
                f"time {float(row['timestamp']):.4f}s",
                f"task: {row['task']}",
                f"quality: {manifest.get('validation', {}).get('status')}",
                f"gripper: fixed at zero; feature included: {manifest.get('gripper_feature_included')}",
            ]
            for side_index, side in enumerate(("right", "left")):
                target_position = target[side]["position"]
                lines.append(
                    f"{side} target xyz: {target_position[0]:.3f}, {target_position[1]:.3f}, "
                    f"{target_position[2]:.3f} m; position error {errors[side_index * 2] * 1000:.3f} mm; "
                    f"orientation error {errors[side_index * 2 + 1]:.3f} deg; "
                    f"warning={bool(warnings[side_index])}"
                )
            recording.log("/info/frame", rr.TextLog("\n".join(lines)))
            logged += 1
            global_index += 1
        if max_frames and logged >= max_frames:
            break
    recording.flush()
    return {
        "dataset": str(root),
        "repo_id": dataset.repo_id,
        "fps": float(dataset.fps),
        "episodes": episodes,
        "logged_frames": logged,
        "quality_status": manifest.get("validation", {}).get("status"),
        "rrd": None if save_path is None else str(save_path),
    }
