"""Validate and replay a simulated joint plan on the existing follower buses."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..full_mujoco import full_model_path
from ..geometry import pose
from ..ik import DualSimulation
from .evaluate import _summary
from .replay import _file_hash, model_fingerprint

SIDES = ("left", "right")
JOINTS = tuple(f"joint{i}" for i in range(7))


def plan_hash(plan):
    encoded = json.dumps(plan, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _joint_map(frame):
    result = {}
    for side in SIDES:
        angles = np.asarray(frame["arms"][side]["joint_deg"], dtype=float)
        if angles.shape != (7,) or not np.isfinite(angles).all():
            raise ValueError(f"{side} plan must have seven finite joint angles")
        result[side] = angles
    return result


def validate_plan(
    plan,
    config,
    simulation,
    *,
    allow_synthetic=False,
    max_sim_error_m=0.025,
    max_sim_orientation_error_deg=None,
    allow_preview_model=False,
):
    """Check every command against the selected URDF before opening motor ports."""
    if (
        plan.get("schema") != "dual_scorpion.mocopi.robot_plan.v1"
        or plan.get("gripper_included") is not False
    ):
        raise ValueError("Unsupported plan or unexpected gripper commands")
    if plan.get("source") not in ("real", "fake") or (plan["source"] == "fake" and not allow_synthetic):
        raise ValueError("Synthetic plans require --allow-synthetic")
    model_kind = plan.get("model_kind", "legacy_urdf")
    if model_kind == "full_mujoco":
        if not allow_preview_model or plan.get("hardware_replay_allowed") is not False:
            raise ValueError("Full CAD model has unmeasured limits; this plan is simulation-only")
        if plan.get("mujoco_model_sha256") != _file_hash(full_model_path(config["robot"])):
            raise ValueError("Plan MuJoCo model differs from selected model; regenerate the plan")
    elif model_kind == "legacy_urdf":
        if plan.get("urdf_sha256") != model_fingerprint(config):
            raise ValueError("Plan URDF differs from selected URDF; regenerate and review the plan")
    else:
        raise ValueError("Unsupported plan model")
    if plan.get("joint_names") != list(JOINTS) or plan.get("failures") != []:
        raise ValueError("Plan contains missing joints or simulation failures")
    fps = plan.get("fps")
    frames = plan.get("frames")
    if (
        not isinstance(fps, int)
        or not 0 < fps <= 120
        or not isinstance(frames, list)
        or len(frames) < 2
        or len(frames) != plan.get("expected_frames")
    ):
        raise ValueError("Plan needs an integer fps and at least two frames")
    if model_kind == "full_mujoco":
        if any(frame.get("collision") is None for frame in frames):
            raise ValueError("Plan lacks modeled-collision results; regenerate the plan")
        if any(frame["collision"].get("count", 0) for frame in frames):
            raise ValueError("Plan contains modeled collisions")
    if not np.isfinite(max_sim_error_m) or not 0 < max_sim_error_m <= 0.10:
        raise ValueError("max_sim_error_m must be in (0, 0.10]")
    if max_sim_orientation_error_deg is not None and (
        not np.isfinite(max_sim_orientation_error_deg) or not 0 < max_sim_orientation_error_deg <= 180
    ):
        raise ValueError("max_sim_orientation_error_deg must be in (0, 180]")
    # At most 0.5 degrees per frame and 15 degrees/second, including if a
    # higher-rate dataset was generated with a different configuration.
    step_limit = min(0.5, 15.0 / fps, float(config["robot"]["max_joint_step_deg"]))
    positions, orientations, previous = {side: [] for side in SIDES}, {side: [] for side in SIDES}, None
    for index, frame in enumerate(frames):
        if not np.isfinite(frame.get("time_s", np.nan)) or abs(frame["time_s"] - index / fps) > 1e-5:
            raise ValueError(f"Invalid plan timing at frame {index}")
        joints = _joint_map(frame)
        for side in SIDES:
            item = frame["arms"][side]
            if (
                item.get("valid") is not True
                or item.get("workspace_clamped") is not False
                or item.get("retarget_clamped") is not False
            ):
                raise ValueError(f"Unsafe/limited IK result at frame {index} ({side})")
            arm = simulation.arms[side]
            if np.any(joints[side] < arm.lower[:7] - 1e-6) or np.any(joints[side] > arm.upper[:7] + 1e-6):
                raise ValueError(f"URDF joint limit exceeded at frame {index} ({side})")
            if previous is not None and np.max(np.abs(joints[side] - previous[side])) > step_limit + 1e-6:
                raise ValueError(f"Joint rate limit exceeded at frame {index} ({side})")
            requested = np.asarray(item["target_pose"], dtype=float)
            simulated = np.asarray(item["simulated_pose"], dtype=float)
            if (
                requested.shape != (7,)
                or simulated.shape != (7,)
                or not np.isfinite([requested, simulated]).all()
            ):
                raise ValueError(f"Invalid TCP pose at frame {index} ({side})")
            target = pose(requested[:3], requested[3:])
            expected = pose(simulated[:3], simulated[3:])
            actual_pos, actual_q = arm.solver.fk_solver.compute(np.r_[joints[side], 0.0])
            actual = pose(actual_pos, actual_q)
            if (
                np.linalg.norm(actual_pos - expected[:3, 3]) > 1e-5
                or Rotation.from_matrix(actual[:3, :3] @ expected[:3, :3].T).magnitude() > 1e-4
            ):
                raise ValueError(f"Plan FK does not match the selected URDF at frame {index} ({side})")
            error = float(np.linalg.norm(actual_pos - target[:3, 3]))
            if error > max_sim_error_m:
                raise ValueError(
                    f"Simulated position error {error:.3f} m exceeds limit at frame {index} ({side})"
                )
            orientation_error = float(
                np.rad2deg(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude())
            )
            if (
                max_sim_orientation_error_deg is not None
                and orientation_error > max_sim_orientation_error_deg
            ):
                raise ValueError(
                    f"Simulated orientation error {orientation_error:.1f} deg exceeds limit "
                    f"at frame {index} ({side})"
                )
            positions[side].append(error)
            orientations[side].append(orientation_error)
        if model_kind == "full_mujoco":
            for side in SIDES:
                simulation.data.qpos[simulation.qpos_indices[side][:7]] = np.deg2rad(joints[side])
                simulation.data.qpos[simulation.qpos_indices[side][7]] = 0
            simulation.mujoco.mj_forward(simulation.model, simulation.data)
            collision = simulation.collision_report()
            if (
                collision["count"] != frame["collision"]["count"]
                or abs(collision["max_penetration_m"] - frame["collision"]["max_penetration_m"]) > 1e-9
            ):
                raise ValueError(f"Plan collision results differ from the selected model at frame {index}")
        previous = joints
    return {
        "frames": len(frames),
        "fps": fps,
        "max_joint_step_deg": step_limit,
        "simulated_position_error_m": {side: _summary(positions[side]) for side in SIDES},
        "simulated_orientation_error_deg": {side: _summary(orientations[side]) for side in SIDES},
    }


def arm_only_follower(left_port, right_port, robot_id, max_step, calibration_dir=None):
    """Construct the existing follower using only joint0..joint6 on each bus."""
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.robots.dual_scorpion_follower import DualScorpionFollower, DualScorpionFollowerConfig

    options = DualScorpionFollowerConfig(
        id=robot_id,
        calibration_dir=Path(calibration_dir) if calibration_dir else None,
        left_arm_port=left_port,
        right_arm_port=right_port,
        use_degrees=True,
        max_relative_target=max_step,
        disable_torque_on_disconnect=False,
    )
    robot = DualScorpionFollower(options)
    for side in SIDES:
        previous = getattr(robot, side + "_bus")
        if not set(JOINTS) <= previous.calibration.keys():
            raise ValueError(f"Missing {side} arm calibration for {robot_id}; calibrate before replay")
        bus = FeetechMotorsBus(
            port=previous.port,
            motors={name: previous.motors[name] for name in JOINTS},
            calibration={name: previous.calibration[name] for name in JOINTS},
        )
        setattr(robot, side + "_bus", bus)
    return robot


def _read_joints(robot):
    observation = robot.get_observation()
    result = {}
    for side in SIDES:
        result[side] = np.array([observation[f"{side}_{name}.pos"] for name in JOINTS], dtype=float)
        if not np.isfinite(result[side]).all():
            raise ValueError(f"Invalid {side} encoder readings")
    return result


def _actions(joints):
    return {f"{side}_{name}.pos": float(joints[side][i]) for side in SIDES for i, name in enumerate(JOINTS)}


def _validate_calibration_limits(robot, plan):
    for side in SIDES:
        bus = getattr(robot, side + "_bus")
        for i, name in enumerate(JOINTS):
            calibration = bus.calibration[name]
            resolution = bus.model_resolution_table[bus.motors[name].model]
            half_range = (calibration.range_max - calibration.range_min) * 180 / (resolution - 1)
            if not np.isfinite(half_range) or half_range <= 0:
                raise ValueError(f"Invalid calibration for {side}_{name}")
            if any(abs(frame["arms"][side]["joint_deg"][i]) > half_range + 1e-6 for frame in plan["frames"]):
                raise ValueError(f"Plan exceeds calibrated motor range for {side}_{name}")


def execute_plan(
    plan,
    config,
    simulation,
    robot,
    report_path,
    *,
    max_start_error_deg=2.0,
    max_tracking_error_deg=5.0,
    allow_synthetic=False,
    max_sim_error_m=0.025,
    max_sim_orientation_error_deg=None,
    realtime=True,
):
    """Replay a prevalidated plan; retain partial quantitative results on failure."""
    if not (0 < max_start_error_deg <= 2 and 0 < max_tracking_error_deg <= 5):
        raise ValueError("Start/tracking tolerances must be positive and at most 2/5 degrees")
    validation = validate_plan(
        plan,
        config,
        simulation,
        allow_synthetic=allow_synthetic,
        max_sim_error_m=max_sim_error_m,
        max_sim_orientation_error_deg=max_sim_orientation_error_deg,
    )
    limit = robot.config.max_relative_target
    if (
        not robot.config.use_degrees
        or not isinstance(limit, (int, float))
        or not np.isfinite(limit)
        or not 0 < limit <= validation["max_joint_step_deg"]
    ):
        raise ValueError("Follower must use calibrated degrees and an enabled per-step motion limit")
    _validate_calibration_limits(robot, plan)
    output = Path(report_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "schema": "dual_scorpion.mocopi.real_metrics.v1",
        "status": "starting",
        "source": plan["source"],
        "episode": plan.get("episode"),
        "task": plan.get("task"),
        "plan_sha256": plan_hash(plan),
        "robot_id": getattr(robot, "id", None),
        "urdf_sha256": plan["urdf_sha256"],
        "fps": plan["fps"],
        "gripper_commanded": False,
        "frames": [],
        "failure": None,
        "limits": "TCP estimated from motor encoders and URDF, not an independent physical measurement",
    }
    active = False
    failure = None
    with output.open("x", encoding="utf-8") as file:
        try:
            for side in SIDES:
                getattr(robot, side + "_bus").connect()
            if not robot.is_calibrated:
                raise ValueError("Motor calibration does not match the robot; recalibrate before replay")
            for side in SIDES:
                bus = getattr(robot, side + "_bus")
                if any(bus.read("Operating_Mode", name, normalize=False) != 0 for name in JOINTS):
                    raise ValueError(f"{side} motors must be in position mode")
            current, first = _read_joints(robot), _joint_map(plan["frames"][0])
            for side in SIDES:
                arm = simulation.arms[side]
                if np.any(current[side] < arm.lower[:7] - 1e-6) or np.any(
                    current[side] > arm.upper[:7] + 1e-6
                ):
                    raise ValueError(f"{side} measured position exceeds the selected URDF joint limits")
                if np.max(np.abs(current[side] - first[side])) > max_start_error_deg:
                    raise ValueError(
                        f"{side} robot start pose differs from plan by more than {max_start_error_deg:g} degrees"
                    )
            # First set the present position as the goal, then enable only
            # the seven arm joints. The grippers are never read or written.
            robot.send_action(_actions(current))
            active = True
            for side in SIDES:
                getattr(robot, side + "_bus").enable_torque(list(JOINTS))
            report["status"] = "running"
            start_time = time.monotonic()
            for index, frame in enumerate(plan["frames"]):
                desired = _joint_map(frame)
                limited = {
                    side: np.clip(desired[side], current[side] - limit, current[side] + limit)
                    for side in SIDES
                }
                sent = robot.send_action(_actions(limited))
                if realtime:
                    time.sleep(1 / plan["fps"])
                measured = _read_joints(robot)
                sample = {
                    "frame": index,
                    "time_s": frame["time_s"],
                    "elapsed_s": time.monotonic() - start_time,
                    "arms": {},
                }
                for side in SIDES:
                    sent_deg = np.array([sent[f"{side}_{name}.pos"] for name in JOINTS], dtype=float)
                    if not np.isfinite(sent_deg).all():
                        raise ValueError(f"Invalid {side} motor command")
                    max_error = float(np.max(np.abs(desired[side] - measured[side])))
                    pos, quat = simulation.arms[side].solver.fk_solver.compute(np.r_[measured[side], 0.0])
                    target = frame["arms"][side]["target_pose"]
                    target_pose = pose(target[:3], target[3:])
                    actual_pose = pose(pos, quat)
                    sample["arms"][side] = {
                        "planned_joint_deg": desired[side].tolist(),
                        "sent_joint_deg": sent_deg.tolist(),
                        "measured_joint_deg": measured[side].tolist(),
                        "max_joint_error_deg": max_error,
                        "tcp_position_error_m": float(np.linalg.norm(pos - target_pose[:3, 3])),
                        "tcp_orientation_error_deg": float(
                            np.rad2deg(
                                Rotation.from_matrix(target_pose[:3, :3] @ actual_pose[:3, :3].T).magnitude()
                            )
                        ),
                    }
                report["frames"].append(sample)
                current = measured
                if any(
                    np.any(current[side] < simulation.arms[side].lower[:7] - 1e-6)
                    or np.any(current[side] > simulation.arms[side].upper[:7] + 1e-6)
                    for side in SIDES
                ):
                    raise RuntimeError(f"Measured robot joint exceeds URDF limits at frame {index}: HOLD")
                if any(
                    sample["arms"][side]["max_joint_error_deg"] > max_tracking_error_deg for side in SIDES
                ):
                    raise RuntimeError(f"Robot fell behind the plan at frame {index}: HOLD")
            report["status"] = "complete"
        except BaseException as exc:
            failure = exc
            report["status"], report["failure"] = "aborted", str(exc) or type(exc).__name__
        finally:
            if active and robot.is_connected:
                try:
                    robot.send_action(_actions(_read_joints(robot)))
                except Exception as exc:
                    report["hold_error"] = str(exc)
            for side in SIDES:
                bus = getattr(robot, side + "_bus")
                if bus.is_connected:
                    try:
                        bus.disconnect(disable_torque=False)
                    except Exception as exc:
                        report[f"{side}_disconnect_error"] = str(exc)
            report["tracking"] = {
                side: {
                    metric: _summary([row["arms"][side][metric] for row in report["frames"]])
                    for metric in ("max_joint_error_deg", "tcp_position_error_m", "tcp_orientation_error_deg")
                }
                for side in SIDES
            }
            elapsed = [row["elapsed_s"] for row in report["frames"]]
            report["measured_fps"] = (
                (len(elapsed) - 1) / (elapsed[-1] - elapsed[0])
                if len(elapsed) > 1 and elapsed[-1] > elapsed[0]
                else None
            )
            file.write(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    if failure:
        raise RuntimeError(f"Real replay stopped: {failure}; report saved to {output}") from failure
    return report


def replay_real(
    config,
    plan_path,
    *,
    execute=False,
    left_port=None,
    right_port=None,
    robot_id=None,
    calibration_dir=None,
    report=None,
    allow_synthetic=False,
    max_sim_error_m=0.025,
    max_sim_orientation_error_deg=None,
):
    plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
    full_model = plan.get("model_kind") == "full_mujoco"
    if full_model and execute:
        raise ValueError("Full CAD model has unmeasured limits; this plan is simulation-only")
    if full_model:
        from ..full_mujoco import FullMujocoSimulation

        simulation = FullMujocoSimulation(config["robot"], gui=False)
    else:
        simulation = DualSimulation(config["robot"], gui=False)
    try:
        result = validate_plan(
            plan,
            config,
            simulation,
            allow_synthetic=allow_synthetic,
            max_sim_error_m=max_sim_error_m,
            max_sim_orientation_error_deg=max_sim_orientation_error_deg,
            allow_preview_model=full_model,
        )
        if not execute:
            return {"mode": "validation_only", "hardware_replay_allowed": not full_model, **result}
        if not left_port or not right_port or not robot_id or not report:
            raise ValueError("--execute requires --left-port, --right-port, --robot-id and --report")
        robot = arm_only_follower(
            left_port, right_port, robot_id, result["max_joint_step_deg"], calibration_dir
        )
        return execute_plan(
            plan,
            config,
            simulation,
            robot,
            report,
            allow_synthetic=allow_synthetic,
            max_sim_error_m=max_sim_error_m,
            max_sim_orientation_error_deg=max_sim_orientation_error_deg,
        )
    finally:
        simulation.close()
