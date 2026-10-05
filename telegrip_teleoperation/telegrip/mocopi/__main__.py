"""Guided CLI. Run from repository root: uv run --no-sync python -m telegrip.mocopi."""

import argparse
import json
import logging
import select
import socket
import sys
import time
from contextlib import ExitStack
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import HeadCalibration, calibrate_head
from .camera import UvcCamera, calibrate_images, camera_list, export_stella
from .config import camera_device, load_config
from .geometry import inverse, pose, world_arm_landmarks
from .mounting import mounting_prompts, mounting_summary
from .posture import ArmPostureRetarget, robot_arm_state
from .retarget import Retarget, alignment_rotation
from .runtime import Streams, TrackingGate
from .sources import fake_pair, record_pair


def await_pair(streams, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pair = streams.poll()
        if (
            pair
            and pair[0].valid
            and (pair[1] is None or pair[1].valid)
            and time.monotonic() - pair[0].timestamp < streams.cfg["tracking"]["timeout_s"]
        ):
            return pair
        time.sleep(0.01)
    raise RuntimeError(
        "15秒以内に同期trackingを取得できません。mocopi SEND、skdf、SLAM初期化、ROS bridgeを確認してください"
    )


def head_calibration(cfg, streams, guided=True):
    if guided:
        print(mounting_summary(cfg))
        await_pair(streams)
        print("Step 1: mocopi head/left/right stream + SLAM: OK")
        for step, text in enumerate(mounting_prompts(cfg), 2):
            input(f"Step {step}: {text}。確認してEnter: ")
        input("neutral poseで静止してEnter（腕anchorは追従起動時に取り直します）: ")
        input(
            f"次の{cfg['calibration']['duration_s']}秒、Headを上下左右へ移動し、首を異なる軸でゆっくり回転します。Enterで開始: "
        )
    duration = cfg["calibration"]["duration_s"]
    pairs = []
    start = time.monotonic()
    next_report = start
    while time.monotonic() - start < duration:
        pair = streams.poll()
        now = time.monotonic()
        if (
            pair
            and pair[1]
            and pair[0].valid
            and pair[1].valid
            and max(now - pair[0].timestamp, now - pair[1].timestamp) < cfg["tracking"]["timeout_s"]
        ):
            pairs.append(pair)
        if guided and now >= next_report:
            print(f"残り{max(0, duration - (now - start)):.0f}秒 / 同期sample={len(pairs)}")
            next_report = now + 1
        time.sleep(1 / cfg["tracking"]["hz"])
    settings = {key: value for key, value in cfg["calibration"].items() if key != "duration_s"}
    calibration = calibrate_head(pairs, **settings)
    print(
        f"SLAM tracking: OK\nmocopi tracking: OK\nestimated scale: {calibration.scale:.6f}\n"
        f"trajectory RMSE: {calibration.quality['rmse_m']:.4f} m\n"
        f"rotation RMSE: {calibration.quality['rotation_rmse_deg']:.2f} deg\nhead-camera extrinsic: valid"
    )
    return calibration


def anchors(cfg, simulation, hands, head, sample):
    mappings = {}
    retarget = cfg["retarget"]
    rotation, source = alignment_rotation(
        head, sample, retarget["world_to_robot_rotation"], retarget["alignment_mode"]
    )
    landmarks = world_arm_landmarks(head, sample)
    for side, arm in simulation.arms.items():
        p, q = arm.solver.fk_solver.compute(arm.last_valid_target)
        robot = pose(p, q)
        arm_posture = None
        if retarget["arm_posture"] == "upper-arm":
            if side + "_shoulder" in landmarks and side + "_elbow" in landmarks:
                arm_posture = ArmPostureRetarget(
                    landmarks[side + "_shoulder"],
                    landmarks[side + "_elbow"],
                    robot_arm_state(arm.solver, arm.last_valid_target),
                    rotation,
                    retarget["arm_posture_gain"],
                )
            else:
                print(f"{side}: shoulder/elbow bones unavailable; hand-only IK")
        mappings[side] = Retarget(
            hands[side],
            robot,
            retarget["translation_scale"],
            rotation,
            retarget["orientation_scale"],
            retarget["orientation_enabled"],
            retarget["max_translation_m"],
            arm_posture,
        )
        solver_reset = getattr(arm.solver, "reset_mode_state", None)
        if solver_reset:
            solver_reset()
    print(f"Neutral alignment: {source}; 左右の開始位置・手首・上腕の姿勢差を吸収しました")
    return mappings


def initialize_tracking(cfg, streams, calibration, guided=False):
    pair = await_pair(streams)
    if guided:
        input(
            "左右sensorを固定し、両肘を曲げて両手を体の前に構えてください。静止してEnterで基準合わせ・開始: "
        )
        # Drain packets accumulated while input() paused reception; use the
        # next incoming sample rather than a queued pre-Enter posture.
        streams.poll()
        pair = await_pair(streams)
    # The operator may move while preparing the neutral pose. Establish the
    # continuity baseline from the fresh sample AFTER Enter, not before it.
    gate = TrackingGate(cfg["tracking"], calibration)
    initial = gate.accept(pair, time.monotonic())
    if initial is None:
        raise ValueError(gate.reason)
    return pair, gate, initial


def debug_frames(head, hands, targets, simulation, pair, calibration, postures=None):
    frames = {"mocopi_head": {"parent": "world", "pose": head.tolist()}}
    for name, landmark in world_arm_landmarks(head, pair[0]).items():
        frames[name] = {"parent": "world", "pose": landmark.tolist()}
    for side in ("left", "right"):
        frames[side + "_hand_target"] = {"parent": "world", "pose": hands[side].tolist()}
        frames[side + "_ee_target"] = {"parent": "robot_stand", "pose": targets[side].tolist()}
        frames[side + "_robot_base"] = {"parent": "robot_stand", "pose": np.eye(4).tolist()}
        p, q = simulation.arms[side].solver.fk_solver.compute(simulation.arms[side].last_valid_target)
        frames[side + "_ee_actual"] = {"parent": "robot_stand", "pose": pose(p, q).tolist()}
        _, elbow, _ = robot_arm_state(simulation.arms[side].solver, simulation.arms[side].last_valid_target)
        frames[side + "_elbow_actual"] = {"parent": "robot_stand", "pose": pose(elbow).tolist()}
        if postures and postures.get(side) is not None:
            frames[side + "_elbow_target"] = {
                "parent": "robot_stand",
                "pose": pose(postures[side].elbow).tolist(),
            }
    if 0 in pair[0].bones:
        frames["mocopi_root"] = {
            "parent": "world",
            "pose": (head @ inverse(pair[0].bones[10]) @ pair[0].bones[0]).tolist(),
        }
    if calibration and pair[1]:
        camera = pair[1].camera.copy()
        camera[:3, 3] *= calibration.scale
        frames["slam_map_metric"] = {"parent": "world", "pose": calibration.world_from_map.tolist()}
        frames["head_camera"] = {"parent": "slam_map_metric", "pose": camera.tolist()}
    return frames


def run(cfg, args):
    if args.sim_backend == "mujoco":
        from .mujoco_sim import MujocoSimulation as Simulation
    else:
        from .ik import DualSimulation as Simulation

    if args.start_pose is not None:
        cfg["robot"]["start_pose"] = args.start_pose
    if args.alignment is not None:
        cfg["retarget"]["alignment_mode"] = args.alignment
    if args.arm_posture is not None:
        cfg["retarget"]["arm_posture"] = args.arm_posture
    mode = "fake" if args.fake else "replay" if args.replay else "mocopi-only" if args.mocopi_only else "live"
    if mode == "fake":
        # Calibrate independent deterministic trajectories, not hand-picked scale.
        pairs = [fake_pair(t, t) for t in np.linspace(0, 15, 150)]
        calibration = calibrate_head(pairs)
        print(f"FAKE inputs: estimated scale={calibration.scale:.6f}; simulated arms only")
    elif mode == "mocopi-only":
        calibration = None
        print("MOCOPI-ONLY: mocopiで頭・腕を推定。頭カメラのvSLAM補正は無効です")
    else:
        calibration = HeadCalibration.load(args.calibration or cfg["slam"]["head_calibration"])
        print("HEAD vSLAM: 頭カメラの姿勢を校正してHeadへ変換し、左右の手・肩・肘を補正します")
    streams = Streams(cfg, mode, args.replay)
    simulation = None
    files = ExitStack()
    log = None
    debug = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if mode in ("live", "mocopi-only"):
            print(mounting_summary(cfg, camera=mode == "live"))
        print(
            f"Simulation backend: {args.sim_backend} (kinematic preview), "
            f"start_pose={cfg['robot']['start_pose']}, alignment={cfg['retarget']['alignment_mode']}\n"
            f"Arm posture: {cfg['retarget']['arm_posture']} (手先優先・上腕/肘を補助目標に使用)\n"
            "追従中もEnterで現在の手とロボット姿勢を再基準化できます"
        )
        simulation = Simulation(cfg["robot"], not args.headless)
        pair, gate, initial = initialize_tracking(cfg, streams, calibration, mode in ("live", "mocopi-only"))
        head, hands = initial
        last_accepted = pair
        mappings = anchors(cfg, simulation, hands, head, pair[0])
        start = time.monotonic()
        last_report = start - 1
        if args.record:
            Path(args.record).parent.mkdir(parents=True, exist_ok=True)
            # ExitStack closes this file on every exit path alongside simulation.
            log = files.enter_context(Path(args.record).open("w", encoding="utf-8"))  # noqa: SIM115
            if calibration:
                calibration.save(str(args.record) + ".calibration.yaml")
        while simulation.is_running() and (args.duration is None or time.monotonic() - start < args.duration):
            pair_new = streams.poll()
            tick = time.monotonic()
            if pair_new:
                pair = pair_new
                result = gate.accept(pair, tick)
                if result:
                    head, hands = result
                    last_accepted = pair
                    if log and pair[1]:
                        record_pair(log, start, pair[0], pair[1])
            gate.tick(tick)
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                try:
                    gate.rearm(tick)
                    mappings = anchors(cfg, simulation, hands, head, last_accepted[0])
                    print("Fresh tracking reanchored; simulation resumed")
                except ValueError as exc:
                    gate.lose(str(exc))
                    print(str(exc))
            targets = {side: mapping.target(hands[side]) for side, mapping in mappings.items()}
            postures = {}
            if gate.tracking_valid:
                try:
                    landmarks = world_arm_landmarks(head, last_accepted[0])
                    postures = {
                        side: mapping.posture_target(landmarks, side) for side, mapping in mappings.items()
                    }
                except (ValueError, KeyError) as exc:
                    gate.lose(f"Upper-arm tracking invalid: {exc}")
            simulation.update(targets, gate.tracking_valid, postures)
            if gate.tracking_valid:
                frames = debug_frames(head, hands, targets, simulation, last_accepted, calibration, postures)
                debug.sendto(
                    json.dumps({"frames": frames, "valid": True, "timestamp": tick}).encode(),
                    ("127.0.0.1", cfg["tracking"]["debug_port"]),
                )
            if tick - last_report >= 1:
                fps = streams.mocopi.fps if streams.mocopi else cfg["tracking"]["hz"]
                status = {
                    side: {
                        "valid": arm.valid,
                        "status": arm.status[:160],
                        "joints_deg": np.round(arm.last_valid_target, 3).tolist(),
                        "arm_posture": arm.posture_info,
                    }
                    for side, arm in simulation.arms.items()
                }
                print(
                    json.dumps(
                        {
                            "mocopi_fps": round(fps, 1),
                            "camera_fps": "see ROS bridge",
                            "slam": "fallback" if calibration is None else gate.reason,
                            "tracking_valid": gate.tracking_valid,
                            "last_update_age": gate.age(tick),
                            "head_xyz": np.round(head[:3, 3], 4).tolist(),
                            "left_xyz": np.round(hands["left"][:3, 3], 4).tolist(),
                            "right_xyz": np.round(hands["right"][:3, 3], 4).tolist(),
                            "arm_landmarks_xyz": {
                                name: np.round(t[:3, 3], 4).tolist()
                                for name, t in world_arm_landmarks(head, last_accepted[0]).items()
                            },
                            "IK": status,
                        }
                    )
                )
                last_report = tick
            time.sleep(max(0, 1 / cfg["tracking"]["hz"] - (time.monotonic() - tick)))
    finally:
        debug.close()
        files.close()
        if simulation:
            simulation.close()
        streams.close()


def doctor(cfg):
    import importlib.util

    print(
        json.dumps(
            {
                "cameras": camera_list(),
                "selected_camera": cfg["camera"],
                "udp_mocopi": cfg["mocopi"]["port"],
                "declared_tracking_mode": cfg["mocopi"]["tracking_mode"],
                "udp_slam_local": cfg["slam"]["pose_port"],
                "rclpy_importable": importlib.util.find_spec("rclpy") is not None,
                "intrinsics_exists": Path(cfg["slam"]["intrinsics"]).exists(),
                "head_calibration_exists": Path(cfg["slam"]["head_calibration"]).exists(),
                "urdf": {k: Path(v).exists() for k, v in cfg["robot"]["urdf"].items()},
                "hardware_control": "disabled",
            },
            indent=2,
        )
    )


def build_parser():
    parser = argparse.ArgumentParser(description="Sony mocopi + C270 Dual Scorpion simulation prototype")
    parser.add_argument("--config", default="config/mocopi/tracking.yaml")
    subs = parser.add_subparsers(dest="command", required=True)
    doctor_parser = subs.add_parser("doctor")
    doctor_parser.add_argument(
        "--camera", type=camera_device, help="Camera index or device path; overrides YAML"
    )
    subs.add_parser("camera-list")
    subs.add_parser("mount-check", help="Show sensor mounting and smartphone mode setup")
    check = subs.add_parser("mocopi-check")
    check.add_argument("--duration", type=float, default=15.0)
    capture = subs.add_parser("camera-capture", help="Save calibration images (headless OpenCV works)")
    capture.add_argument("--camera", type=camera_device, help="Camera index or device path; overrides YAML")
    capture.add_argument("--output", default="outputs/mocopi/checkerboard")
    capture.add_argument("--count", type=int, default=20)
    capture.add_argument("--interval", type=float, default=2.0)
    camera = subs.add_parser("calibrate-camera")
    camera.add_argument("--images", default="outputs/mocopi/checkerboard")
    camera.add_argument("--board", nargs=2, type=int, default=[9, 6], help="INNER corner counts")
    camera.add_argument("--square-m", type=float, default=0.025)
    head = subs.add_parser("calibrate-head")
    head.add_argument("--fake", action="store_true")
    execute = subs.add_parser("run")
    execute.add_argument("--headless", action="store_true")
    execute.add_argument("--duration", type=float)
    execute.add_argument("--record")
    execute.add_argument("--calibration", help="Override calibration, e.g. replay.jsonl.calibration.yaml")
    execute.add_argument(
        "--start-pose", choices=["work", "backwards", "config"], help="Robot neutral preset; overrides YAML"
    )
    execute.add_argument(
        "--alignment", choices=["body", "world"], help="Neutral body heading or configured world axes"
    )
    execute.add_argument(
        "--arm-posture",
        choices=["upper-arm", "hand-only"],
        help="Soft elbow/upper-arm IK guidance, or the previous hand-only IK",
    )
    execute.add_argument(
        "--sim-backend",
        choices=["mujoco", "pybullet"],
        default="mujoco",
        help="Simulation viewer (default: mujoco)",
    )
    source = execute.add_mutually_exclusive_group()
    source.add_argument("--fake", action="store_true")
    source.add_argument("--replay")
    source.add_argument("--mocopi-only", action="store_true", help="Disable head-camera vSLAM correction")
    backend = execute.add_mutually_exclusive_group()
    backend.add_argument("--sim", action="store_true", help="Default; kinematic simulation preview")
    backend.add_argument("--hardware", action="store_true", help="Not implemented; always rejected")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s: %(message)s")
    if getattr(args, "hardware", False):
        parser.error("Hardware control is disabled in this MVP. Use --sim.")
    try:
        cfg = load_config(args.config, camera=getattr(args, "camera", None))
        if args.command == "doctor":
            doctor(cfg)
        elif args.command == "camera-list":
            print(json.dumps(camera_list(), indent=2))
        elif args.command == "mount-check":
            print(mounting_summary(cfg))
        elif args.command == "mocopi-check":
            print(mounting_summary(cfg))
            streams = Streams(cfg, "mocopi-only")
            try:
                start, report, count = time.monotonic(), 0.0, 0
                while time.monotonic() - start < args.duration:
                    pair = streams.poll()
                    if pair:
                        count += 1
                    if pair and time.monotonic() - report > 1:
                        print(
                            f"mocopi FPS={streams.mocopi.fps:.1f} "
                            + str(
                                {
                                    k: pair[0].bones[k][:3, 3].round(3).tolist()
                                    for k in (10, 12, 13, 14, 16, 17, 18)
                                }
                            )
                            + " upper_arm_rotation_deg="
                            + str(
                                {
                                    side: Rotation.from_matrix(pair[0].bones[bone][:3, :3])
                                    .as_rotvec(degrees=True)
                                    .round(1)
                                    .tolist()
                                    for side, bone in (("left", 12), ("right", 16))
                                }
                            )
                        )
                        report = time.monotonic()
                    time.sleep(0.01)
                if not count:
                    raise RuntimeError("mocopi motion frameなし。PC IPv4/12351/mocopi(UDP)/SEND/skdfを確認")
            finally:
                streams.close()
        elif args.command == "camera-capture":
            import cv2

            directory = Path(args.output)
            directory.mkdir(parents=True, exist_ok=True)
            camera = UvcCamera(cfg["camera"])
            try:
                print(f"Camera: {cfg['camera']['device']}")
                input(
                    "9x6内角checkerboardを用意してください。Enter後、2秒ごとに撮影します。位置/傾き/距離を変えてください: "
                )
                for i in range(args.count):
                    deadline = time.monotonic() + args.interval
                    image = None
                    while time.monotonic() < deadline or image is None:
                        _, image = camera.read()
                    path = directory / f"checkerboard_{time.time_ns()}_{i:03}.png"
                    if not cv2.imwrite(str(path), image):
                        raise RuntimeError(f"Failed to save {path}")
                    print(str(path))
            finally:
                camera.close()
        elif args.command == "calibrate-camera":
            import cv2

            files = sorted(Path(args.images).glob("*.png"))
            images = [cv2.imread(str(path)) for path in files]
            if not images or any(image is None for image in images):
                raise ValueError("No readable PNG images")
            print(
                json.dumps(
                    calibrate_images(images, args.board, args.square_m, cfg["slam"]["intrinsics"]), indent=2
                )
            )
            export_stella(cfg["slam"]["intrinsics"], cfg["camera"]["fps"], cfg["slam"]["stella_config"])
        elif args.command == "calibrate-head":
            streams = Streams(cfg, "fake" if args.fake else "live")
            try:
                result = head_calibration(cfg, streams, not args.fake)
                output = cfg["slam"]["head_calibration"]
                if args.fake:
                    output = str(Path(output).with_name("fake_head_tracking.yaml"))
                result.save(output)
                print("Saved: " + output)
            finally:
                streams.close()
        elif args.command == "run":
            run(cfg, args)
    except KeyboardInterrupt:
        print("Stopped. Simulation closed; motor hardware was never connected.")
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        parser.exit(
            2,
            f"Error: {exc}\n校正不良の場合は保存しません。HOW_TO_USE.mdのtroubleshootingを確認してください。\n",
        )


if __name__ == "__main__":
    main()
