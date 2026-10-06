"""Hardware-free math, protocol, calibration and existing-IK integration checks."""

import struct
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "telegrip_teleoperation"))
from telegrip.mocopi.calibration import HeadCalibration, calibrate_head, umeyama
from telegrip.mocopi.config import load_config
from telegrip.mocopi.geometry import MocopiSample, SlamSample, inverse, pose, sony_pose, world_hands
from telegrip.mocopi.receiver import PARENTS, MocopiDecoder
from telegrip.mocopi.retarget import Retarget
from telegrip.mocopi.runtime import TrackingGate
from telegrip.mocopi.sources import Replay, SampleBuffer, fake_pair, record_pair

CONFIG = Path(__file__).resolve().parents[2] / "config/mocopi/tracking.yaml"


@pytest.mark.parametrize("device", ["0", "/dev/video2", "/dev/v4l/by-id/head-camera"])
def test_camera_selection_reaches_opencv_without_changing_yaml(device, monkeypatch):
    from telegrip.mocopi.__main__ import build_parser
    from telegrip.mocopi.camera import UvcCamera
    from telegrip.mocopi.ros_bridge import build_parser as bridge_parser

    from lerobot.cameras.opencv import camera_opencv

    original = CONFIG.read_text()
    expected = 0 if device == "0" else device
    configs = []

    class CameraStub:
        def __init__(self, config):
            configs.append(config)
            self.is_connected = False

        def connect(self):
            self.is_connected = True

        def disconnect(self):
            self.is_connected = False

    monkeypatch.setattr(camera_opencv, "OpenCVCamera", CameraStub)
    for parser, prefix in (
        (build_parser(), ["camera-capture"]),
        (build_parser(), ["doctor"]),
        (bridge_parser(), []),
    ):
        args = parser.parse_args([*prefix, "--camera", device])
        cfg = load_config(CONFIG, camera=args.camera)
        assert cfg["camera"]["device"] == expected
        camera = UvcCamera(cfg["camera"])
        assert configs[-1].index_or_path == (expected if isinstance(expected, int) else Path(expected))
        camera.close()
    assert CONFIG.read_text() == original
    assert load_config(CONFIG)["camera"]["device"] == "/dev/video2"


@pytest.mark.parametrize("device", ["-1", "", " "])
def test_invalid_camera_selection_is_rejected(device):
    from telegrip.mocopi.__main__ import build_parser
    from telegrip.mocopi.ros_bridge import build_parser as bridge_parser

    for parser, prefix in ((build_parser(), ["camera-capture"]), (bridge_parser(), [])):
        with pytest.raises(SystemExit) as exc:
            parser.parse_args([*prefix, "--camera", device])
        assert exc.value.code == 2


def test_camera_failure_suggests_capture_device_of_same_camera(monkeypatch):
    from telegrip.mocopi import camera as camera_module

    from lerobot.cameras.opencv import camera_opencv

    class ClosedCamera:
        is_connected = False

        def __init__(self, config):
            self.config = config

        def connect(self):
            raise ConnectionError("non-capture node")

    monkeypatch.setattr(camera_opencv, "OpenCVCamera", ClosedCamera)
    monkeypatch.setattr(
        camera_module,
        "camera_list",
        lambda: [
            {"id": "/dev/video0", "name": "Internal webcam"},
            {"id": "/dev/video2", "name": "C270 HD WEBCAM"},
            {"id": "/dev/video3", "name": "C270 HD WEBCAM", "error": "cannot open"},
        ],
    )
    cfg = load_config(CONFIG, camera="/dev/video3")
    with pytest.raises(RuntimeError, match="--camera /dev/video2"):
        camera_module.UvcCamera(cfg["camera"])


def test_se3_inverse_composition():
    a = pose([1, 2, 3], Rotation.from_rotvec([0.3, 0.2, -0.1]).as_quat())
    b = pose([-0.2, 0.5, 0.1])
    np.testing.assert_allclose(inverse(a) @ a, np.eye(4), atol=1e-12)
    np.testing.assert_allclose(inverse(a @ b), inverse(b) @ inverse(a), atol=1e-12)


def test_head_relative_world_and_left_right():
    head = pose([2, 3, 4], Rotation.from_rotvec([0.2, 0, 0]).as_quat())
    left, right = pose([0, 0.3, -0.2]), pose([0, -0.3, -0.2])
    sample = MocopiSample(1, {10: head, 14: head @ left, 18: head @ right})
    w = pose([-0.2, 0, 1])
    hands = world_hands(w, sample)
    np.testing.assert_allclose(hands["left"], w @ left, atol=1e-12)
    np.testing.assert_allclose(hands["right"], w @ right, atol=1e-12)
    assert hands["left"][1, 3] > hands["right"][1, 3]


def test_umeyama_scale_and_world_alignment():
    points = np.random.default_rng(1).normal(size=(80, 3))
    rot = Rotation.from_rotvec([0.2, -0.3, 0.4]).as_matrix()
    expected = 2.4 * (points @ rot.T) + [1, 2, 3]
    scale, rotation, translation = umeyama(points, expected)
    assert scale == pytest.approx(2.4)
    np.testing.assert_allclose(rotation, rot, atol=1e-12)
    np.testing.assert_allclose(translation, [1, 2, 3], atol=1e-12)


def test_joint_scale_extrinsic_calibration_and_reload(tmp_path):
    pairs = [fake_pair(t, t) for t in np.linspace(0, 15, 180)]
    # Non-identity global alignment: test both sides of the hand-eye equation.
    align = pose([0.3, -0.2, 0.4], Rotation.from_rotvec([0.35, -0.4, 0.3]).as_quat())
    for m, _ in pairs:
        m.bones = {bone: align @ t for bone, t in m.bones.items()}
    calibration = calibrate_head(pairs)
    assert calibration.scale == pytest.approx(1.7, abs=1e-5)
    assert calibration.quality["rmse_m"] < 1e-6
    np.testing.assert_allclose(calibration.world_from_map, align, atol=1e-5)
    for m, s in pairs[::20]:
        np.testing.assert_allclose(calibration.head(s.camera), m.bones[10], atol=1e-5)
    path = tmp_path / "head.yaml"
    calibration.save(path)
    restored = HeadCalibration.load(path)
    np.testing.assert_allclose(restored.head(pairs[-1][1].camera), pairs[-1][0].bones[10], atol=1e-5)


def test_calibration_rejects_stationary_and_single_axis():
    stationary = [fake_pair(0, t) for t in np.linspace(0, 15, 100)]
    with pytest.raises(ValueError, match="平行移動"):
        calibrate_head(stationary)
    single = []
    for t in np.linspace(0, 15, 100):
        h = pose([0.1 * np.sin(t), 0, 1.5], Rotation.from_rotvec([0, 0, 0.2 * np.sin(t)]).as_quat())
        single.append((MocopiSample(t, {10: h}), SlamSample(t, h)))
    with pytest.raises(ValueError, match="2軸"):
        calibrate_head(single)


def test_calibration_rejects_noise_and_map_change():
    pairs = [fake_pair(t, t) for t in np.linspace(0, 15, 100)]
    pairs[-1][1].session = "new-map"
    with pytest.raises(ValueError, match="Map"):
        calibrate_head(pairs)
    pairs[-1][1].session = "fake"
    for i, (m, _) in enumerate(pairs):
        m.bones[10][:3, 3] += [0, 0.2 * (-1) ** i, 0]
    with pytest.raises(ValueError, match="rejected"):
        calibrate_head(pairs)


def test_retarget_anchor_scale_rotation_and_workspace():
    h0 = pose([3, 4, 5])
    r0 = pose([0.2, 0.3, 0.4])
    a = Rotation.from_rotvec([0, 0, np.pi / 2]).as_matrix()
    retarget = Retarget(h0, r0, 0.8, a, orientation_scale=0.5)
    np.testing.assert_allclose(retarget.target(h0), r0)
    h1 = pose([3.1, 4, 5], Rotation.from_rotvec([0, 0, 0.4]).as_quat())
    out = retarget.target(h1)
    np.testing.assert_allclose(out[:3, 3], [0.2, 0.38, 0.4], atol=1e-12)
    assert Rotation.from_matrix(out[:3, :3]).magnitude() == pytest.approx(0.2)
    out = retarget.target(pose([30, 40, 50]))
    assert np.linalg.norm(out[:3, 3] - r0[:3, 3]) == pytest.approx(0.35)


@pytest.mark.parametrize("heading_deg", [0, 90, -135])
def test_body_alignment_absorbs_heading_position_and_wrist_pose_offsets(heading_deg):
    from telegrip.mocopi.retarget import alignment_rotation

    config = load_config(CONFIG)
    transform = pose([1, -2, 0.4], Rotation.from_euler("z", heading_deg, degrees=True).as_quat())
    sample, _ = fake_pair(0, 10)
    sample.bones = {bone: transform @ value for bone, value in sample.bones.items()}
    head = sample.bones[10]
    frame, _ = alignment_rotation(head, sample, config["retarget"]["world_to_robot_rotation"])
    operator_rotation = transform[:3, :3]
    expected_rotation = np.array(config["retarget"]["world_to_robot_rotation"])
    for bone, side in ((14, "left"), (18, "right")):
        human = sample.bones[bone].copy()
        human[:3, :3] = Rotation.from_euler("xyz", [80, -45, 130], degrees=True).as_matrix()
        robot = pose([0.25 if side == "left" else -0.25, -0.35, 0.25], [0.2, 0.3, -0.4, 0.8])
        mapping = Retarget(human, robot, 0.8, frame, orientation_scale=0.5)
        np.testing.assert_allclose(mapping.target(human), robot, atol=1e-12)
        for axis in range(3):
            moved = human.copy()
            offset = np.eye(3)[axis] * 0.025
            moved[:3, 3] += operator_rotation @ offset
            target = mapping.target(moved)
            np.testing.assert_allclose(
                target[:3, 3] - robot[:3, 3], 0.8 * expected_rotation @ offset, atol=1e-12
            )
        moved = human.copy()
        operator_roll = operator_rotation @ np.array([0.2, 0, 0])
        moved[:3, :3] = Rotation.from_rotvec(operator_roll).as_matrix() @ human[:3, :3]
        target = mapping.target(moved)
        expected_delta = Rotation.from_rotvec(expected_rotation @ np.array([0.1, 0, 0])).as_matrix()
        np.testing.assert_allclose(target[:3, :3], expected_delta @ robot[:3, :3], atol=1e-12)


def test_body_alignment_root_fallback_and_degenerate_shoulders():
    from telegrip.mocopi.retarget import alignment_rotation

    cfg = load_config(CONFIG)
    sample, _ = fake_pair(0, 10)
    sample.bones[16][:3, 3] = sample.bones[12][:3, 3]
    with pytest.raises(ValueError, match="body heading from shoulders"):
        alignment_rotation(sample.bones[10], sample, cfg["retarget"]["world_to_robot_rotation"])
    del sample.bones[12], sample.bones[16]
    frame, source = alignment_rotation(sample.bones[10], sample, cfg["retarget"]["world_to_robot_rotation"])
    assert "root" in source
    np.testing.assert_allclose(frame, cfg["retarget"]["world_to_robot_rotation"])


def test_neutral_reanchor_keeps_current_robot_pose_and_updates_body_heading():
    from telegrip.mocopi.__main__ import anchors
    from telegrip.mocopi.geometry import world_hands
    from telegrip.mocopi.ik import DualSimulation

    pytest.importorskip("pybullet")
    cfg = load_config(CONFIG)
    sim = DualSimulation(cfg["robot"], gui=False)
    try:
        m, _ = fake_pair(0, 10)
        head = m.bones[10]
        hands = world_hands(head, m)
        mappings = anchors(cfg, sim, hands, head, m)
        for side, mapping in mappings.items():
            np.testing.assert_allclose(mapping.target(hands[side]), sim.initial_ee[side], atol=1e-12)
            assert sim.initial_ee[side][1, 3] < -0.2
            assert sim.initial_ee[side][2, 3] > 0.1
        targets = {side: target.copy() for side, target in sim.initial_ee.items()}
        for target in targets.values():
            target[:3, 3] += [0, 0, 0.012]
        for _ in range(50):
            sim.update(targets)
        current = {side: arm.last_valid_target.copy() for side, arm in sim.arms.items()}
        turn = pose([0.5, -0.3, 0], Rotation.from_euler("z", 90, degrees=True).as_quat())
        m.bones = {bone: turn @ value for bone, value in m.bones.items()}
        head = m.bones[10]
        hands = world_hands(head, m)
        mappings = anchors(cfg, sim, hands, head, m)
        for side, mapping in mappings.items():
            position, quaternion = sim.arms[side].solver.fk_solver.compute(current[side])
            np.testing.assert_allclose(mapping.target(hands[side]), pose(position, quaternion), atol=1e-12)
            np.testing.assert_array_equal(sim.arms[side].last_valid_target, current[side])
            from telegrip.mocopi.geometry import world_arm_landmarks
            from telegrip.mocopi.posture import robot_arm_state

            posture = mapping.posture_target(world_arm_landmarks(head, m), side)
            _, elbow, upper = robot_arm_state(sim.arms[side].solver, current[side])
            np.testing.assert_allclose(posture.elbow, elbow, atol=1e-8)
            np.testing.assert_allclose(posture.upper_rotation, upper, atol=1e-8)
    finally:
        sim.close()


def chunk(tag, payload):
    return struct.pack("<I4s", len(payload), tag) + payload


def packet(skeleton=False, number=1, rotations=None):
    bones = []
    for i in range(27):
        # Native Y-up: metre unit, skeleton offset retained on motion.
        xyz = (0.0, 1.0, 0.0) if skeleton else (0.0, 99.0, 0.0)
        quaternion = (rotations or {}).get(i, (0.0, 0.0, 0.0, 1.0))
        bones.append(
            chunk(
                b"bndt" if skeleton else b"btdt",
                chunk(b"bnid", struct.pack("<H", i)) + chunk(b"tran", struct.pack("<7f", *quaternion, *xyz)),
            )
        )
    head = chunk(b"head", chunk(b"ftyp", b"mmfd") + chunk(b"vrsn", struct.pack("<I", 1)))
    if skeleton:
        return head + chunk(b"skdf", chunk(b"bons", b"".join(bones)))
    return head + chunk(b"fram", chunk(b"fnum", struct.pack("<I", number))) + chunk(b"btrs", b"".join(bones))


def test_official_chunk_layout_hierarchy_and_reordered_packets():
    decoder = MocopiDecoder()
    with pytest.raises(ValueError, match="skdf"):
        decoder.decode(packet(), 0)
    assert decoder.decode(packet(True), 0) is None
    frame = decoder.decode(packet(), 1)
    assert frame.bones[0][2, 3] == pytest.approx(99)
    assert frame.bones[10][2, 3] == pytest.approx(109)
    assert frame.bones[14][2, 3] == pytest.approx(110)
    assert PARENTS[11] == PARENTS[15] == 7
    assert decoder.decode(packet(number=1), 2) is None
    assert decoder.decode(packet(number=0), 2) is None
    with pytest.raises(ValueError):
        decoder.decode(packet()[:-1], 2)


def test_sony_axes_quaternion_change_of_basis():
    t = sony_pose([0, 0, 0, 1, 1, 2, 3])
    np.testing.assert_allclose(t[:3, 3], [3, 1, 2])
    raw_q = Rotation.from_rotvec([0, 0.3, 0]).as_quat()
    t = sony_pose([*raw_q, 0, 0, 0])
    np.testing.assert_allclose(Rotation.from_matrix(t[:3, :3]).as_rotvec(), [0, 0, 0.3], atol=1e-12)


def test_timestamp_nearest_bounded_and_replay(tmp_path):
    buffer = SampleBuffer(2)
    m, s = fake_pair(1.0, 10.0)
    assert buffer.add(m)
    assert not buffer.add(m)
    assert buffer.nearest(10.2, 0.08) is None
    assert buffer.nearest(10.02, 0.08) is m
    path = tmp_path / "replay.jsonl"
    with path.open("w") as f:
        record_pair(f, 10, m, s)
        m2, s2 = fake_pair(2, 11)
        record_pair(f, 10, m2, s2)
    replay = Replay(path)
    first = replay.poll(200)
    assert first[0].timestamp == 200
    assert replay.poll(200.1) is None
    assert replay.poll(201)[0].timestamp == 201


def test_tracking_timeout_latches_and_reanchor():
    cfg = load_config(CONFIG)["tracking"]
    gate = TrackingGate(cfg)
    m, _ = fake_pair(0, 10)
    assert gate.accept((m, None), 10)
    assert gate.tick(10)
    assert not gate.tick(11)
    assert gate.latched
    m, _ = fake_pair(0.1, 11)
    assert gate.accept((m, None), 11)
    assert not gate.tracking_valid
    gate.rearm(11)
    assert gate.tracking_valid
    gate.tick(12)
    with pytest.raises(ValueError):
        gate.rearm(12)


def test_startup_uses_neutral_pose_after_enter_and_still_rejects_jumps(monkeypatch):
    from types import SimpleNamespace

    from telegrip.mocopi import __main__ as cli

    cfg = load_config(CONFIG)
    before, _ = fake_pair(0, 10)
    after, _ = fake_pair(0, 12)
    after.bones[14][:3, 3] += [0.6, 0, 0]
    sequence = iter([(before, None), (after, None)])
    prompts = []
    discarded = []
    streams = SimpleNamespace(poll=lambda: discarded.append((before, None)))
    monkeypatch.setattr(cli, "await_pair", lambda streams: next(sequence))
    monkeypatch.setattr("builtins.input", lambda text: prompts.append(text))
    monkeypatch.setattr(cli.time, "monotonic", lambda: 12)
    pair, gate, (_, hands) = cli.initialize_tracking(cfg, streams, None, guided=True)
    assert prompts and pair[0] is after and gate.tracking_valid
    assert len(discarded) == 1
    np.testing.assert_allclose(hands["left"], after.bones[14])
    after.bones[14][:3, 3] += [0.6, 0, 0]
    after.timestamp = 12.03
    assert gate.accept((after, None), 12.03) is None
    assert gate.latched and "left hand pose discontinuity" in gate.reason


def test_startup_after_enter_still_checks_saved_alignment(monkeypatch):
    from types import SimpleNamespace

    from telegrip.mocopi import __main__ as cli

    cfg = load_config(CONFIG)
    m, s = fake_pair(0, 12)

    class Calibration:
        session = s.session
        intrinsics_id = s.intrinsics_id

        def head(self, camera):
            wrong = m.bones[10].copy()
            wrong[:3, 3] += [1, 0, 0]
            return wrong

    monkeypatch.setattr(cli, "await_pair", lambda streams: (m, s))
    monkeypatch.setattr("builtins.input", lambda text: None)
    monkeypatch.setattr(cli.time, "monotonic", lambda: 12)
    with pytest.raises(ValueError, match="Saved map alignment inconsistent"):
        cli.initialize_tracking(cfg, SimpleNamespace(poll=lambda: None), Calibration(), guided=True)


def test_ik_invalid_handling_and_joint_jump():
    pytest.importorskip("pybullet")
    from telegrip.mocopi.ik import SafeIK

    class Solver:
        last_solve_info = {"residual_final_m": 0.0}
        result = np.zeros(7)

        def solve(self, *args):
            return self.result

    solver = Solver()
    safe = SafeIK(solver, np.zeros(8), np.full(8, -180), np.full(8, 180))
    assert np.isfinite(safe.solve(pose())).all()
    assert safe.valid
    for bad in (np.full(7, np.nan), np.full(7, 100), np.zeros(6)):
        solver.result = bad
        np.testing.assert_array_equal(safe.solve(pose()), np.zeros(8))
        assert not safe.valid
    solver.result = np.zeros(7)
    solver.last_solve_info = {"error": "singular"}
    safe.solve(pose())
    assert not safe.valid


def test_existing_dual_ik_both_arms_move_and_hold():
    pytest.importorskip("pybullet")
    from telegrip.mocopi.ik import DualSimulation

    config = load_config(CONFIG)
    sim = DualSimulation(config["robot"], gui=False)
    try:
        before = {side: arm.last_valid_target.copy() for side, arm in sim.arms.items()}
        targets = {side: t.copy() for side, t in sim.initial_ee.items()}
        for t in targets.values():
            t[:3, 3] += [0.0, 0.0, 0.012]
        for _ in range(50):
            sim.update(targets)
        for side, arm in sim.arms.items():
            assert arm.valid, arm.status
            assert np.linalg.norm(arm.last_valid_target - before[side]) > 0.01
        held = {side: arm.last_valid_target.copy() for side, arm in sim.arms.items()}
        sim.update(targets, valid=False)
        for side, arm in sim.arms.items():
            np.testing.assert_array_equal(arm.last_valid_target, held[side])
    finally:
        sim.close()


def test_mujoco_urdf_frames_match_existing_fk_for_both_arms():
    pytest.importorskip("mujoco")
    pytest.importorskip("pybullet")
    from telegrip.mocopi.mujoco_sim import MujocoSimulation

    cfg = load_config(CONFIG)
    sources = {side: Path(path).read_bytes() for side, path in cfg["robot"]["urdf"].items()}
    source_visuals = Path(cfg["robot"]["visual_urdf"]).read_bytes()
    sim = MujocoSimulation(cfg["robot"], gui=False)
    try:
        assert sim.model.nq == 16 and sim.model.nmocap == 4
        # 17 CAD meshes: shared stand plus eight links per arm. No capsules.
        assert sim.model.nmesh == 17
        assert sim.model.ngeom == 20
        for fraction in (0.0, 0.25, 0.75):
            for arm in sim.arms.values():
                arm.last_valid_target[:] = arm.lower + fraction * (arm.upper - arm.lower)
            sim._sync_state()
            for side, arm in sim.arms.items():
                position, quaternion = arm.solver.fk_solver.compute(arm.last_valid_target)
                body = sim.data.body(f"{side}/tcp_link")
                np.testing.assert_allclose(body.xpos, position, atol=2e-6)
                np.testing.assert_allclose(
                    body.xmat.reshape(3, 3), Rotation.from_quat(quaternion).as_matrix(), atol=2e-6
                )
                np.testing.assert_allclose(
                    sim.data.qpos[sim.qpos_indices[side]], np.deg2rad(arm.last_valid_target)
                )
    finally:
        sim.close()
    for side, path in cfg["robot"]["urdf"].items():
        assert Path(path).read_bytes() == sources[side]
    assert Path(cfg["robot"]["visual_urdf"]).read_bytes() == source_visuals


def test_mujoco_both_arms_move_and_hold_on_tracking_loss_and_invalid_ik():
    pytest.importorskip("mujoco")
    pytest.importorskip("pybullet")
    from telegrip.mocopi.mujoco_sim import MujocoSimulation

    sim = MujocoSimulation(load_config(CONFIG)["robot"], gui=False)
    try:
        before = sim.data.qpos.copy()
        targets = {side: target.copy() for side, target in sim.initial_ee.items()}
        for target in targets.values():
            target[:3, 3] += [0, 0, 0.012]
        for _ in range(50):
            sim.update(targets)
        for side, arm in sim.arms.items():
            assert arm.valid, arm.status
            indices = sim.qpos_indices[side]
            assert np.linalg.norm(sim.data.qpos[indices] - before[indices]) > 1e-4
        held = sim.data.qpos.copy()
        sim.update(targets, valid=False)
        np.testing.assert_array_equal(sim.data.qpos, held)
        for target in targets.values():
            target[0, 3] = np.nan
        sim.update(targets)
        assert all(not arm.valid for arm in sim.arms.values())
        np.testing.assert_array_equal(sim.data.qpos, held)
    finally:
        sim.close()


def test_tracking_rejects_map_change_and_pose_jump():
    cfg = load_config(CONFIG)["tracking"]
    pairs = [fake_pair(t, t) for t in np.linspace(0, 15, 100)]
    calibration = calibrate_head(pairs)
    gate = TrackingGate(cfg, calibration)
    m, s = fake_pair(0.0, 10.0)
    assert gate.accept((m, s), 10.0)
    s.session = "different-map"
    assert gate.accept((m, s), 10.0) is None
    assert gate.latched and not gate.tracking_valid
    gate = TrackingGate(cfg)
    m, _ = fake_pair(0.0, 10.0)
    gate.accept((m, None), 10.0)
    m, _ = fake_pair(0.0, 10.03)
    m.bones[14][:3, 3] += [1.0, 0.0, 0.0]
    assert gate.accept((m, None), 10.03) is None
    assert "left hand pose discontinuity" in gate.reason


def test_slam_transport_malformed_stale_and_basis():
    import json
    import socket
    import time

    from telegrip.mocopi.slam import SlamReceiver

    receiver = SlamReceiver(0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = receiver.socket.getsockname()
    try:
        sender.sendto(b"not-json", destination)
        row = {
            "schema": 1,
            "convention": "T_S_C_ros",
            "timestamp": time.monotonic(),
            "position": [1, 2, 3],
            "quaternion": [0, 0, 0, 1],
            "valid": True,
            "session": "map1",
            "intrinsics_id": "id",
        }
        sender.sendto(json.dumps(row).encode(), destination)
        samples = receiver.poll()
        assert receiver.invalid == 1
        assert len(samples) == 1
        np.testing.assert_array_equal(samples[0].camera[:3, 3], [1, 2, 3])
        row["timestamp"] = time.monotonic() + 10
        sender.sendto(json.dumps(row).encode(), destination)
        assert not receiver.poll()
        assert receiver.invalid == 2
    finally:
        sender.close()
        receiver.close()


def test_intrinsics_export_and_resolution_validation(tmp_path):
    import yaml
    from telegrip.mocopi.camera import export_stella, intrinsics_id, load_intrinsics

    path = tmp_path / "intrinsics.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "fx": 540.0,
                "fy": 535.0,
                "cx": 320.0,
                "cy": 240.0,
                "distortion": [0.1, -0.01, 0, 0, 0],
                "resolution": [640, 480],
            }
        )
    )
    _, k, d = load_intrinsics(path, [640, 480])
    assert k[0, 0] == 540 and d[0] == 0.1
    with pytest.raises(ValueError, match="resolution"):
        load_intrinsics(path, [1280, 720])
    output = tmp_path / "stella.yaml"
    export_stella(path, 30, output)
    config = yaml.safe_load(output.read_text())["Camera"]
    assert config["fx"] == 540
    assert config["k1"] == 0  # images are rectified exactly once in bridge
    assert intrinsics_id(path) == intrinsics_id(path)


def test_ik_rejects_nan_residual():
    pytest.importorskip("pybullet")
    from telegrip.mocopi.ik import SafeIK

    class Solver:
        last_solve_info = {"residual_final_m": np.nan}

        def solve(self, *args):
            return np.full(7, 0.1)

    safe = SafeIK(Solver(), np.zeros(8), np.full(8, -180), np.full(8, 180))
    np.testing.assert_array_equal(safe.solve(pose()), np.zeros(8))
    assert not safe.valid


def test_numerical_import_does_not_load_robot_driver():
    import os
    import subprocess

    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import telegrip.mocopi.geometry; "
            "assert 'telegrip.core.robot_interface' not in sys.modules; "
            "assert 'telegrip.control_loop' not in sys.modules",
        ],
        env={**os.environ, "PYTHONPATH": str(CONFIG.parents[2] / "telegrip_teleoperation")},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_checkerboard_intrinsic_calibration_from_rendered_images(tmp_path):
    cv2 = pytest.importorskip("cv2")
    from telegrip.mocopi.camera import calibrate_images

    k = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
    rng = np.random.default_rng(5)
    images = []
    for _ in range(24):
        image = np.full((480, 640, 3), 220, dtype=np.uint8)
        rvec = rng.uniform(-0.4, 0.4, 3)
        tvec = np.array([rng.uniform(-0.22, 0), rng.uniform(-0.17, 0.02), rng.uniform(0.6, 0.85)])
        for x in range(-1, 9):
            for y in range(-1, 6):
                points = (
                    np.array([[x, y, 0], [x + 1, y, 0], [x + 1, y + 1, 0], [x, y + 1, 0]], dtype=float)
                    * 0.025
                )
                projected, _ = cv2.projectPoints(points, rvec, tvec, k, np.zeros(5))
                corners = np.round(projected[:, 0]).astype(np.int32)
                color = (0, 0, 0) if (x + y) % 2 == 0 else (255, 255, 255)
                cv2.fillConvexPoly(image, corners, color)
        images.append(image)
    output = tmp_path / "calibration.yaml"
    result = calibrate_images(images, [9, 6], 0.025, output)
    assert output.exists()
    assert result["rms_px"] < 1.0
    assert result["fx"] == pytest.approx(600, rel=0.08)
    assert result["fy"] == pytest.approx(600, rel=0.08)


def test_external_stella_tracking_guard_is_strict_and_idempotent(tmp_path):
    from telegrip.mocopi.prepare_stella import FUNCTION, GUARD, HEADER, prepare

    source = tmp_path / "src/stella_vslam_ros.cc"
    source.parent.mkdir()
    source.write_text("#include <stella_vslam/publish/map_publisher.h>\n" + FUNCTION + "\n}\n")
    assert prepare(tmp_path)
    assert FUNCTION + GUARD in source.read_text()
    assert HEADER in source.read_text()
    assert not prepare(tmp_path)
    source.write_text("unrecognized upstream version")
    with pytest.raises(ValueError, match="Unsupported"):
        prepare(tmp_path)
    assert source.read_text() == "unrecognized upstream version"


def test_upper_arm_rotation_drives_its_hand_without_ankle_bone_remapping():
    decoder = MocopiDecoder()
    decoder.decode(packet(True), 0)
    before = decoder.decode(packet(number=1), 1)
    quaternion = Rotation.from_rotvec([0, 0, 0.5]).as_quat()
    upper_arm = decoder.decode(packet(number=2, rotations={12: quaternion}), 2)
    assert np.linalg.norm(upper_arm.bones[14][:3, 3] - before.bones[14][:3, 3]) > 0.1
    np.testing.assert_allclose(upper_arm.bones[18], before.bones[18])
    ankle_bone = decoder.decode(packet(number=3, rotations={20: quaternion}), 3)
    for bone in (14, 18):
        np.testing.assert_allclose(ankle_bone.bones[bone], before.bones[bone])


def test_world_arm_landmarks_share_head_relative_alignment_and_keep_sides():
    from telegrip.mocopi.geometry import world_arm_landmarks

    sample, _ = fake_pair(1, 2)
    world_head = pose([1.0, -0.5, 2.0], Rotation.from_rotvec([0.3, 0.4, 0.1]).as_quat())
    points = world_arm_landmarks(world_head, sample)
    for name, bone in {
        "left_shoulder": 12,
        "left_elbow": 13,
        "right_shoulder": 16,
        "right_elbow": 17,
    }.items():
        np.testing.assert_allclose(points[name], world_head @ inverse(sample.bones[10]) @ sample.bones[bone])
    assert not np.allclose(points["left_elbow"], points["right_elbow"])
    sample.bones = {k: v for k, v in sample.bones.items() if k in (0, 10, 14, 18)}
    assert world_arm_landmarks(world_head, sample) == {}  # old minimal replay compatibility


def test_upper_body_and_standard_mounting_guidance():
    from telegrip.mocopi.mounting import mounting_prompts, mounting_summary

    config = load_config(CONFIG)
    upper = mounting_prompts(config)
    assert len(upper) == 6
    assert "ANKLE/L" in upper[3] and "左二の腕" in upper[3]
    assert "ANKLE/R" in upper[4] and "右二の腕" in upper[4]
    assert "HIP" in upper[5]
    assert "自動検出しません" in mounting_summary(config)
    config["mocopi"]["tracking_mode"] = "standard"
    standard = mounting_prompts(config)
    assert "左足首" in standard[3] and "右足首" in standard[4]


def test_mounting_config_rejects_unknown_mode_and_defaults_legacy(tmp_path):
    import yaml

    data = yaml.safe_load(CONFIG.read_text())
    data["mocopi"]["tracking_mode"] = "ankle-bone-is-arm"
    config_file = tmp_path / "config.yaml"
    config_file.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="tracking_mode"):
        load_config(config_file)
    del data["mocopi"]["tracking_mode"]
    config_file.write_text(yaml.safe_dump(data))
    assert load_config(config_file)["mocopi"]["tracking_mode"] == "standard"


@pytest.mark.parametrize("heading_deg", [0, 90, -135])
def test_arm_posture_retarget_maps_backwards_elbow_and_axial_twist(heading_deg):
    from telegrip.mocopi.posture import ArmPostureRetarget

    heading = Rotation.from_euler("z", heading_deg, degrees=True).as_matrix()
    robot_from_operator = np.array(load_config(CONFIG)["retarget"]["world_to_robot_rotation"])
    shoulder = pose([1, 2, 1.2], Rotation.from_matrix(heading).as_quat())
    elbow = pose(shoulder[:3, 3] + heading @ [0, 0, -0.3])
    robot = (np.array([0.25, 0, 0.4]), np.array([0.25, -0.15, 0.2]), np.eye(3))
    mapping = ArmPostureRetarget(shoulder, elbow, robot, robot_from_operator @ heading.T)
    baseline = mapping.target(shoulder, elbow)
    np.testing.assert_allclose(baseline.elbow, robot[1])
    np.testing.assert_allclose(baseline.upper_rotation, robot[2], atol=1e-12)
    # Pulling the human elbow backwards (-forward) moves it to robot +Y.
    pulled = elbow.copy()
    pulled[:3, 3] += heading @ [-0.06, 0, 0]
    target = mapping.target(shoulder, pulled)
    assert target.elbow[1] > baseline.elbow[1] + 0.03
    np.testing.assert_allclose(target.elbow[[0, 2]], baseline.elbow[[0, 2]])
    # Axial upper-arm rotation must remain observable even with fixed points.
    twisted = shoulder.copy()
    delta = Rotation.from_rotvec([0, 0, 0.4]).as_matrix()
    twisted[:3, :3] = heading @ delta
    target = mapping.target(twisted, elbow)
    np.testing.assert_allclose(target.elbow, baseline.elbow)
    expected = robot_from_operator @ Rotation.from_rotvec([0, 0, 0.32]).as_matrix() @ robot_from_operator.T
    np.testing.assert_allclose(target.upper_rotation, expected, atol=1e-12)


def test_head_vslam_rotation_does_not_move_stationary_hands_or_upper_arms():
    from telegrip.mocopi.geometry import world_arm_landmarks

    sample, _ = fake_pair(0, 10)
    extrinsic = pose([0.035, -0.015, 0.06], Rotation.from_rotvec([0.2, -0.1, 0.15]).as_quat())
    calibration = HeadCalibration(1.7, pose(), extrinsic, {}, "head-map", "camera")

    def pair():
        camera = sample.bones[10] @ inverse(extrinsic)
        camera[:3, 3] /= calibration.scale
        return sample, SlamSample(sample.timestamp, camera, session="head-map", intrinsics_id="camera")

    gate = TrackingGate(load_config(CONFIG)["tracking"], calibration)
    before_head, before_hands = gate.accept(pair(), 10)
    before_arms = world_arm_landmarks(before_head, sample)
    sample.timestamp = 10.03
    sample.bones[10][:3, :3] = Rotation.from_rotvec([0.1, -0.1, 0.2]).as_matrix()
    head, hands = gate.accept(pair(), 10.03)
    assert gate.tracking_valid
    assert Rotation.from_matrix(head[:3, :3]).magnitude() > 0.2
    for side in hands:
        np.testing.assert_allclose(hands[side], before_hands[side], atol=1e-12)
    for name, landmark in world_arm_landmarks(head, sample).items():
        np.testing.assert_allclose(landmark, before_arms[name], atol=1e-12)


@pytest.mark.parametrize("fault", ["rotation-jump", "nan", "missing"])
def test_upper_arm_fault_holds_even_when_hands_are_stationary(fault):
    sample, _ = fake_pair(0, 10)
    gate = TrackingGate(load_config(CONFIG)["tracking"])
    assert gate.accept((sample, None), 10)
    sample.timestamp = 10.03
    if fault == "rotation-jump":
        sample.bones[12][:3, :3] = Rotation.from_rotvec([1, 0, 0]).as_matrix()
    elif fault == "nan":
        sample.bones[13][0, 3] = np.nan
    else:
        del sample.bones[12]
    assert gate.accept((sample, None), 10.03) is None
    assert gate.latched and not gate.tracking_valid


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("motion", ["elbow-out", "upper-twist", "backwards"])
def test_mujoco_upper_arm_guidance_moves_posture_preserves_tcp_and_speed(side, motion):
    pytest.importorskip("mujoco")
    pytest.importorskip("pybullet")
    from telegrip.mocopi.mujoco_sim import MujocoSimulation
    from telegrip.mocopi.posture import ArmPostureTarget, robot_arm_state

    sim = MujocoSimulation(load_config(CONFIG)["robot"], gui=False)
    try:
        arm = sim.arms[side]
        before = arm.last_valid_target.copy()
        shoulder, elbow, upper = robot_arm_state(arm.solver, before)
        targets = {name: target.copy() for name, target in sim.initial_ee.items()}
        if motion == "elbow-out":
            delta = Rotation.from_rotvec([0, -0.2 if side == "left" else 0.2, 0]).as_matrix()
            goal = ArmPostureTarget(shoulder + delta @ (elbow - shoulder), delta @ upper)
        elif motion == "upper-twist":
            delta = Rotation.from_rotvec(upper[:, 2] * 0.4).as_matrix()
            goal = ArmPostureTarget(elbow.copy(), delta @ upper)
        else:
            delta = Rotation.from_rotvec([0.25, 0, 0]).as_matrix()
            goal = ArmPostureTarget(shoulder + delta @ (elbow - shoulder), delta @ upper)
            targets[side][:3, 3] += [0, 0.04, -0.015]
        initial_error = Rotation.from_matrix(goal.upper_rotation @ upper.T).magnitude()
        for _ in range(180):
            previous = arm.last_valid_target.copy()
            sim.update(targets, postures={side: goal})
            assert arm.valid, arm.status
            assert np.max(np.abs(arm.last_valid_target - previous)) <= arm.max_step + 1e-6
            assert np.all(arm.last_valid_target >= arm.lower - 1e-6)
            assert np.all(arm.last_valid_target <= arm.upper + 1e-6)
        _, achieved_elbow, achieved_upper = robot_arm_state(arm.solver, arm.last_valid_target)
        actual_tcp = sim.data.body(f"{side}/tcp_link").xpos
        assert np.linalg.norm(actual_tcp - targets[side][:3, 3]) < 0.005
        if motion == "upper-twist":
            final_error = Rotation.from_matrix(goal.upper_rotation @ achieved_upper.T).magnitude()
            assert final_error < initial_error * 0.85
        else:
            assert np.linalg.norm(achieved_elbow - elbow) > 0.01
            assert np.linalg.norm(goal.elbow - achieved_elbow) < np.linalg.norm(goal.elbow - elbow)
        other = "right" if side == "left" else "left"
        assert sim.arms[other].posture_info["mode"] == "hand-only"
        np.testing.assert_allclose(sim.data.qpos[sim.qpos_indices[side]], np.deg2rad(arm.last_valid_target))
        marker = sim.model.body(f"{side}/elbow_target").mocapid[0]
        np.testing.assert_allclose(sim.data.mocap_pos[marker], goal.elbow)
        held = sim.data.qpos.copy()
        sim.update(targets, valid=False, postures={side: goal})
        np.testing.assert_array_equal(sim.data.qpos, held)
        assert arm.posture_info["mode"] == "HOLD"
        # Invalid limb observations cannot be displayed or commanded.
        goal.elbow[0] = np.nan
        sim.update(targets, postures={side: goal})
        assert not arm.valid
        np.testing.assert_array_equal(sim.data.qpos[sim.qpos_indices[side]], held[sim.qpos_indices[side]])
        assert np.isfinite(sim.data.mocap_pos).all()
    finally:
        sim.close()


def test_old_hand_only_replay_can_start_without_upper_arm_bones():
    pytest.importorskip("pybullet")
    from telegrip.mocopi.__main__ import anchors, build_parser
    from telegrip.mocopi.ik import DualSimulation

    cfg = load_config(CONFIG)
    sample, _ = fake_pair(0, 10)
    sample.bones = {bone: value for bone, value in sample.bones.items() if bone in (0, 10, 14, 18)}
    sim = DualSimulation(cfg["robot"], gui=False)
    try:
        head = sample.bones[10]
        mappings = anchors(cfg, sim, world_hands(head, sample), head, sample)
        assert all(mapping.posture_target({}, side) is None for side, mapping in mappings.items())
        args = build_parser().parse_args(["run", "--arm-posture", "hand-only"])
        assert args.arm_posture == "hand-only"
    finally:
        sim.close()


def test_default_hand_rotation_is_not_halved():
    cfg = load_config(CONFIG)["retarget"]
    human = pose([0.2, 0.3, 1.1])
    robot = pose([0.25, -0.3, 0.25], Rotation.from_euler("x", 20, degrees=True).as_quat())
    mapping = Retarget(
        human,
        robot,
        rotation=np.array(cfg["world_to_robot_rotation"]),
        orientation_scale=cfg["orientation_scale"],
        orientation_enabled=cfg["orientation_enabled"],
    )
    turned = human.copy()
    turned[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    target = mapping.target(turned)
    angle = Rotation.from_matrix(target[:3, :3] @ robot[:3, :3].T).magnitude()
    assert np.rad2deg(angle) == pytest.approx(90)
    np.testing.assert_array_equal(target[:3, 3], robot[:3, 3])


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("axis,angle_deg", [(0, -30), (1, 20), (2, 45)])
def test_wrist_turns_with_upper_arm_guidance_and_orientation_markers(side, axis, angle_deg):
    pytest.importorskip("mujoco")
    pytest.importorskip("pybullet")
    from telegrip.mocopi.mujoco_sim import MujocoSimulation
    from telegrip.mocopi.posture import ArmPostureTarget, robot_arm_state

    sim = MujocoSimulation(load_config(CONFIG)["robot"], gui=False)
    try:
        arm = sim.arms[side]
        _, elbow, upper = robot_arm_state(arm.solver, arm.last_valid_target)
        targets = {name: target.copy() for name, target in sim.initial_ee.items()}
        targets[side][:3, :3] = (
            targets[side][:3, :3] @ Rotation.from_rotvec(np.eye(3)[axis] * np.deg2rad(angle_deg)).as_matrix()
        )
        for _ in range(240):
            before = arm.last_valid_target.copy()
            sim.update(targets, postures={side: ArmPostureTarget(elbow, upper)})
            assert arm.valid, arm.status
            assert np.max(np.abs(arm.last_valid_target - before)) <= arm.max_step + 1e-6
        actual = sim.data.body(f"{side}/tcp_link")
        error = Rotation.from_matrix(targets[side][:3, :3] @ actual.xmat.reshape(3, 3).T).magnitude()
        assert np.rad2deg(error) < (5 if axis == 2 else abs(angle_deg) * 0.5)
        assert np.linalg.norm(actual.xpos - targets[side][:3, 3]) < 0.005
        for number in range(3):
            np.testing.assert_allclose(
                sim.data.site(f"{side}/target_axis_{number}").xmat.reshape(3, 3),
                targets[side][:3, :3],
                atol=1e-7,
            )
            np.testing.assert_allclose(
                sim.data.site(f"{side}/actual_axis_{number}").xmat,
                actual.xmat,
                atol=1e-7,
            )
        held = sim.data.qpos.copy(), sim.data.mocap_quat.copy()
        sim.update(targets, valid=False)
        np.testing.assert_array_equal(sim.data.qpos, held[0])
        np.testing.assert_array_equal(sim.data.mocap_quat, held[1])
    finally:
        sim.close()
