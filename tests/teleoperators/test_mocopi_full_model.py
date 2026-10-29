"""Native full-CAD MuJoCo preview without physical motor access."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "telegrip_teleoperation"))
pytest.importorskip("mujoco")
from telegrip.mocopi.config import load_config
from telegrip.mocopi.full_mujoco import FullMujocoSimulation, full_model_path
from telegrip.mocopi.posture import ArmPostureTarget, robot_arm_state

CONFIG = Path(__file__).resolve().parents[2] / "config/mocopi/tracking.yaml"


def test_full_model_maps_cad_sides_and_fixes_gripper():
    simulation = FullMujocoSimulation(load_config(CONFIG)["robot"], gui=False)
    try:
        assert simulation.model.nq == 16
        for side, suffix in (("left", "L"), ("right", "R")):
            expected = [simulation.model.joint(f"joint{i}_{suffix}").qposadr[0] for i in range(8)]
            np.testing.assert_array_equal(simulation.qpos_indices[side], expected)
            assert simulation.data.qpos[expected[7]] == 0
            position, _ = simulation.arms[side].solver.fk_solver.compute(
                simulation.arms[side].last_valid_target
            )
            np.testing.assert_allclose(position, simulation.data.site(f"{side}/tcp_site").xpos, atol=1e-9)
        assert simulation.initial_ee["left"][0, 3] < 0
        assert simulation.initial_ee["right"][0, 3] > 0
        assert all(simulation.initial_ee[side][1, 3] < 0 for side in ("left", "right"))
        assert all(simulation.arms[side].last_valid_target[3] == -30 for side in ("left", "right"))

        left = simulation.arms["left"]
        initial_left, initial_right = (
            left.last_valid_target.copy(),
            simulation.arms["right"].last_valid_target.copy(),
        )
        _, elbow, upper = robot_arm_state(left.solver, initial_left)
        targets = {side: value.copy() for side, value in simulation.initial_ee.items()}
        targets["left"][:3, 3] += [0, 0.01, 0.005]
        posture = {"left": ArmPostureTarget(elbow, upper)}
        for _ in range(40):
            previous = left.last_valid_target.copy()
            simulation.update(targets, postures=posture)
            assert left.valid, left.status
            assert np.max(np.abs(left.last_valid_target - previous)) <= left.max_step + 1e-6
            assert simulation.data.qpos[simulation.qpos_indices["left"][7]] == 0
            assert simulation.data.qpos[simulation.qpos_indices["right"][7]] == 0
        actual, _ = left.solver.fk_solver.compute(left.last_valid_target)
        assert np.linalg.norm(actual - targets["left"][:3, 3]) < 0.002
        assert np.linalg.norm(left.last_valid_target[:7] - initial_left[:7]) > 0.1
        np.testing.assert_allclose(simulation.arms["right"].last_valid_target, initial_right, atol=1e-4)

        held = simulation.data.qpos.copy()
        simulation.update(targets, valid=False, postures=posture)
        np.testing.assert_array_equal(simulation.data.qpos, held)
        assert all(not arm.valid and arm.posture_info["mode"] == "HOLD" for arm in simulation.arms.values())

        unreachable = {side: value.copy() for side, value in targets.items()}
        unreachable["left"][:3, 3] += [5, 0, 0]
        before = left.last_valid_target.copy()
        simulation.update(unreachable, postures=posture)
        assert not left.valid
        np.testing.assert_array_equal(left.last_valid_target, before)
    finally:
        simulation.close()


def test_full_model_ik_rejects_a_clipped_unreachable_candidate():
    """The safety verdict must describe the command actually returned."""
    cfg = load_config(CONFIG)["robot"]
    work = FullMujocoSimulation(cfg, gui=False)
    try:
        targets = {side: value.copy() for side, value in work.initial_ee.items()}
        targets["left"][:3, 3] += [0, 0.01, 0.005]
        work.update(targets)
        left = work.arms["left"]
        actual, _ = left.solver.fk_solver.compute(left.last_valid_target)
        returned_residual = np.linalg.norm(actual - targets["left"][:3, 3])
        assert left.valid
        assert abs(left.solver.last_solve_info["residual_final_m"] - returned_residual) < 1e-9
        assert returned_residual <= left.tolerance
        work_target = targets["left"]
    finally:
        work.close()

    backwards_cfg = {**cfg, "start_pose": "backwards"}
    backwards = FullMujocoSimulation(backwards_cfg, gui=False)
    try:
        assert backwards.start_pose == "backwards"
        left = backwards.arms["left"]
        before = left.last_valid_target.copy()
        candidate = left.solver.solve(work_target[:3, 3], None, before)
        candidate_position, _ = left.solver.fk_solver.compute(candidate)
        candidate_residual = np.linalg.norm(candidate_position - work_target[:3, 3])
        assert np.max(np.abs(candidate[:7] - before[:7])) <= left.max_step + 1e-6
        assert candidate_residual > left.tolerance
        np.testing.assert_allclose(
            left.solver.last_solve_info["residual_final_m"], candidate_residual, atol=1e-12
        )

        backwards.update(targets)
        assert not left.valid
        assert "IK not reachable" in left.status
        np.testing.assert_array_equal(left.last_valid_target, before)
    finally:
        backwards.close()


@pytest.mark.parametrize("side", ["left", "right"])
def test_full_model_jacobian_matches_mujoco_finite_difference(side):
    simulation = FullMujocoSimulation(load_config(CONFIG)["robot"], gui=False)
    try:
        fk = simulation.arms[side].solver.fk_solver
        joints = simulation.arms[side].last_valid_target.copy()
        position, _ = fk.compute(joints)
        jacobian, _ = fk.jacobian()
        epsilon = 1e-5
        for joint in (0, 3, 6):
            shifted = joints.copy()
            shifted[joint] += np.rad2deg(epsilon)
            moved, _ = fk.compute(shifted)
            np.testing.assert_allclose((moved - position) / epsilon, jacobian[:, joint], atol=2e-5)
    finally:
        simulation.close()


def test_missing_full_model_path_is_a_clear_error():
    robot = dict(load_config(CONFIG)["robot"])
    assert Path(full_model_path(robot)).is_file()
    robot.pop("mujoco_full_model")
    with pytest.raises(ValueError, match="mujoco_full_model"):
        full_model_path(robot)
    with pytest.raises(ValueError, match="mujoco_full_model"):
        FullMujocoSimulation(robot, gui=False)


def test_doctor_reports_full_model_and_lfs_state(monkeypatch, capsys, tmp_path):
    import json

    from telegrip.mocopi import __main__ as cli

    # Camera probing is unrelated here and can be slow or unavailable.
    monkeypatch.setattr(cli, "camera_list", lambda: [])
    cfg = load_config(CONFIG)
    cli.doctor(cfg)
    report = json.loads(capsys.readouterr().out)
    assert report["mujoco_importable"] is True
    assert report["mujoco_full_model"]["exists"] is True
    assert report["mujoco_full_model"]["mesh_state"] == "ok"
    assert report["visual_urdf_exists"] is True and report["hardware_control"] == "disabled"

    # A config without the key reports the model as absent instead of guessing.
    cfg["robot"].pop("mujoco_full_model")
    cli.doctor(cfg)
    assert json.loads(capsys.readouterr().out)["mujoco_full_model"] is None

    # Un-pulled Git LFS meshes are reported as pointers, not as usable models.
    model = tmp_path / "model.xml"
    model.write_text("<mujoco/>")
    cfg = load_config(CONFIG)
    cfg["robot"]["mujoco_full_model"] = str(model)
    cli.doctor(cfg)
    assert json.loads(capsys.readouterr().out)["mujoco_full_model"]["mesh_state"] == "missing"
    mesh = tmp_path / "meshes" / "visual" / "base_link.obj"
    mesh.parent.mkdir(parents=True)
    mesh.write_bytes(b"version https://git-lfs.github.com/spec/v1\noid sha256:0\n")
    cli.doctor(cfg)
    assert json.loads(capsys.readouterr().out)["mujoco_full_model"]["mesh_state"] == "lfs_pointer"
