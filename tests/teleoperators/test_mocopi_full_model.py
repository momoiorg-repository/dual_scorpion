"""Native full-CAD MuJoCo preview without physical motor access."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "telegrip_teleoperation"))
pytest.importorskip("mujoco")
from telegrip.mocopi.config import load_config
from telegrip.mocopi.full_mujoco import FullMujocoSimulation
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
