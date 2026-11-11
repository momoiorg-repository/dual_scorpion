"""Controlled IK modes keep the default mocopi path unchanged."""

import csv
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "telegrip_teleoperation"))
from telegrip.mocopi.config import load_config
from telegrip.mocopi.dataset.capture import capture_session
from telegrip.mocopi.dataset.replay import replay_sim

CONFIG = Path(__file__).resolve().parents[2] / "config/mocopi/tracking.yaml"
RESEARCH_MODES = ("position-only", "hand-6dof", "hand-6dof-upper-arm")
LOCAL_REFERENCE_DATASET = (
    Path(__file__).resolve().parents[2] / "outputs/mocopi/datasets/episode-001-fps15-sync16"
)
LOCAL_REFERENCE_PLAN = (
    Path(__file__).resolve().parents[2]
    / "outputs/mocopi/plans/build-robot-episode-001-v12/episode-000-plan.json"
)


def _read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _read_csv(path):
    with Path(path).open(encoding="utf-8", newline="") as file:
        return list(csv.DictReader(file))


@pytest.fixture(scope="module")
def stationary_dataset(tmp_path_factory):
    pytest.importorskip("cv2")
    pytest.importorskip("mujoco")
    pytest.importorskip("lerobot.datasets.lerobot_dataset")
    from telegrip.mocopi.dataset.convert import convert_sessions

    cfg = load_config(CONFIG)
    cfg["camera"].update(width=96, height=64, fps=20)
    raw = tmp_path_factory.mktemp("raw") / "stationary"
    capture_session(cfg, raw, 0.35, "stationary comparison", fake=True)
    motion_path = raw / "mocopi.jsonl"
    motion = [json.loads(line) for line in motion_path.read_text().splitlines() if line]
    bones = motion[0]["bones"]
    for row in motion[1:]:
        row["bones"] = bones
    motion_path.write_text(
        "".join(json.dumps(row, allow_nan=False) + "\n" for row in motion), encoding="utf-8"
    )
    dataset = tmp_path_factory.mktemp("dataset") / "stationary"
    convert_sessions([raw], dataset, "local/mocopi-ik-modes", fps=20)
    return cfg, dataset


@pytest.fixture(scope="module")
def plans(stationary_dataset):
    cfg, dataset = stationary_dataset
    options = {"gui": False, "realtime": False, "sim_model": "full", "preview_max_step_deg": 20}
    result = {
        "implicit": replay_sim(cfg, dataset, **options)[0],
        "explicit": replay_sim(cfg, dataset, ik_mode="default", **options)[0],
    }
    for mode in RESEARCH_MODES:
        result[mode] = replay_sim(cfg, dataset, ik_mode=mode, **options)[0]
    return result


def test_default_mode_is_unchanged_and_deterministic(plans):
    implicit, explicit = plans["implicit"], plans["explicit"]
    assert implicit == explicit
    assert implicit["ik_mode"] == explicit["ik_mode"] == "default"
    assert implicit["arm_posture"] == explicit["arm_posture"] == "upper-arm"
    assert implicit["retarget"]["arm_posture_gain"] == 0.8
    assert implicit["retarget"]["orientation_enabled"] is True
    assert not implicit["failures"]


@pytest.mark.skipif(
    not (LOCAL_REFERENCE_DATASET.is_dir() and LOCAL_REFERENCE_PLAN.is_file()),
    reason="local pre-change v12 plan and recorded dataset are not present",
)
def test_default_matches_recorded_pre_change_plan():
    """Numerical regression against the plan generated before this change set."""
    cfg = load_config(CONFIG)
    plan, report = replay_sim(
        cfg,
        LOCAL_REFERENCE_DATASET,
        gui=False,
        realtime=False,
        sim_model="full",
        preview_max_step_deg=20,
    )
    reference = _read_json(LOCAL_REFERENCE_PLAN)
    assert len(plan["frames"]) == len(reference["frames"])
    for expected_frame, actual_frame in zip(reference["frames"], plan["frames"], strict=True):
        assert set(actual_frame) == set(expected_frame)
        for side in ("left", "right"):
            assert set(actual_frame["arms"][side]) == set(expected_frame["arms"][side])
    for side in ("left", "right"):
        expected = np.asarray(
            [frame["arms"][side]["joint_deg"] for frame in reference["frames"]], dtype=float
        )
        actual = np.asarray([frame["arms"][side]["joint_deg"] for frame in plan["frames"]], dtype=float)
        np.testing.assert_allclose(actual, expected, rtol=0.0, atol=0.0)
    assert not plan["failures"] and not reference["failures"]
    assert report["ik_failures"] == []


def test_console_reports_the_ik_mode(stationary_dataset, capsys):
    cfg, dataset = stationary_dataset
    replay_sim(cfg, dataset, gui=False, realtime=False, sim_model="full", ik_mode="hand-6dof")
    captured = capsys.readouterr().out
    assert "IK mode: hand-6dof" in captured


def test_ik_mode_and_legacy_posture_conflicts_are_explicit(tmp_path):
    cfg = load_config(CONFIG)
    missing = tmp_path / "missing-dataset"
    with pytest.raises(ValueError, match="position-only conflicts"):
        replay_sim(cfg, missing, ik_mode="position-only", arm_posture="upper-arm", sim_model="full")
    with pytest.raises(ValueError, match="hand-6dof conflicts"):
        replay_sim(cfg, missing, ik_mode="hand-6dof", arm_posture="upper-arm", sim_model="full")
    with pytest.raises(ValueError, match="requires --arm-posture upper-arm"):
        replay_sim(
            cfg,
            missing,
            ik_mode="hand-6dof-upper-arm",
            arm_posture="hand-only",
            sim_model="full",
        )
    with pytest.raises(ValueError, match="Unsupported ik_mode"):
        replay_sim(cfg, missing, ik_mode="not-a-mode", sim_model="full")


def test_research_modes_run_independently(plans):
    for mode in RESEARCH_MODES:
        plan = plans[mode]
        assert plan["ik_mode"] == mode
        assert len(plan["frames"]) >= 3
        assert plan["gripper_included"] is False
        assert plan["arm_posture"] == ("upper-arm" if mode == "hand-6dof-upper-arm" else "hand-only")
        for frame in plan["frames"]:
            for side in ("left", "right"):
                arm = frame["arms"][side]
                assert len(arm["joint_deg"]) == 7
                assert arm["research_ik"]["mode"] == mode
                if not arm["valid"]:
                    assert arm["research_ik"]["failure_kind"]


def test_hand_6dof_and_upper_arm_share_targets_initial_pose_and_limits(plans):
    baseline = plans["hand-6dof"]
    posture = plans["hand-6dof-upper-arm"]
    assert len(baseline["frames"]) == len(posture["frames"])
    for first, second in zip(baseline["frames"], posture["frames"], strict=True):
        assert first["source_frame"] == second["source_frame"]
        for side in ("left", "right"):
            np.testing.assert_array_equal(
                first["arms"][side]["target_pose"], second["arms"][side]["target_pose"]
            )
    for side in ("left", "right"):
        np.testing.assert_array_equal(
            baseline["frames"][0]["arms"][side]["joint_deg"],
            posture["frames"][0]["arms"][side]["joint_deg"],
        )
    for key in (
        "position_tolerance_m",
        "preview_max_joint_step_deg",
        "hardware_max_joint_step_deg",
        "ik_orientation_tolerance_deg",
        "mujoco_model_sha256",
        "start_pose",
    ):
        assert baseline[key] == posture[key]
    assert baseline["retarget"]["orientation_enabled"] == posture["retarget"]["orientation_enabled"]


def test_upper_arm_refinement_stays_inside_tcp_tolerances(plans):
    plan = plans["hand-6dof-upper-arm"]
    position_tolerance = plan["position_tolerance_m"]
    orientation_tolerance = plan["ik_orientation_tolerance_deg"]
    refined_frames = 0
    for frame in plan["frames"]:
        for side in ("left", "right"):
            arm = frame["arms"][side]
            if not arm["valid"]:
                continue
            target = np.asarray(arm["target_pose"], dtype=float)
            actual = np.asarray(arm["simulated_pose"], dtype=float)
            assert np.linalg.norm(target[:3] - actual[:3]) <= position_tolerance + 1e-9
            error = Rotation.from_quat(target[3:]).inv() * Rotation.from_quat(actual[3:])
            assert np.rad2deg(error.magnitude()) <= orientation_tolerance + 1e-9
            if arm["posture_info"]["mode"] in ("settled", "limited", "active"):
                refined_frames += 1
    assert refined_frames > 0


def test_research_mode_is_reproducible(stationary_dataset, plans):
    cfg, dataset = stationary_dataset
    again = replay_sim(
        cfg,
        dataset,
        gui=False,
        realtime=False,
        sim_model="full",
        preview_max_step_deg=20,
        ik_mode="hand-6dof-upper-arm",
    )[0]
    assert again == plans["hand-6dof-upper-arm"]


def test_failure_ranges_ignore_empty_csv_values_and_sort_numerically():
    from telegrip.mocopi.dataset.ik_plots import _failure_ranges

    rows = [
        {"mode": "m", "side": "left", "plan_frame": "10", "failure_kind": "unreachable"},
        {"mode": "m", "side": "left", "plan_frame": "2", "failure_kind": "unreachable"},
        {"mode": "m", "side": "left", "plan_frame": "3", "failure_kind": "unreachable"},
        {"mode": "m", "side": "left", "plan_frame": "4", "failure_kind": ""},
        {"mode": "m", "side": "left", "plan_frame": "5", "failure_kind": None},
        {"mode": "m", "side": "right", "plan_frame": "7", "failure_kind": "joint_jump"},
    ]
    assert _failure_ranges(rows, "m", "left") == "2-3, 10"
    assert _failure_ranges(rows, "m", "right") == "7"
    assert _failure_ranges(rows, "other", "left") == "none"


def test_comparison_artifacts_are_consistent(stationary_dataset, tmp_path):
    pytest.importorskip("matplotlib")
    from telegrip.mocopi.dataset.fingerprint import dataset_fingerprint
    from telegrip.mocopi.dataset.ik_experiment import run_ik_comparison

    cfg, dataset = stationary_dataset
    output = tmp_path / "experiment"
    source_fingerprint = dataset_fingerprint(dataset)
    result = run_ik_comparison(
        cfg,
        dataset,
        output,
        preview_max_step_deg=20,
    )
    assert dataset_fingerprint(dataset) == source_fingerprint
    assert result["modes"] == ["default", "position-only", "hand-6dof", "hand-6dof-upper-arm"]
    for folder in ("default", "position_only", "hand_6dof", "hand_6dof_upper_arm"):
        assert (output / folder / "plan.json").is_file()
        assert (output / folder / "report.json").is_file()
        assert (output / folder / "evaluation.json").is_file()
        assert (output / folder / "per_frame.csv").is_file()
        assert (output / folder / "joint_trajectory.csv").is_file()

    metadata = _read_json(output / "metadata.json")
    assert metadata["orientation_tolerance_deg"] == 15.0
    fairness = metadata["fairness_check"]
    assert fairness["same_tcp_target"] is True
    assert fairness["same_initial_joint_pose"] is True
    assert fairness["same_limits_and_motion"] is True
    assert fairness["c_success_frames_within_tcp_tolerance"] is True
    assert fairness["c_success_frames_within_orientation_tolerance"] is True
    assert fairness["c_posture_correction_within_0p5_mm"] is True
    assert fairness["joint_trajectory_difference_deg"]["max_abs"] is not None

    summary_json = _read_json(output / "comparison" / "summary.json")
    summary_rows = _read_csv(output / "comparison" / "summary.csv")
    frame_rows = _read_csv(output / "comparison" / "per_frame_comparison.csv")
    plan = _read_json(output / "hand_6dof" / "plan.json")
    assert len(frame_rows) == len(plan["frames"]) * 2 * len(result["modes"])
    metric_row = next(
        row
        for row in summary_rows
        if row["mode"] == "hand-6dof" and row["side"] == "left" and row["metric"] == "tcp_position_error_mm"
    )
    json_stats = summary_json["modes"]["hand-6dof"]["left"]["stats"]["tcp_position_error_mm"]
    assert float(metric_row["mean"]) == pytest.approx(json_stats["mean"], abs=1e-12)
    values = [
        float(row["position_error_mm"])
        for row in frame_rows
        if row["mode"] == "hand-6dof" and row["side"] == "left"
    ]
    assert float(metric_row["mean"]) == pytest.approx(float(np.mean(values)), abs=1e-12)
    assert int(metric_row["count"]) == len(values)

    graphs = output / "comparison" / "graphs"
    stems = {path.stem for path in graphs.iterdir()}
    assert len(stems) == 16
    for stem in stems:
        png, svg = graphs / f"{stem}.png", graphs / f"{stem}.svg"
        assert png.stat().st_size > 0 and svg.stat().st_size > 0
    image_module = pytest.importorskip("PIL.Image")
    with image_module.open(graphs / "tcp_position_error.png") as image:
        dpi = image.info.get("dpi")
    assert dpi is not None and min(dpi) >= 299

    report = (output / "comparison" / "report.md").read_text(encoding="utf-8")
    assert "Orientation tolerance for research 6DoF modes: 15.0 deg" in report
    assert "None deg" not in report
    assert "Same TCP target (B vs C): True" in report
    baseline_holds = sum(summary_json["modes"]["hand-6dof"][side]["hold_count"] for side in ("left", "right"))
    if baseline_holds == 0:
        assert "| hand-6DOF | none | none |" in report
    with pytest.raises(FileExistsError):
        run_ik_comparison(cfg, dataset, output, preview_max_step_deg=20)


def test_same_tcp_can_hold_different_upper_arm_postures():
    pytest.importorskip("mujoco")
    from telegrip.mocopi.full_mujoco import FullMujocoSimulation
    from telegrip.mocopi.posture import ArmPostureTarget, robot_arm_state

    cfg = load_config(CONFIG)["robot"]
    cfg = {
        **cfg,
        "start_pose": "neutral",
        "ik_mode": "hand-6dof-upper-arm",
        "ik_orientation_tolerance_deg": 15.0,
        "max_joint_step_deg": 20.0,
    }
    simulation = FullMujocoSimulation(cfg, gui=False)
    try:
        arm = simulation.arms["left"]
        initial = arm.last_valid_target.copy()
        position, quaternion = arm.solver.fk_solver.compute(initial)
        target = np.eye(4)
        target[:3, 3] = position
        target[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        _, elbow, upper = robot_arm_state(arm.solver, initial)

        def settle(shift):
            arm.last_valid_target[:] = initial
            arm.valid = False
            arm.posture_solver = None
            posture = ArmPostureTarget(elbow + shift, upper)
            for _ in range(60):
                joints = arm.solve(target, posture)
            assert arm.valid, arm.status
            tcp, quat = arm.solver.fk_solver.compute(joints)
            _, final_elbow, _ = robot_arm_state(arm.solver, joints)
            return tcp, quat, final_elbow

        first_tcp, first_quat, first_elbow = settle(np.zeros(3))
        second_tcp, second_quat, second_elbow = settle(np.array([0.0, 0.0, -0.04]))
        assert np.linalg.norm(first_tcp - second_tcp) < 0.001
        assert np.linalg.norm(first_elbow - second_elbow) > 0.003
        for quat in (first_quat, second_quat):
            error = Rotation.from_quat(quat).inv() * Rotation.from_quat(quaternion)
            assert np.rad2deg(error.magnitude()) <= 15.0
        np.testing.assert_array_equal(simulation.data.qpos[simulation.qpos_indices["left"][7]], 0)
    finally:
        simulation.close()


def test_comparison_subset_omitting_primary_modes_still_reports(stationary_dataset, tmp_path):
    pytest.importorskip("matplotlib")
    from telegrip.mocopi.dataset.ik_experiment import run_ik_comparison

    cfg, dataset = stationary_dataset
    output = tmp_path / "default-only"
    run_ik_comparison(cfg, dataset, output, modes=["default"], preview_max_step_deg=20)
    metadata = _read_json(output / "metadata.json")
    assert metadata["fairness_check"]["available"] is False
    report = (output / "comparison" / "report.md").read_text(encoding="utf-8")
    assert "both primary-comparison modes must be run" in report
