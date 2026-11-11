"""Static thesis-ready figures and factual comparison report."""

import csv
from collections import Counter
from pathlib import Path

import numpy as np

MODE_LABELS = {
    "default": "default (existing)",
    "position-only": "position-only",
    "hand-6dof": "hand-6DOF",
    "hand-6dof-upper-arm": "hand-6DOF + upper-arm",
}
MODE_COLORS = {
    "default": "#5f6368",
    "position-only": "#d97706",
    "hand-6dof": "#2563eb",
    "hand-6dof-upper-arm": "#059669",
}
MODE_STYLES = {
    "default": "--",
    "position-only": ":",
    "hand-6dof": "-",
    "hand-6dof-upper-arm": "-.",
}


def _plt():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "figure.dpi": 120,
            "savefig.dpi": 300,
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "legend.fontsize": 8,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.unicode_minus": False,
            "svg.fonttype": "none",
        }
    )
    return plt


def _save(fig, directory, stem):
    png = directory / f"{stem}.png"
    svg = directory / f"{stem}.svg"
    fig.savefig(png, dpi=300, bbox_inches="tight")
    fig.savefig(svg, bbox_inches="tight")
    return [f"graphs/{png.name}", f"graphs/{svg.name}"]


def _series(results, mode, side, metric):
    rows = [row for row in results[mode]["per_frame"] if row["side"] == side]
    return np.asarray([np.nan if row[metric] is None else float(row[metric]) for row in rows], dtype=float)


def _line_by_side(directory, results, modes, metric, title, ylabel, stem):
    plt = _plt()
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2), sharey=True)
    for axis, side in zip(axes, ("left", "right"), strict=True):
        for mode in modes:
            values = _series(results, mode, side, metric)
            axis.plot(
                np.arange(len(values)),
                values,
                label=MODE_LABELS[mode],
                color=MODE_COLORS[mode],
                linestyle=MODE_STYLES[mode],
                linewidth=1.5,
            )
        axis.set_title(f"{side.capitalize()} arm")
        axis.set_xlabel("Plan frame")
        axis.set_ylabel(ylabel)
    axes[-1].legend(loc="best", frameon=False)
    fig.suptitle(title, x=0.01, ha="left", fontweight="bold")
    fig.tight_layout()
    paths = _save(fig, directory, stem)
    plt.close(fig)
    return paths


def _bars(directory, results, modes, metrics, title, ylabel, stem, sides=("left", "right")):
    plt = _plt()
    x = np.arange(len(metrics))
    width = 0.8 / max(len(modes) * len(sides), 1)
    fig, ax = plt.subplots(figsize=(max(9, len(metrics) * 1.8), 4.4))
    offset = 0
    for mode in modes:
        for side in sides:
            values = []
            for metric in metrics:
                value = results[mode]["sides"][side].get(metric)
                values.append(0.0 if value is None else float(value))
            ax.bar(
                x + offset - 0.4 + width / 2,
                values,
                width,
                label=f"{MODE_LABELS[mode]} / {side}",
                color=MODE_COLORS[mode],
                alpha=0.95 if side == "left" else 0.55,
                edgecolor="white",
                linewidth=0.6,
            )
            offset += width
    ax.set_xticks(x, metrics, rotation=20, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontweight="bold")
    ax.legend(frameon=False, ncol=2)
    fig.tight_layout()
    paths = _save(fig, directory, stem)
    plt.close(fig)
    return paths


def _stat_bars(directory, results, modes, metric, title, ylabel, stem):
    plt = _plt()
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    for axis, side in zip(axes, ("left", "right"), strict=True):
        values = [
            results[mode]["sides"][side]["stats"][metric]["mean"]
            if results[mode]["sides"][side]["stats"][metric]["mean"] is not None
            else 0.0
            for mode in modes
        ]
        axis.bar(
            np.arange(len(modes)),
            values,
            color=[MODE_COLORS[mode] for mode in modes],
            edgecolor="white",
            linewidth=0.8,
        )
        axis.set_xticks(np.arange(len(modes)), [MODE_LABELS[mode] for mode in modes], rotation=20, ha="right")
        axis.set_title(f"{side.capitalize()} arm")
        axis.set_ylabel(ylabel)
    fig.suptitle(title, x=0.01, ha="left", fontweight="bold")
    fig.tight_layout()
    paths = _save(fig, directory, stem)
    plt.close(fig)
    return paths


def _scalar_bars(directory, results, modes, metric, title, ylabel, stem):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(9, 4.2))
    width = 0.34
    for index, side in enumerate(("left", "right")):
        values = [results[mode]["sides"][side].get(metric) or 0.0 for mode in modes]
        ax.bar(
            np.arange(len(modes)) + (index - 0.5) * width,
            values,
            width,
            label=side,
            color=["#2563eb", "#d97706"][index],
            alpha=0.9 if side == "left" else 0.65,
            edgecolor="white",
            linewidth=0.7,
        )
    ax.set_xticks(np.arange(len(modes)), [MODE_LABELS[mode] for mode in modes], rotation=20, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontweight="bold")
    ax.legend(frameon=False)
    fig.tight_layout()
    paths = _save(fig, directory, stem)
    plt.close(fig)
    return paths


def _collision_bars(directory, results, modes):
    plt = _plt()
    metrics = (
        ("self_collision_contacts", "Self"),
        ("cross_arm_collision_contacts", "Cross-arm"),
        ("obstacle_collision_contacts", "Obstacle"),
    )
    fig, ax = plt.subplots(figsize=(9, 4.2))
    width = 0.25
    for index, (metric, label) in enumerate(metrics):
        values = [results[mode]["collision_totals"][metric] for mode in modes]
        ax.bar(
            np.arange(len(modes)) + (index - 1) * width,
            values,
            width,
            label=label,
            color=["#2563eb", "#d97706", "#059669"][index],
            edgecolor="white",
            linewidth=0.7,
        )
    ax.set_xticks(np.arange(len(modes)), [MODE_LABELS[mode] for mode in modes], rotation=20, ha="right")
    ax.set_ylabel("Contact count")
    ax.set_title("Modeled collision contacts by IK mode", loc="left", fontweight="bold")
    ax.legend(frameon=False)
    fig.tight_layout()
    paths = _save(fig, directory, "collision_contacts")
    plt.close(fig)
    return paths


def generate_comparison_plots(output, results):
    directory = Path(output) / "comparison" / "graphs"
    directory.mkdir(parents=True, exist_ok=True)
    modes = list(results)
    graphs = []
    graphs += _line_by_side(
        directory,
        results,
        modes,
        "position_error_mm",
        "TCP position error by IK mode",
        "Position error (mm)",
        "tcp_position_error",
    )
    graphs += _line_by_side(
        directory,
        results,
        modes,
        "orientation_error_deg",
        "TCP orientation error by IK mode",
        "Orientation error (deg)",
        "tcp_orientation_error",
    )
    graphs += _line_by_side(
        directory,
        results,
        modes,
        "shoulder_elbow_angle_error_deg",
        "Shoulder-to-elbow direction error",
        "Angle error (deg)",
        "shoulder_elbow_angle_error",
    )
    graphs += _line_by_side(
        directory,
        results,
        modes,
        "swivel_angle_error_deg",
        "Elbow swivel-angle difference",
        "Swivel angle difference (deg)",
        "swivel_angle_error",
    )
    graphs += _scalar_bars(
        directory,
        results,
        modes,
        "ik_success_rate",
        "Strict IK success rate",
        "Success rate",
        "ik_success_rate",
    )
    graphs += _bars(
        directory,
        results,
        modes,
        ["hold_count", "fallback_count", "unreachable_count", "orientation_failure_count"],
        "IK failure and fallback counts",
        "Frames / arms",
        "ik_outcomes",
    )
    graphs += _bars(
        directory,
        results,
        modes,
        ["hold_rate", "fallback_rate", "unreachable_rate", "orientation_failure_rate"],
        "IK outcome rates",
        "Rate",
        "ik_outcome_rates",
    )
    graphs += _scalar_bars(
        directory,
        results,
        modes,
        "joint_reversal_count",
        "Joint direction reversals",
        "Reversal count",
        "joint_reversals",
    )
    graphs += _collision_bars(directory, results, modes)
    graphs += _stat_bars(
        directory,
        results,
        modes,
        "joint_speed_abs_deg_s",
        "Joint speed comparison",
        "Mean |joint speed| (deg/s)",
        "joint_speed",
    )
    graphs += _stat_bars(
        directory,
        results,
        modes,
        "joint_acceleration_abs_deg_s2",
        "Joint acceleration comparison",
        "Mean |joint acceleration| (deg/s^2)",
        "joint_acceleration",
    )
    graphs += _stat_bars(
        directory,
        results,
        modes,
        "tcp_position_error_mm",
        "TCP position error statistics",
        "Mean error (mm)",
        "tcp_position_stats",
    )
    graphs += _stat_bars(
        directory,
        results,
        modes,
        "tcp_orientation_error_deg",
        "TCP orientation error statistics",
        "Mean error (deg)",
        "tcp_orientation_stats",
    )
    graphs += _stat_bars(
        directory,
        results,
        modes,
        "shoulder_elbow_angle_error_deg",
        "Upper-arm posture statistics",
        "Mean angle error (deg)",
        "upper_arm_stats",
    )
    graphs += _stat_bars(
        directory,
        results,
        modes,
        "swivel_angle_error_deg",
        "Swivel-angle statistics",
        "Mean absolute difference (deg)",
        "swivel_stats",
    )

    plt = _plt()
    fig, axes = plt.subplots(7, 2, figsize=(14, 19), sharex=True)
    for column, side in enumerate(("left", "right")):
        for joint_index in range(7):
            axis = axes[joint_index, column]
            for mode in modes:
                rows = list(results[mode]["joint_trajectory"])
                values = np.asarray([row[f"{side}_joint_deg"][joint_index] for row in rows], dtype=float)
                axis.plot(
                    np.arange(len(values)),
                    values,
                    label=MODE_LABELS[mode],
                    color=MODE_COLORS[mode],
                    linestyle=MODE_STYLES[mode],
                    linewidth=1.1,
                )
            axis.set_ylabel(f"joint{joint_index} (deg)")
            if joint_index == 0:
                axis.set_title(f"{side.capitalize()} arm")
            if joint_index == 6:
                axis.set_xlabel("Plan frame")
    axes[0, 0].legend(frameon=False, ncol=2)
    fig.suptitle("14 arm-joint trajectories by IK mode", x=0.01, ha="left", fontweight="bold")
    fig.tight_layout()
    graphs += _save(fig, directory, "joint_trajectories")
    plt.close(fig)
    return graphs


def _csv_rows(path):
    with Path(path).open(encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))
    for row in rows:
        if row.get("failure_kind") in ("", "None"):
            row["failure_kind"] = None
    return rows


def _format(value, digits=4):
    if value is None:
        return "missing"
    return f"{value:.{digits}f}"


def _failure_ranges(rows, mode, side):
    frames = sorted(
        int(row["plan_frame"])
        for row in rows
        if row["mode"] == mode and row["side"] == side and row["failure_kind"] not in (None, "", "None")
    )
    ranges = []
    for value in frames:
        if ranges and value == ranges[-1][1] + 1:
            ranges[-1][1] = value
        else:
            ranges.append([value, value])
    return ", ".join(f"{start}-{end}" if start != end else str(start) for start, end in ranges) or "none"


def generate_comparison_report(output, metadata, results, graphs, fairness=None):
    output = Path(output)
    rows = _csv_rows(output / "comparison" / "per_frame_comparison.csv")
    primary = metadata["primary_comparison"]
    b, c = primary
    lines = [
        "# IKモード比較実験レポート",
        "",
        f"- Episode: {metadata['episode']}",
        f"- Dataset: `{metadata['dataset']}`",
        f"- Dataset content SHA-256: `{metadata.get('dataset_content_sha256')}`",
        f"- MuJoCo model SHA-256: `{metadata.get('mujoco_model_sha256')}`",
        f"- Obstacle YAML: `{metadata.get('obstacles') or 'none (MJCF floor/link collisions only)'}`",
        f"- Modes: `{', '.join(metadata['modes'])}`",
        f"- Primary comparison: `{b}` vs `{c}`",
        "- Default mode is the existing Mocopi behavior and is retained unchanged.",
        "",
        "## 実験条件",
        "",
        f"- Preview joint step: {metadata.get('preview_max_joint_step_deg')} deg/frame",
        f"- Slowdown: {metadata.get('slowdown')}",
        f"- Position tolerance: {metadata['ik']['position_tolerance_m']} m",
        f"- Orientation tolerance for research 6DoF modes: {metadata.get('orientation_tolerance_deg')} deg",
        f"- Hardware step limit: {metadata['ik']['hardware_max_joint_step_deg']} deg/frame",
        "- Simulation-only; no real motor or encoder observations.",
        "- Human elbow poses are Mocopi body-model estimates.",
        "",
        "## 比較条件の公平性",
        "",
    ]
    if fairness and fairness.get("available"):
        lines += [
            f"- Same source frames: {fairness['same_source_frames']}",
            f"- Same TCP target (B vs C): {fairness['same_tcp_target']} "
            f"(max abs difference {fairness['max_target_pose_abs_diff']:.3e})",
            f"- Same initial joint pose (B vs C): {fairness['same_initial_joint_pose']} "
            f"(max abs difference {fairness['max_initial_joint_abs_diff_deg']:.3e} deg)",
            f"- Same limits and motion settings: {fairness['same_limits_and_motion']}",
            f"- C success frames within TCP tolerance: "
            f"{fairness['c_success_frames_within_tcp_tolerance']} "
            f"({fairness['c_success_frames']} success, {fairness['c_failure_frames']} failure)",
            f"- C success frames within orientation tolerance: "
            f"{fairness['c_success_frames_within_orientation_tolerance']}",
            f"- C posture correction keeps TCP movement within 0.5 mm: "
            f"{fairness['c_posture_correction_within_0p5_mm']} "
            f"(max {fairness['max_posture_tcp_position_delta_mm']:.4f} mm)",
            f"- Robot joint trajectory difference B vs C (mean abs / max abs): "
            f"{_format(fairness['joint_trajectory_difference_deg']['mean_abs'])} / "
            f"{_format(fairness['joint_trajectory_difference_deg']['max_abs'])} deg",
            "- B and C differ in whether the upper-arm/elbow posture is supplied as a secondary objective.",
        ]
    else:
        lines.append("- Not evaluated: both hand-6dof and hand-6dof-upper-arm must be run.")
    lines += ["", "## 主比較の結果", ""]
    if b in results and c in results:
        lines += [
            "| Metric | Side | hand-6DOF | hand-6DOF + upper-arm |",
            "| --- | --- | ---: | ---: |",
        ]
        primary_metrics = (
            ("ik_success_rate", "IK success rate"),
            ("unreachable_rate", "Unreachable rate"),
            ("hold_count", "HOLD count"),
            ("fallback_count", "Fallback count"),
            ("elbow_pose_different_frames", "Elbow posture >10 deg"),
            ("joint_reversal_count", "Joint reversals"),
        )
        for metric, label in primary_metrics:
            for side in ("left", "right"):
                lines.append(
                    f"| {label} | {side} | {_format(results[b]['sides'][side].get(metric))} | "
                    f"{_format(results[c]['sides'][side].get(metric))} |"
                )
        for metric, label, digits in (
            ("tcp_position_error_mm", "TCP position mean (mm)", 4),
            ("tcp_orientation_error_deg", "TCP orientation mean (deg)", 4),
            ("shoulder_elbow_angle_error_deg", "Shoulder-elbow direction mean error (deg)", 4),
            ("arm_plane_angle_error_deg", "Arm-plane mean error (deg)", 4),
            ("swivel_angle_error_deg", "Swivel mean absolute difference (deg)", 4),
            ("upper_arm_orientation_error_deg", "Upper-arm rotation mean error (deg)", 4),
            ("posture_tcp_position_delta_mm", "C posture TCP movement mean (mm)", 4),
            ("joint_speed_abs_deg_s", "Mean |joint speed| (deg/s)", 4),
            ("joint_acceleration_abs_deg_s2", "Mean |joint acceleration| (deg/s^2)", 4),
        ):
            for side in ("left", "right"):
                value_b = results[b]["sides"][side]["stats"][metric]["mean"]
                value_c = results[c]["sides"][side]["stats"][metric]["mean"]
                lines.append(
                    f"| {label} | {side} | {_format(value_b, digits)} | {_format(value_c, digits)} |"
                )
        for metric, label in (
            ("self_collision_contacts", "Self-collision contacts"),
            ("cross_arm_collision_contacts", "Cross-arm collision contacts"),
            ("obstacle_collision_contacts", "Obstacle collision contacts"),
        ):
            lines.append(
                f"| {label} | both | {_format(results[b]['collision_totals'][metric])} | "
                f"{_format(results[c]['collision_totals'][metric])} |"
            )
    else:
        lines.append("Not evaluated: both primary-comparison modes must be run.")
    lines += [
        "",
        "## 全モード概要",
        "",
        "| Mode | Side | Success rate | TCP position mean (mm) | TCP orientation mean (deg) | Upper-arm mean error (deg) | Swivel mean abs diff (deg) |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for mode in metadata["modes"]:
        for side in ("left", "right"):
            side_result = results[mode]["sides"][side]
            lines.append(
                f"| {MODE_LABELS[mode]} | {side} | {_format(side_result['ik_success_rate'])} | "
                f"{_format(side_result['stats']['tcp_position_error_mm']['mean'])} | "
                f"{_format(side_result['stats']['tcp_orientation_error_deg']['mean'])} | "
                f"{_format(side_result['stats']['shoulder_elbow_angle_error_deg']['mean'])} | "
                f"{_format(side_result['stats']['swivel_angle_error_deg']['mean'])} |"
            )
    lines += [
        "",
        "## 失敗区間",
        "",
        "| Mode | Left failure frames | Right failure frames |",
        "| --- | --- | --- |",
    ]
    for mode in metadata["modes"]:
        lines.append(
            f"| {MODE_LABELS[mode]} | {_failure_ranges(rows, mode, 'left')} | "
            f"{_failure_ranges(rows, mode, 'right')} |"
        )
    lines += [
        "",
        "## 失敗要因",
        "",
        "| Mode | Side | Failure kinds (count) |",
        "| --- | --- | --- |",
    ]
    for mode in metadata["modes"]:
        for side in ("left", "right"):
            counts = Counter(
                row["failure_kind"]
                for row in rows
                if row["mode"] == mode
                and row["side"] == side
                and row["failure_kind"] not in (None, "", "None")
            )
            lines.append(
                f"| {MODE_LABELS[mode]} | {side} | "
                f"{', '.join(f'{kind}: {count}' for kind, count in sorted(counts.items())) or 'none'} |"
            )
    lines += [
        "",
        "## グラフ",
        "",
    ]
    for graph in graphs:
        if graph.endswith(".svg"):
            lines.append(f"- [{Path(graph).stem}]({graph})")
    lines += [
        "",
        "## 指標定義",
        "",
        "- TCP position/orientation error: target TCP pose versus MuJoCo FK TCP pose.",
        "- IK success: the selected mode satisfies its position constraint and, for 6DoF modes, its orientation tolerance.",
        "- HOLD: a frame that failed strict IK validation and reused the last valid joint command.",
        "- Fallback: a position-only fallback used by the existing default solver path.",
        "- Shoulder-elbow error: angle between mapped human upper-arm direction and robot upper-arm direction.",
        "- Arm-plane error: angle between human and robot shoulder-elbow-hand plane normals.",
        "- Swivel error: signed elbow swivel-angle difference around the shoulder-to-hand axis, wrapped to [-180, 180] deg.",
        "- Upper-arm rotation error: geodesic error after mapping the human upper-arm frame into the robot initial-frame offset.",
        "- Joint reversal: sign change between consecutive joint velocities when both absolute speeds are at least 15 deg/s.",
        "",
        "## 研究上の限界",
        "",
        "- Mocopi elbow orientation is inferred by the body model and is not an independent physical measurement.",
        "- Full MJCF joint limits and motor mapping are provisional/unmeasured.",
        "- Collision results cover the modeled floor, non-adjacent robot links, and declared box obstacles only.",
        "- Results use simulation FK, not real robot encoder feedback.",
        "- The experiment does not establish causal superiority of one mode; it reports observed differences under the stated conditions.",
        "",
        "## 生成物",
        "",
        "- `summary.csv` / `summary.json`: mode/side statistics and outcome counts.",
        "- `per_frame_comparison.csv`: per-frame metrics for all modes.",
        "- `graphs/`: PNG at 300 dpi and SVG.",
    ]
    (output / "comparison" / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
