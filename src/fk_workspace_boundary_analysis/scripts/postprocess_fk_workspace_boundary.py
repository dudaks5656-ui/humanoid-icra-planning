#!/usr/bin/env python3
"""Extract observed FK inner/outer boundaries and presentation figures.

No IK or robot-model computation occurs here.  The script consumes only the
self-collision-validated FK state CSV emitted by the C++ sampler.
"""

import csv
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Patch, Rectangle


WORKSPACE = "/home/openarm/humanoid_sim_ws"
VALIDATION = os.path.join(WORKSPACE, "validation")
PRESENTATION = os.path.join(WORKSPACE, "presentation")
PACKAGE = os.path.join(WORKSPACE, "src", "fk_workspace_boundary_analysis")
STATES = os.path.join(VALIDATION, "fk_workspace_boundary_states.csv")
GRID = os.path.join(VALIDATION, "workspace_projection_summary.csv")
CONFIGS = ["LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH"]
LABELS = {
    "LIFT_ONLY": "C0 Arm + Lift",
    "LIFT_YAW": "C1 Arm + Lift + Yaw",
    "LIFT_PITCH": "C2 Arm + Lift + Pitch",
    "LIFT_YAW_PITCH": "C3 Arm + Lift + Yaw + Pitch",
}
COLORS = {
    "LIFT_ONLY": "#2792ff",
    "LIFT_YAW": "#ff9f1c",
    "LIFT_PITCH": "#e052d1",
    "LIFT_YAW_PITCH": "#20d6c7",
}
OUTPUTS = [
    "fk_workspace_boundary_front.csv", "fk_workspace_boundary_right.csv",
    "fk_workspace_boundary_summary.csv", "fk_vs_grid_workspace_comparison.csv",
    "fk_workspace_boundary_convergence.csv", "fk_workspace_boundary_figure_index.csv",
]


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, fieldnames, rows):
    with open(path, "x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def finite(values):
    values = np.asarray(values, dtype=float)
    return values[np.isfinite(values)]


def boundary(points, view, angle_bin_deg, gap_threshold, minimum_gap_samples):
    horizontal_key = "tcp_y" if view == "front" else "tcp_x"
    grouped = defaultdict(list)
    for row in points:
        u, z = float(row[horizontal_key]), float(row["tcp_z"])
        angle = math.degrees(math.atan2(z, u))
        index = math.floor((angle + 180.0) / angle_bin_deg)
        grouped[index].append((math.hypot(u, z), u, z))
    records = []
    for index in sorted(grouped):
        samples = sorted(grouped[index])
        radii = np.array([item[0] for item in samples])
        min_item, max_item = samples[0], samples[-1]
        gaps = int(np.sum(np.diff(radii) > gap_threshold)) if len(samples) >= minimum_gap_samples else 0
        center = -180.0 + (index + 0.5) * angle_bin_deg
        prefix = "y" if view == "front" else "x"
        records.append({
            "angle_deg": center,
            "min_radius": min_item[0], "max_radius": max_item[0],
            f"min_{prefix}": min_item[1], "min_z": min_item[2],
            f"max_{prefix}": max_item[1], "max_z": max_item[2],
            "valid_sample_count": len(samples), "observed_gap_count": gaps,
        })
    return records


def observed_band_area(records, angle_bin_deg):
    radians = math.radians(angle_bin_deg)
    return sum(0.5 * (row["max_radius"] ** 2 - row["min_radius"] ** 2) * radians for row in records)


def summary_row(config, all_rows, valid_rows, front, right):
    xyz = np.array([[float(row[key]) for key in ("tcp_x", "tcp_y", "tcp_z")] for row in valid_rows])
    margins = finite([row["joint_margin"] for row in valid_rows])
    clearances = finite([row["self_clearance"] for row in valid_rows])
    rf = np.hypot(xyz[:, 1], xyz[:, 2])
    rr = np.hypot(xyz[:, 0], xyz[:, 2])
    return {
        "configuration": config,
        "total_samples": len(all_rows), "valid_states": len(valid_rows),
        "valid_rate": len(valid_rows) / len(all_rows),
        "x_min": xyz[:, 0].min(), "x_max": xyz[:, 0].max(),
        "y_min": xyz[:, 1].min(), "y_max": xyz[:, 1].max(),
        "z_min": xyz[:, 2].min(), "z_max": xyz[:, 2].max(),
        "max_forward_reach": xyz[:, 0].max(),
        "min_observed_radius_front": rf.min(), "max_observed_radius_front": rf.max(),
        "min_observed_radius_right": rr.min(), "max_observed_radius_right": rr.max(),
        "mean_joint_margin": margins.mean(), "min_joint_margin": margins.min(),
        "mean_self_clearance": clearances.mean(), "min_self_clearance": clearances.min(),
        "front_observed_band_area": observed_band_area(front, SETTINGS["angle_bin_deg"]),
        "right_observed_band_area": observed_band_area(right, SETTINGS["angle_bin_deg"]),
    }


def add_reference(ax, view):
    ax.scatter([0], [0], color="#ffd45c", marker="x", s=90, linewidth=2.5, zorder=10)
    ax.annotate("base_link origin", (0, 0), xytext=(10, 12), textcoords="offset points",
                color="#ffd45c", weight="bold", fontsize=10)
    if view == "front":
        ax.axvline(0, color="#ffd45c", linestyle="--", linewidth=1.5, alpha=0.9)
        ax.add_patch(Rectangle((-0.22, 0), 0.44, 0.28, facecolor="#8a929c", alpha=0.20,
                               edgecolor="#bdc5ce", linewidth=1.2))
        ax.set_xlabel("Y — left (+) / right (−) [m]", weight="bold")
    else:
        ax.axvline(0, color="#ffd45c", linestyle="--", linewidth=1.5, alpha=0.9)
        ax.add_patch(Rectangle((-0.16, 0), 0.36, 0.28, facecolor="#8a929c", alpha=0.20,
                               edgecolor="#bdc5ce", linewidth=1.2))
        ax.text(0.55, 0.34, "FORWARD +X →", color="#ffd45c", fontsize=11, weight="bold",
                ha="center", zorder=9)
        ax.set_xlabel("X — backward / forward (+) [m]", weight="bold")
    ax.set_ylabel("Z — vertical [m]", weight="bold")
    ax.grid(color="#9aa6b2", alpha=0.18, linewidth=0.7)
    ax.set_aspect("equal", adjustable="box")


def line_coordinates(records, view, which):
    prefix = "y" if view == "front" else "x"
    u, z = [], []
    previous = None
    for row in records:
        if previous is not None and row["angle_deg"] - previous > SETTINGS["angle_bin_deg"] * 1.5:
            u.append(np.nan); z.append(np.nan)
        u.append(row[f"{which}_{prefix}"])
        z.append(row[f"{which}_z"])
        previous = row["angle_deg"]
    return u, z


def style_figure(title, subtitle):
    fig, ax = plt.subplots(figsize=(16, 9), dpi=120)
    fig.patch.set_facecolor("#10151d"); ax.set_facecolor("#151c25")
    for spine in ax.spines.values(): spine.set_color("#8fa1b3")
    ax.tick_params(colors="#e8eef5", labelsize=10)
    ax.xaxis.label.set_color("#e8eef5"); ax.yaxis.label.set_color("#e8eef5")
    fig.suptitle(title, color="#f4f7fb", fontsize=24, weight="bold", y=0.965)
    ax.set_title(subtitle, color="#aebdca", fontsize=12, pad=10)
    fig.subplots_adjust(left=0.08, right=0.94, bottom=0.13, top=0.86)
    return fig, ax


def plot_single(path, config, points, records, view, summary):
    horizontal = "tcp_y" if view == "front" else "tcp_x"
    view_label = "FRONT Y-Z" if view == "front" else "RIGHT-SIDE X-Z"
    fig, ax = style_figure(LABELS[config], f"{view_label} positional FK endpoint boundary · base_link origin")
    ax.scatter([float(row[horizontal]) for row in points], [float(row["tcp_z"]) for row in points],
               s=5, alpha=0.08, color=COLORS[config], edgecolors="none", label="valid FK endpoints")
    inner_u, inner_z = line_coordinates(records, view, "min")
    outer_u, outer_z = line_coordinates(records, view, "max")
    ax.plot(inner_u, inner_z, color="#ffd45c", linewidth=2.2, label="observed inner boundary")
    ax.plot(outer_u, outer_z, color="#ffffff", linewidth=2.5, label="observed outer boundary")
    add_reference(ax, view)
    all_u = np.array([float(row[horizontal]) for row in points]); all_z = np.array([float(row["tcp_z"]) for row in points])
    ax.set_xlim(min(-0.25, all_u.min() - 0.08), max(0.85, all_u.max() + 0.08))
    ax.set_ylim(min(0.0, all_z.min() - 0.08), all_z.max() + 0.10)
    min_key = "min_observed_radius_front" if view == "front" else "min_observed_radius_right"
    max_key = "max_observed_radius_front" if view == "front" else "max_observed_radius_right"
    ax.legend(loc="upper right", framealpha=0.88, fontsize=10)
    fig.text(0.5, 0.04,
             f"Observed radius: {summary[min_key]:.3f}–{summary[max_key]:.3f} m  |  "
             f"Max X: {summary['x_max']:.3f} m  |  Halton samples: {summary['total_samples']:,}, valid: {summary['valid_states']:,}",
             ha="center", color="#ffd45c", fontsize=9.5, weight="bold")
    fig.savefig(path, dpi=120, facecolor=fig.get_facecolor()); plt.close(fig)


def plot_all(path, valid_by_config, boundary_by_config, view, summaries):
    horizontal = "tcp_y" if view == "front" else "tcp_x"
    view_label = "FRONT Y-Z" if view == "front" else "RIGHT-SIDE X-Z"
    fig, ax = style_figure("C0–C3 Positional FK Boundary Comparison",
                           f"{view_label} · outer boundaries from valid self-collision-free RobotStates")
    for config in CONFIGS:
        points = valid_by_config[config]
        ax.scatter([float(row[horizontal]) for row in points[::8]], [float(row["tcp_z"]) for row in points[::8]],
                   s=3, alpha=0.035, color=COLORS[config], edgecolors="none")
        outer_u, outer_z = line_coordinates(boundary_by_config[config], view, "max")
        ax.plot(outer_u, outer_z, color=COLORS[config], linewidth=2.2,
                label=f"{LABELS[config]} outer")
    add_reference(ax, view)
    all_u = np.array([float(row[horizontal]) for config in CONFIGS for row in valid_by_config[config]])
    all_z = np.array([float(row["tcp_z"]) for config in CONFIGS for row in valid_by_config[config]])
    ax.set_xlim(min(-0.25, all_u.min() - 0.08), max(0.85, all_u.max() + 0.08))
    ax.set_ylim(min(0.0, all_z.min() - 0.08), all_z.max() + 0.10)
    ax.legend(loc="upper right", framealpha=0.88, fontsize=9)
    max_text = " · ".join(f"C{i} Xmax={summaries[c]['x_max']:.3f}" for i, c in enumerate(CONFIGS))
    fig.text(0.5, 0.04, max_text + " m", ha="center", color="#ffd45c", fontsize=11, weight="bold")
    fig.savefig(path, dpi=120, facecolor=fig.get_facecolor()); plt.close(fig)


def main():
    global SETTINGS
    with open(os.path.join(PACKAGE, "config", "fk_workspace_boundary.yaml"), encoding="utf-8") as stream:
        SETTINGS = yaml.safe_load(stream)["fk_workspace_boundary"]["ros__parameters"]
    for name in OUTPUTS:
        if os.path.exists(os.path.join(VALIDATION, name)):
            raise RuntimeError(f"Refusing to overwrite FK postprocess output: {name}")
    os.makedirs(PRESENTATION, exist_ok=True)
    figure_paths = [os.path.join(PRESENTATION, f"fk_workspace_{view}_{suffix}.png")
                    for view in ("front", "right") for suffix in ("c0", "c1", "c2", "c3", "all")]
    if any(os.path.exists(path) for path in figure_paths):
        raise RuntimeError("Refusing to overwrite FK presentation figure")

    states = read_csv(STATES)
    if len(states) != 40000:
        raise RuntimeError(f"Expected exactly 40,000 sampled RobotStates, got {len(states)}")
    by_config = {config: [row for row in states if row["configuration"] == config] for config in CONFIGS}
    valid = {config: [row for row in by_config[config] if row["valid"] == "1"] for config in CONFIGS}
    if any(len(by_config[config]) != 10000 or not valid[config] for config in CONFIGS):
        raise RuntimeError("Per-configuration state integrity failed")

    front = {config: boundary(valid[config], "front", SETTINGS["angle_bin_deg"],
                              SETTINGS["observed_gap_threshold_m"], SETTINGS["minimum_gap_bin_samples"])
             for config in CONFIGS}
    right = {config: boundary(valid[config], "right", SETTINGS["angle_bin_deg"],
                              SETTINGS["observed_gap_threshold_m"], SETTINGS["minimum_gap_bin_samples"])
             for config in CONFIGS}
    front_rows, right_rows = [], []
    for config in CONFIGS:
        front_rows.extend({"configuration": config, **row} for row in front[config])
        right_rows.extend({"configuration": config, **row} for row in right[config])
    write_csv(os.path.join(VALIDATION, OUTPUTS[0]),
              ["configuration", "angle_deg", "min_radius", "max_radius", "min_y", "min_z", "max_y", "max_z", "valid_sample_count", "observed_gap_count"], front_rows)
    write_csv(os.path.join(VALIDATION, OUTPUTS[1]),
              ["configuration", "angle_deg", "min_radius", "max_radius", "min_x", "min_z", "max_x", "max_z", "valid_sample_count", "observed_gap_count"], right_rows)

    summaries = {config: summary_row(config, by_config[config], valid[config], front[config], right[config]) for config in CONFIGS}
    summary_fields = list(next(iter(summaries.values())).keys())
    write_csv(os.path.join(VALIDATION, OUTPUTS[2]), summary_fields, [summaries[c] for c in CONFIGS])

    grid_rows = {row["configuration"]: row for row in read_csv(GRID)}
    comparisons = []
    for config in CONFIGS:
        fk, grid = summaries[config], grid_rows[config]
        dx = float(fk["x_max"]) - float(grid["x_max"])
        comparisons.append({
            "configuration": config, "fk_max_x": fk["x_max"], "grid_max_x": grid["x_max"], "delta_x": dx,
            "fk_y_span": fk["y_max"] - fk["y_min"], "grid_y_span": float(grid["y_max"]) - float(grid["y_min"]),
            "fk_z_span": fk["z_max"] - fk["z_min"], "grid_z_span": float(grid["z_max_front"]) - float(grid["z_min_front"]),
            "qualitative_consistency": "POSITIONAL_FK_EXTENDS_BEYOND_FIXED_ORIENTATION_GRID" if dx > 0.03
            else "MAX_X_WITHIN_ONE_GRID_STEP",
        })
    write_csv(os.path.join(VALIDATION, OUTPUTS[3]), list(comparisons[0]), comparisons)

    convergence = []
    for config in CONFIGS:
        for milestone in SETTINGS["convergence_milestones"]:
            subset_all = [row for row in by_config[config] if int(row["sample_id"]) < milestone]
            subset = [row for row in subset_all if row["valid"] == "1"]
            xyz = np.array([[float(row[key]) for key in ("tcp_x", "tcp_y", "tcp_z")] for row in subset])
            fb = boundary(subset, "front", SETTINGS["angle_bin_deg"], SETTINGS["observed_gap_threshold_m"], SETTINGS["minimum_gap_bin_samples"])
            rb = boundary(subset, "right", SETTINGS["angle_bin_deg"], SETTINGS["observed_gap_threshold_m"], SETTINGS["minimum_gap_bin_samples"])
            convergence.append({
                "configuration": config, "sample_milestone": milestone, "valid_states": len(subset),
                "max_x": xyz[:, 0].max(), "x_span": np.ptp(xyz[:, 0]), "y_span": np.ptp(xyz[:, 1]), "z_span": np.ptp(xyz[:, 2]),
                "front_observed_band_area": observed_band_area(fb, SETTINGS["angle_bin_deg"]),
                "right_observed_band_area": observed_band_area(rb, SETTINGS["angle_bin_deg"]),
            })
    write_csv(os.path.join(VALIDATION, OUTPUTS[4]), list(convergence[0]), convergence)

    figures = []
    for view, boundaries in (("front", front), ("right", right)):
        for index, config in enumerate(CONFIGS):
            path = os.path.join(PRESENTATION, f"fk_workspace_{view}_c{index}.png")
            plot_single(path, config, valid[config], boundaries[config], view, summaries[config])
            figures.append({"view": view.upper(), "scene": f"C{index}", "path": os.path.relpath(path, WORKSPACE)})
        path = os.path.join(PRESENTATION, f"fk_workspace_{view}_all.png")
        plot_all(path, valid, boundaries, view, summaries)
        figures.append({"view": view.upper(), "scene": "ALL", "path": os.path.relpath(path, WORKSPACE)})
    write_csv(os.path.join(VALIDATION, OUTPUTS[5]), ["view", "scene", "path"], figures)
    print({
        "states": len(states), "valid": {c: len(valid[c]) for c in CONFIGS},
        "max_x": {c: summaries[c]["x_max"] for c in CONFIGS},
        "front_bins": {c: len(front[c]) for c in CONFIGS},
        "right_bins": {c: len(right[c]) for c in CONFIGS},
    })


if __name__ == "__main__":
    main()
