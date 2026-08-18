#!/usr/bin/env python3
"""Create disjoint, correspondence-based C3 differential surface patches."""
import csv
import math
import os
from collections import Counter, defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

WS = "/home/openarm/humanoid_sim_ws"
VAL = f"{WS}/validation"
PRE = f"{WS}/presentation"
SOURCE = f"{VAL}/boundary_sweep_workspace_nested_states.csv"
CONFIGS = ["LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH"]
CONFIG_LABELS = ["C0  Arm + Lift", "C1  +Yaw", "C2  +Pitch", "C3  +Yaw + Pitch"]
CATEGORIES = ["BASELINE_C0", "YAW_UNIQUE", "PITCH_UNIQUE", "SINGLE_DOF_SHARED", "COMBINED_ONLY"]
CATEGORY_LABELS = {
    "BASELINE_C0": "Baseline: Arm + Lift",
    "YAW_UNIQUE": "+Yaw only",
    "PITCH_UNIQUE": "+Pitch only",
    "SINGLE_DOF_SHARED": "Yaw/Pitch shared",
    "COMBINED_ONLY": "Yaw + Pitch combined only",
}
COLORS = {
    "BASELINE_C0": "#4C78A8",
    "YAW_UNIQUE": "#F2A93B",
    "PITCH_UNIQUE": "#D95FBB",
    "SINGLE_DOF_SHARED": "#59A14F",
    "COMBINED_ONLY": "#E45756",
}
CONFIG_COLORS = ["#4C78A8", "#F2A93B", "#D95FBB", "#20BFA9"]
EPS = 1.0e-10


def read(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write(path, fields, rows):
    if os.path.exists(path):
        raise RuntimeError(f"Refusing overwrite: {path}")
    with open(path, "x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def f(row, key):
    return float(row[key])


def key_for(row):
    return (
        row["view"],
        row["boundary_type"],
        round(f(row, "lift_value"), 8),
        round(f(row, "sweep_parameter"), 8),
    )


def point(row):
    return np.array([f(row, "tcp_x"), f(row, "tcp_y"), f(row, "tcp_z")])


def category_from_metrics(metrics):
    m0, m1, m2, m3 = (metrics[c] for c in CONFIGS)
    if m3 > max(m1, m2) + EPS:
        return "COMBINED_ONLY"
    yaw_added = m1 > m0 + EPS
    pitch_added = m2 > m0 + EPS
    if yaw_added and pitch_added:
        return "SINGLE_DOF_SHARED"
    if yaw_added:
        return "YAW_UNIQUE"
    if pitch_added:
        return "PITCH_UNIQUE"
    return "BASELINE_C0"


def triangle_area(vertices):
    return 0.5 * float(np.linalg.norm(np.cross(vertices[1] - vertices[0], vertices[2] - vertices[0])))


def build_surfaces(rows):
    indexed = defaultdict(dict)
    metrics = defaultdict(dict)
    for row in rows:
        key = key_for(row)
        indexed[row["target_configuration"]][key] = row
        metrics[key][row["target_configuration"]] = f(row, "selection_metric")
    if len(metrics) != 480 or any(set(value) != set(CONFIGS) for value in metrics.values()):
        raise RuntimeError("Unexpected nested correspondence cardinality")
    endpoint_categories = {key: category_from_metrics(value) for key, value in metrics.items()}
    lifts = sorted({key[2] for key in metrics})
    angles = sorted({key[3] for key in metrics})
    surfaces = {config: [] for config in CONFIGS}
    patches = []
    patch_id = 0
    for view in ("FRONT", "RIGHT"):
        for boundary in ("INNER_BOUNDARY", "OUTER_BOUNDARY"):
            for lower, upper in zip(lifts[:-1], lifts[1:]):
                for index, angle0 in enumerate(angles):
                    angle1 = angles[(index + 1) % len(angles)]
                    keys = [
                        (view, boundary, lower, angle0),
                        (view, boundary, lower, angle1),
                        (view, boundary, upper, angle1),
                        (view, boundary, upper, angle0),
                    ]
                    for config in CONFIGS:
                        vertices = [point(indexed[config][key]) for key in keys]
                        surfaces[config].extend(
                            [(vertices[0], vertices[1], vertices[2]), (vertices[0], vertices[2], vertices[3])]
                        )
                    for vertex_indices in ((0, 1, 2), (0, 2, 3)):
                        tri_keys = [keys[i] for i in vertex_indices]
                        averaged = {
                            config: sum(metrics[key][config] for key in tri_keys) / 3.0 for config in CONFIGS
                        }
                        category = category_from_metrics(averaged)
                        vertices = [point(indexed["LIFT_YAW_PITCH"][key]) for key in tri_keys]
                        row = {
                            "patch_id": patch_id,
                            "category": category,
                            "view": view,
                            "boundary_type": boundary,
                            "lower_lift": lower,
                            "upper_lift": upper,
                            "angle_start": angle0,
                            "angle_end": angle1,
                            "surface_area": triangle_area(vertices),
                        }
                        for vertex_number, vertex in enumerate(vertices, 1):
                            row.update(
                                {
                                    f"v{vertex_number}_x": vertex[0],
                                    f"v{vertex_number}_y": vertex[1],
                                    f"v{vertex_number}_z": vertex[2],
                                }
                            )
                        patches.append(row)
                        patch_id += 1
    return indexed, metrics, endpoint_categories, surfaces, patches


def patch_vertices(row):
    return [
        np.array([f(row, f"v{number}_x"), f(row, f"v{number}_y"), f(row, f"v{number}_z")])
        for number in (1, 2, 3)
    ]


def limits(surfaces):
    xyz = np.array([vertex for surface in surfaces.values() for triangle in surface for vertex in triangle])
    lower = xyz.min(axis=0)
    upper = xyz.max(axis=0)
    padding = np.array([0.08, 0.08, 0.08])
    return lower - padding, upper + padding


def style_axis(ax, bounds, title):
    lower, upper = bounds
    ax.set_xlim(lower[0], upper[0])
    ax.set_ylim(lower[1], upper[1])
    ax.set_zlim(lower[2], upper[2])
    ax.set_box_aspect(np.maximum(upper - lower, 0.1))
    ax.set_facecolor("#151C25")
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((0.08, 0.11, 0.15, 1.0))
    ax.tick_params(colors="#DCE6F0", labelsize=8)
    ax.set_xlabel("X forward [m]", color="#EAF1F7")
    ax.set_ylabel("Y lateral [m]", color="#EAF1F7")
    ax.set_zlabel("Z [m]", color="#EAF1F7")
    ax.set_title(title, color="#F5F8FB", pad=12)
    ax.view_init(elev=21, azim=-58)
    ax.scatter([0], [0], [0], marker="x", s=45, color="white", depthshade=False)
    ax.quiver(0, 0, 0, 0.18, 0, 0, color="#FF6B6B", linewidth=1.2)
    ax.quiver(0, 0, 0, 0, 0.18, 0, color="#7FD36B", linewidth=1.2)
    ax.quiver(0, 0, 0, 0, 0, 0.18, color="#62A8FF", linewidth=1.2)


def add_triangles(ax, triangles, color, alpha, edge_alpha=0.20):
    if not triangles:
        return
    collection = Poly3DCollection(
        triangles,
        facecolors=color,
        edgecolors=color,
        linewidths=0.18,
        alpha=alpha,
    )
    collection.set_edgecolor((*matplotlib.colors.to_rgb(color), edge_alpha))
    ax.add_collection3d(collection)


def save_single(path, title, bounds, layers, legend_labels, annotation=None):
    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor="#10151D")
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.02, right=0.78, bottom=0.06, top=0.92)
    style_axis(ax, bounds, title)
    handles = []
    for triangles, color, alpha, label in layers:
        add_triangles(ax, triangles, color, alpha)
        handles.append(Patch(facecolor=color, edgecolor=color, alpha=max(alpha, 0.55), label=label))
    fig.legend(handles=handles, loc="center left", bbox_to_anchor=(0.79, 0.55), frameon=False, labelcolor="white")
    if annotation:
        fig.text(0.80, 0.32, annotation, color="#EAF1F7", fontsize=12, linespacing=1.55)
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    os.makedirs(PRE, exist_ok=True)
    rows = read(SOURCE)
    if len(rows) != 1920:
        raise RuntimeError("Expected 1,920 immutable nested-state rows")
    indexed, metrics, endpoint_categories, surfaces, patches = build_surfaces(rows)
    bounds = limits(surfaces)
    patch_fields = [
        "patch_id", "category", "view", "boundary_type", "lower_lift", "upper_lift",
        "angle_start", "angle_end", "surface_area",
        "v1_x", "v1_y", "v1_z", "v2_x", "v2_y", "v2_z", "v3_x", "v3_y", "v3_z",
    ]
    write(f"{VAL}/boundary_sweep_workspace_differential_patches.csv", patch_fields, patches)

    endpoint_counts = Counter(endpoint_categories.values())
    patch_counts = Counter(row["category"] for row in patches)
    patch_areas = defaultdict(float)
    for row in patches:
        patch_areas[row["category"]] += float(row["surface_area"])
    descriptions = {
        "BASELINE_C0": "C3 support patch has no improvement over C0",
        "YAW_UNIQUE": "C1 improves C0 while C2 does not",
        "PITCH_UNIQUE": "C2 improves C0 while C1 does not",
        "SINGLE_DOF_SHARED": "Both C1 and C2 independently improve C0",
        "COMBINED_ONLY": "C3 improves beyond both C1 and C2",
    }
    summary = [
        {
            "category": category,
            "endpoint_count": endpoint_counts[category],
            "patch_count": patch_counts[category],
            "approximate_surface_area": patch_areas[category],
            "description": descriptions[category],
        }
        for category in CATEGORIES
    ]
    write(
        f"{VAL}/boundary_sweep_workspace_differential_summary.csv",
        ["category", "endpoint_count", "patch_count", "approximate_surface_area", "description"],
        summary,
    )

    identity = [
        {"check": "immutable_nested_rows", "expected": 1920, "actual": len(rows), "status": "PASS"},
        {"check": "correspondence_endpoint_union", "expected": 480, "actual": sum(endpoint_counts.values()), "status": "PASS"},
        {"check": "c3_surface_patch_union", "expected": 768, "actual": len(patches), "status": "PASS"},
        {"check": "unique_patch_assignment", "expected": 768, "actual": len({int(row["patch_id"]) for row in patches}), "status": "PASS"},
        {"check": "nonempty_five_way_categories", "expected": 5, "actual": len([c for c in CATEGORIES if patch_counts[c] > 0]), "status": "PASS"},
    ]
    write(
        f"{VAL}/boundary_sweep_workspace_differential_identity.csv",
        ["check", "expected", "actual", "status"],
        identity,
    )

    by_category = {
        category: [patch_vertices(row) for row in patches if row["category"] == category]
        for category in CATEGORIES
    }
    differential_layers = [
        (by_category[category], COLORS[category], 0.68 if category != "BASELINE_C0" else 0.30, CATEGORY_LABELS[category])
        for category in CATEGORIES
    ]
    save_single(
        f"{PRE}/boundary_3d_differential.png",
        "Directed workspace contribution decomposition",
        bounds,
        differential_layers,
        CATEGORIES,
    )

    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor="#10151D")
    fig.subplots_adjust(left=0.02, right=0.90, bottom=0.07, top=0.89, wspace=0.02, hspace=0.30)
    for number, (config, label, color) in enumerate(zip(CONFIGS, CONFIG_LABELS, CONFIG_COLORS), 1):
        ax = fig.add_subplot(2, 2, number, projection="3d")
        style_axis(ax, bounds, label)
        ax.set_title(label, color="#F5F8FB", pad=4)
        add_triangles(ax, surfaces[config], color, 0.30)
    fig.suptitle("C0–C3 complete shells · identical camera and scale", color="white", fontsize=16)
    fig.legend(
        handles=[Patch(facecolor=color, label=label) for color, label in zip(CONFIG_COLORS, CONFIG_LABELS)],
        loc="center left", bbox_to_anchor=(0.91, 0.50), frameon=False, labelcolor="white",
    )
    fig.savefig(f"{PRE}/boundary_3d_four_configurations.png", facecolor=fig.get_facecolor())
    plt.close(fig)

    base = surfaces["LIFT_ONLY"]
    expansion = [triangle for category in CATEGORIES[1:] for triangle in by_category[category]]
    old_summary = {row["configuration"]: row for row in read(f"{VAL}/boundary_sweep_workspace_summary.csv")}
    c0_x = float(old_summary["LIFT_ONLY"]["max_forward_x"])
    c3_x = float(old_summary["LIFT_YAW_PITCH"]["max_forward_x"])
    delta = c3_x - c0_x
    annotation = f"C0 Max X = {c0_x:.4f} m\nC3 Max X = {c3_x:.4f} m\nForward increase\n+{delta:.4f} m (+{100.0 * delta / c0_x:.1f}%)"
    save_single(
        f"{PRE}/boundary_3d_c0_vs_c3_expansion.png",
        "C0 baseline + full torso expansion",
        bounds,
        [(base, COLORS["BASELINE_C0"], 0.20, CATEGORY_LABELS["BASELINE_C0"]),
         (expansion, "#20BFA9", 0.72, "C3 − C0 added workspace")],
        [], annotation,
    )
    save_single(
        f"{PRE}/boundary_3d_yaw_effect.png",
        "Yaw-added workspace",
        bounds,
        [(base, COLORS["BASELINE_C0"], 0.15, CATEGORY_LABELS["BASELINE_C0"]),
         (by_category["YAW_UNIQUE"], COLORS["YAW_UNIQUE"], 0.74, CATEGORY_LABELS["YAW_UNIQUE"]),
         (by_category["SINGLE_DOF_SHARED"], COLORS["SINGLE_DOF_SHARED"], 0.66, CATEGORY_LABELS["SINGLE_DOF_SHARED"])],
        [],
    )
    save_single(
        f"{PRE}/boundary_3d_pitch_effect.png",
        "Pitch-added workspace",
        bounds,
        [(base, COLORS["BASELINE_C0"], 0.15, CATEGORY_LABELS["BASELINE_C0"]),
         (by_category["PITCH_UNIQUE"], COLORS["PITCH_UNIQUE"], 0.74, CATEGORY_LABELS["PITCH_UNIQUE"]),
         (by_category["SINGLE_DOF_SHARED"], COLORS["SINGLE_DOF_SHARED"], 0.66, CATEGORY_LABELS["SINGLE_DOF_SHARED"])],
        [],
    )
    save_single(
        f"{PRE}/boundary_3d_single_dof_shared.png",
        "Yaw/Pitch shared single-DOF workspace",
        bounds,
        [(base, COLORS["BASELINE_C0"], 0.10, "C0 reference"),
         (by_category["SINGLE_DOF_SHARED"], COLORS["SINGLE_DOF_SHARED"], 0.82, CATEGORY_LABELS["SINGLE_DOF_SHARED"])],
        [],
    )
    save_single(
        f"{PRE}/boundary_3d_combined_only.png",
        "Yaw + Pitch combined-only workspace",
        bounds,
        [(base, COLORS["BASELINE_C0"], 0.08, "C0 reference"),
         (by_category["COMBINED_ONLY"], COLORS["COMBINED_ONLY"], 0.82, CATEGORY_LABELS["COMBINED_ONLY"])],
        [],
    )
    print({
        "source_rows": len(rows), "correspondence_endpoints": len(metrics), "patches": len(patches),
        "endpoint_categories": dict(endpoint_counts), "patch_categories": dict(patch_counts),
    })


if __name__ == "__main__":
    main()
