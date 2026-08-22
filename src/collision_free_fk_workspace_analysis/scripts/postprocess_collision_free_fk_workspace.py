#!/usr/bin/env python3
"""Postprocess validated FK endpoints without convex hulls or Cartesian IK resampling."""
import csv
import hashlib
import math
import os
import subprocess
from collections import defaultdict, deque

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

WS = "/home/openarm/humanoid_sim_ws"
VAL = f"{WS}/validation"
PRE = f"{WS}/presentation"
PKG = f"{WS}/src/collision_free_fk_workspace_analysis"
STATES = f"{VAL}/collision_free_fk_workspace_states.csv"
META = f"{VAL}/collision_free_fk_workspace_sampling_metadata.csv"
CONFIGS = ["LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH"]
SHORT = dict(zip(CONFIGS, ["C0", "C1", "C2", "C3"]))
LABEL = dict(zip(CONFIGS, ["Arm + Lift", "+ Waist Yaw", "+ Waist Pitch", "+ Yaw + Pitch"]))
COLOR = dict(zip(CONFIGS, ["#3a86ff", "#ff9f1c", "#d65db1", "#00c2a8"]))
DIFF_COLORS = {
    "BASELINE_C0": "#4977e8", "YAW_UNIQUE": "#ff9f1c", "PITCH_UNIQUE": "#e65ec7",
    "SINGLE_DOF_SHARED": "#ffd166", "COMBINED_ONLY": "#06d6a0",
}


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, fields, rows):
    if os.path.exists(path):
        raise RuntimeError(f"Refusing to overwrite: {path}")
    with open(path, "x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def metadata():
    return {row["key"]: row["value"] for row in read_csv(META)}


def value(row, key):
    return float(row[key])


def voxel(row, resolution):
    return tuple(math.floor(value(row, key) / resolution) for key in ("tcp_x", "tcp_y", "tcp_z"))


def voxel_center(key, resolution):
    return tuple((index + 0.5) * resolution for index in key)


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def exposed_edges(cells):
    edges = []
    for a, b in cells:
        for da, db, side in ((-1, 0, "MIN_A"), (1, 0, "MAX_A"), (0, -1, "MIN_B"), (0, 1, "MAX_B")):
            if (a + da, b + db) not in cells:
                edges.append((a, b, side))
    return edges


def hole_metrics(cells):
    if not cells:
        return 0, 0
    amin, amax = min(a for a, _ in cells) - 1, max(a for a, _ in cells) + 1
    bmin, bmax = min(b for _, b in cells) - 1, max(b for _, b in cells) + 1
    empty = {(a, b) for a in range(amin, amax + 1) for b in range(bmin, bmax + 1) if (a, b) not in cells}
    exterior = set()
    queue = deque([(amin, bmin)])
    while queue:
        cell = queue.popleft()
        if cell in exterior or cell not in empty:
            continue
        exterior.add(cell)
        a, b = cell
        queue.extend([(a - 1, b), (a + 1, b), (a, b - 1), (a, b + 1)])
    internal = empty - exterior
    components = 0
    remaining = set(internal)
    while remaining:
        components += 1
        queue = [remaining.pop()]
        while queue:
            a, b = queue.pop()
            for neighbor in ((a - 1, b), (a + 1, b), (a, b - 1), (a, b + 1)):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
    return components, len(internal)


def differential(occupancy):
    c0, c1, c2, c3 = (occupancy[c] for c in CONFIGS)
    return {
        "BASELINE_C0": set(c0),
        "YAW_UNIQUE": c1 - (c0 | c2),
        "PITCH_UNIQUE": c2 - (c0 | c1),
        "SINGLE_DOF_SHARED": (c1 & c2) - c0,
        "COMBINED_ONLY": c3 - (c1 | c2),
    }


def plot_style(fig, axes):
    fig.patch.set_facecolor("#0f141c")
    for ax in np.atleast_1d(axes).flat:
        ax.set_facecolor("#151d28")
        ax.tick_params(colors="#e7edf5")
        ax.grid(alpha=0.12)
        for spine in ax.spines.values():
            spine.set_color("#8fa1b3")


def plot_single_3d(path, config, occupancy, resolution):
    fig = plt.figure(figsize=(16, 9), dpi=120)
    ax = fig.add_subplot(111, projection="3d")
    plot_style(fig, [ax])
    xyz = np.array([voxel_center(cell, resolution) for cell in sorted(occupancy[config])])
    ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], s=8, marker="s", alpha=0.28, color=COLOR[config])
    ax.scatter([0], [0], [0], marker="x", s=90, color="white")
    ax.set(xlabel="X forward [m]", ylabel="Y left/right [m]", zlabel="Z vertical [m]")
    ax.set_title(f"{SHORT[config]} {LABEL[config]} · collision-free FK occupancy\n10 mm cells · corrected grasp TCP", color="white")
    ax.view_init(22, -125)
    fig.savefig(path, dpi=120, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_four(path, occupancy, resolution):
    fig = plt.figure(figsize=(16, 9), dpi=120)
    axes = [fig.add_subplot(2, 2, i + 1, projection="3d") for i in range(4)]
    fig.patch.set_facecolor("#0f141c")
    all_xyz = np.array([voxel_center(cell, resolution) for c in CONFIGS for cell in occupancy[c]])
    limits = [(all_xyz[:, i].min(), all_xyz[:, i].max()) for i in range(3)]
    for ax, config in zip(axes, CONFIGS):
        ax.set_facecolor("#151d28")
        xyz = np.array([voxel_center(cell, resolution) for cell in sorted(occupancy[config])])
        ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], s=3, marker="s", alpha=0.24, color=COLOR[config])
        ax.set(xlim=limits[0], ylim=limits[1], zlim=limits[2])
        ax.set_title(f"{SHORT[config]} {LABEL[config]}", color="white")
        ax.view_init(22, -125)
        ax.tick_params(colors="#e7edf5", labelsize=7)
    fig.suptitle("Collision-Free FK Workspace · identical axes · corrected grasp TCP", color="white", fontsize=16)
    fig.savefig(path, dpi=120, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_projection(path, projection, view, resolution):
    fig, axes = plt.subplots(2, 2, figsize=(16, 9), dpi=120, sharex=True, sharey=True)
    plot_style(fig, axes)
    for ax, config in zip(axes.flat, CONFIGS):
        cells = projection[config]
        points = np.array([((a + 0.5) * resolution, (b + 0.5) * resolution) for a, b in sorted(cells)])
        ax.scatter(points[:, 0], points[:, 1], marker="s", s=18, color=COLOR[config], alpha=0.55)
        ax.axvline(0, color="white", linewidth=0.7, alpha=0.45)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(f"{SHORT[config]} {LABEL[config]} · {len(cells)} cells", color="white")
    horizontal = "Y left/right [m]" if view == "front" else "X backward/forward [m]"
    fig.supxlabel(horizontal, color="white")
    fig.supylabel("Z vertical [m]", color="white")
    fig.suptitle(f"{'Front Y–Z' if view == 'front' else 'Right X–Z'} union projection · holes preserved", color="white", fontsize=16)
    fig.savefig(path, dpi=120, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_differential(path, categories, resolution):
    fig = plt.figure(figsize=(16, 9), dpi=120)
    ax = fig.add_subplot(111, projection="3d")
    fig.patch.set_facecolor("#0f141c")
    ax.set_facecolor("#151d28")
    for category, cells in categories.items():
        if not cells:
            continue
        xyz = np.array([voxel_center(cell, resolution) for cell in sorted(cells)])
        ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], s=7, marker="s", alpha=0.34,
                   color=DIFF_COLORS[category], label=f"{category} ({len(cells)})")
    ax.scatter([0], [0], [0], marker="x", s=90, color="white")
    ax.set(xlabel="X forward [m]", ylabel="Y left/right [m]", zlabel="Z vertical [m]")
    ax.set_title("Torso contribution by 10 mm occupancy cell\nNo convex hull · disjoint categories", color="white")
    ax.view_init(22, -125)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0))
    fig.savefig(path, dpi=120, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_c0_vs_c3(path, occupancy, resolution):
    fig = plt.figure(figsize=(16, 9), dpi=120)
    ax = fig.add_subplot(111, projection="3d")
    fig.patch.set_facecolor("#0f141c")
    ax.set_facecolor("#151d28")
    layers = [(occupancy[CONFIGS[0]], "C0 baseline", "#4977e8"),
              (occupancy[CONFIGS[3]] - occupancy[CONFIGS[0]], "C3 − C0 added", "#06d6a0")]
    for cells, label, color in layers:
        xyz = np.array([voxel_center(cell, resolution) for cell in sorted(cells)])
        ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], s=7, marker="s", alpha=0.34, color=color,
                   label=f"{label} ({len(cells)})")
    ax.set(xlabel="X forward [m]", ylabel="Y left/right [m]", zlabel="Z vertical [m]")
    ax.set_title("Corrected grasp TCP · C0 baseline and full-torso added occupancy", color="white")
    ax.view_init(22, -125)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0))
    fig.savefig(path, dpi=120, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    os.makedirs(PRE, exist_ok=True)
    settings = yaml.safe_load(open(f"{PKG}/config/collision_free_fk_workspace.yaml", encoding="utf-8"))[
        "collision_free_fk_workspace"]["ros__parameters"]
    meta = metadata()
    rows = read_csv(STATES)
    valid = {config: [row for row in rows if row["configuration"] == config and row["failure_reason"] == "VALID"]
             for config in CONFIGS}
    if any(not valid[config] for config in CONFIGS):
        raise RuntimeError("Every configuration must have collision-free FK states")
    if any(row["valid_joint_limits"] != "1" or row["self_collision"] != "0"
           for config in CONFIGS for row in valid[config]):
        raise RuntimeError("Invalid or colliding row entered workspace pool")

    resolutions = [float(x) for x in settings["voxel_sensitivity_m"]]
    primary = float(settings["primary_voxel_size_m"])
    occupancies = {resolution: {config: {voxel(row, resolution) for row in valid[config]}
                                for config in CONFIGS} for resolution in resolutions}
    occupancy = occupancies[primary]
    front = {config: {(cell[1], cell[2]) for cell in occupancy[config]} for config in CONFIGS}
    right = {config: {(cell[0], cell[2]) for cell in occupancy[config]} for config in CONFIGS}

    nested_rows = []
    state_keys = {config: {row["state_id"] for row in valid[config]} for config in CONFIGS}
    relations = [(CONFIGS[0], CONFIGS[1]), (CONFIGS[0], CONFIGS[2]),
                 (CONFIGS[1], CONFIGS[3]), (CONFIGS[2], CONFIGS[3])]
    for lower, upper in relations:
        missing = state_keys[lower] - state_keys[upper]
        nested_rows.append(dict(check_type="STATE_POOL", resolution_m="", lower_configuration=lower,
                                upper_configuration=upper, lower_count=len(state_keys[lower]),
                                upper_count=len(state_keys[upper]), missing_count=len(missing),
                                status="PASS" if not missing else "FAIL"))
    for resolution in resolutions:
        for lower, upper in relations:
            missing = occupancies[resolution][lower] - occupancies[resolution][upper]
            nested_rows.append(dict(check_type="SPATIAL_OCCUPANCY", resolution_m=resolution,
                                    lower_configuration=lower, upper_configuration=upper,
                                    lower_count=len(occupancies[resolution][lower]),
                                    upper_count=len(occupancies[resolution][upper]),
                                    missing_count=len(missing), status="PASS" if not missing else "FAIL"))
    if any(row["status"] != "PASS" for row in nested_rows):
        raise RuntimeError("Nested inclusion failed")
    write_csv(f"{VAL}/collision_free_fk_workspace_nested_check.csv", list(nested_rows[0]), nested_rows)

    summary = []
    projection_summary = []
    for config in CONFIGS:
        points = np.array([[value(row, key) for key in ("tcp_x", "tcp_y", "tcp_z")] for row in valid[config]])
        margins = np.array([value(row, "joint_margin") for row in valid[config]])
        clearances = np.array([value(row, "self_clearance") for row in valid[config]])
        manips = np.array([value(row, "manipulability") for row in valid[config]])
        radii = np.linalg.norm(points, axis=1)
        source_family = {"LIFT_ONLY": "BASE", "LIFT_YAW": "YAW", "LIFT_PITCH": "PITCH", "LIFT_YAW_PITCH": "COMBINED"}[config]
        attempts = int(meta[f"{source_family}_attempts"])
        rejects = int(meta[f"{source_family}_self_collision_rejections"])
        inherited = sum(row["inherited_or_new"] == "INHERITED" for row in valid[config])
        summary.append(dict(configuration=config,generated_attempts=attempts,nested_valid_states=len(valid[config]),
            inherited_states=inherited,new_valid_states=len(valid[config])-inherited,self_collision_rejections=rejects,
            valid_rate_new=(len(valid[config])-inherited)/attempts,x_min=points[:,0].min(),x_max=points[:,0].max(),
            y_min=points[:,1].min(),y_max=points[:,1].max(),z_min=points[:,2].min(),z_max=points[:,2].max(),
            max_forward_x=points[:,0].max(),max_lateral_abs_y=np.abs(points[:,1]).max(),
            min_radial_reach=radii.min(),max_radial_reach=radii.max(),occupied_voxels=len(occupancy[config]),
            voxel_size_m=primary,occupied_workspace_measure_m3=len(occupancy[config])*primary**3,
            front_reachable_cells=len(front[config]),front_projected_area_m2=len(front[config])*primary**2,
            right_reachable_cells=len(right[config]),right_projected_area_m2=len(right[config])*primary**2,
            mean_joint_margin=margins.mean(),min_joint_margin=margins.min(),mean_self_clearance=clearances.mean(),
            min_self_clearance=clearances.min(),mean_manipulability=manips.mean(),median_manipulability=np.median(manips)))
        fh, fhc = hole_metrics(front[config]); rh, rhc = hole_metrics(right[config])
        projection_summary.append(dict(configuration=config,voxel_size_m=primary,
            front_reachable_cells=len(front[config]),front_projected_area_m2=len(front[config])*primary**2,
            front_hole_components=fh,front_internal_empty_cells=fhc,right_reachable_cells=len(right[config]),
            right_projected_area_m2=len(right[config])*primary**2,right_hole_components=rh,
            right_internal_empty_cells=rhc,x_min=points[:,0].min(),x_max=points[:,0].max(),
            y_min=points[:,1].min(),y_max=points[:,1].max(),z_min=points[:,2].min(),z_max=points[:,2].max()))
    write_csv(f"{VAL}/collision_free_fk_workspace_summary.csv", list(summary[0]), summary)
    write_csv(f"{VAL}/collision_free_fk_workspace_projection_summary.csv", list(projection_summary[0]), projection_summary)

    boundary_rows = []
    for view, projections in (("FRONT_YZ", front), ("RIGHT_XZ", right)):
        for config in CONFIGS:
            for a, b, side in exposed_edges(projections[config]):
                boundary_rows.append(dict(view=view,configuration=config,voxel_size_m=primary,
                                          cell_a=a,cell_b=b,exposed_side=side))
    write_csv(f"{VAL}/collision_free_fk_workspace_projection_boundaries.csv", list(boundary_rows[0]), boundary_rows)

    differential_rows = []
    sensitivity_rows = []
    main_categories = None
    for resolution in resolutions:
        categories = differential(occupancies[resolution])
        if resolution == primary:
            main_categories = categories
        identity = set().union(*categories.values()) == occupancies[resolution][CONFIGS[3]]
        disjoint = sum(len(cells) for cells in categories.values()) == len(set().union(*categories.values()))
        for category, cells in categories.items():
            differential_rows.append(dict(voxel_size_m=resolution,category=category,occupied_cells=len(cells),
                                          occupied_measure_m3=len(cells)*resolution**3,
                                          description={"BASELINE_C0":"Arm + Lift baseline","YAW_UNIQUE":"C1 minus C0 and C2",
                                          "PITCH_UNIQUE":"C2 minus C0 and C1","SINGLE_DOF_SHARED":"C1 and C2 overlap beyond C0",
                                          "COMBINED_ONLY":"C3 minus union of C1 and C2"}[category]))
        sensitivity_rows.append(dict(voxel_size_m=resolution,c0_cells=len(occupancies[resolution][CONFIGS[0]]),
            c1_cells=len(occupancies[resolution][CONFIGS[1]]),c2_cells=len(occupancies[resolution][CONFIGS[2]]),
            c3_cells=len(occupancies[resolution][CONFIGS[3]]),c0_measure_m3=len(occupancies[resolution][CONFIGS[0]])*resolution**3,
            c3_measure_m3=len(occupancies[resolution][CONFIGS[3]])*resolution**3,
            c3_gain_percent=(len(occupancies[resolution][CONFIGS[3]])/len(occupancies[resolution][CONFIGS[0]])-1)*100,
            decomposition_identity="PASS" if identity else "FAIL",categories_disjoint="PASS" if disjoint else "FAIL"))
        if not identity or not disjoint:
            raise RuntimeError("Differential decomposition identity failed")
    write_csv(f"{VAL}/collision_free_fk_workspace_differential_summary.csv", list(differential_rows[0]), differential_rows)
    write_csv(f"{VAL}/collision_free_fk_workspace_voxel_sensitivity.csv", list(sensitivity_rows[0]), sensitivity_rows)

    convergence_rows = []
    fractions = [float(x) for x in settings["convergence_fractions"]]
    previous = {}
    for fraction in fractions:
        for config in CONFIGS:
            subset = [row for row in valid[config] if int(row["origin_sample_id"]) < math.ceil(fraction*int(row["origin_sample_cap"]))]
            points = np.array([[value(row, key) for key in ("tcp_x", "tcp_y", "tcp_z")] for row in subset])
            cells = {voxel(row, primary) for row in subset}
            metrics = np.array([points[:,0].min(),points[:,0].max(),points[:,1].min(),points[:,1].max(),points[:,2].min(),points[:,2].max()])
            if config in previous:
                extent_change = float(np.max(np.abs(metrics-previous[config][0])))
                occupancy_change = (len(cells)-len(previous[config][1]))/max(1,len(previous[config][1]))
            else:
                extent_change = ""; occupancy_change = ""
            convergence_rows.append(dict(configuration=config,fraction=fraction,source_valid_states=len(subset),
                occupied_voxels=len(cells),occupied_measure_m3=len(cells)*primary**3,x_min=metrics[0],x_max=metrics[1],
                y_min=metrics[2],y_max=metrics[3],z_min=metrics[4],z_max=metrics[5],
                max_extent_change_from_previous_m=extent_change,occupancy_change_from_previous=occupancy_change))
            previous[config]=(metrics,cells)
    write_csv(f"{VAL}/collision_free_fk_workspace_convergence.csv", list(convergence_rows[0]), convergence_rows)
    final_changes=[row for row in convergence_rows if row["fraction"]==1.0]
    converged=all(float(row["max_extent_change_from_previous_m"]) <= float(settings["convergence_extent_tolerance_m"])
                  and float(row["occupancy_change_from_previous"]) <= float(settings["convergence_occupancy_change_tolerance"])
                  for row in final_changes)

    representatives = []
    rows_by_cell = {config: defaultdict(list) for config in CONFIGS}
    for config in CONFIGS:
        for row in valid[config]: rows_by_cell[config][voxel(row, primary)].append(row)
    def choose(config, cells, scenario, score):
        candidates=[row for cell in cells for row in rows_by_cell[config].get(cell,[])]
        if not candidates: return
        row=score(candidates)
        representatives.append(dict(scenario=scenario,configuration=config,state_id=row["state_id"],tcp_x=row["tcp_x"],
            tcp_y=row["tcp_y"],tcp_z=row["tcp_z"],ik_exists="PASS_EXACT_SOURCE_STATE_FK_WITNESS",
            joint_limits="PASS",self_collision="PASS_COLLISION_FREE",fk_position_error_m=0,
            joint_margin=row["joint_margin"],self_clearance=row["self_clearance"],source_configuration=row["source_configuration"],
            joint_names=row["joint_names"],joint_values=row["joint_values"],notes="No Cartesian grid sweep; exact generating state is an IK witness"))
    c0=CONFIGS[0];c3=CONFIGS[3]
    interior_cells={cell for cell in occupancy[c0] if all((cell[0]+d[0],cell[1]+d[1],cell[2]+d[2]) in occupancy[c0]
        for d in ((1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)))}
    choose(c0, interior_cells or occupancy[c0], "C0_INTERIOR", lambda xs:max(xs,key=lambda r:value(r,"joint_margin")))
    choose(c0, occupancy[c0], "C0_FORWARD_BOUNDARY", lambda xs:max(xs,key=lambda r:value(r,"tcp_x")))
    choose(CONFIGS[1], main_categories["YAW_UNIQUE"], "C1_YAW_ADDED", lambda xs:max(xs,key=lambda r:value(r,"tcp_x")))
    choose(CONFIGS[2], main_categories["PITCH_UNIQUE"], "C2_PITCH_ADDED", lambda xs:max(xs,key=lambda r:value(r,"tcp_x")))
    choose(CONFIGS[3], main_categories["SINGLE_DOF_SHARED"], "SINGLE_DOF_SHARED", lambda xs:max(xs,key=lambda r:value(r,"joint_margin")))
    choose(c3, main_categories["COMBINED_ONLY"], "C3_COMBINED_ONLY", lambda xs:max(xs,key=lambda r:value(r,"tcp_x")))
    choose(c3, occupancy[c3], "MAX_FORWARD_NEAR_BOUNDARY", lambda xs:max(xs,key=lambda r:value(r,"tcp_x")))
    choose(c3, occupancy[c3], "MAX_LATERAL", lambda xs:max(xs,key=lambda r:abs(value(r,"tcp_y"))))
    choose(c3, occupancy[c3], "LOW_Z", lambda xs:min(xs,key=lambda r:value(r,"tcp_z")))
    choose(c3, occupancy[c3], "HIGH_Z", lambda xs:max(xs,key=lambda r:value(r,"tcp_z")))
    if len(representatives)>int(settings["representative_validation_max_points"]):
        raise RuntimeError("Representative validation hard cap exceeded")
    write_csv(f"{VAL}/collision_free_fk_workspace_representative_validation.csv", list(representatives[0]), representatives)

    old_rows={row["configuration"]:row for row in read_csv(f"{VAL}/boundary_sweep_workspace_summary.csv")}
    new_rows={row["configuration"]:row for row in summary}
    comparisons=[]
    for config in CONFIGS:
        old=old_rows[config];new=new_rows[config]
        comparisons.append(dict(configuration=config,old_tcp_basis="PRE_CORRECTION_DIRECTED_BOUNDARY_SWEEP",
            new_tcp_basis="CORRECTED_34P5MM_COLLISION_FREE_FK_OCCUPANCY",old_max_x=old["max_forward_x"],
            new_max_x=new["max_forward_x"],delta_max_x=float(new["max_forward_x"])-float(old["max_forward_x"]),
            old_max_lateral_y=old["max_lateral_y"],new_max_lateral_abs_y=new["max_lateral_abs_y"],
            old_front_area=old["front_area"],new_front_projected_area=new["front_projected_area_m2"],
            old_right_area=old["right_area"],new_right_projected_area=new["right_projected_area_m2"],
            comparability="TREND_ONLY_DIFFERENT_SAMPLING_AND_MEASURE_DEFINITIONS"))
    write_csv(f"{VAL}/collision_free_fk_workspace_tcp_comparison.csv", list(comparisons[0]), comparisons)

    figure_paths=[]
    for index,config in enumerate(CONFIGS):
        path=f"{PRE}/collision_free_fk_workspace_c{index}.png";plot_single_3d(path,config,occupancy,primary);figure_paths.append(path)
    path=f"{PRE}/collision_free_fk_workspace_four_configurations.png";plot_four(path,occupancy,primary);figure_paths.append(path)
    path=f"{PRE}/collision_free_fk_workspace_front_compare.png";plot_projection(path,front,"front",primary);figure_paths.append(path)
    path=f"{PRE}/collision_free_fk_workspace_right_compare.png";plot_projection(path,right,"right",primary);figure_paths.append(path)
    path=f"{PRE}/collision_free_fk_workspace_3d_compare.png";plot_four(path,occupancy,primary);figure_paths.append(path)
    path=f"{PRE}/collision_free_fk_workspace_3d_differential.png";plot_differential(path,main_categories,primary);figure_paths.append(path)
    path=f"{PRE}/collision_free_fk_workspace_c0_vs_c3.png";plot_c0_vs_c3(path,occupancy,primary);figure_paths.append(path)
    write_csv(f"{VAL}/collision_free_fk_workspace_figure_index.csv",["figure","sha256"],
              [dict(figure=os.path.relpath(path,WS),sha256=sha256(path)) for path in figure_paths])

    git_commit=subprocess.check_output(["git","rev-parse","HEAD"],cwd=WS,text=True).strip()
    model_files=[f"{WS}/src/humanoid_sim_description/urdf/openarm_arms_adapter.xacro",
      f"{WS}/src/humanoid_description/urdf/openarm_v10_arms_adapter.xacro",
      f"{WS}/src/openarm_description/assets/robot/openarm_v1.0/urdf/ee/ee_dispatcher.xacro",
      f"{WS}/src/humanoid_sim_moveit_config/config/humanoid_sim.srdf",
      f"{WS}/src/humanoid_sim_moveit_config/config/joint_limits.yaml",
      f"{WS}/src/humanoid_sim_moveit_config/config/kinematics.yaml",
      f"{WS}/src/humanoid_sim_moveit_config/config/ompl_planning.yaml"]
    summary_by={row["configuration"]:row for row in summary}
    final_delta={row["configuration"]:row for row in final_changes}
    audit=f"""# Collision-Free FK Workspace Audit

## Definition

- Workspace type: collision-free configuration FK workspace
- Source commit: `{git_commit}`
- Analysis hand: LEFT (same convention as the prior workspace research)
- Base frame: `{meta['base_frame']}`; AMR fixed
- TCP: `{meta['tcp_frame']}`, parent `{meta['tcp_parent']}`, xyz `{meta['tcp_xyz']}`, RPY `{meta['tcp_rpy']}`
- Right TCP preflight: `{meta['right_tcp_frame']}`, parent `{meta['right_tcp_parent']}`, xyz `{meta['right_tcp_xyz']}`, RPY `{meta['right_tcp_rpy']}`
- Environment collision objects: none
- Sampling: deterministic Halton nested state pool; seed `{meta['random_seed']}`
- Collision checks: `{meta['total_collision_checks']}` (hard cap 40000)
- Quantitative occupancy: validated endpoints at 10 mm; sensitivity at 5/10/15 mm
- Convex hull: not used; empty cells and projection holes are preserved

## Nested construction

- C0 = BASE valid states
- C1 = all C0 states + YAW enrichment
- C2 = all C0 states + PITCH enrichment
- C3 = all C0/C1/C2 source states + COMBINED enrichment
- State-level and 5/10/15 mm occupancy inclusion checks: **PASS**
- Differential partition identity and disjointness at every tested resolution: **PASS**

## Results at 10 mm

|Configuration|Nested valid states|Collision rejects (new attempts)|X max [m]|Y range [m]|Z range [m]|Occupied measure [m^3]|Front area [m^2]|Right area [m^2]|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
"""
    for config in CONFIGS:
        row=summary_by[config]
        audit += f"|{config}|{row['nested_valid_states']}|{row['self_collision_rejections']}|{float(row['max_forward_x']):.6f}|{float(row['y_min']):.6f}..{float(row['y_max']):.6f}|{float(row['z_min']):.6f}..{float(row['z_max']):.6f}|{float(row['occupied_workspace_measure_m3']):.6f}|{float(row['front_projected_area_m2']):.6f}|{float(row['right_projected_area_m2']):.6f}|\n"
    audit += f"""

The occupied measure is a sampling- and resolution-dependent FK endpoint occupancy estimate, not a convex-hull volume.

## Convergence

- Milestones: 25%, 50%, 75%, 100%
- Final criterion: extent change <= {settings['convergence_extent_tolerance_m']} m and occupancy growth <= {settings['convergence_occupancy_change_tolerance']}
- Judgment: **{'PASS' if converged else 'NOT CONVERGED'}**
"""
    for config in CONFIGS:
        row=final_delta[config]
        audit += f"- {config}: final extent change `{float(row['max_extent_change_from_previous_m']):.6f} m`, occupancy change `{float(row['occupancy_change_from_previous']):.6f}`\n"
    audit += f"""

## Representative cross-check

- Points tested: {len(representatives)} (hard cap {settings['representative_validation_max_points']})
- All selected endpoints retain their exact generating joint state, satisfying bounds and self-collision checks: **PASS**
- The exact source state is a constructive IK-existence witness; no Cartesian grid or broad IK sweep was executed.
- This does not claim a collision-free path from neutral to every state.

## Old versus corrected TCP

- Old results are retained read-only and are not mixed into the new quantitative estimate.
- Old method: directed boundary sweep using the pre-correction TCP.
- New method: nested collision-free joint-state sampling and FK of the corrected grasp TCP.
- A 34.5 mm link-local offset does not imply a constant +34.5 mm base-frame X change because TCP orientation varies.

## Integrity and safety

"""
    for path in model_files:
        audit += f"- `{os.path.relpath(path,WS)}` SHA-256: `{sha256(path)}`\n"
    audit += """
- Every workspace endpoint originates from a state with valid joint limits and no self-collision: **PASS**
- Deterministic sequence contract: fixed Halton index, dimensions, seed, and hard caps recorded in metadata.
- IK grid workspace rerun: **NO**
- OMPL/path planning: **NO**
- Trajectory execution: **NO**
- Controller/ros2_control/hardware: **NO**
- AMR motion: **NO**

## Interpretation boundary

This dataset proves that a collision-free robot configuration exists for each recorded endpoint. It does not prove that every endpoint has a collision-free trajectory from a neutral/reference pose. Representative path feasibility remains a separate task-level MoveIt experiment.
"""
    audit_path=f"{VAL}/collision_free_fk_workspace_audit.md"
    if os.path.exists(audit_path): raise RuntimeError(f"Refusing to overwrite: {audit_path}")
    with open(audit_path,"x",encoding="utf-8") as stream:stream.write(audit)
    print({"rows":len(rows),"valid":{c:len(valid[c]) for c in CONFIGS},"summary":summary,
           "converged":converged,"representatives":len(representatives),"figures":len(figure_paths)})


if __name__ == "__main__":
    main()
