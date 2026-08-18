#!/usr/bin/env python3
"""Integrity-check, audit, and manifest FK workspace boundary evidence."""

import csv
import hashlib
import json
import math
import os
import subprocess

from PIL import Image


WORKSPACE = "/home/openarm/humanoid_sim_ws"
VALIDATION = os.path.join(WORKSPACE, "validation")
PRESENTATION = os.path.join(WORKSPACE, "presentation")
PACKAGE = "src/fk_workspace_boundary_analysis"
AUDIT = os.path.join(VALIDATION, "fk_workspace_boundary_audit.md")
MANIFEST = os.path.join(VALIDATION, "fk_workspace_boundary_manifest_sha256.txt")
CONFIGS = ["LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH"]
PROTECTED = {
    "src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro": "e0bcfb64859d94203eeec45f823dec8d238d72bec6809d81c2c99091942be4e4",
    "src/humanoid_sim_moveit_config/config/humanoid_sim.srdf": "7892b122fe28e91650f8919a66c256ebc55d3b34472e006701a63cadab5484e8",
    "src/humanoid_sim_moveit_config/config/joint_limits.yaml": "1e851fa791dab1c95eeba4a698e620c0334afd12a0c9964b774f24868d03ccfb",
    "src/humanoid_sim_moveit_config/config/kinematics.yaml": "7a825c85e6d3bfa06b3ff6d195b3aced8c490d6132d8d94e2d0b01f52e73204e",
    "src/humanoid_sim_moveit_config/config/ompl_planning.yaml": "3e47c074ffcb9a72ea62e8821a8923f80c7f346e246993b3131038380aaf1335",
}


def digest(path):
    value = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def bytes_digest(data):
    return hashlib.sha256(data).hexdigest()


def read_csv(name):
    with open(os.path.join(VALIDATION, name), newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def metadata(name):
    return {row["key"]: row["value"] for row in read_csv(name)}


def verify_manifest(name, base):
    path = os.path.join(VALIDATION, name)
    count = 0
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if not line.strip(): continue
            expected, relative = line.rstrip("\n").split("  ", 1)
            target = os.path.join(base, relative)
            if digest(target) != expected:
                raise RuntimeError(f"Protected manifest mismatch: {target}")
            count += 1
    if not count: raise RuntimeError(f"Empty manifest: {path}")
    return count


def fmt(value):
    return f"{float(value):.6f}"


def main():
    if os.path.exists(AUDIT) or os.path.exists(MANIFEST):
        raise RuntimeError("Refusing to overwrite FK audit/manifest")
    for relative, expected in PROTECTED.items():
        if digest(os.path.join(WORKSPACE, relative)) != expected:
            raise RuntimeError(f"Protected robot/MoveIt source changed: {relative}")
    existing = [
        ("fixed_base_workspace_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_fine_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_dof_ablation_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_demo_manifest_sha256.txt", WORKSPACE),
        ("fixed_base_workspace_envelope_demo_manifest_sha256.txt", WORKSPACE),
        ("radial_workspace_validation_manifest_sha256.txt", WORKSPACE),
        ("workspace_projection_manifest_sha256.txt", WORKSPACE),
    ]
    manifest_counts = {name: verify_manifest(name, base) for name, base in existing}

    xacro_path = os.path.join(WORKSPACE, "src/humanoid_sim_description/urdf/humanoid_sim.urdf.xacro")
    robot_xml = subprocess.check_output(["xacro", xacro_path])
    robot_model_hash = bytes_digest(robot_xml)
    states = read_csv("fk_workspace_boundary_states.csv")
    sample_meta = metadata("fk_workspace_boundary_sampling_metadata.csv")
    summary_rows = read_csv("fk_workspace_boundary_summary.csv")
    summary = {row["configuration"]: row for row in summary_rows}
    front = read_csv("fk_workspace_boundary_front.csv")
    right = read_csv("fk_workspace_boundary_right.csv")
    grid = read_csv("fk_vs_grid_workspace_comparison.csv")
    convergence = read_csv("fk_workspace_boundary_convergence.csv")
    if len(states) != 40000 or len(summary) != 4:
        raise RuntimeError("FK state/summary cardinality mismatch")
    for config in CONFIGS:
        rows = [row for row in states if row["configuration"] == config]
        valid = [row for row in rows if row["valid"] == "1"]
        if len(rows) != 10000 or len(valid) != int(summary[config]["valid_states"]):
            raise RuntimeError(f"FK state count mismatch: {config}")
        if len(valid) != int(sample_meta[f"{config}_valid"]):
            raise RuntimeError(f"Sampling metadata mismatch: {config}")
        if any(not all(math.isfinite(float(row[key])) for key in ("tcp_x", "tcp_y", "tcp_z", "joint_margin", "self_clearance")) for row in valid):
            raise RuntimeError(f"NaN/Inf valid FK state: {config}")
        if any(abs(float(row["yaw"])) > 1e-12 for row in valid) and config in {"LIFT_ONLY", "LIFT_PITCH"}:
            raise RuntimeError(f"Fixed yaw moved: {config}")
        if any(abs(float(row["pitch"])) > 1e-12 for row in valid) and config in {"LIFT_ONLY", "LIFT_YAW"}:
            raise RuntimeError(f"Fixed pitch moved: {config}")
    if sample_meta["ik_used"] != "false" or sample_meta["total_states"] != "40000":
        raise RuntimeError("FK sampling scope drift")

    figures = [row["path"] for row in read_csv("fk_workspace_boundary_figure_index.csv")]
    if len(figures) != 10 or len(set(figures)) != 10:
        raise RuntimeError("Expected ten unique FK boundary figures")
    for relative in figures:
        with Image.open(os.path.join(WORKSPACE, relative)) as image:
            if image.size != (1920, 1080):
                raise RuntimeError(f"Figure size mismatch: {relative}")
            image.verify()
    video_path = os.path.join(PRESENTATION, "fk_workspace_boundary_demo.mp4")
    with open(os.path.join(PRESENTATION, "fk_workspace_boundary_demo_metadata.json"), encoding="utf-8") as stream:
        video = json.load(stream)
    if video["decoded_frames"] != 1050 or video["codec"] != "H.264" or video["resolution"] != "1920x1080":
        raise RuntimeError("FK video verification metadata mismatch")

    convergence_lines = []
    for config in CONFIGS:
        rows = [row for row in convergence if row["configuration"] == config]
        if [int(row["sample_milestone"]) for row in rows] != [2500, 5000, 7500, 10000]:
            raise RuntimeError(f"Convergence milestone mismatch: {config}")
        before, final = rows[-2], rows[-1]
        convergence_lines.append(
            f"| {config} | {float(before['max_x']):.6f} → {float(final['max_x']):.6f} | "
            f"{float(final['max_x'])-float(before['max_x']):+.6f} | "
            f"{float(before['front_observed_band_area']):.6f} → {float(final['front_observed_band_area']):.6f} | "
            f"{float(before['right_observed_band_area']):.6f} → {float(final['right_observed_band_area']):.6f} |"
        )

    audit = [
        "# FK Workspace Boundary Audit", "", "## Model and scope", "",
        f"- Expanded robot model SHA-256: `{robot_model_hash}`",
        "- Joint position limits source: validated URDF loaded through MoveIt RobotModel; `joint_limits.yaml` supplies velocity policy only.",
        "- Base/TCP: `base_link` / `openarm_left_hand_tcp`.",
        "- Coordinate convention: +X forward, +Y left, +Z vertical.",
        "- AMR/base/world transform: fixed.",
        "- Workspace: positional FK endpoints; TCP orientation is unconstrained.",
        "- IK, OMPL, Cartesian planning, box/object, controller and hardware: not used.",
        "", "## Sampling and validity", "",
        "- Method: deterministic 10-dimensional Halton low-discrepancy sequence.",
        f"- Seed/index offset: `{sample_meta['random_seed']}`.",
        "- Shared dimensions: identical lift and seven arm samples at each sample ID across C0-C3.",
        "- Samples: 10,000 per configuration; 40,000 total; no early stop or automatic rerun.",
        "- Valid state criteria: joint bounds, no exact active bound, existing SRDF ACM self-collision check, finite FK update.",
        "- Boundary reference origin: `base_link` origin.",
        "- Front boundary: Y-Z angle bins; right boundary: X-Z angle bins; 2° bin centers.",
        "- Inner/outer boundaries are observed per-bin radial minima/maxima; convex hull and spline filling are not used.",
        "- Observed gap flag: adjacent sampled radii differ by >0.05 m in a bin with at least 3 samples. It is sampling evidence, not proof of infeasibility.",
        "", "## Configuration results", "",
        "| Configuration | Valid / 10000 | Collision rejects | X range (m) | Y range (m) | Z range (m) | Front radius (m) | Right radius (m) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for config in CONFIGS:
        row = summary[config]
        audit.append(
            f"| {config} | {row['valid_states']} | {sample_meta[f'{config}_self_collision_rejections']} | "
            f"{fmt(row['x_min'])}–{fmt(row['x_max'])} | {fmt(row['y_min'])}–{fmt(row['y_max'])} | "
            f"{fmt(row['z_min'])}–{fmt(row['z_max'])} | {fmt(row['min_observed_radius_front'])}–{fmt(row['max_observed_radius_front'])} | "
            f"{fmt(row['min_observed_radius_right'])}–{fmt(row['max_observed_radius_right'])} |"
        )
    audit += [
        "", "## Observed radial gaps", "",
        "| Configuration | Front bins | Front observed gaps | Right bins | Right observed gaps |",
        "|---|---:|---:|---:|---:|",
    ]
    for config in CONFIGS:
        frows = [row for row in front if row["configuration"] == config]
        rrows = [row for row in right if row["configuration"] == config]
        audit.append(f"| {config} | {len(frows)} | {sum(int(row['observed_gap_count']) for row in frows)} | "
                     f"{len(rrows)} | {sum(int(row['observed_gap_count']) for row in rrows)} |")
    audit += [
        "", "## Convergence observations (7,500 → 10,000 samples)", "",
        "| Configuration | max X (m) | Δ max X (m) | Front observed band area (m²) | Right observed band area (m²) |",
        "|---|---:|---:|---:|---:|", *convergence_lines,
        "", "No arbitrary early-stop threshold was applied; all 10,000 states per configuration were evaluated.",
        "", "## FK versus existing fixed-orientation targeted grid", "",
        "| Configuration | FK max X | Grid max X | ΔX | FK Y span | Grid Y span | FK Z span | Grid Z span | Interpretation |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in grid:
        audit.append(f"| {row['configuration']} | {fmt(row['fk_max_x'])} | {fmt(row['grid_max_x'])} | {fmt(row['delta_x'])} | "
                     f"{fmt(row['fk_y_span'])} | {fmt(row['grid_y_span'])} | {fmt(row['fk_z_span'])} | {fmt(row['grid_z_span'])} | "
                     f"{row['qualitative_consistency']} |")
    audit += [
        "", "The FK set is orientation-unconstrained and samples joint space, while the grid set uses a fixed grasp orientation "
        "inside a targeted Cartesian box. Differences therefore quantify method/scope, not a contradiction or replacement.",
        "", "## Presentation and integrity", "",
        "- Figures: 10 PNG, 1920×1080, with raw endpoint scatter plus observed inner/outer boundaries.",
        f"- Video: `presentation/fk_workspace_boundary_demo.mp4` ({video['duration_seconds']:.1f} s, "
        f"{video['resolution']}, {video['fps']:.0f} fps, H.264, {video['file_size_bytes']} bytes); full {video['decoded_frames']}-frame decode PASS.",
        "- Existing seven workspace/demo manifests: PASS before and after analysis.",
        "- Protected Xacro/URDF, SRDF/ACM, joint limits, kinematics and OMPL hashes: unchanged.",
        "- Actual trajectory execution: false; controller: false; ros2_control: false; hardware: false; AMR motion: false.", "",
    ]
    with open(AUDIT, "x", encoding="utf-8") as stream:
        stream.write("\n".join(audit))

    package_files = [
        f"{PACKAGE}/CMakeLists.txt", f"{PACKAGE}/package.xml", f"{PACKAGE}/config/fk_workspace_boundary.yaml",
        f"{PACKAGE}/launch/fk_workspace_boundary.launch.py", f"{PACKAGE}/launch/fk_workspace_boundary_demo.launch.py",
        f"{PACKAGE}/rviz/fk_workspace_boundary_front.rviz", f"{PACKAGE}/rviz/fk_workspace_boundary_right.rviz",
        f"{PACKAGE}/src/fk_workspace_boundary.cpp",
        f"{PACKAGE}/scripts/postprocess_fk_workspace_boundary.py", f"{PACKAGE}/scripts/fk_workspace_boundary_rviz.py",
        f"{PACKAGE}/scripts/generate_fk_workspace_boundary_video.py", f"{PACKAGE}/scripts/finalize_fk_workspace_boundary.py",
    ]
    validation_files = [
        "validation/fk_workspace_boundary_states.csv", "validation/fk_workspace_boundary_sampling_metadata.csv",
        "validation/fk_workspace_boundary_front.csv", "validation/fk_workspace_boundary_right.csv",
        "validation/fk_workspace_boundary_summary.csv", "validation/fk_vs_grid_workspace_comparison.csv",
        "validation/fk_workspace_boundary_convergence.csv", "validation/fk_workspace_boundary_figure_index.csv",
        "validation/fk_workspace_boundary_audit.md",
    ]
    presentation_files = figures + ["presentation/fk_workspace_boundary_demo.mp4",
                                    "presentation/fk_workspace_boundary_demo_metadata.json"]
    targets = package_files + validation_files + presentation_files
    with open(MANIFEST, "x", encoding="utf-8") as stream:
        for relative in sorted(targets):
            path = os.path.join(WORKSPACE, relative)
            if not os.path.isfile(path) or os.path.getsize(path) == 0:
                raise RuntimeError(f"Missing manifest target: {path}")
            stream.write(f"{digest(path)}  {relative}\n")
    verify_manifest(os.path.basename(MANIFEST), WORKSPACE)
    print(json.dumps({
        "status": "PASS", "robot_model_sha256": robot_model_hash,
        "valid_states": {config: int(summary[config]["valid_states"]) for config in CONFIGS},
        "manifest_entries": len(targets), "protected_manifest_entries": manifest_counts,
        "video": video,
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
