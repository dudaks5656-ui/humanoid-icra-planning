#!/usr/bin/env python3
"""Audit and manifest the validated workspace projection deliverables."""

import csv
import hashlib
import json
import math
import os

from PIL import Image


WORKSPACE = "/home/openarm/humanoid_sim_ws"
VALIDATION = os.path.join(WORKSPACE, "validation")
PRESENTATION = os.path.join(WORKSPACE, "presentation")
AUDIT = os.path.join(VALIDATION, "workspace_projection_audit.md")
MANIFEST = os.path.join(VALIDATION, "workspace_projection_manifest_sha256.txt")
SOURCE = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_comparison.csv")


def digest(path):
    value = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def read_csv(name):
    with open(os.path.join(VALIDATION, name), newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def verify_manifest(name, base):
    path = os.path.join(VALIDATION, name)
    count = 0
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            expected, relative = line.rstrip("\n").split("  ", 1)
            target = os.path.join(base, relative)
            if digest(target) != expected:
                raise RuntimeError(f"Existing manifest mismatch: {target}")
            count += 1
    if not count:
        raise RuntimeError(f"Empty existing manifest: {path}")
    return count


def truth(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def main():
    if os.path.exists(AUDIT) or os.path.exists(MANIFEST):
        raise RuntimeError("Projection audit/manifest already exists; refusing to overwrite")
    existing = [
        ("fixed_base_workspace_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_fine_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_dof_ablation_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_demo_manifest_sha256.txt", WORKSPACE),
        ("fixed_base_workspace_envelope_demo_manifest_sha256.txt", WORKSPACE),
        ("radial_workspace_validation_manifest_sha256.txt", WORKSPACE),
    ]
    existing_counts = {name: verify_manifest(name, base) for name, base in existing}
    source = read_csv("fixed_base_workspace_dof_ablation_comparison.csv")
    front = read_csv("workspace_projection_front.csv")
    right = read_csv("workspace_projection_right.csv")
    summary = read_csv("workspace_projection_summary.csv")
    comparison = read_csv("workspace_projection_comparison.csv")
    metadata = {row["key"]: row["value"] for row in read_csv("workspace_projection_metadata.csv")}
    keys = ["c0_lift_success", "c1_lift_yaw_success", "c2_lift_pitch_success", "c3_lift_yaw_pitch_success"]
    source_counts = [sum(int(row[key]) for row in source) for key in keys]
    if len(source) != 1440 or len({row["point_id"] for row in source}) != 1440:
        raise RuntimeError("Projection source is not the exact unique 1,440-point set")
    if source_counts != [833, 1030, 976, 1119]:
        raise RuntimeError(f"Source occupancy drift: {source_counts}")
    if digest(SOURCE) != metadata["source_sha256"]:
        raise RuntimeError("Projection source hash drift")
    if len(front) != 4 * 10 * 12 or len(right) != 4 * 12 * 12:
        raise RuntimeError(f"Projection row count mismatch: front={len(front)} right={len(right)}")

    dx, dy, dz = (float(metadata[key]) for key in ("dx", "dy", "dz"))
    front_area = dy * dz
    right_area = dx * dz
    expected_max = [0.677083333333333, 0.70625, 0.70625, 0.735416666666667]
    for index, row in enumerate(summary):
        front_cells = int(row["front_reachable_cells"])
        right_cells = int(row["right_reachable_cells"])
        if not math.isclose(front_cells * front_area, float(row["front_projected_area"]), abs_tol=1e-12):
            raise RuntimeError(f"Front projected area mismatch: {row['configuration']}")
        if not math.isclose(right_cells * right_area, float(row["right_projected_area"]), abs_tol=1e-12):
            raise RuntimeError(f"Right projected area mismatch: {row['configuration']}")
        if not math.isclose(float(row["x_max"]), expected_max[index], abs_tol=1e-12):
            raise RuntimeError(f"Projected maximum X drift: {row['configuration']}")
    if any(not math.isfinite(float(value)) for row in summary for key, value in row.items() if key != "configuration"):
        raise RuntimeError("NaN/Inf in projection summary")

    figures = [row["path"] for row in read_csv("workspace_projection_figure_index.csv")]
    if len(figures) != 18 or len(set(figures)) != 18:
        raise RuntimeError("Expected 18 unique projection figures")
    for relative in figures:
        with Image.open(os.path.join(WORKSPACE, relative)) as image:
            if image.size != (1920, 1080):
                raise RuntimeError(f"Figure resolution mismatch: {relative} {image.size}")
            image.verify()
    video_meta_path = os.path.join(PRESENTATION, "workspace_front_right_projection_demo_metadata.json")
    with open(video_meta_path, encoding="utf-8") as stream:
        video = json.load(stream)
    if video["decoded_frames"] != 1080 or video["codec"] != "H.264" or video["resolution"] != "1920x1080":
        raise RuntimeError(f"Projection video metadata mismatch: {video}")

    names = [row["configuration"] for row in summary]
    audit_lines = [
        "# Workspace Front/Right Projection Audit",
        "",
        "## Source and coordinate frame",
        "",
        f"- Source CSV: `validation/{os.path.basename(SOURCE)}`",
        f"- Source SHA-256: `{digest(SOURCE)}`",
        "- Source physical TCP points: **1,440 unique points**",
        f"- Source reachable counts C0/C1/C2/C3: **{source_counts}**",
        "- Base frame: `base_link`",
        "- Verified convention: **+X forward, +Y left, +Z up**",
        "- Front view: **Y-Z plane**, viewed from the robot front; X is collapsed.",
        "- Right-side view: **X-Z plane**, viewed from the robot right; Y is collapsed.",
        "",
        "## Projection method",
        "",
        "- Definition: **2D union projection of validated 3D reachable workspace**.",
        "- Front cell is reachable if any validated X-depth sample at that (Y,Z) is reachable.",
        "- Right cell is reachable if any validated Y-depth sample at that (X,Z) is reachable.",
        "- This union does not imply that every collapsed-depth coordinate is reachable.",
        "- Convex hull: **not used**.",
        "- Smoothing/interpolation/morphological closing: **not used**.",
        "- Unreachable 2D holes: **preserved exactly at sampled grid resolution**.",
        "- New IK/workspace sampling: **none**.",
        "",
        "## Grid and projected cell areas",
        "",
        "- Grid dimensions X/Y/Z: **12 / 10 / 12**",
        f"- dx/dy/dz: **{dx:.15f} / {dy:.15f} / {dz:.15f} m**",
        f"- Front Y-Z cell area: **{front_area:.15f} m²**",
        f"- Right X-Z cell area: **{right_area:.15f} m²**",
        "",
        "## Results",
        "",
        "| Configuration | Front cells | Front area (m²) | Δ front vs C0 | Right cells | Right area (m²) | Δ right vs C0 | Max X (m) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row, delta in zip(summary, comparison):
        audit_lines.append(
            f"| {row['configuration']} | {row['front_reachable_cells']} | "
            f"{float(row['front_projected_area']):.6f} | {float(delta['front_percent_delta_vs_c0']):+.2f}% | "
            f"{row['right_reachable_cells']} | {float(row['right_projected_area']):.6f} | "
            f"{float(delta['right_percent_delta_vs_c0']):+.2f}% | {float(row['x_max']):.6f} |"
        )
    audit_lines += [
        "",
        "The projected maximum X values exactly match the validated C0-C3 maxima "
        "(0.677083, 0.706250, 0.706250, 0.735417 m).",
        "",
        "## Presentation and RViz",
        "",
        "- Figures: **18 PNG files, each 1920×1080**, including all requested 12 primary figures.",
        "- RViz scenes: `front_c0`, `front_c1`, `front_c2`, `front_c3`, `front_compare`, "
        "`right_c0`, `right_c1`, `right_c2`, `right_c3`, `right_compare`.",
        "- RViz uses the same generated projection CSV cells and the fixed `base_link` frame.",
        f"- Video: `presentation/workspace_front_right_projection_demo.mp4` "
        f"({video['duration_seconds']:.1f} s, {video['resolution']}, {video['fps']:.0f} fps, {video['codec']}, "
        f"{video['file_size_bytes']} bytes).",
        f"- Full video decode: **PASS ({video['decoded_frames']} frames)**.",
        "",
        "## Safety scope",
        "",
        "- AMR/base fixed: **true**",
        "- Actual trajectory execution: **false**",
        "- Controller: **false**",
        "- ros2_control: **false**",
        "- Hardware/motor/sensor use: **false**",
        "- Existing coarse/fine/ablation/envelope/radial manifests: **PASS and unchanged**",
        "",
    ]
    with open(AUDIT, "x", encoding="utf-8") as stream:
        stream.write("\n".join(audit_lines))

    package_files = [
        "src/workspace_projection_demo/CMakeLists.txt",
        "src/workspace_projection_demo/package.xml",
        "src/workspace_projection_demo/config/workspace_projection.yaml",
        "src/workspace_projection_demo/launch/workspace_projection_demo.launch.py",
        "src/workspace_projection_demo/rviz/workspace_projection_front.rviz",
        "src/workspace_projection_demo/rviz/workspace_projection_right.rviz",
        "src/workspace_projection_demo/scripts/generate_workspace_projections.py",
        "src/workspace_projection_demo/scripts/workspace_projection_rviz.py",
        "src/workspace_projection_demo/scripts/generate_workspace_projection_video.py",
        "src/workspace_projection_demo/scripts/finalize_workspace_projection.py",
    ]
    validation_files = [
        "validation/workspace_projection_front.csv",
        "validation/workspace_projection_right.csv",
        "validation/workspace_projection_summary.csv",
        "validation/workspace_projection_comparison.csv",
        "validation/workspace_projection_differential.csv",
        "validation/workspace_projection_metadata.csv",
        "validation/workspace_projection_figure_index.csv",
        "validation/workspace_projection_audit.md",
    ]
    presentation_files = figures + [
        "presentation/workspace_front_right_projection_demo.mp4",
        "presentation/workspace_front_right_projection_demo_metadata.json",
    ]
    manifest_files = package_files + validation_files + presentation_files
    if len(manifest_files) != len(set(manifest_files)):
        raise RuntimeError("Duplicate path in projection manifest")
    with open(MANIFEST, "x", encoding="utf-8") as stream:
        for relative in sorted(manifest_files):
            target = os.path.join(WORKSPACE, relative)
            if not os.path.isfile(target) or os.path.getsize(target) == 0:
                raise RuntimeError(f"Missing or empty manifest target: {target}")
            stream.write(f"{digest(target)}  {relative}\n")
    verify_manifest(os.path.basename(MANIFEST), WORKSPACE)
    print(json.dumps({
        "status": "PASS",
        "source_points": len(source),
        "source_reachable": source_counts,
        "front_areas": [float(row["front_projected_area"]) for row in summary],
        "right_areas": [float(row["right_projected_area"]) for row in summary],
        "existing_manifest_entries": existing_counts,
        "new_manifest_entries": len(manifest_files),
        "video": video,
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
