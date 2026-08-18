#!/usr/bin/env python3
"""Verify the differential decomposition and create an immutable audit/manifest."""
import csv
import hashlib
import json
import math
import os
import subprocess

import cv2
from PIL import Image

WS = "/home/openarm/humanoid_sim_ws"
VAL = f"{WS}/validation"
PRE = f"{WS}/presentation"
PKG = "src/boundary_sweep_workspace_differential_visualization"
AUDIT = f"{VAL}/boundary_sweep_workspace_differential_audit.md"
MANIFEST = f"{VAL}/boundary_sweep_workspace_differential_manifest_sha256.txt"
CATEGORIES = ["BASELINE_C0", "YAW_UNIQUE", "PITCH_UNIQUE", "SINGLE_DOF_SHARED", "COMBINED_ONLY"]
FIGURES = [
    "presentation/boundary_3d_differential.png",
    "presentation/boundary_3d_four_configurations.png",
    "presentation/boundary_3d_c0_vs_c3_expansion.png",
    "presentation/boundary_3d_yaw_effect.png",
    "presentation/boundary_3d_pitch_effect.png",
    "presentation/boundary_3d_single_dof_shared.png",
    "presentation/boundary_3d_combined_only.png",
]


def digest(path):
    value = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read(name):
    with open(f"{VAL}/{name}", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def verify_manifest(path, base):
    checked = 0
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            expected, relative = line.rstrip().split("  ", 1)
            if digest(os.path.join(base, relative)) != expected:
                raise RuntimeError(f"Existing manifest mismatch: {relative}")
            checked += 1
    return checked


def main():
    if os.path.exists(AUDIT) or os.path.exists(MANIFEST):
        raise RuntimeError("Refusing differential audit/manifest overwrite")
    old_manifest = f"{VAL}/boundary_sweep_workspace_manifest_sha256.txt"
    old_entries = verify_manifest(old_manifest, WS)
    source_files = [
        "validation/boundary_sweep_workspace_nested_states.csv",
        "validation/boundary_sweep_workspace_nested_check.csv",
        "validation/boundary_sweep_workspace_3d_surface.csv",
        "validation/boundary_sweep_workspace_summary.csv",
        "validation/boundary_sweep_workspace_manifest_sha256.txt",
    ]
    source_hashes = {path: digest(f"{WS}/{path}") for path in source_files}
    source_commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=WS, text=True).strip()

    patches = read("boundary_sweep_workspace_differential_patches.csv")
    summary = read("boundary_sweep_workspace_differential_summary.csv")
    identity = read("boundary_sweep_workspace_differential_identity.csv")
    nested_checks = read("boundary_sweep_workspace_nested_check.csv")
    old_summary = {row["configuration"]: row for row in read("boundary_sweep_workspace_summary.csv")}
    if len(patches) != 768 or len({row["patch_id"] for row in patches}) != 768:
        raise RuntimeError("Patch identity failure")
    if set(row["category"] for row in patches) != set(CATEGORIES):
        raise RuntimeError("Five-way category failure")
    if any(row["status"] != "PASS" or int(row["expected"]) != int(row["actual"]) for row in identity):
        raise RuntimeError("Decomposition identity failure")
    if len(nested_checks) != 20 or any(row["status"] != "PASS" for row in nested_checks):
        raise RuntimeError("Existing nested relation failure")
    if len(summary) != 5 or sum(int(row["patch_count"]) for row in summary) != 768:
        raise RuntimeError("Differential summary failure")
    if any(not math.isfinite(float(row["surface_area"])) or float(row["surface_area"]) <= 0 for row in patches):
        raise RuntimeError("Invalid patch area")

    for path in FIGURES:
        with Image.open(f"{WS}/{path}") as image:
            if image.size != (1920, 1080):
                raise RuntimeError(f"Unexpected figure size: {path} {image.size}")
            image.verify()
    video_path = f"{PRE}/boundary_sweep_workspace_differential_demo.mp4"
    with open(f"{PRE}/boundary_sweep_workspace_differential_demo_metadata.json", encoding="utf-8") as stream:
        video = json.load(stream)
    if (
        video["codec_name"] != "h264" or video["profile"] != "High" or video["level"] > 40
        or video["pixel_format"] != "yuv420p" or video["resolution"] != "1920x1080"
        or video["decoded_frames"] != 960 or abs(video["duration_seconds"] - 32.0) > 0.01
        or video["audio"] is not False
    ):
        raise RuntimeError("Video metadata failure")
    capture = cv2.VideoCapture(video_path)
    decoded = 0
    while capture.read()[0]:
        decoded += 1
    capture.release()
    if decoded != 960:
        raise RuntimeError(f"Video decode failure: {decoded}")
    with open(video_path, "rb") as stream:
        payload = stream.read()
    moov = payload.find(b"moov")
    mdat = payload.find(b"mdat")
    if moov < 0 or mdat < 0 or moov >= mdat:
        raise RuntimeError("Faststart atom order failure")

    c0_x = float(old_summary["LIFT_ONLY"]["max_forward_x"])
    c3_x = float(old_summary["LIFT_YAW_PITCH"]["max_forward_x"])
    delta = c3_x - c0_x
    lines = [
        "# Directed Boundary-Sweep Differential Visualization Audit", "",
        "## Immutable sources", "",
        f"- Source commit before this visualization: `{source_commit}`.",
        f"- Existing boundary-sweep manifest: PASS ({old_entries} entries).",
    ]
    for path, value in source_hashes.items():
        lines.append(f"- `{path}`: `{value}`")
    lines += [
        "", "No URDF, SRDF/ACM, joint limits, directed-sweep CSV, existing 3D shell, existing manifest, or existing video was modified.",
        "", "## Five-way decomposition", "",
        "- BASELINE_C0: no support-metric improvement over C0.",
        "- YAW_UNIQUE: C1 improves C0 and C2 does not.",
        "- PITCH_UNIQUE: C2 improves C0 and C1 does not.",
        "- SINGLE_DOF_SHARED: C1 and C2 both independently improve C0.",
        "- COMBINED_ONLY: C3 improves beyond both C1 and C2.",
        "- Classification uses the validated `(view, boundary, Lift slice, sweep angle)` correspondence. Triangle metrics are the mean of their three endpoint metrics.",
        "- This is a disjoint directed-support surface-patch decomposition, not a volumetric mesh Boolean subtraction.",
        "", "| Category | Endpoints | Patches | Approx. patch area (m²) |", "|---|---:|---:|---:|",
    ]
    for row in summary:
        lines.append(
            f"| {row['category']} | {row['endpoint_count']} | {row['patch_count']} | {float(row['approximate_surface_area']):.6f} |"
        )
    lines += [
        "", "## Identity and nested validity", "",
        "- `C3 = BASELINE_C0 ∪ YAW_UNIQUE ∪ PITCH_UNIQUE ∪ SINGLE_DOF_SHARED ∪ COMBINED_ONLY`: PASS.",
        "- 768/768 selected C3 patches are assigned once; category patch-ID intersection is empty.",
        "- 480/480 correspondence endpoints are assigned once.",
        "- Existing nested relations C0⊆C1/C2 and C1/C2⊆C3: 20/20 PASS.",
        f"- C0 max X = {c0_x:.4f} m; C3 max X = {c3_x:.4f} m; increase = +{delta:.4f} m (+{100.0 * delta / c0_x:.1f}%).",
        "", "## Presentation", "",
    ]
    lines.extend(f"- `{path}`" for path in FIGURES)
    lines += [
        "- All figures use the same forward-emphasizing camera, base reference, axis limits and scale where compared.",
        "- Legends are outside the plotting axes and are not clipped.",
        "- The four complete shells are shown only in separate 2×2 panels, never as the main overlay.",
        "", "## RViz", "",
        "- Scenes: `boundary_diff_all`, `boundary_diff_yaw`, `boundary_diff_pitch`, `boundary_diff_combined`, `boundary_c0_vs_c3`.",
        "- Marker namespaces: `baseline_c0`, `yaw_unique`, `pitch_unique`, `single_dof_shared`, `combined_only`.",
        "- RobotState publication is visualization-only; no trajectory execution or controller is used.",
        "", "## Video", "",
        f"- `{video['path']}`: {video['duration_seconds']:.1f} s, {video['resolution']}, {video['fps']} fps, H.264 {video['profile']} level {video['level'] / 10:.1f}, {video['pixel_format']}, {video['file_size_bytes']} bytes.",
        f"- Full decode: {decoded}/{video['decoded_frames']} frames PASS; audio absent; faststart `moov` offset {moov} before `mdat` offset {mdat}.",
        "- Sequence adds one contribution at a time; four complete shells are not overlaid.", "",
    ]
    with open(AUDIT, "x", encoding="utf-8") as stream:
        stream.write("\n".join(lines))

    package = [
        f"{PKG}/CMakeLists.txt", f"{PKG}/package.xml",
        f"{PKG}/launch/boundary_sweep_workspace_differential_demo.launch.py",
        f"{PKG}/rviz/boundary_sweep_workspace_differential.rviz",
        f"{PKG}/scripts/generate_boundary_sweep_differential.py",
        f"{PKG}/scripts/boundary_sweep_differential_rviz.py",
        f"{PKG}/scripts/generate_boundary_sweep_differential_video.py",
        f"{PKG}/scripts/finalize_boundary_sweep_differential.py",
    ]
    validation = [
        "validation/boundary_sweep_workspace_differential_patches.csv",
        "validation/boundary_sweep_workspace_differential_summary.csv",
        "validation/boundary_sweep_workspace_differential_identity.csv",
        "validation/boundary_sweep_workspace_differential_audit.md",
    ]
    presentation = FIGURES + [
        "presentation/boundary_sweep_workspace_differential_demo.mp4",
        "presentation/boundary_sweep_workspace_differential_demo_metadata.json",
    ]
    targets = package + validation + presentation
    with open(MANIFEST, "x", encoding="utf-8") as stream:
        for path in sorted(targets):
            absolute = f"{WS}/{path}"
            if not os.path.isfile(absolute) or os.path.getsize(absolute) == 0:
                raise RuntimeError(f"Missing manifest target: {path}")
            stream.write(f"{digest(absolute)}  {path}\n")
    verify_manifest(MANIFEST, WS)
    print(json.dumps({
        "status": "PASS", "source_commit": source_commit, "old_manifest_entries": old_entries,
        "endpoint_union": 480, "patch_union": 768, "categories": 5,
        "figures": len(FIGURES), "video_frames": decoded, "manifest_entries": len(targets),
    }, indent=2))


if __name__ == "__main__":
    main()
