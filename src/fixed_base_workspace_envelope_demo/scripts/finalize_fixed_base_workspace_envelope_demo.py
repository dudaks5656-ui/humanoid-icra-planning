#!/usr/bin/env python3
"""Validate envelope outputs and write an audit plus root-relative SHA-256 manifest."""

import csv
import hashlib
import json
import math
import pathlib


WORKSPACE = pathlib.Path("/home/openarm/humanoid_sim_ws")
VALIDATION = WORKSPACE / "validation"
PRESENTATION = WORKSPACE / "presentation"
METRICS = VALIDATION / "fixed_base_workspace_envelope_demo_metrics.csv"
AUDIT = VALIDATION / "fixed_base_workspace_envelope_demo_audit.md"
MANIFEST = VALIDATION / "fixed_base_workspace_envelope_demo_manifest_sha256.txt"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def csv_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main():
    comparison = VALIDATION / "fixed_base_workspace_dof_ablation_comparison.csv"
    summary = VALIDATION / "fixed_base_workspace_dof_ablation_summary.csv"
    contributions = VALIDATION / "fixed_base_workspace_dof_ablation_contributions.csv"
    runtime_path = PRESENTATION / "fixed_base_workspace_envelope_demo_runtime.json"
    full_recording = PRESENTATION / "fixed_base_workspace_envelope_demo_recording_full.json"
    short_recording = PRESENTATION / "fixed_base_workspace_envelope_demo_recording_short.json"
    metrics = csv_rows(METRICS)
    if [row["configuration"] for row in metrics] != ["C0", "C1", "C2", "C3", "COMBINED_ONLY"]:
        raise RuntimeError("Envelope metrics must contain C0-C3 plus COMBINED_ONLY")
    expected_counts = [833, 1030, 976, 1119, 65]
    expected_faces = [636, 674, 694, 730, 266]
    expected_volumes = [0.0977907291666664, 0.120917708333333, 0.114578333333333,
                        0.1313659375, 0.00763072916666665]
    for row, count, faces, volume in zip(metrics, expected_counts, expected_faces, expected_volumes):
        if int(row["occupied_voxels"]) != count or int(row["exposed_faces"]) != faces:
            raise RuntimeError(f"Occupancy/surface mismatch: {row}")
        if int(row["triangles"]) != faces * 2:
            raise RuntimeError(f"Triangle count mismatch: {row}")
        if not math.isclose(float(row["reconstructed_volume"]), volume, rel_tol=0.0, abs_tol=1e-12):
            raise RuntimeError(f"Volume mismatch: {row}")
        if float(row["absolute_difference"]) > 1e-12:
            raise RuntimeError(f"Validated/reconstructed volume mismatch: {row}")
    runtime = json.loads(runtime_path.read_text(encoding="utf-8"))
    if runtime["convex_hull_used"] or not runtime["holes_preserved"]:
        raise RuntimeError("Envelope integrity flags are invalid")
    if runtime["point_1360"] != [0.735416666666667, 0.147, 1.03125]:
        raise RuntimeError("Point 1360 drift")
    if not runtime["animation_collision_free"][3] or runtime["animated_point_ids"][3] != 1360:
        raise RuntimeError("Required C3 point 1360 animation did not validate")
    for flag in ("trajectory_execution", "controller", "ros2_control", "hardware", "amr_motion"):
        if runtime[flag]:
            raise RuntimeError(f"Forbidden runtime safety flag enabled: {flag}")
    recordings = [json.loads(path.read_text(encoding="utf-8")) for path in (full_recording, short_recording)]
    for recording in recordings:
        video = recording["video"]
        if not video["all_frames_decoded"] or video["width"] != 1920 or video["height"] != 1080:
            raise RuntimeError("Recording decode/resolution validation failed")
        if abs(video["fps"] - 30.0) > 0.1 or "H.264" not in recording["gst_discoverer"]:
            raise RuntimeError("Recording codec/fps validation failed")

    selected = "; ".join(f"{row['configuration']}=[{row['representative_point_ids']}]" for row in metrics[:4])
    animated = "; ".join(
        f"{row['configuration']}={row['animated_point_id']} ({row['animation_collision_free']})"
        for row in metrics[:4]
    )
    lines = [
        "# Fixed-base workspace envelope demo audit", "", "## Immutable sources", "",
        f"- `{comparison}` — `{sha256(comparison)}`",
        f"- `{summary}` — `{sha256(summary)}`",
        f"- `{contributions}` — `{sha256(contributions)}`",
        "- Existing coarse/fine/ablation/demo manifests verified before implementation: PASS",
        "", "## Grid and reconstruction", "",
        "- Grid dimensions: `12 × 10 × 12` = 1,440 physical TCP points",
        f"- Spacing: dx `{metrics[0]['dx']}` m, dy `{metrics[0]['dy']}` m, dz `{metrics[0]['dz']}` m",
        f"- Voxel volume: `{metrics[0]['voxel_volume']}` m³",
        "- Method: occupied voxel + six-neighbor exposed-face extraction",
        "- Convex hull: not used", "- Hole preservation: yes; no filling or smoothing",
        "", "| Region | Occupied | Exposed faces | Triangles | Reconstructed m³ | Validated m³ | Difference |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in metrics:
        lines.append(
            f"| {row['configuration']} | {row['occupied_voxels']} | {row['exposed_faces']} | "
            f"{row['triangles']} | {float(row['reconstructed_volume']):.15f} | "
            f"{float(row['validated_volume']):.15f} | {float(row['absolute_difference']):.3e} |"
        )
    lines += [
        "", "## Representative RobotState visualization", "", f"- Boundary point sets: {selected}",
        f"- Animated point states: {animated}",
        "- Combined-only representative: point `1360`, TCP `(0.735416666666667, 0.147, 1.03125)` m",
        "- Point 1360 classification: C0/C1/C2 FAIL, C3 PASS",
        "- Interpolation policy: neutral-to-valid RobotState; every intermediate sample checked for bounds and self-collision",
        "- Failed interpolation is not displayed as motion; static state/markers are used instead",
        "", "## Recording", "",
    ]
    for label, recording in zip(("Full", "Short"), recordings):
        video = recording["video"]
        lines.append(
            f"- {label}: `{recording['video_path']}`, {video['duration_s']:.3f} s, "
            f"{video['width']}×{video['height']}, {video['fps']:.1f} fps, H.264/MP4, "
            f"{video['file_size_bytes']} bytes, all frames decoded `{video['all_frames_decoded']}`"
        )
    lines += [
        "- Screenshots: `envelope_c0.png`, `envelope_c1.png`, `envelope_c2.png`, `envelope_c3.png`, "
        "`envelope_c0_vs_c3.png`, `envelope_combined_only.png`",
        "", "## Safety", "", "- AMR/base fixed: true", "- trajectory execution: false",
        "- controller: false", "- ros2_control: false", "- hardware: false", "- navigation: false",
    ]
    temporary = AUDIT.with_suffix(AUDIT.suffix + ".tmp")
    temporary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    temporary.replace(AUDIT)

    files = [
        WORKSPACE / "src/fixed_base_workspace_envelope_demo/CMakeLists.txt",
        WORKSPACE / "src/fixed_base_workspace_envelope_demo/package.xml",
        WORKSPACE / "src/fixed_base_workspace_envelope_demo/config/fixed_base_workspace_envelope_demo.yaml",
        WORKSPACE / "src/fixed_base_workspace_envelope_demo/launch/fixed_base_workspace_envelope_demo.launch.py",
        WORKSPACE / "src/fixed_base_workspace_envelope_demo/rviz/fixed_base_workspace_envelope_demo.rviz",
        WORKSPACE / "src/fixed_base_workspace_envelope_demo/src/fixed_base_workspace_envelope_demo.cpp",
        WORKSPACE / "src/fixed_base_workspace_envelope_demo/scripts/fixed_base_workspace_envelope_overlay.py",
        WORKSPACE / "src/fixed_base_workspace_envelope_demo/scripts/record_fixed_base_workspace_envelope_demo.py",
        WORKSPACE / "src/fixed_base_workspace_envelope_demo/scripts/finalize_fixed_base_workspace_envelope_demo.py",
        PRESENTATION / "README_envelope.md", runtime_path, full_recording, short_recording,
        PRESENTATION / "fixed_base_workspace_envelope_demo.mp4",
        PRESENTATION / "fixed_base_workspace_envelope_demo_short.mp4",
        *(PRESENTATION / name for name in (
            "envelope_c0.png", "envelope_c1.png", "envelope_c2.png", "envelope_c3.png",
            "envelope_c0_vs_c3.png", "envelope_combined_only.png")),
        METRICS, AUDIT,
    ]
    missing = [str(path) for path in files if not path.is_file() or path.stat().st_size <= 0]
    if missing:
        raise RuntimeError(f"Missing/empty manifest targets: {missing}")
    manifest_text = "".join(f"{sha256(path)}  {path.relative_to(WORKSPACE)}\n" for path in sorted(files))
    temporary = MANIFEST.with_suffix(MANIFEST.suffix + ".tmp")
    temporary.write_text(manifest_text, encoding="utf-8")
    temporary.replace(MANIFEST)
    print(json.dumps({"status": "PASS", "metrics_rows": len(metrics), "manifest_files": len(files),
                      "audit": str(AUDIT), "manifest": str(MANIFEST)}, indent=2))


if __name__ == "__main__":
    main()
