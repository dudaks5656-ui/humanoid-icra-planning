#!/usr/bin/env python3
"""Generate the radial validation audit and immutable SHA-256 manifest."""

import csv
import hashlib
import json
import pathlib


WORKSPACE = pathlib.Path("/home/openarm/humanoid_sim_ws")
VALIDATION = WORKSPACE / "validation"
PRESENTATION = WORKSPACE / "presentation"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(name):
    with (VALIDATION / name).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main():
    points = rows("radial_workspace_validation_points.csv")
    intervals = rows("radial_workspace_validation_intervals.csv")
    holes = rows("radial_workspace_validation_holes.csv")
    summary = rows("radial_workspace_validation_summary.csv")
    states = rows("radial_workspace_validation_states.csv")
    metadata_rows = rows("radial_workspace_validation_metadata.csv")
    metadata = {row["key"]: row["value"] for row in metadata_rows}
    if len(summary) != 10 or {row["configuration"] for row in summary} != {"LIFT_ONLY", "LIFT_YAW_PITCH"}:
        raise RuntimeError("Summary is not 5 rays x C0/C3")
    if len(points) > 400 or len(points) != 2 * int(metadata["physical_points"]):
        raise RuntimeError("Point/evaluation hard cap integrity failure")
    if len(states) != 20:
        raise RuntimeError(f"Expected first/last states for every ray/config, got {len(states)}")
    full = json.loads((PRESENTATION / "radial_workspace_validation_demo_recording_full.json").read_text())
    short = json.loads((PRESENTATION / "radial_workspace_validation_demo_recording_short.json").read_text())
    source = VALIDATION / "fixed_base_workspace_dof_ablation_comparison.csv"
    source_hashes = {
        "comparison": sha(source),
        "coarse_manifest": sha(VALIDATION / "fixed_base_workspace_manifest_sha256.txt"),
        "fine_manifest": sha(VALIDATION / "fixed_base_workspace_fine_manifest_sha256.txt"),
        "dof_manifest": sha(VALIDATION / "fixed_base_workspace_dof_ablation_manifest_sha256.txt"),
        "envelope_manifest": sha(VALIDATION / "fixed_base_workspace_envelope_demo_manifest_sha256.txt"),
    }
    lines = [
        "# Radial workspace validation audit", "",
        "## Scope and immutable sources", "",
        "- Purpose: lightweight presentation-oriented boundary validation; not a full workspace recomputation.",
        "- Configurations: C0 `ARM + LIFT` and C3 `ARM + LIFT + WAIST_YAW + WAIST_PITCH` only.",
        "- Base/TCP frames: `base_link` / `openarm_left_hand_tcp`; +X forward, +Y left, +Z up.",
        f"- Source comparison SHA-256: `{source_hashes['comparison']}`.",
        f"- Coarse manifest SHA-256: `{source_hashes['coarse_manifest']}`.",
        f"- Fine manifest SHA-256: `{source_hashes['fine_manifest']}`.",
        f"- DOF ablation manifest SHA-256: `{source_hashes['dof_manifest']}`.",
        f"- Envelope demo manifest SHA-256: `{source_hashes['envelope_manifest']}`.",
        f"- Reference origin in base_link: ({metadata['origin_x']}, {metadata['origin_y']}, {metadata['origin_z']}) m.",
        f"- Radial limits/step: {metadata['radial_min']}–{metadata['radial_max']} m / {metadata['radial_step']} m.",
        f"- Physical points / configuration evaluations: {metadata['physical_points']} / {metadata['configuration_evaluations']}.",
        f"- Maximum IK seeds: {metadata['max_ik_seeds']}.",
        f"- Internal anomaly revalidation: {metadata['special_revalidated_points']} points, up to {metadata['max_special_ik_seeds']} seeds each.", "",
        "## Ray vectors", "",
    ]
    for name in ("FRONT", "FRONT_LEFT", "FRONT_RIGHT", "FRONT_UP", "FRONT_DOWN"):
        lines.append(f"- {name}: `{metadata['ray_' + name]}`")
    lines += ["", "## Feasible intervals and holes", "",
              "| Configuration | Ray | First (m) | Last (m) | Intervals | Holes | Largest hole (m) | PASS/FAIL |",
              "|---|---|---:|---:|---:|---:|---:|---:|"]
    for row in summary:
        lines.append(f"| {row['configuration']} | {row['ray_name']} | {row['first_feasible_distance'] or 'NONE'} | "
                     f"{row['last_feasible_distance'] or 'NONE'} | {row['feasible_interval_count']} | {row['hole_count']} | "
                     f"{row['largest_hole'] or '0'} | {row['pass_count']}/{row['fail_count']} |")
    lines += ["", "Each interval is the exact contiguous 20 mm PASS sample run. Holes include only FAIL runs bounded by PASS on both sides; exterior FAIL samples are not mislabeled as holes.", ""]
    if intervals:
        lines += ["### Explicit intervals", ""]
        for row in intervals:
            lines.append(f"- {row['configuration']} / {row['ray_name']} / #{row['interval_index']}: "
                         f"{row['start_distance']}–{row['end_distance']} m ({row['sample_count']} samples).")
    lines += ["", "### Internal holes", ""]
    if holes:
        for row in holes:
            lines.append(f"- {row['configuration']} / {row['ray_name']} / #{row['hole_index']}: "
                         f"{row['start_distance']}–{row['end_distance']} m; sampled width {row['hole_length']} m.")
    else:
        lines.append("- No internal hole was observed on these five sampled rays at 20 mm spacing.")
    lines += ["", "## Representative RobotState and presentation evidence", "",
              "- First/last feasible RobotState rows: 20; each state passed IK, joint bounds, exact-bound, fixed-orientation, and self-collision checks.",
              "- Demo uses static switching between validated states; it does not claim a planned or executable trajectory.",
              f"- Full video: `{full['video_path']}`, {full['video']['duration_s']:.3f} s, "
              f"{full['video']['width']}x{full['video']['height']}, H.264, {full['video']['file_size_bytes']} bytes.",
              f"- Short video: `{short['video_path']}`, {short['video']['duration_s']:.3f} s, "
              f"{short['video']['width']}x{short['video']['height']}, H.264, {short['video']['file_size_bytes']} bytes.",
              "- Screenshots: `presentation/radial_front_c0.png`, `radial_front_c3.png`, `radial_hole_example.png`, `radial_min_max_pose.png`.", "",
              "## Safety", "",
              "- AMR/base fixed: yes.", "- Trajectory execution: **false**.", "- Controller: **false**.",
              "- ros2_control: **false**.", "- Hardware: **false**.", "- Box/environment collision objects: none.", ""]
    audit = VALIDATION / "radial_workspace_validation_audit.md"
    audit.write_text("\n".join(lines), encoding="utf-8")

    manifest_files = [
        "src/radial_workspace_validation/CMakeLists.txt", "src/radial_workspace_validation/package.xml",
        "src/radial_workspace_validation/src/radial_workspace_validation.cpp",
        "src/radial_workspace_validation/config/radial_workspace_validation.yaml",
        "src/radial_workspace_validation/launch/radial_workspace_validation.launch.py",
        "src/radial_workspace_validation/launch/radial_workspace_validation_demo.launch.py",
        "src/radial_workspace_validation/rviz/radial_workspace_validation.rviz",
        "src/radial_workspace_validation/scripts/radial_workspace_validation_demo.py",
        "src/radial_workspace_validation/scripts/radial_workspace_validation_overlay.py",
        "src/radial_workspace_validation/scripts/record_radial_workspace_validation_demo.py",
        "src/radial_workspace_validation/scripts/finalize_radial_workspace_validation.py",
        "validation/radial_workspace_validation_points.csv", "validation/radial_workspace_validation_intervals.csv",
        "validation/radial_workspace_validation_holes.csv", "validation/radial_workspace_validation_summary.csv",
        "validation/radial_workspace_validation_states.csv", "validation/radial_workspace_validation_metadata.csv",
        "validation/radial_workspace_validation_audit.md",
        "presentation/radial_workspace_validation_demo.mp4",
        "presentation/radial_workspace_validation_demo_short.mp4",
        "presentation/radial_workspace_validation_demo_recording_full.json",
        "presentation/radial_workspace_validation_demo_recording_short.json",
        "presentation/radial_front_c0.png", "presentation/radial_front_c3.png",
        "presentation/radial_hole_example.png", "presentation/radial_min_max_pose.png",
    ]
    manifest = VALIDATION / "radial_workspace_validation_manifest_sha256.txt"
    manifest.write_text("".join(f"{sha(WORKSPACE / relative)}  {relative}\n" for relative in manifest_files), encoding="utf-8")
    for line in manifest.read_text(encoding="utf-8").splitlines():
        expected, relative = line.split("  ", 1)
        if sha(WORKSPACE / relative) != expected:
            raise RuntimeError(f"Manifest verification failed: {relative}")
    print(json.dumps({"status": "PASS", "evaluations": len(points), "summary_rows": len(summary),
                      "intervals": len(intervals), "holes": len(holes), "manifest_files": len(manifest_files)}, indent=2))


if __name__ == "__main__":
    main()
