#!/usr/bin/env python3
"""Generate presentation documentation, audit, and SHA-256 manifest."""

import csv
import hashlib
import json
import math
import os
import pathlib

import cv2


WORKSPACE = pathlib.Path("/home/openarm/humanoid_sim_ws")
VALIDATION = WORKSPACE / "validation"
PRESENTATION = WORKSPACE / "presentation"
PACKAGE = WORKSPACE / "src/fixed_base_workspace_demo"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_manifest(path):
    count = 0
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            line = line.rstrip("\n")
            if not line:
                continue
            expected, name = line.split("  ", 1)
            target = path.parent / name
            if sha256(target) != expected:
                raise RuntimeError(f"Immutable manifest mismatch: {target}")
            count += 1
    if not count:
        raise RuntimeError(f"Empty immutable manifest: {path}")
    return count


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def key_values(path):
    return {row["key"]: row["value"] for row in read_csv(path)}


def image_check(path):
    image = cv2.imread(str(path))
    if image is None or image.size == 0:
        raise RuntimeError(f"Representative frame is not decodable: {path}")
    return int(image.shape[1]), int(image.shape[0])


def format_command(parts):
    return " ".join(str(item) for item in parts)


def main():
    old_manifest_counts = {
        "coarse": verify_manifest(VALIDATION / "fixed_base_workspace_manifest_sha256.txt"),
        "fine": verify_manifest(VALIDATION / "fixed_base_workspace_fine_manifest_sha256.txt"),
        "ablation": verify_manifest(VALIDATION / "fixed_base_workspace_dof_ablation_manifest_sha256.txt"),
    }
    summary_path = VALIDATION / "fixed_base_workspace_dof_ablation_summary.csv"
    contributions_path = VALIDATION / "fixed_base_workspace_dof_ablation_contributions.csv"
    comparison_path = VALIDATION / "fixed_base_workspace_dof_ablation_comparison.csv"
    common_metrics_path = VALIDATION / "fixed_base_workspace_dof_ablation_common_point_metrics.csv"
    summary = read_csv(summary_path)
    contribution = read_csv(contributions_path)[0]
    comparison = read_csv(comparison_path)
    common_metrics = read_csv(common_metrics_path)
    state_path = PRESENTATION / "fixed_base_workspace_demo_selected_state.csv"
    state = key_values(state_path)
    runtime = json.loads((PRESENTATION / "fixed_base_workspace_demo_runtime.json").read_text(encoding="utf-8"))
    full = json.loads((PRESENTATION / "fixed_base_workspace_demo_recording_full.json").read_text(encoding="utf-8"))
    short = json.loads((PRESENTATION / "fixed_base_workspace_demo_recording_short.json").read_text(encoding="utf-8"))

    expected_counts = [833, 1030, 976, 1119]
    if [int(row["reachable_points"]) for row in summary] != expected_counts:
        raise RuntimeError("Presentation source counts do not match validated 4-way summary")
    if len(comparison) != 1440 or len({row["point_id"] for row in comparison}) != 1440:
        raise RuntimeError("Comparison point set is not exactly 1,440 unique points")
    if not common_metrics:
        raise RuntimeError("Validated common-point metrics source is empty")
    point = next(row for row in comparison if row["point_id"] == state["point_id"])
    if not (
        point["classification"] == "COMBINED_TORSO_ONLY"
        and [point[key] for key in (
            "c0_lift_success", "c1_lift_yaw_success", "c2_lift_pitch_success",
            "c3_lift_yaw_pitch_success")]
        == ["0", "0", "0", "1"]
        and state["confidence"] == "HIGH"
    ):
        raise RuntimeError("Representative point is not a verified high-confidence 0/0/0/1 point")
    if runtime.get("representative_point_id") != int(state["point_id"]):
        raise RuntimeError("Runtime representative point drift")
    if any(runtime.get(key) for key in (
        "trajectory_execution", "controller", "ros2_control", "hardware", "amr_motion")):
        raise RuntimeError("Forbidden execution flag was enabled")

    for recording in (full, short):
        video = pathlib.Path(recording["video_path"])
        meta = recording["video"]
        if not video.is_file() or video.stat().st_size <= 0 or not meta["all_frames_decoded"]:
            raise RuntimeError(f"Invalid recording evidence: {video}")
        if meta["width"] != 1920 or meta["height"] != 1080 or abs(meta["fps"] - 30.0) > 0.1:
            raise RuntimeError(f"Recording format mismatch: {video}")
        if "H.264" not in recording["gst_discoverer"] or "Quicktime" not in recording["gst_discoverer"]:
            raise RuntimeError(f"Recording codec/container evidence mismatch: {video}")
        required_scenes = {"ROBOT", "C0", "C1", "C2", "C3", "COMBINED_C0",
                           "COMBINED_C1", "COMBINED_C2", "COMBINED_C3", "ANIMATION", "FINAL"}
        missing = required_scenes.difference(recording["scene_first_seen_s"])
        if missing:
            raise RuntimeError(f"Recording omitted required scenes {sorted(missing)}: {video}")
    frame_paths = [PRESENTATION / name for name in (
        "frame_c0.png", "frame_c3.png", "frame_combined_only.png")]
    frame_dimensions = {path.name: image_check(path) for path in frame_paths}

    presentation_summary = PRESENTATION / "fixed_base_workspace_presentation_summary.csv"
    with presentation_summary.open("w", newline="", encoding="utf-8") as stream:
        fields = ["configuration", "reachable_points", "total_points", "workspace_volume",
                  "increase_percent", "max_forward_x", "mean_manipulability", "mean_joint_margin"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in summary:
            writer.writerow({
                "configuration": row["configuration"],
                "reachable_points": row["reachable_points"],
                "total_points": row["total_points"],
                "workspace_volume": row["targeted_workspace_volume"],
                "increase_percent": row["percent_delta_vs_lift_only"],
                "max_forward_x": row["x_max"],
                "mean_manipulability": row["mean_manipulability"],
                "mean_joint_margin": row["mean_joint_margin"],
            })

    readme = PRESENTATION / "README.md"
    table_lines = []
    labels = {
        "LIFT_ONLY": "C0 Arm + Lift", "LIFT_YAW": "C1 Arm + Lift + Yaw",
        "LIFT_PITCH": "C2 Arm + Lift + Pitch", "LIFT_YAW_PITCH": "C3 Arm + Lift + Yaw + Pitch",
    }
    for row in summary:
        delta = "baseline" if row["configuration"] == "LIFT_ONLY" else f"+{float(row['percent_delta_vs_lift_only']):.2f}%"
        table_lines.append(
            f"| {labels[row['configuration']]} | {row['reachable_points']} / {row['total_points']} | "
            f"{float(row['targeted_workspace_volume']):.6f} m³ | {delta} | {float(row['x_max']):.4f} m |"
        )
    readme.write_text(
        "# Fixed-base workspace RViz presentation demo\n\n"
        "## 연구 목적\n\n"
        "AMR을 고정한 상태에서 Waist Yaw/Pitch 자유도가 manipulation workspace에 주는 영향을 "
        "동일한 1,440개 TCP 점으로 보여주는 planning-only 발표 자료입니다.\n\n"
        "## 네 configuration과 검증 수치\n\n"
        "| Configuration | Reachable | Targeted volume | C0 대비 | Max X |\n"
        "|---|---:|---:|---:|---:|\n" + "\n".join(table_lines) + "\n\n"
        f"- Yaw unique expansion: {contribution['yaw_expanded_unique_count']} points\n"
        f"- Pitch unique expansion: {contribution['pitch_expanded_unique_count']} points\n"
        f"- Yaw/Pitch overlap expansion: {contribution['yaw_pitch_overlap_count']} points\n"
        f"- Combined-torso-only: {contribution['combined_torso_only_count']} points, "
        f"{float(contribution['combined_torso_only_volume']):.6f} m³\n\n"
        "## 영상 장면\n\n"
        "1. 고정 AMR과 휴머노이드 모델\n"
        "2. C0 Arm+Lift workspace\n"
        "3. C1 Yaw 추가 workspace\n"
        "4. C2 Pitch 추가 workspace\n"
        "5. C3 Yaw+Pitch 및 category overlay\n"
        "6. 동일 combined-only TCP에서 C0/C1/C2 실패\n"
        "7. C3 성공 RobotState와 collision-checked visualization-only interpolation\n\n"
        "RViz는 RobotState, TF, TCP marker와 workspace point cloud를 표시합니다. 수치 패널은 "
        "RViz 장면 topic과 동기화되는 visualization-only 2D overlay이며 validated CSV만 읽습니다.\n\n"
        "## 실행\n\n"
        "```bash\n"
        "export ROS_DOMAIN_ID=42\nexport ROS_LOCALHOST_ONLY=1\n"
        "source /opt/ros/humble/setup.bash\nsource install/setup.bash\n"
        "ros2 launch fixed_base_workspace_demo fixed_base_workspace_demo.launch.py demo_scene:=auto\n"
        "```\n\n"
        "수동 장면은 `demo_scene:=c0`, `c1`, `c2`, `c3`, `combined_only`, `robot`으로 선택합니다. "
        "측면 시점은 `rviz_config:=.../fixed_base_workspace_demo_side.rviz`를 사용합니다.\n\n"
        "## 발표 시 말할 핵심\n\n"
        "1. AMR은 움직이지 않습니다.\n"
        "2. 동일한 1,440 TCP points를 비교했습니다.\n"
        "3. IK, joint limit, self-collision 조건이 적용되었습니다.\n"
        "4. Yaw 단독은 +23.65%, Pitch 단독은 +17.17%입니다.\n"
        "5. Yaw+Pitch는 +34.33%입니다.\n"
        "6. 65점은 두 torso DOF를 동시에 사용해야 했습니다.\n"
        "7. 애니메이션은 RobotState 시각화이며 실제 trajectory execution이 아닙니다.\n"
        "8. 다음 단계는 box task-feasible workspace와 recovery planning입니다.\n\n"
        "## 녹화 파일\n\n"
        "- `fixed_base_workspace_demo.mp4`: 전체 발표 버전\n"
        "- `fixed_base_workspace_demo_short.mp4`: 단축 버전\n"
        "- `frame_c0.png`, `frame_c3.png`, `frame_combined_only.png`: 대표 화면\n",
        encoding="utf-8",
    )

    audit = VALIDATION / "fixed_base_workspace_demo_audit.md"
    joint_rows = sorted((key.removeprefix("joint."), value) for key, value in state.items() if key.startswith("joint."))
    audit.write_text(
        "# Fixed-base workspace presentation demo audit\n\n"
        "## Immutable sources\n\n"
        f"- `{comparison_path}` — `{sha256(comparison_path)}`\n"
        f"- `{summary_path}` — `{sha256(summary_path)}`\n"
        f"- `{contributions_path}` — `{sha256(contributions_path)}`\n"
        f"- `{common_metrics_path}` — `{sha256(common_metrics_path)}`\n"
        f"- Existing manifest verification: PASS {old_manifest_counts}\n\n"
        "## Point clouds and representative\n\n"
        f"- Point-cloud counts C0/C1/C2/C3: {expected_counts}\n"
        f"- Combined-torso-only count: {contribution['combined_torso_only_count']}\n"
        f"- Representative point: ID `{state['point_id']}`, XYZ "
        f"(`{state['tcp_x']}`, `{state['tcp_y']}`, `{state['tcp_z']}`) m\n"
        "- Source classification/results: `COMBINED_TORSO_ONLY`, C0/C1/C2 FAIL, C3 PASS\n"
        f"- Confidence: `{state['confidence']}`\n"
        f"- Selected torso: Lift `{state['selected_lift']}`, Yaw `{state['selected_yaw']}`, "
        f"Pitch `{state['selected_pitch']}` rad\n"
        f"- Reproduced state margin/clearance: `{state['computed_joint_margin']}` / "
        f"`{state['computed_self_clearance']}` m\n"
        f"- TCP position/orientation error: `{state['tcp_position_error']}` m / "
        f"`{state['orientation_error']}` rad\n"
        f"- Selected C3 joint state: `{joint_rows}`\n\n"
        "## Demo\n\n"
        "- Scene sequence: ROBOT → C0 → C1 → C2 → C3 → C0/C1/C2 same-target failures → "
        "C3 same-target success → visualization interpolation → FINAL\n"
        f"- Animation collision-free: `{runtime['animation_collision_free']}`, samples: "
        f"`{runtime['animation_samples']}`\n"
        "- RViz configs: `fixed_base_workspace_demo.rviz`, `fixed_base_workspace_demo_side.rviz`\n\n"
        "- Presentation text: synchronized 2D overlay; read-only CSV/status subscriber, no command publisher\n\n"
        "## Recording\n\n"
        f"- Full command: `{format_command(full['recording_command'])}`\n"
        f"- Full video: `{full['video_path']}`, {full['video']['duration_s']:.3f} s, "
        f"{full['video']['width']}×{full['video']['height']}, 30 fps, H.264/MP4, "
        f"{full['video']['file_size_bytes']} bytes\n"
        f"- Short video: `{short['video_path']}`, {short['video']['duration_s']:.3f} s, "
        f"{short['video']['width']}×{short['video']['height']}, 30 fps, H.264/MP4, "
        f"{short['video']['file_size_bytes']} bytes\n"
        f"- All frames decoded: full `{full['video']['all_frames_decoded']}`, "
        f"short `{short['video']['all_frames_decoded']}`\n"
        f"- Representative frames: `{[(str(path), frame_dimensions[path.name]) for path in frame_paths]}`\n"
        "- Representative frame visual review: PASS — robot, workspace cloud, validated text panel, and "
        "combined-only target/valid C3 pose are visible and readable.\n\n"
        "## Safety\n\n"
        "- trajectory execution = false\n- controller = false\n- ros2_control = false\n"
        "- hardware = false\n- AMR motion = false\n- navigation = false\n- Gazebo physics = false\n",
        encoding="utf-8",
    )

    manifest_targets = [
        PACKAGE / "package.xml", PACKAGE / "CMakeLists.txt",
        PACKAGE / "src/fixed_base_workspace_demo.cpp",
        PACKAGE / "launch/fixed_base_workspace_demo.launch.py",
        PACKAGE / "config/fixed_base_workspace_demo.yaml",
        PACKAGE / "rviz/fixed_base_workspace_demo.rviz",
        PACKAGE / "rviz/fixed_base_workspace_demo_side.rviz",
        PACKAGE / "scripts/fixed_base_workspace_overlay.py",
        PACKAGE / "scripts/record_fixed_base_workspace_demo.py",
        PACKAGE / "scripts/finalize_fixed_base_workspace_demo.py",
        audit, readme, presentation_summary, state_path,
        PRESENTATION / "fixed_base_workspace_demo_runtime.json",
        PRESENTATION / "fixed_base_workspace_demo_recording_full.json",
        PRESENTATION / "fixed_base_workspace_demo_recording_short.json",
        PRESENTATION / "fixed_base_workspace_demo.mp4",
        PRESENTATION / "fixed_base_workspace_demo_short.mp4",
        *frame_paths,
    ]
    for path in manifest_targets:
        if not path.is_file() or path.stat().st_size <= 0:
            raise RuntimeError(f"Manifest target missing or empty: {path}")
    manifest = VALIDATION / "fixed_base_workspace_demo_manifest_sha256.txt"
    with manifest.open("w", encoding="utf-8") as stream:
        for path in sorted(manifest_targets):
            stream.write(f"{sha256(path)}  {path.relative_to(WORKSPACE)}\n")
    for line in manifest.read_text(encoding="utf-8").splitlines():
        expected, relative = line.split("  ", 1)
        if sha256(WORKSPACE / relative) != expected:
            raise RuntimeError(f"New demo manifest verification failed: {relative}")
    print(json.dumps({
        "status": "PASS", "manifest": str(manifest), "manifest_files": len(manifest_targets),
        "representative_point_id": int(state["point_id"]), "old_manifests": old_manifest_counts,
    }, indent=2))


if __name__ == "__main__":
    main()
