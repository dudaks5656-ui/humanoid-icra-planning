import csv
import hashlib
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORKSPACE = "/home/openarm/humanoid_sim_ws"
VALIDATION = os.path.join(WORKSPACE, "validation")
PRESENTATION = os.path.join(WORKSPACE, "presentation")


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _verify_manifest(path, base):
    count = 0
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            line = line.rstrip("\n")
            if not line:
                continue
            expected, name = line.split("  ", 1)
            target = os.path.join(base, name)
            if _sha256(target) != expected:
                raise RuntimeError(f"Immutable manifest mismatch: {target}")
            count += 1
    if not count:
        raise RuntimeError(f"Empty manifest: {path}")
    return count


def _read_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _text(path):
    with open(path, encoding="utf-8") as stream:
        return stream.read()


def _yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _launch(context):
    manifests = [
        (os.path.join(VALIDATION, "fixed_base_workspace_manifest_sha256.txt"), VALIDATION),
        (os.path.join(VALIDATION, "fixed_base_workspace_fine_manifest_sha256.txt"), VALIDATION),
        (os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_manifest_sha256.txt"), VALIDATION),
        (os.path.join(VALIDATION, "fixed_base_workspace_demo_manifest_sha256.txt"), WORKSPACE),
    ]
    manifest_counts = [_verify_manifest(path, base) for path, base in manifests]

    comparison = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_comparison.csv")
    summary = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_summary.csv")
    contributions = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_contributions.csv")
    selected_state = os.path.join(PRESENTATION, "fixed_base_workspace_demo_selected_state.csv")
    fine_points = os.path.join(VALIDATION, "fixed_base_workspace_fine_points.csv")
    new_points = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_new_points.csv")
    points = _read_csv(comparison)
    if len(points) != 1440 or len({row["point_id"] for row in points}) != 1440:
        raise RuntimeError("Envelope source is not the immutable 1,440-point grid")
    expected = [833, 1030, 976, 1119]
    keys = ["c0_lift_success", "c1_lift_yaw_success", "c2_lift_pitch_success", "c3_lift_yaw_pitch_success"]
    counts = [sum(int(row[key]) for row in points) for key in keys]
    if counts != expected:
        raise RuntimeError(f"Validated occupancy drift: {counts} != {expected}")
    representative = next((row for row in points if row["point_id"] == "1360"), None)
    if representative is None or [representative[key] for key in keys] != ["0", "0", "0", "1"]:
        raise RuntimeError("Combined-only representative point 1360 drifted")

    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    share = get_package_share_directory("fixed_base_workspace_envelope_demo")
    xacro_path = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    srdf_path = os.path.join(moveit, "config", "humanoid_sim.srdf")
    kinematics_path = os.path.join(moveit, "config", "kinematics.yaml")
    limits_path = os.path.join(moveit, "config", "joint_limits.yaml")
    config_path = os.path.abspath(LaunchConfiguration("config").perform(context))
    rviz_config = os.path.abspath(LaunchConfiguration("rviz_config").perform(context))
    config = _yaml(config_path)["fixed_base_workspace_envelope_demo"]["ros__parameters"]
    os.makedirs(PRESENTATION, exist_ok=True)

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_path]), value_type=str)
    }
    semantic = {"robot_description_semantic": _text(srdf_path)}
    kinematics = {"robot_description_kinematics": _yaml(kinematics_path)}
    limits = {"robot_description_planning": _yaml(limits_path)}
    runtime = {
        "comparison_csv": comparison,
        "summary_csv": summary,
        "contributions_csv": contributions,
        "stored_c3_state_csv": selected_state,
        "fine_points_csv": fine_points,
        "new_points_csv": new_points,
        "runtime_status_json": os.path.join(PRESENTATION, "fixed_base_workspace_envelope_demo_runtime.json"),
        "metrics_csv": os.path.join(VALIDATION, "fixed_base_workspace_envelope_demo_metrics.csv"),
        "source_comparison_sha256": _sha256(comparison),
        "source_summary_sha256": _sha256(summary),
        "source_contributions_sha256": _sha256(contributions),
        "demo_scene": LaunchConfiguration("demo_scene"),
        "visualization_mode": LaunchConfiguration("visualization_mode"),
        "duration_scale": ParameterValue(LaunchConfiguration("duration_scale"), value_type=float),
        "preflight_only": ParameterValue(LaunchConfiguration("preflight_only"), value_type=bool),
    }
    print(
        "FIXED_BASE_WORKSPACE_ENVELOPE PREFLIGHT "
        f"manifests=PASS{tuple(manifest_counts)} grid_points=1440 occupancy={counts} "
        "convex_hull=NO execution=NO controller=NO hardware=NO"
    )

    demo = Node(
        package="fixed_base_workspace_envelope_demo",
        executable="fixed_base_workspace_envelope_demo",
        name="fixed_base_workspace_envelope_demo",
        output="screen",
        parameters=[robot_description, semantic, kinematics, limits, config, runtime],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="fixed_base_workspace_envelope_robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="fixed_base_workspace_envelope_demo_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[robot_description, semantic, kinematics],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    overlay = Node(
        package="fixed_base_workspace_envelope_demo",
        executable="fixed_base_workspace_envelope_overlay.py",
        name="fixed_base_workspace_envelope_overlay",
        output="screen",
        parameters=[{
            "summary_csv": summary,
            "contributions_csv": contributions,
            "runtime_status_json": os.path.join(PRESENTATION, "fixed_base_workspace_envelope_demo_runtime.json"),
        }],
        condition=IfCondition(LaunchConfiguration("use_overlay")),
    )
    del share
    return [state_publisher, demo, rviz, overlay]


def generate_launch_description():
    share = get_package_share_directory("fixed_base_workspace_envelope_demo")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "fixed_base_workspace_envelope_demo.yaml")),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(share, "rviz", "fixed_base_workspace_envelope_demo.rviz")),
        DeclareLaunchArgument("demo_scene", default_value="auto"),
        DeclareLaunchArgument("visualization_mode", default_value="surface"),
        DeclareLaunchArgument("duration_scale", default_value="1.0"),
        DeclareLaunchArgument("preflight_only", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_overlay", default_value="true"),
        OpaqueFunction(function=_launch),
    ])
