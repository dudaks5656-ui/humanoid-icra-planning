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


def _verify_manifest(path):
    base = os.path.dirname(path)
    count = 0
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            line = line.rstrip("\n")
            if not line:
                continue
            expected, name = line.split("  ", 1)
            target = os.path.join(base, name)
            actual = _sha256(target)
            if actual != expected:
                raise RuntimeError(f"Manifest mismatch: {target}: {actual} != {expected}")
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
    coarse = os.path.join(VALIDATION, "fixed_base_workspace_manifest_sha256.txt")
    fine = os.path.join(VALIDATION, "fixed_base_workspace_fine_manifest_sha256.txt")
    ablation = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_manifest_sha256.txt")
    manifest_counts = [_verify_manifest(path) for path in (coarse, fine, ablation)]

    comparison = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_comparison.csv")
    summary = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_summary.csv")
    contributions = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_contributions.csv")
    fine_points = os.path.join(VALIDATION, "fixed_base_workspace_fine_points.csv")
    c3_overrides = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_c3_special_overrides.csv")

    summaries = _read_csv(summary)
    expected = {
        "LIFT_ONLY": (833, 0.0977907291666664, 0.677083333333333),
        "LIFT_YAW": (1030, 0.120917708333333, 0.70625),
        "LIFT_PITCH": (976, 0.114578333333333, 0.70625),
        "LIFT_YAW_PITCH": (1119, 0.1313659375, 0.735416666666667),
    }
    if len(summaries) != 4:
        raise RuntimeError("Presentation source summary must contain four rows")
    for row in summaries:
        want = expected.get(row["configuration"])
        got = (int(row["reachable_points"]), float(row["targeted_workspace_volume"]), float(row["x_max"]))
        if want is None or any(abs(a - b) > 1e-12 for a, b in zip(got, want)):
            raise RuntimeError(f"Validated summary drift: {row['configuration']} {got} != {want}")
    contribution = _read_csv(contributions)
    if len(contribution) != 1:
        raise RuntimeError("Contribution source must contain one row")
    expected_contribution = {
        "yaw_expanded_unique_count": 78,
        "pitch_expanded_unique_count": 24,
        "yaw_pitch_overlap_count": 119,
        "combined_torso_only_count": 65,
    }
    for key, value in expected_contribution.items():
        if int(contribution[0][key]) != value:
            raise RuntimeError(f"Validated contribution drift: {key}")
    points = _read_csv(comparison)
    if len(points) != 1440 or len({row["point_id"] for row in points}) != 1440:
        raise RuntimeError("Presentation comparison is not the exact 1,440-point set")

    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    demo_share = get_package_share_directory("fixed_base_workspace_demo")
    xacro_path = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    srdf_path = os.path.join(moveit, "config", "humanoid_sim.srdf")
    kinematics_path = os.path.join(moveit, "config", "kinematics.yaml")
    limits_path = os.path.join(moveit, "config", "joint_limits.yaml")
    config_path = os.path.abspath(LaunchConfiguration("config").perform(context))
    demo_config = _yaml(config_path)["fixed_base_workspace_demo"]["ros__parameters"]
    rviz_config = os.path.abspath(LaunchConfiguration("rviz_config").perform(context))
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
        "fine_points_csv": fine_points,
        "c3_overrides_csv": c3_overrides,
        "selected_state_csv": os.path.join(PRESENTATION, "fixed_base_workspace_demo_selected_state.csv"),
        "runtime_status_json": os.path.join(PRESENTATION, "fixed_base_workspace_demo_runtime.json"),
        "source_comparison_sha256": _sha256(comparison),
        "source_summary_sha256": _sha256(summary),
        "source_contributions_sha256": _sha256(contributions),
        "demo_scene": LaunchConfiguration("demo_scene"),
        "duration_scale": ParameterValue(LaunchConfiguration("duration_scale"), value_type=float),
        "preflight_only": ParameterValue(LaunchConfiguration("preflight_only"), value_type=bool),
    }
    print(
        "FIXED_BASE_WORKSPACE_DEMO PREFLIGHT "
        f"manifests=PASS{tuple(manifest_counts)} points=1440 clouds=[833,1030,976,1119] "
        "combined_only=65 robot_state_only=YES execution=NO controller=NO hardware=NO"
    )

    demo = Node(
        package="fixed_base_workspace_demo",
        executable="fixed_base_workspace_demo",
        name="fixed_base_workspace_demo",
        output="screen",
        parameters=[robot_description, semantic, kinematics, limits, demo_config, runtime],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="fixed_base_workspace_robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="fixed_base_workspace_demo_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[robot_description, semantic, kinematics],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    overlay = Node(
        package="fixed_base_workspace_demo",
        executable="fixed_base_workspace_overlay.py",
        name="fixed_base_workspace_demo_overlay",
        output="screen",
        parameters=[{
            "summary_csv": summary,
            "contributions_csv": contributions,
            "runtime_status_json": os.path.join(PRESENTATION, "fixed_base_workspace_demo_runtime.json"),
        }],
        condition=IfCondition(LaunchConfiguration("use_overlay")),
    )
    return [state_publisher, demo, rviz, overlay]


def generate_launch_description():
    share = get_package_share_directory("fixed_base_workspace_demo")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=os.path.join(share, "config", "fixed_base_workspace_demo.yaml")),
        DeclareLaunchArgument(
            "rviz_config", default_value=os.path.join(share, "rviz", "fixed_base_workspace_demo.rviz")),
        DeclareLaunchArgument("demo_scene", default_value="auto"),
        DeclareLaunchArgument("duration_scale", default_value="1.0"),
        DeclareLaunchArgument("preflight_only", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_overlay", default_value="true"),
        OpaqueFunction(function=_launch),
    ])
