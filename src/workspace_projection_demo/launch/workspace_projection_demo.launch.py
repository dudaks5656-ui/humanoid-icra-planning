import csv
import hashlib
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORKSPACE = "/home/openarm/humanoid_sim_ws"
VALIDATION = os.path.join(WORKSPACE, "validation")


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_manifest(path, base):
    with open(path, encoding="utf-8") as stream:
        entries = [line.rstrip("\n") for line in stream if line.strip()]
    if not entries:
        raise RuntimeError(f"Empty manifest: {path}")
    for entry in entries:
        expected, relative = entry.split("  ", 1)
        target = os.path.join(base, relative)
        if sha256(target) != expected:
            raise RuntimeError(f"Immutable manifest mismatch: {target}")


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def launch_setup(context):
    manifests = [
        ("fixed_base_workspace_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_fine_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_dof_ablation_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_envelope_demo_manifest_sha256.txt", WORKSPACE),
        ("radial_workspace_validation_manifest_sha256.txt", WORKSPACE),
    ]
    for name, base in manifests:
        verify_manifest(os.path.join(VALIDATION, name), base)
    comparison = os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_comparison.csv")
    rows = read_csv(comparison)
    keys = ["c0_lift_success", "c1_lift_yaw_success", "c2_lift_pitch_success", "c3_lift_yaw_pitch_success"]
    counts = [sum(int(row[key]) for row in rows) for key in keys]
    if len(rows) != 1440 or counts != [833, 1030, 976, 1119]:
        raise RuntimeError(f"Validated source drift: points={len(rows)} reachable={counts}")

    share = get_package_share_directory("workspace_projection_demo")
    description = get_package_share_directory("humanoid_sim_description")
    xacro = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro]), value_type=str
        )
    }
    scene = LaunchConfiguration("scene").perform(context)
    right = scene.startswith("right")
    rviz_file = os.path.join(share, "rviz", "workspace_projection_right.rviz" if right else "workspace_projection_front.rviz")
    runtime = {
        "front_csv": os.path.join(VALIDATION, "workspace_projection_front.csv"),
        "right_csv": os.path.join(VALIDATION, "workspace_projection_right.csv"),
        "summary_csv": os.path.join(VALIDATION, "workspace_projection_summary.csv"),
        "state_csv": os.path.join(VALIDATION, "radial_workspace_validation_states.csv"),
        "runtime_json": os.path.join(WORKSPACE, "presentation", "workspace_projection_rviz_runtime.json"),
        "scene": scene,
    }
    print(
        "WORKSPACE_PROJECTION_PREFLIGHT manifests=PASS points=1440 "
        f"reachable={counts} scene={scene} new_ik=NO execution=NO controller=NO hardware=NO"
    )
    projection = Node(
        package="workspace_projection_demo", executable="workspace_projection_rviz.py",
        name="workspace_projection_rviz", output="screen", parameters=[runtime],
    )
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="workspace_projection_robot_state_publisher", output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="workspace_projection_rviz_view",
        output="screen", arguments=["-d", rviz_file], parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return [rsp, projection, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="front_compare"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
