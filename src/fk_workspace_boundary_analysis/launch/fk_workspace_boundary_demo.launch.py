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


def digest(path):
    value = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def verify_manifest(path):
    with open(path, encoding="utf-8") as stream:
        entries = [line.rstrip("\n") for line in stream if line.strip()]
    for entry in entries:
        expected, relative = entry.split("  ", 1)
        target = os.path.join(WORKSPACE, relative)
        if digest(target) != expected:
            raise RuntimeError(f"FK workspace manifest mismatch: {target}")


def setup(context):
    verify_manifest(os.path.join(VALIDATION, "fk_workspace_boundary_manifest_sha256.txt"))
    scene = LaunchConfiguration("scene").perform(context)
    view = LaunchConfiguration("view").perform(context)
    if view not in {"front", "right"}:
        raise RuntimeError("view must be front or right")
    description = get_package_share_directory("humanoid_sim_description")
    share = get_package_share_directory("fk_workspace_boundary_analysis")
    xacro = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro]), value_type=str
        )
    }
    rviz = os.path.join(share, "rviz", f"fk_workspace_boundary_{view}.rviz")
    runtime = {
        "states_csv": os.path.join(VALIDATION, "fk_workspace_boundary_states.csv"),
        "scene": scene,
        "runtime_json": os.path.join(WORKSPACE, "presentation", "fk_workspace_boundary_rviz_runtime.json"),
    }
    print(f"FK_WORKSPACE_DEMO PREFLIGHT manifest=PASS scene={scene} view={view} IK=NO execution=NO")
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="fk_workspace_robot_state_publisher", output="screen",
             parameters=[robot_description], condition=IfCondition(LaunchConfiguration("use_rviz"))),
        Node(package="fk_workspace_boundary_analysis", executable="fk_workspace_boundary_rviz.py",
             name="fk_workspace_boundary_rviz", output="screen", parameters=[runtime]),
        Node(package="rviz2", executable="rviz2", name="fk_workspace_boundary_view",
             output="screen", arguments=["-d", rviz], parameters=[robot_description],
             condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="fk_compare"),
        DeclareLaunchArgument("view", default_value="right"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=setup),
    ])
