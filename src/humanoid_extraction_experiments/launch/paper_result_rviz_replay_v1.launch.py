import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORKSPACE = "/home/openarm/humanoid_sim_ws"
SOURCE_PACKAGE = os.path.join(WORKSPACE, "src", "humanoid_extraction_experiments")
DEFAULT_ROOT = os.path.join(
    WORKSPACE, "validation", "paper_main_simulation_dataset_v1", "run_20260815_223216")
DEFAULT_AUDIT_DIR = os.path.join(
    WORKSPACE, "validation", "paper_result_rviz_replay_v1", "run_20260816_125837")


def generate_launch_description():
    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    xacro_path = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_path]), value_type=str)}
    semantic_path = os.path.join(moveit, "config", "humanoid_sim.srdf")
    with open(semantic_path, "r", encoding="utf-8") as stream:
        semantic = stream.read()
    case = LaunchConfiguration("case_label")
    result_root = LaunchConfiguration("result_root")
    audit_dir = LaunchConfiguration("audit_dir")
    replay = ExecuteProcess(
        cmd=[
            "python3", os.path.join(SOURCE_PACKAGE, "scripts", "paper_result_rviz_replay_v1.py"),
            "--result-root", result_root,
            "--selection-audit", [audit_dir, "/representative_case_audit.yaml"],
            "--case-label", case,
            "--scene-config", os.path.join(SOURCE_PACKAGE, "config", "top_open_reference_scene.yaml"),
            "--robot-xacro", xacro_path,
            "--runtime-audit", [audit_dir, "/runtime_", case, ".yaml"],
            "--playback-speed", LaunchConfiguration("playback_speed"),
        ],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("case_label", default_value="LOCKED_COMMON_SUCCESS",
                              description="One of the six labels in representative_case_audit.yaml"),
        DeclareLaunchArgument("result_root", default_value=DEFAULT_ROOT),
        DeclareLaunchArgument("audit_dir", default_value=DEFAULT_AUDIT_DIR),
        DeclareLaunchArgument("playback_speed", default_value="0.5"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="paper_result_robot_state_publisher", output="screen", parameters=[robot_description]),
        replay,
        Node(
            package="rviz2", executable="rviz2", name="paper_result_rviz",
            arguments=["-d", os.path.join(SOURCE_PACKAGE, "rviz", "paper_result_rviz_replay_v1.rviz")],
            parameters=[robot_description, {"robot_description_semantic": semantic}],
            output="screen", condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
