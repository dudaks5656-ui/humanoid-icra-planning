import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORKSPACE = "/home/openarm/humanoid_sim_ws"
VALIDATION = os.path.join(WORKSPACE, "validation")
PRESENTATION = os.path.join(WORKSPACE, "presentation")


def _yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _text(path):
    with open(path, encoding="utf-8") as stream:
        return stream.read()


def generate_launch_description():
    package = get_package_share_directory("radial_workspace_validation")
    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    xacro = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    robot_description = {"robot_description": ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro]), value_type=str)}
    semantic = {"robot_description_semantic": _text(os.path.join(moveit, "config", "humanoid_sim.srdf"))}
    runtime = os.path.join(PRESENTATION, "radial_workspace_validation_demo_runtime.json")
    common = {
        "base_frame": "base_link",
        "comparison_csv": os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_comparison.csv"),
        "points_csv": os.path.join(VALIDATION, "radial_workspace_validation_points.csv"),
        "intervals_csv": os.path.join(VALIDATION, "radial_workspace_validation_intervals.csv"),
        "holes_csv": os.path.join(VALIDATION, "radial_workspace_validation_holes.csv"),
        "summary_csv": os.path.join(VALIDATION, "radial_workspace_validation_summary.csv"),
        "states_csv": os.path.join(VALIDATION, "radial_workspace_validation_states.csv"),
        "metadata_csv": os.path.join(VALIDATION, "radial_workspace_validation_metadata.csv"),
        "runtime_json": runtime,
        "demo_scene": LaunchConfiguration("demo_scene"),
        "selected_ray": LaunchConfiguration("selected_ray"),
        "duration_scale": ParameterValue(LaunchConfiguration("duration_scale"), value_type=float),
    }
    return LaunchDescription([
        DeclareLaunchArgument("demo_scene", default_value="auto"),
        DeclareLaunchArgument("selected_ray", default_value="FRONT"),
        DeclareLaunchArgument("duration_scale", default_value="1.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_overlay", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="radial_workspace_robot_state_publisher", parameters=[robot_description],
             condition=IfCondition(LaunchConfiguration("use_rviz"))),
        Node(package="radial_workspace_validation", executable="radial_workspace_validation_demo.py",
             name="radial_workspace_validation_demo", output="screen", parameters=[common]),
        Node(package="rviz2", executable="rviz2", name="radial_workspace_validation_rviz",
             arguments=["-d", os.path.join(package, "rviz", "radial_workspace_validation.rviz")],
             parameters=[robot_description, semantic], condition=IfCondition(LaunchConfiguration("use_rviz"))),
        Node(package="radial_workspace_validation", executable="radial_workspace_validation_overlay.py",
             name="radial_workspace_validation_overlay", output="screen",
             parameters=[{"summary_csv": common["summary_csv"], "runtime_json": runtime}],
             condition=IfCondition(LaunchConfiguration("use_overlay"))),
    ])
