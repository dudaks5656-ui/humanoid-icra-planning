import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _text(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def _launch(context):
    output_dir = LaunchConfiguration("output_dir").perform(context)
    os.makedirs(output_dir, exist_ok=False)
    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    experiment = get_package_share_directory("humanoid_extraction_experiments")
    xacro = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro]), value_type=str
        )
    }
    semantic = {
        "robot_description_semantic": _text(os.path.join(moveit, "config", "humanoid_sim.srdf"))
    }
    kinematics = {
        "robot_description_kinematics": _yaml(os.path.join(moveit, "config", "kinematics.yaml"))
    }
    limits = {
        "robot_description_planning": _yaml(os.path.join(moveit, "config", "joint_limits.yaml"))
    }

    pilot = Node(
        package="humanoid_extraction_experiments",
        executable="box_target_yaw_pitch_feasibility_pilot_v1",
        output="screen",
        parameters=[robot_description, semantic, kinematics, limits, {
            "scene_config": os.path.join(experiment, "config", "top_open_reference_scene.yaml"),
            "pilot_config": os.path.join(
                experiment, "config", "box_target_yaw_pitch_feasibility_pilot_v1.yaml"
            ),
            "position_catalog_csv": os.path.join(output_dir, "position_catalog.csv"),
            "feasibility_results_csv": os.path.join(output_dir, "feasibility_results.csv"),
            "comparison_csv": os.path.join(output_dir, "locked_vs_yaw_pitch_comparison.csv"),
            "trajectory_samples_csv": os.path.join(output_dir, "lift_trajectory_samples.csv"),
            "result_yaml": os.path.join(output_dir, "pilot_result.yaml"),
            "audit_md": os.path.join(output_dir, "pilot_audit.md"),
        }],
    )
    return [pilot]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir"),
        OpaqueFunction(function=_launch),
    ])
