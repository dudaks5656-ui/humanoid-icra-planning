import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_text(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    description_share = get_package_share_directory("humanoid_sim_description")
    moveit_share = get_package_share_directory("humanoid_sim_moveit_config")
    xacro_path = os.path.join(description_share, "urdf", "humanoid_sim.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_path]), value_type=str
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": _load_text(
            os.path.join(moveit_share, "config", "humanoid_sim.srdf")
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/collision_results.csv",
            ),
            DeclareLaunchArgument(
                "trajectory_output_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/trajectory_validation.csv",
            ),
            Node(
                package="humanoid_sim_moveit_config",
                executable="collision_state_checker",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {
                        "output_csv": LaunchConfiguration("output_csv"),
                        "trajectory_output_csv": LaunchConfiguration("trajectory_output_csv"),
                        "collision_detector_name": "FCL",
                        "input_topic": "/collision_state_input",
                        "trajectory_topic": "/display_planned_path",
                        "coarse_subdivisions": 1,
                        "medium_subdivisions": 5,
                        "fine_subdivisions": 20,
                    },
                ],
            ),
        ]
    )
