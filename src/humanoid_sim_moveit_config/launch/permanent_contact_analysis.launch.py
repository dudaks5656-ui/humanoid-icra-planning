import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_text(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def generate_launch_description():
    description_share = get_package_share_directory("humanoid_sim_description")
    moveit_share = get_package_share_directory("humanoid_sim_moveit_config")
    xacro_path = os.path.join(description_share, "urdf", "humanoid_sim.urdf.xacro")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "comparison_output",
                default_value="/home/openarm/humanoid_sim_ws/validation/arm_mount_collision_comparison.txt",
            ),
            DeclareLaunchArgument(
                "contact_output",
                default_value="/home/openarm/humanoid_sim_ws/validation/permanent_contact_analysis.csv",
            ),
            Node(
                package="humanoid_sim_moveit_config",
                executable="permanent_contact_analyzer",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command([FindExecutable(name="xacro"), " ", xacro_path]),
                            value_type=str,
                        ),
                        "robot_description_semantic": _load_text(
                            os.path.join(moveit_share, "config", "humanoid_sim.srdf")
                        ),
                        "comparison_output": LaunchConfiguration("comparison_output"),
                        "contact_output": LaunchConfiguration("contact_output"),
                    }
                ],
            ),
        ]
    )
