import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_text(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def generate_launch_description():
    description_share = get_package_share_directory("humanoid_sim_description")
    moveit_share = get_package_share_directory("humanoid_sim_moveit_config")
    xacro_path = os.path.join(description_share, "urdf", "humanoid_sim.urdf.xacro")

    planning_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "planning_only.launch.py")
        ),
        launch_arguments={
            "ompl_profile": "baseline",
            "start_joint_state_gui": "true",
            "start_rviz": "true",
        }.items(),
    )

    marker_node = Node(
        package="humanoid_sim_moveit_config",
        executable="contact_marker_publisher",
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
            }
        ],
    )

    return LaunchDescription([planning_only, marker_node])
