import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory("humanoid_sim_moveit_config")
    experiment_share = get_package_share_directory("humanoid_extraction_experiments")

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "planning_only.launch.py")
        ),
        launch_arguments={
            "ompl_profile": LaunchConfiguration("ompl_profile"),
            "start_joint_state_gui": "true",
            "start_rviz": "true",
        }.items(),
    )
    boundary = Node(
        package="humanoid_extraction_experiments",
        executable="workspace_boundary_visualizer",
        name="workspace_boundary_visualizer",
        output="screen",
        parameters=[
            {
                "boundary_csv": LaunchConfiguration("boundary_csv"),
                "scene_config": LaunchConfiguration("scene_config"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ompl_profile", default_value="baseline"),
            DeclareLaunchArgument(
                "boundary_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/target_boundary_search.csv",
            ),
            DeclareLaunchArgument(
                "scene_config",
                default_value=os.path.join(
                    experiment_share, "config", "confined_scene.yaml"
                ),
            ),
            moveit,
            boundary,
        ]
    )
