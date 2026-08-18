#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    description = get_package_share_directory("humanoid_sim_description")
    package = get_package_share_directory("boundary_sweep_workspace_differential_visualization")
    robot_description = {
        "robot_description": xacro.process_file(
            os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
        ).toxml()
    }
    scene = LaunchConfiguration("scene").perform(context)
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
        ),
        Node(
            package="boundary_sweep_workspace_differential_visualization",
            executable="boundary_sweep_differential_rviz.py",
            parameters=[{"scene": scene}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(package, "rviz", "boundary_sweep_workspace_differential.rviz")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scene", default_value="boundary_diff_all"),
            OpaqueFunction(function=setup),
        ]
    )
