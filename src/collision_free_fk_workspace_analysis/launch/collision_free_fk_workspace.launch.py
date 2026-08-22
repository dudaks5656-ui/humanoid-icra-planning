#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def setup(context):
    desc = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    package = get_package_share_directory("collision_free_fk_workspace_analysis")
    import xacro
    robot_description = {
        "robot_description": xacro.process_file(
            os.path.join(desc, "urdf", "humanoid_sim.urdf.xacro")
        ).toxml()
    }
    with open(os.path.join(moveit, "config", "humanoid_sim.srdf"), encoding="utf-8") as stream:
        semantic = {"robot_description_semantic": stream.read()}
    with open(os.path.join(moveit, "config", "joint_limits.yaml"), encoding="utf-8") as stream:
        limits = {"robot_description_planning": yaml.safe_load(stream)}
    config = os.path.join(package, "config", "collision_free_fk_workspace.yaml")
    return [Node(
        package="collision_free_fk_workspace_analysis",
        executable="collision_free_fk_workspace",
        output="screen",
        parameters=[robot_description, semantic, limits, config],
        arguments=["--ros-args", "-p", ["output_dir:=", LaunchConfiguration("output_dir")]],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="/home/openarm/humanoid_sim_ws/validation"),
        OpaqueFunction(function=setup),
    ])
