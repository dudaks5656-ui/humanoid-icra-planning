#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def setup(context):
    ws = "/home/openarm/humanoid_sim_ws"
    desc = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    package = get_package_share_directory("lift_slice_fk_workspace_analysis")
    xacro_path = os.path.join(desc, "urdf", "humanoid_sim.urdf.xacro")
    import xacro
    robot_description = {"robot_description": xacro.process_file(xacro_path).toxml()}
    with open(os.path.join(moveit, "config", "humanoid_sim.srdf"), encoding="utf-8") as f:
        semantic = {"robot_description_semantic": f.read()}
    with open(os.path.join(moveit, "config", "joint_limits.yaml"), encoding="utf-8") as f:
        limits = {"robot_description_planning": yaml.safe_load(f)}
    config = os.path.join(package, "config", "lift_slice_fk_workspace.yaml")
    return [Node(package="lift_slice_fk_workspace_analysis", executable="lift_slice_fk_workspace",
                 output="screen", parameters=[robot_description, semantic, limits, config])]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=setup)])
