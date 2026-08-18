#!/usr/bin/env python3
import os,yaml,xacro
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def setup(context):
    desc=get_package_share_directory("humanoid_sim_description");moveit=get_package_share_directory("humanoid_sim_moveit_config");pkg=get_package_share_directory("boundary_sweep_workspace_analysis")
    rd={"robot_description":xacro.process_file(os.path.join(desc,"urdf","humanoid_sim.urdf.xacro")).toxml()}
    with open(os.path.join(moveit,"config","humanoid_sim.srdf"),encoding="utf-8") as f:semantic={"robot_description_semantic":f.read()}
    with open(os.path.join(moveit,"config","joint_limits.yaml"),encoding="utf-8") as f:limits={"robot_description_planning":yaml.safe_load(f)}
    return [Node(package="boundary_sweep_workspace_analysis",executable="boundary_sweep_workspace",output="screen",parameters=[rd,semantic,limits,os.path.join(pkg,"config","boundary_sweep_workspace.yaml")])]
def generate_launch_description():return LaunchDescription([OpaqueFunction(function=setup)])
