#!/usr/bin/env python3
import os,xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def setup(context):
 d=get_package_share_directory("humanoid_sim_description");p=get_package_share_directory("boundary_sweep_workspace_analysis");rd={"robot_description":xacro.process_file(os.path.join(d,"urdf","humanoid_sim.urdf.xacro")).toxml()};scene=LaunchConfiguration("scene").perform(context);pose=LaunchConfiguration("pose_type").perform(context)
 return [Node(package="robot_state_publisher",executable="robot_state_publisher",parameters=[rd]),Node(package="boundary_sweep_workspace_analysis",executable="boundary_sweep_workspace_rviz.py",parameters=[{"scene":scene,"pose_type":pose}]),Node(package="rviz2",executable="rviz2",arguments=["-d",os.path.join(p,"rviz","boundary_sweep_workspace.rviz")])]
def generate_launch_description():return LaunchDescription([DeclareLaunchArgument("scene",default_value="boundary_compare"),DeclareLaunchArgument("pose_type",default_value="MAX_REACH_POSE"),OpaqueFunction(function=setup)])
