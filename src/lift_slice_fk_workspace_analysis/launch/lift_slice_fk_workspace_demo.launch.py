#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def setup(context):
    desc=get_package_share_directory("humanoid_sim_description");pkg=get_package_share_directory("lift_slice_fk_workspace_analysis")
    robot={"robot_description":xacro.process_file(os.path.join(desc,"urdf","humanoid_sim.urdf.xacro")).toxml()}
    scene=LaunchConfiguration("scene").perform(context);pose=LaunchConfiguration("pose_slice").perform(context);view=LaunchConfiguration("view").perform(context)
    rviz=os.path.join(pkg,"rviz",f"lift_slice_fk_workspace_{view}.rviz")
    return [Node(package="robot_state_publisher",executable="robot_state_publisher",parameters=[robot]),
            Node(package="lift_slice_fk_workspace_analysis",executable="lift_slice_fk_workspace_rviz.py",parameters=[{"scene":scene,"pose_slice":pose}]),
            Node(package="rviz2",executable="rviz2",arguments=["-d",rviz],output="screen")]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("scene",default_value="lift_compare"),DeclareLaunchArgument("pose_slice",default_value="mid"),DeclareLaunchArgument("view",default_value="3d"),OpaqueFunction(function=setup)])
