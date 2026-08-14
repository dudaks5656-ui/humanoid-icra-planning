import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _load_text(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def _launch_nodes(context):
    profile = LaunchConfiguration("ompl_profile").perform(context)
    if profile not in ("baseline", "fine"):
        raise RuntimeError("ompl_profile must be 'baseline' or 'fine'")

    description_share = get_package_share_directory("humanoid_sim_description")
    moveit_share = get_package_share_directory("humanoid_sim_moveit_config")
    xacro_path = os.path.join(description_share, "urdf", "humanoid_sim.urdf.xacro")
    srdf_path = os.path.join(moveit_share, "config", "humanoid_sim.srdf")
    ompl_name = "ompl_planning.yaml" if profile == "baseline" else "ompl_planning_fine.yaml"

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_path]), value_type=str
        )
    }
    robot_description_semantic = {"robot_description_semantic": _load_text(srdf_path)}
    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml(
            os.path.join(moveit_share, "config", "kinematics.yaml")
        )
    }
    robot_description_planning = {
        "robot_description_planning": _load_yaml(
            os.path.join(moveit_share, "config", "joint_limits.yaml")
        )
    }
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": _load_yaml(os.path.join(moveit_share, "config", ompl_name)),
    }
    planning_only = {
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "capabilities": "",
        "disable_capabilities": "move_group/MoveGroupExecuteTrajectoryAction",
    }

    common = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        planning_pipeline,
    ]

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            parameters=[robot_description],
            condition=IfCondition(LaunchConfiguration("start_joint_state_gui")),
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=common + [planning_only],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(moveit_share, "rviz", "planning_only.rviz")],
            parameters=common,
            condition=IfCondition(LaunchConfiguration("start_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ompl_profile",
                default_value="baseline",
                description="baseline (0.01) or provisional fine (0.0025)",
            ),
            DeclareLaunchArgument("start_joint_state_gui", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
