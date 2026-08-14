import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _load_text(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def _launch_nodes(context):
    description_share = get_package_share_directory("humanoid_sim_description")
    moveit_share = get_package_share_directory("humanoid_sim_moveit_config")
    experiment_share = get_package_share_directory("humanoid_extraction_experiments")

    xacro_path = os.path.join(description_share, "urdf", "humanoid_sim.urdf.xacro")
    srdf_path = os.path.join(moveit_share, "config", "humanoid_sim.srdf")
    scene_path = LaunchConfiguration("scene_config").perform(context)
    candidate_path = LaunchConfiguration("candidate_config").perform(context)

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_path]), value_type=str
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": _load_text(srdf_path)
    }
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
        "ompl": _load_yaml(os.path.join(moveit_share, "config", "ompl_planning.yaml")),
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

    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        )
    move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=common + [planning_only],
        )
    experiment = Node(
            package="humanoid_extraction_experiments",
            executable="differential_repeat_trials",
            output="screen",
            parameters=common
            + [
                {
                    "scene_config": scene_path,
                    "candidate_config": candidate_path,
                    "output_csv": LaunchConfiguration("output_csv"),
                    "summary_path": LaunchConfiguration("summary_path"),
                    "target_history_csv": LaunchConfiguration("target_history_csv"),
                    "extraction_clearance_csv": LaunchConfiguration("extraction_clearance_csv"),
                    "scene_translation_csv": LaunchConfiguration("scene_translation_csv"),
                    "planning_attempt_id": LaunchConfiguration("planning_attempt_id"),
                    "boundary_csv": LaunchConfiguration("boundary_csv"),
                    "comparison_csv": LaunchConfiguration("comparison_csv"),
                    "ik_audit_csv": LaunchConfiguration("ik_audit_csv"),
                    "geometric_csv": LaunchConfiguration("geometric_csv"),
                    "comparison_input_csv": LaunchConfiguration("comparison_input_csv"),
                    "repeat_trials_csv": LaunchConfiguration("repeat_trials_csv"),
                    "budget_sensitivity_csv": LaunchConfiguration("budget_sensitivity_csv"),
                    "analysis_path": LaunchConfiguration("analysis_path"),
                    "hold_for_rviz": ParameterValue(
                        LaunchConfiguration("hold_for_rviz"), value_type=bool
                    ),
                }
            ],
        )
    rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                os.path.join(experiment_share, "rviz", "single_case_extraction.rviz"),
            ],
            parameters=common,
            condition=IfCondition(LaunchConfiguration("start_rviz")),
        )

    return [
        robot_state_publisher,
        move_group,
        TimerAction(period=3.0, actions=[experiment]),
        rviz,
        RegisterEventHandler(
            OnProcessExit(
                target_action=experiment,
                on_exit=[EmitEvent(event=Shutdown(reason="single-case experiment completed"))],
            )
        ),
    ]


def generate_launch_description():
    experiment_share = get_package_share_directory("humanoid_extraction_experiments")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene_config",
                default_value=os.path.join(experiment_share, "config", "confined_scene.yaml"),
            ),
            DeclareLaunchArgument(
                "candidate_config",
                default_value=os.path.join(experiment_share, "config", "torso_candidates.yaml"),
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("hold_for_rviz", default_value="true"),
            DeclareLaunchArgument(
                "output_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_internal_stages.csv",
            ),
            DeclareLaunchArgument(
                "summary_path",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_internal_summary.md",
            ),
            DeclareLaunchArgument(
                "target_history_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_internal_history.csv",
            ),
            DeclareLaunchArgument(
                "extraction_clearance_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_extraction_clearance.csv",
            ),
            DeclareLaunchArgument(
                "scene_translation_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_scene_translation.csv",
            ),
            DeclareLaunchArgument("planning_attempt_id", default_value="differential_repeat_1"),
            DeclareLaunchArgument(
                "boundary_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_internal_boundary.csv",
            ),
            DeclareLaunchArgument(
                "comparison_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_internal_comparison.csv",
            ),
            DeclareLaunchArgument(
                "ik_audit_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_internal_ik.csv",
            ),
            DeclareLaunchArgument(
                "geometric_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_internal_geometric.csv",
            ),
            DeclareLaunchArgument(
                "comparison_input_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/lift_only_vs_lift_yaw_pitch.csv",
            ),
            DeclareLaunchArgument(
                "repeat_trials_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_target_repeat_trials.csv",
            ),
            DeclareLaunchArgument(
                "budget_sensitivity_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/planning_budget_sensitivity.csv",
            ),
            DeclareLaunchArgument(
                "analysis_path",
                default_value="/home/openarm/humanoid_sim_ws/validation/differential_target_analysis.md",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
