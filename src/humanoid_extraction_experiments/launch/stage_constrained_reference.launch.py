import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


VALIDATION = "/home/openarm/humanoid_sim_ws/validation"


def _yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _text(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def _launch(context):
    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    experiment = get_package_share_directory("humanoid_extraction_experiments")
    xacro = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro]), value_type=str
        )
    }
    semantic = {"robot_description_semantic": _text(os.path.join(moveit, "config", "humanoid_sim.srdf"))}
    kinematics = {"robot_description_kinematics": _yaml(os.path.join(moveit, "config", "kinematics.yaml"))}
    limits = {"robot_description_planning": _yaml(os.path.join(moveit, "config", "joint_limits.yaml"))}
    pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": _yaml(os.path.join(moveit, "config", "ompl_planning.yaml")),
    }
    common = [robot_description, semantic, kinematics, limits, pipeline]
    planning_only = {
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "disable_capabilities": "move_group/MoveGroupExecuteTrajectoryAction",
    }
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[robot_description])
    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                      parameters=common + [planning_only])
    generator = Node(
        package="humanoid_extraction_experiments",
        executable="stage_constrained_reference_generator",
        output="screen",
        parameters=common + [{
            "scene_config": os.path.join(experiment, "config", "top_open_reference_scene.yaml"),
            "candidate_config": os.path.join(experiment, "config", "torso_candidates.yaml"),
            "output_csv": os.path.join(VALIDATION, "stage_constrained_internal_stages.csv"),
            "summary_path": os.path.join(VALIDATION, "stage_constrained_internal_summary.md"),
            "target_history_csv": os.path.join(VALIDATION, "stage_constrained_internal_history.csv"),
            "extraction_clearance_csv": os.path.join(VALIDATION, "stage_constrained_internal_clearance.csv"),
            "scene_translation_csv": os.path.join(VALIDATION, "stage_constrained_internal_scene_translation.csv"),
            "planning_attempt_id": "stage_constrained_offline_reference",
            "boundary_csv": os.path.join(VALIDATION, "stage_constrained_internal_boundary.csv"),
            "comparison_csv": os.path.join(VALIDATION, "stage_constrained_internal_comparison.csv"),
            "ik_audit_csv": os.path.join(VALIDATION, "stage_constrained_internal_ik.csv"),
            "geometric_csv": os.path.join(VALIDATION, "stage_constrained_internal_geometric.csv"),
            "comparison_input_csv": os.path.join(VALIDATION, "lift_only_vs_lift_yaw_pitch.csv"),
            "repeat_trials_csv": os.path.join(VALIDATION, "stage_constrained_internal_repeat.csv"),
            "budget_sensitivity_csv": os.path.join(VALIDATION, "stage_constrained_internal_budget.csv"),
            "analysis_path": os.path.join(VALIDATION, "stage_constrained_internal_analysis.md"),
            "reference_trials_csv": os.path.join(VALIDATION, "stage_constrained_internal_random_trials_unused.csv"),
            "reference_trajectory_csv": os.path.join(VALIDATION, "stage_constrained_internal_random_trajectory_unused.csv"),
            "reference_waypoints_yaml": os.path.join(VALIDATION, "stage_constrained_internal_random_waypoints_unused.yaml"),
            "reference_audit_md": os.path.join(VALIDATION, "stage_constrained_internal_random_audit_unused.md"),
            "stage_trials_csv": os.path.join(VALIDATION, "stage_constrained_reference_trials.csv"),
            "stage_trajectory_csv": os.path.join(VALIDATION, "stage_constrained_reference_trajectory.csv"),
            "stage_waypoints_yaml": os.path.join(VALIDATION, "stage_constrained_reference_waypoints.yaml"),
            "stage_audit_md": os.path.join(VALIDATION, "stage_constrained_reference_audit.md"),
            "hold_for_rviz": ParameterValue(LaunchConfiguration("hold_for_rviz"), value_type=bool),
        }],
    )
    return [
        rsp,
        move_group,
        TimerAction(period=3.0, actions=[generator]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=generator,
                on_exit=[EmitEvent(event=Shutdown(reason="stage-constrained generator completed"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hold_for_rviz", default_value="true"),
        OpaqueFunction(function=_launch),
    ])
