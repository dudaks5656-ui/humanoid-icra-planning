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


def _yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _text(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def _launch(context):
    validation = LaunchConfiguration("output_dir").perform(context)
    os.makedirs(validation, exist_ok=False)
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
    rsp = Node(
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
    baseline = Node(
        package="humanoid_extraction_experiments",
        executable="lift_actuated_extraction_baseline_v1",
        output="screen",
        parameters=common + [{
            "scene_config": os.path.join(experiment, "config", "top_open_reference_scene.yaml"),
            "candidate_config": os.path.join(experiment, "config", "torso_candidates.yaml"),
            "lift_baseline_config": os.path.join(
                experiment, "config", "lift_actuated_extraction_baseline_v1.yaml"
            ),
            "output_csv": os.path.join(validation, "internal_stages_unused.csv"),
            "summary_path": os.path.join(validation, "internal_summary_unused.md"),
            "target_history_csv": os.path.join(validation, "internal_history_unused.csv"),
            "extraction_clearance_csv": os.path.join(validation, "internal_clearance_unused.csv"),
            "scene_translation_csv": os.path.join(validation, "internal_translation_unused.csv"),
            "planning_attempt_id": "lift_actuated_strict_planning_only",
            "boundary_csv": os.path.join(validation, "internal_boundary_unused.csv"),
            "comparison_csv": os.path.join(validation, "internal_comparison_unused.csv"),
            "ik_audit_csv": os.path.join(validation, "internal_ik_unused.csv"),
            "geometric_csv": os.path.join(validation, "internal_geometric_unused.csv"),
            "comparison_input_csv": os.path.join(validation, "internal_comparison_input_unused.csv"),
            "repeat_trials_csv": os.path.join(validation, "internal_repeat_unused.csv"),
            "budget_sensitivity_csv": os.path.join(validation, "internal_budget_unused.csv"),
            "analysis_path": os.path.join(validation, "internal_analysis_unused.md"),
            "reference_trials_csv": os.path.join(validation, "internal_trials_unused.csv"),
            "reference_trajectory_csv": os.path.join(validation, "internal_trajectory_unused.csv"),
            "reference_waypoints_yaml": os.path.join(validation, "internal_waypoints_unused.yaml"),
            "reference_audit_md": os.path.join(validation, "internal_audit_unused.md"),
            "lift_baseline_samples_csv": os.path.join(validation, "lift_actuated_samples.csv"),
            "lift_baseline_grasp_candidates_csv": os.path.join(validation, "grasp_candidates.csv"),
            "lift_baseline_trials_csv": os.path.join(validation, "strict_candidate_trials.csv"),
            "lift_baseline_summary_csv": os.path.join(validation, "lift_actuated_summary.csv"),
            "lift_baseline_result_yaml": os.path.join(validation, "lift_actuated_result.yaml"),
            "lift_baseline_audit_md": os.path.join(validation, "lift_actuated_audit.md"),
            "hold_for_rviz": False,
        }],
    )
    return [
        rsp,
        move_group,
        TimerAction(period=3.0, actions=[baseline]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=baseline,
                on_exit=[EmitEvent(event=Shutdown(reason="lift-actuated baseline audit completed"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir"),
        OpaqueFunction(function=_launch),
    ])
