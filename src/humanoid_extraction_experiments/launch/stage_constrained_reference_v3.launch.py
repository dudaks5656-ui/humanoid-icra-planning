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
    # Refuse reuse so previous validation evidence cannot be overwritten.
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
    generator = Node(
        package="humanoid_extraction_experiments",
        executable="stage_constrained_reference_generator_v3",
        output="screen",
        parameters=common + [{
            "scene_config": os.path.join(experiment, "config", "top_open_reference_scene.yaml"),
            "candidate_config": os.path.join(experiment, "config", "torso_candidates.yaml"),
            "output_csv": os.path.join(validation, "stage_v3_internal_stages.csv"),
            "summary_path": os.path.join(validation, "stage_v3_internal_summary.md"),
            "target_history_csv": os.path.join(validation, "stage_v3_internal_history.csv"),
            "extraction_clearance_csv": os.path.join(validation, "stage_v3_internal_clearance.csv"),
            "scene_translation_csv": os.path.join(validation, "stage_v3_internal_scene_translation.csv"),
            "planning_attempt_id": "stage_v3_layered_ik_graph",
            "boundary_csv": os.path.join(validation, "stage_v3_internal_boundary_unused.csv"),
            "comparison_csv": os.path.join(validation, "stage_v3_internal_comparison.csv"),
            "ik_audit_csv": os.path.join(validation, "stage_v3_internal_ik.csv"),
            "geometric_csv": os.path.join(validation, "stage_v3_internal_geometric.csv"),
            "comparison_input_csv": os.path.join(validation, "lift_only_vs_lift_yaw_pitch.csv"),
            "repeat_trials_csv": os.path.join(validation, "stage_v3_internal_repeat.csv"),
            "budget_sensitivity_csv": os.path.join(validation, "stage_v3_internal_budget.csv"),
            "analysis_path": os.path.join(validation, "stage_v3_internal_analysis.md"),
            "reference_trials_csv": os.path.join(validation, "stage_v3_internal_random_trials_unused.csv"),
            "reference_trajectory_csv": os.path.join(validation, "stage_v3_internal_random_trajectory_unused.csv"),
            "reference_waypoints_yaml": os.path.join(validation, "stage_v3_internal_random_waypoints_unused.yaml"),
            "reference_audit_md": os.path.join(validation, "stage_v3_internal_random_audit_unused.md"),
            "stage_v3_summary_csv": os.path.join(validation, "stage_v3_summary.csv"),
            "stage_v3_candidates_csv": os.path.join(validation, "stage_v3_candidates.csv"),
            "stage_v3_layers_csv": os.path.join(validation, "stage_v3_layers.csv"),
            "stage_v3_edges_csv": os.path.join(validation, "stage_v3_edges.csv"),
            "stage_v3_boundary_csv": os.path.join(validation, "stage_v3_failure_boundary.csv"),
            "stage_v3_graph_path_csv": os.path.join(validation, "stage_v3_selected_graph_path.csv"),
            "stage_v3_waypoints_yaml": os.path.join(validation, "stage_v3_waypoints.yaml"),
            "stage_v3_audit_md": os.path.join(validation, "stage_v3_audit.md"),
            "stage_v3_boundary_audit_summary_csv": os.path.join(validation, "stage_v3_boundary_audit_summary.csv"),
            "stage_v3_boundary_audit_edges_csv": os.path.join(validation, "stage_v3_boundary_audit_edges.csv"),
            "stage_v3_boundary_audit_joints_csv": os.path.join(validation, "stage_v3_boundary_audit_joint_deltas.csv"),
            "stage_v3_boundary_audit_closest_csv": os.path.join(validation, "stage_v3_boundary_audit_closest_pair.csv"),
            "stage_v3_boundary_audit_jacobian_csv": os.path.join(validation, "stage_v3_boundary_audit_jacobian.csv"),
            "stage_v3_boundary_audit_continuation_csv": os.path.join(validation, "stage_v3_boundary_audit_continuation.csv"),
            "stage_v3_boundary_audit_refinement_csv": os.path.join(validation, "stage_v3_boundary_audit_refinement.csv"),
            "stage_v3_boundary_audit_md": os.path.join(validation, "stage_v3_boundary_audit.md"),
            "stage_v3_boundary_audit_only": ParameterValue(
                LaunchConfiguration("boundary_audit_only"), value_type=bool
            ),
            "stage_v3_ik_seeds_per_waypoint": 100,
            "stage_v3_max_candidates_per_waypoint": 48,
            "stage_v3_cartesian_spacing_m": 0.001,
            "stage_v3_duplicate_tolerance_rad": 0.0001,
            "stage_v3_max_edge_revolute_step_rad": 0.12,
            "stage_v3_max_edge_prismatic_step_m": 0.005,
            "stage_v3_displacement_weight": 1.0,
            "stage_v3_limit_proximity_weight": 0.002,
            "hold_for_rviz": False,
        }],
    )
    return [
        rsp,
        move_group,
        TimerAction(period=3.0, actions=[generator]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=generator,
                on_exit=[EmitEvent(event=Shutdown(reason="stage-constrained v3 diagnostic completed"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("boundary_audit_only", default_value="false"),
        DeclareLaunchArgument("output_dir"),
        OpaqueFunction(function=_launch),
    ])
