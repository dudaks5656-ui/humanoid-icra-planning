import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context):
    description_share = get_package_share_directory("humanoid_sim_description")
    moveit_share = get_package_share_directory("humanoid_sim_moveit_config")
    experiment_share = get_package_share_directory("humanoid_extraction_experiments")

    xacro_path = os.path.join(description_share, "urdf", "humanoid_sim.urdf.xacro")
    srdf_path = os.path.join(moveit_share, "config", "humanoid_sim.srdf")
    with open(srdf_path, "r", encoding="utf-8") as stream:
        robot_description_semantic = stream.read()

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_path]), value_type=str
        )
    }
    semantic = {"robot_description_semantic": robot_description_semantic}

    # Deliberately no move_group, OMPL, IK, controller, ros2_control, or trajectory node.
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    geometry_publisher = Node(
        package="humanoid_extraction_experiments",
        executable="top_open_box_geometry_publisher",
        output="screen",
        parameters=[
            robot_description,
            semantic,
            {
                "geometry_config": LaunchConfiguration("geometry_config"),
                "static_collision_csv": LaunchConfiguration("static_collision_csv"),
                "aabb_csv": LaunchConfiguration("aabb_csv"),
                "placement_candidates_csv": LaunchConfiguration("placement_candidates_csv"),
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[robot_description],
    )
    return [state_publisher, geometry_publisher, rviz]


def generate_launch_description():
    experiment_share = get_package_share_directory("humanoid_extraction_experiments")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "geometry_config",
                default_value=os.path.join(
                    experiment_share, "config", "top_open_box_600x400x150.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "static_collision_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/top_open_box_static_collision_check.csv",
            ),
            DeclareLaunchArgument(
                "aabb_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/robot_box_aabb_comparison.csv",
            ),
            DeclareLaunchArgument(
                "placement_candidates_csv",
                default_value="/home/openarm/humanoid_sim_ws/validation/top_open_box_placement_candidates.csv",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="/home/openarm/humanoid_sim_ws/validation/top_open_box_600x400x150.rviz",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
