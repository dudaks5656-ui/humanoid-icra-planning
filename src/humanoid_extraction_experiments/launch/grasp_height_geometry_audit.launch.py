import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _launch_node(context):
    description_share = get_package_share_directory("humanoid_sim_description")
    moveit_share = get_package_share_directory("humanoid_sim_moveit_config")
    xacro_path = os.path.join(description_share, "urdf", "humanoid_sim.urdf.xacro")
    srdf_path = os.path.join(moveit_share, "config", "humanoid_sim.srdf")
    with open(srdf_path, "r", encoding="utf-8") as stream:
        srdf = stream.read()
    return [
        Node(
            package="humanoid_extraction_experiments",
            executable="grasp_height_geometry_audit",
            output="screen",
            parameters=[
                {
                    "robot_description": ParameterValue(
                        Command([FindExecutable(name="xacro"), " ", xacro_path]), value_type=str
                    ),
                    "robot_description_semantic": srdf,
                    "robot_description_kinematics": _load_yaml(
                        os.path.join(moveit_share, "config", "kinematics.yaml")
                    ),
                    "geometry_config": LaunchConfiguration("geometry_config"),
                    "geometry_report": "/home/openarm/humanoid_sim_ws/validation/50mm_grasp_height_geometry_audit.md",
                    "aperture_csv": "/home/openarm/humanoid_sim_ws/validation/50mm_finger_aperture_sweep.csv",
                    "heights_csv": "/home/openarm/humanoid_sim_ws/validation/50mm_grasp_height_candidates.csv",
                    "corrected_csv": "/home/openarm/humanoid_sim_ws/validation/top_open_center_reachability_grasp_corrected.csv",
                    "corrected_report": "/home/openarm/humanoid_sim_ws/validation/top_open_center_reachability_grasp_corrected.md",
                }
            ],
        )
    ]


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
            OpaqueFunction(function=_launch_node),
        ]
    )
