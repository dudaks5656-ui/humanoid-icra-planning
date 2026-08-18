import glob
import hashlib
import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _text(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def _verify_manifest(directory, manifest_name):
    manifest = os.path.join(directory, manifest_name)
    if not os.path.isfile(manifest):
        raise RuntimeError("Required coarse manifest is missing: " + manifest)
    with open(manifest, "r", encoding="utf-8") as stream:
        for line in stream:
            expected, name = line.rstrip("\n").split("  ", 1)
            path = os.path.join(directory, name)
            digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
            if digest != expected:
                raise RuntimeError("Coarse manifest verification failed: " + name)


def _launch(context):
    output_dir = os.path.abspath(LaunchConfiguration("output_dir").perform(context))
    os.makedirs(output_dir, exist_ok=True)
    _verify_manifest(output_dir, "fixed_base_workspace_manifest_sha256.txt")
    existing = sorted(glob.glob(os.path.join(output_dir, "fixed_base_workspace_fine_*")))
    if existing:
        raise RuntimeError("Refusing to overwrite fine evidence: " + ", ".join(existing))

    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    package = get_package_share_directory("fixed_base_workspace_analysis")
    prefix = get_package_prefix("fixed_base_workspace_analysis")
    config_path = os.path.abspath(LaunchConfiguration("config").perform(context))
    config = _yaml(config_path)["fixed_base_workspace_fine"]["ros__parameters"]
    xacro = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro]), value_type=str
        )
    }
    semantic = {"robot_description_semantic": _text(os.path.join(moveit, "config", "humanoid_sim.srdf"))}
    kinematics = {"robot_description_kinematics": _yaml(os.path.join(moveit, "config", "kinematics.yaml"))}
    limits = {"robot_description_planning": _yaml(os.path.join(moveit, "config", "joint_limits.yaml"))}
    runtime = {
        "output_dir": output_dir,
        "coarse_points_csv": os.path.join(output_dir, "fixed_base_workspace_points.csv"),
        "coarse_comparison_csv": os.path.join(output_dir, "fixed_base_workspace_comparison.csv"),
        "coarse_manifest": os.path.join(output_dir, "fixed_base_workspace_manifest_sha256.txt"),
        "fine_config_path": config_path,
        "postprocess_executable": os.path.join(
            prefix, "lib", "fixed_base_workspace_analysis", "fixed_base_workspace_fine_postprocess"
        ),
    }
    runner = Node(
        package="fixed_base_workspace_analysis",
        executable="fixed_base_workspace_fine",
        name="fixed_base_workspace_fine",
        output="screen",
        parameters=[robot_description, semantic, kinematics, limits, config, runtime],
    )
    return [runner]


def generate_launch_description():
    package = get_package_share_directory("fixed_base_workspace_analysis")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(package, "config", "fixed_base_workspace_fine.yaml")),
        DeclareLaunchArgument("output_dir", default_value="/home/openarm/humanoid_sim_ws/validation"),
        OpaqueFunction(function=_launch),
    ])
