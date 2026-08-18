import glob
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
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
    output_dir = os.path.abspath(LaunchConfiguration("output_dir").perform(context))
    os.makedirs(output_dir, exist_ok=True)
    existing = sorted(glob.glob(os.path.join(output_dir, "fixed_base_workspace_*")))
    if existing:
        raise RuntimeError(
            "Refusing to overwrite prior fixed-base workspace evidence: " + ", ".join(existing)
        )

    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    package = get_package_share_directory("fixed_base_workspace_analysis")
    xacro = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    config_path = LaunchConfiguration("config").perform(context)
    config = _yaml(config_path)["fixed_base_workspace"]["ros__parameters"]

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro]), value_type=str
        )
    }
    semantic = {
        "robot_description_semantic": _text(os.path.join(moveit, "config", "humanoid_sim.srdf"))
    }
    kinematics = {
        "robot_description_kinematics": _yaml(os.path.join(moveit, "config", "kinematics.yaml"))
    }
    limits = {
        "robot_description_planning": _yaml(os.path.join(moveit, "config", "joint_limits.yaml"))
    }

    runner = Node(
        package="fixed_base_workspace_analysis",
        executable="fixed_base_workspace",
        name="fixed_base_workspace",
        output="screen",
        parameters=[robot_description, semantic, kinematics, limits, config, {"output_dir": output_dir}],
    )
    postprocess = Node(
        package="fixed_base_workspace_analysis",
        executable="fixed_base_workspace_postprocess",
        name="fixed_base_workspace_postprocess",
        output="screen",
        arguments=["--output-dir", output_dir, "--config", config_path],
    )

    return [
        runner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=runner,
                on_exit=[postprocess],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=postprocess,
                on_exit=[EmitEvent(event=Shutdown(reason="fixed-base workspace batch completed"))],
            )
        ),
    ]


def generate_launch_description():
    package = get_package_share_directory("fixed_base_workspace_analysis")
    default_output = "/home/openarm/humanoid_sim_ws/validation"
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(package, "config", "fixed_base_workspace.yaml"),
        ),
        DeclareLaunchArgument("output_dir", default_value=default_output),
        OpaqueFunction(function=_launch),
    ])
