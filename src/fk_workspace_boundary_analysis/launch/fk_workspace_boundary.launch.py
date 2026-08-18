import hashlib
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORKSPACE = "/home/openarm/humanoid_sim_ws"
VALIDATION = os.path.join(WORKSPACE, "validation")


def sha256(path):
    value = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def verify_manifest(name, base):
    path = os.path.join(VALIDATION, name)
    with open(path, encoding="utf-8") as stream:
        entries = [line.rstrip("\n") for line in stream if line.strip()]
    if not entries:
        raise RuntimeError(f"Empty protected manifest: {path}")
    for entry in entries:
        expected, relative = entry.split("  ", 1)
        target = os.path.join(base, relative)
        if sha256(target) != expected:
            raise RuntimeError(f"Protected manifest mismatch: {target}")


def load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_text(path):
    with open(path, encoding="utf-8") as stream:
        return stream.read()


def setup(context):
    for name, base in (
        ("fixed_base_workspace_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_fine_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_dof_ablation_manifest_sha256.txt", VALIDATION),
        ("fixed_base_workspace_demo_manifest_sha256.txt", WORKSPACE),
        ("fixed_base_workspace_envelope_demo_manifest_sha256.txt", WORKSPACE),
        ("radial_workspace_validation_manifest_sha256.txt", WORKSPACE),
        ("workspace_projection_manifest_sha256.txt", WORKSPACE),
    ):
        verify_manifest(name, base)
    output_dir = os.path.abspath(LaunchConfiguration("output_dir").perform(context))
    os.makedirs(output_dir, exist_ok=True)
    existing = [name for name in os.listdir(output_dir) if name.startswith("fk_workspace_boundary_")]
    if existing:
        raise RuntimeError("Refusing to overwrite FK workspace outputs: " + ", ".join(sorted(existing)))

    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    share = get_package_share_directory("fk_workspace_boundary_analysis")
    config_path = os.path.abspath(LaunchConfiguration("config").perform(context))
    config = load_yaml(config_path)["fk_workspace_boundary"]["ros__parameters"]
    config["output_dir"] = output_dir
    config["samples_per_configuration"] = int(LaunchConfiguration("samples_per_configuration").perform(context))
    xacro = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro]), value_type=str
        )
    }
    semantic = {"robot_description_semantic": load_text(os.path.join(moveit, "config", "humanoid_sim.srdf"))}
    limits = {"robot_description_planning": load_yaml(os.path.join(moveit, "config", "joint_limits.yaml"))}
    print(
        "FK_WORKSPACE_BOUNDARY LAUNCH PREFLIGHT manifests=PASS configs=4 "
        f"samples_per_config={config['samples_per_configuration']} output={output_dir} "
        "halton=YES full_cartesian=NO IK=NO trajectory=NO hardware=NO"
    )
    del share
    return [Node(
        package="fk_workspace_boundary_analysis",
        executable="fk_workspace_boundary",
        name="fk_workspace_boundary",
        output="screen",
        parameters=[robot_description, semantic, limits, config],
    )]


def generate_launch_description():
    share = get_package_share_directory("fk_workspace_boundary_analysis")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "fk_workspace_boundary.yaml")),
        DeclareLaunchArgument("output_dir", default_value=VALIDATION),
        DeclareLaunchArgument("samples_per_configuration", default_value="10000"),
        OpaqueFunction(function=setup),
    ])
