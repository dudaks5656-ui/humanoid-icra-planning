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


def _yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _text(path):
    with open(path, encoding="utf-8") as stream:
        return stream.read()


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _verify_manifest(path, base):
    with open(path, encoding="utf-8") as stream:
        rows = [line.rstrip("\n") for line in stream if line.strip()]
    if not rows:
        raise RuntimeError(f"Empty immutable manifest: {path}")
    for row in rows:
        expected, relative = row.split("  ", 1)
        target = os.path.join(base, relative)
        if _sha256(target) != expected:
            raise RuntimeError(f"Immutable manifest mismatch: {target}")


def _launch(context):
    manifests = [
        (os.path.join(VALIDATION, "fixed_base_workspace_manifest_sha256.txt"), VALIDATION),
        (os.path.join(VALIDATION, "fixed_base_workspace_fine_manifest_sha256.txt"), VALIDATION),
        (os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_manifest_sha256.txt"), VALIDATION),
        (os.path.join(VALIDATION, "fixed_base_workspace_demo_manifest_sha256.txt"), WORKSPACE),
        (os.path.join(VALIDATION, "fixed_base_workspace_envelope_demo_manifest_sha256.txt"), WORKSPACE),
    ]
    for manifest, base in manifests:
        _verify_manifest(manifest, base)
    existing = [name for name in os.listdir(VALIDATION) if name.startswith("radial_workspace_validation_")]
    if existing:
        raise RuntimeError("Refusing to overwrite radial evidence: " + ", ".join(sorted(existing)))

    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    package = get_package_share_directory("radial_workspace_validation")
    config_path = os.path.abspath(LaunchConfiguration("config").perform(context))
    config = _yaml(config_path)["radial_workspace_validation"]["ros__parameters"]
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
        "output_dir": VALIDATION,
        "comparison_csv": os.path.join(VALIDATION, "fixed_base_workspace_dof_ablation_comparison.csv"),
    }
    print("RADIAL_WORKSPACE LAUNCH PREFLIGHT manifests=PASS source=immutable_1440 C0_C3_only=true")
    return [Node(
        package="radial_workspace_validation",
        executable="radial_workspace_validation",
        name="radial_workspace_validation",
        output="screen",
        parameters=[robot_description, semantic, kinematics, limits, config, runtime],
    )]


def generate_launch_description():
    package = get_package_share_directory("radial_workspace_validation")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(package, "config", "radial_workspace_validation.yaml")),
        OpaqueFunction(function=_launch),
    ])
