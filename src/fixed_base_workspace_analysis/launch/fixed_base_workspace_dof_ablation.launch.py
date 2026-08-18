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


def _sha(path):
    with open(path, "rb") as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def _verify_manifest(directory, name):
    path = os.path.join(directory, name)
    if not os.path.isfile(path):
        raise RuntimeError("Required manifest missing: " + path)
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            expected, filename = line.rstrip("\n").split("  ", 1)
            if _sha(os.path.join(directory, filename)) != expected:
                raise RuntimeError(f"Manifest verification failed: {name}: {filename}")


def _launch(context):
    output_dir = os.path.abspath(LaunchConfiguration("output_dir").perform(context))
    os.makedirs(output_dir, exist_ok=True)
    _verify_manifest(output_dir, "fixed_base_workspace_manifest_sha256.txt")
    _verify_manifest(output_dir, "fixed_base_workspace_fine_manifest_sha256.txt")
    existing = sorted(glob.glob(os.path.join(output_dir, "fixed_base_workspace_dof_ablation_*")))
    if existing:
        raise RuntimeError("Refusing to overwrite ablation evidence: " + ", ".join(existing))

    description = get_package_share_directory("humanoid_sim_description")
    moveit = get_package_share_directory("humanoid_sim_moveit_config")
    package = get_package_share_directory("fixed_base_workspace_analysis")
    prefix = get_package_prefix("fixed_base_workspace_analysis")
    config_path = os.path.abspath(LaunchConfiguration("config").perform(context))
    ablation = _yaml(config_path)["fixed_base_workspace_dof_ablation"]["ros__parameters"]
    fine_config_path = os.path.join(package, "config", "fixed_base_workspace_fine.yaml")
    fine = _yaml(fine_config_path)["fixed_base_workspace_fine"]["ros__parameters"]
    shared = ["base_frame", "tcp_frame", "arm_group", "full_group", "target_x_min", "target_x_max",
              "target_y_min", "target_y_max", "target_z_min", "target_z_max", "grid_x", "grid_y", "grid_z",
              "max_ik_seeds", "ik_timeout_s", "orientation_tolerance_rad", "exact_bound_epsilon", "random_seed",
              "target_qx", "target_qy", "target_qz", "target_qw", "max_torso_candidates"]
    differences = [key for key in shared if ablation.get(key) != fine.get(key)]
    if differences:
        raise RuntimeError("Ablation/fine contract mismatch: " + ", ".join(differences))

    xacro = os.path.join(description, "urdf", "humanoid_sim.urdf.xacro")
    srdf = os.path.join(moveit, "config", "humanoid_sim.srdf")
    limits_path = os.path.join(moveit, "config", "joint_limits.yaml")
    kinematics_path = os.path.join(moveit, "config", "kinematics.yaml")
    protected = {"urdf_xacro": (xacro, ablation["expected_urdf_xacro_sha256"]),
                 "srdf": (srdf, ablation["expected_srdf_sha256"]),
                 "joint_limits": (limits_path, ablation["expected_joint_limits_sha256"]),
                 "kinematics": (kinematics_path, ablation["expected_kinematics_sha256"])}
    hashes = {}
    for name, (path, expected) in protected.items():
        actual = _sha(path)
        if actual != expected:
            raise RuntimeError(f"Protected {name} hash mismatch: {actual} != {expected}")
        hashes[name + "_sha256"] = actual

    robot_description = {"robot_description": ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro]), value_type=str)}
    semantic = {"robot_description_semantic": _text(srdf)}
    kinematics = {"robot_description_kinematics": _yaml(kinematics_path)}
    limits = {"robot_description_planning": _yaml(limits_path)}
    runtime = {
        "output_dir": output_dir,
        # The ablation runner reuses the fine runner implementation.  These
        # paths satisfy that runner's immutable-evidence contract; ablation
        # itself reconstructs its grid exclusively from fine_points_csv.
        "coarse_points_csv": os.path.join(output_dir, "fixed_base_workspace_points.csv"),
        "coarse_comparison_csv": os.path.join(output_dir, "fixed_base_workspace_comparison.csv"),
        "coarse_manifest": os.path.join(output_dir, "fixed_base_workspace_manifest_sha256.txt"),
        "fine_manifest": os.path.join(output_dir, "fixed_base_workspace_fine_manifest_sha256.txt"),
        "fine_points_csv": os.path.join(output_dir, "fixed_base_workspace_fine_points.csv"),
        "fine_metadata_csv": os.path.join(output_dir, "fixed_base_workspace_fine_metadata.csv"),
        "fine_config_path": fine_config_path,
        "ablation_config_path": config_path,
        "postprocess_executable": os.path.join(
            prefix, "lib", "fixed_base_workspace_analysis", "fixed_base_workspace_dof_ablation_postprocess"),
        **hashes,
    }
    return [Node(
        package="fixed_base_workspace_analysis",
        executable="fixed_base_workspace_dof_ablation",
        name="fixed_base_workspace_dof_ablation",
        output="screen",
        parameters=[robot_description, semantic, kinematics, limits, ablation, runtime],
    )]


def generate_launch_description():
    package = get_package_share_directory("fixed_base_workspace_analysis")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(
            package, "config", "fixed_base_workspace_dof_ablation.yaml")),
        DeclareLaunchArgument("output_dir", default_value="/home/openarm/humanoid_sim_ws/validation"),
        OpaqueFunction(function=_launch),
    ])
