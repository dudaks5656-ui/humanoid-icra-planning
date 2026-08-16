import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def _yaml(path):
    with open(path, "r", encoding="utf-8") as stream: return yaml.safe_load(stream)
def _text(path):
    with open(path, "r", encoding="utf-8") as stream: return stream.read()
def _launch(context):
    output=LaunchConfiguration("output_dir").perform(context); os.makedirs(output,exist_ok=False)
    input_dir=LaunchConfiguration("input_dir").perform(context)
    description=get_package_share_directory("humanoid_sim_description")
    moveit=get_package_share_directory("humanoid_sim_moveit_config")
    experiment=get_package_share_directory("humanoid_extraction_experiments")
    robot={"robot_description":ParameterValue(Command([FindExecutable(name="xacro")," ",os.path.join(description,"urdf","humanoid_sim.urdf.xacro")]),value_type=str)}
    params=[robot,{"robot_description_semantic":_text(os.path.join(moveit,"config","humanoid_sim.srdf"))},{"robot_description_kinematics":_yaml(os.path.join(moveit,"config","kinematics.yaml"))},{"robot_description_planning":_yaml(os.path.join(moveit,"config","joint_limits.yaml"))},{
      "scene_config":os.path.join(experiment,"config","top_open_reference_scene.yaml"),
      "pilot_config":os.path.join(experiment,"config","adaptive_target_boundary_search_v1.yaml"),
      "input_dir":input_dir,"ray_state_sequence_csv":os.path.join(output,"internal_unused_sequence.csv"),
      "boundary_summary_csv":os.path.join(output,"internal_unused_boundary.csv"),"gripper_envelope_csv":os.path.join(output,"internal_unused_envelope.csv"),
      "result_yaml":os.path.join(output,"torso_axis_ablation_result.yaml"),"audit_md":os.path.join(output,"torso_axis_ablation_audit.md"),
      "results_csv":os.path.join(output,"axis_ablation_results.csv"),"summary_csv":os.path.join(output,"target_axis_summary.csv") }]
    return [Node(package="humanoid_extraction_experiments",executable="torso_axis_ablation_v1",output="screen",parameters=params)]
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("input_dir",default_value="/home/openarm/humanoid_sim_ws/validation/adaptive_target_boundary_search_v1/run_20260815_201241"),DeclareLaunchArgument("output_dir"),OpaqueFunction(function=_launch)])
