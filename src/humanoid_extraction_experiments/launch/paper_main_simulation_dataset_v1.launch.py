import datetime
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def _yaml(path):
    with open(path,"r",encoding="utf-8") as stream:return yaml.safe_load(stream)
def _text(path):
    with open(path,"r",encoding="utf-8") as stream:return stream.read()
def _launch(context):
    root=LaunchConfiguration("run_root").perform(context)
    desc=get_package_share_directory("humanoid_sim_description"); moveit=get_package_share_directory("humanoid_sim_moveit_config"); exp=get_package_share_directory("humanoid_extraction_experiments")
    deadline=int(datetime.datetime.fromisoformat("2026-08-16T10:30:00+09:00").timestamp())
    robot={"robot_description":ParameterValue(Command([FindExecutable(name="xacro")," ",os.path.join(desc,"urdf","humanoid_sim.urdf.xacro")]),value_type=str)}
    params=[robot,{"robot_description_semantic":_text(os.path.join(moveit,"config","humanoid_sim.srdf"))},{"robot_description_kinematics":_yaml(os.path.join(moveit,"config","kinematics.yaml"))},{"robot_description_planning":_yaml(os.path.join(moveit,"config","joint_limits.yaml"))},{
      "scene_config":os.path.join(exp,"config","top_open_reference_scene.yaml"),"pilot_config":os.path.join(exp,"config","adaptive_target_boundary_search_v1.yaml"),
      "input_dir":"/home/openarm/humanoid_sim_ws/validation/torso_axis_ablation_v1/run_20260815_210000",
      "ray_state_sequence_csv":os.path.join(root,"logs","internal_sequence.csv"),"boundary_summary_csv":os.path.join(root,"logs","internal_boundary.csv"),"gripper_envelope_csv":os.path.join(root,"raw","gripper_swept_envelope.csv"),
      "result_yaml":os.path.join(root,"logs","internal_axis.yaml"),"audit_md":os.path.join(root,"logs","internal_axis.md"),"results_csv":os.path.join(root,"logs","internal_axis_results.csv"),"summary_csv":os.path.join(root,"logs","internal_axis_summary.csv"),
      "run_root":root,"deadline_epoch":deadline}]
    return [Node(package="humanoid_extraction_experiments",executable="paper_main_simulation_dataset_v1",output="screen",parameters=params)]
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("run_root"),OpaqueFunction(function=_launch)])
