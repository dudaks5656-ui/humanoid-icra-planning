cmake_minimum_required(VERSION 3.8)
project(humanoid_extraction_experiments)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(single_case_extraction src/single_case_extraction.cpp)
add_executable(target_boundary_search src/target_boundary_search.cpp)
add_executable(differential_repeat_trials src/differential_repeat_trials.cpp)
add_executable(reference_trajectory_generator src/reference_trajectory_generator.cpp)
add_executable(stage_constrained_reference_generator src/stage_constrained_reference_generator.cpp)
add_executable(grasp_seeded_reference_generator src/grasp_seeded_reference_generator.cpp)
add_executable(stage_constrained_reference_generator_v2 src/stage_constrained_reference_generator_v2.cpp)
add_executable(stage_constrained_reference_generator_v3 src/stage_constrained_reference_generator_v3.cpp)
add_executable(stage_constrained_torso_recovery_v1 src/stage_constrained_torso_recovery_v1.cpp)
add_executable(lift_actuated_extraction_baseline_v1 src/lift_actuated_extraction_baseline_v1.cpp)
add_executable(box_target_yaw_pitch_feasibility_pilot_v1 src/box_target_yaw_pitch_feasibility_pilot_v1.cpp)
add_executable(adaptive_target_boundary_search_v1 src/adaptive_target_boundary_search_v1.cpp)
add_executable(torso_axis_ablation_v1 src/torso_axis_ablation_v1.cpp)
add_executable(paper_main_simulation_dataset_v1 src/paper_main_simulation_dataset_v1.cpp)
add_executable(top_open_box_geometry_publisher src/top_open_box_geometry_publisher.cpp)
add_executable(top_open_center_reachability src/top_open_center_reachability.cpp)
add_executable(grasp_height_geometry_audit src/grasp_height_geometry_audit.cpp)
ament_target_dependencies(
  single_case_extraction
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(single_case_extraction yaml-cpp)

ament_target_dependencies(
  target_boundary_search
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(target_boundary_search yaml-cpp)

ament_target_dependencies(
  differential_repeat_trials
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(differential_repeat_trials yaml-cpp)

ament_target_dependencies(
  reference_trajectory_generator
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(reference_trajectory_generator yaml-cpp)

ament_target_dependencies(
  stage_constrained_reference_generator
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(stage_constrained_reference_generator yaml-cpp)

ament_target_dependencies(
  grasp_seeded_reference_generator
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(grasp_seeded_reference_generator yaml-cpp)

ament_target_dependencies(
  stage_constrained_reference_generator_v2
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(stage_constrained_reference_generator_v2 yaml-cpp)

ament_target_dependencies(
  stage_constrained_reference_generator_v3
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(stage_constrained_reference_generator_v3 yaml-cpp)

ament_target_dependencies(
  stage_constrained_torso_recovery_v1
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(stage_constrained_torso_recovery_v1 yaml-cpp)

ament_target_dependencies(
  lift_actuated_extraction_baseline_v1
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(lift_actuated_extraction_baseline_v1 yaml-cpp)

ament_target_dependencies(
  box_target_yaw_pitch_feasibility_pilot_v1
  rclcpp
  geometry_msgs
  sensor_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(box_target_yaw_pitch_feasibility_pilot_v1 yaml-cpp)

ament_target_dependencies(
  adaptive_target_boundary_search_v1
  rclcpp
  geometry_msgs
  geometric_shapes
  sensor_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
)
target_link_libraries(adaptive_target_boundary_search_v1 yaml-cpp)

ament_target_dependencies(torso_axis_ablation_v1 rclcpp geometry_msgs geometric_shapes sensor_msgs shape_msgs visualization_msgs moveit_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface tf2)
target_link_libraries(torso_axis_ablation_v1 yaml-cpp)

ament_target_dependencies(paper_main_simulation_dataset_v1 rclcpp geometry_msgs geometric_shapes sensor_msgs shape_msgs visualization_msgs moveit_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface tf2)
target_link_libraries(paper_main_simulation_dataset_v1 yaml-cpp)

ament_target_dependencies(
  top_open_box_geometry_publisher
  rclcpp
  geometry_msgs
  geometric_shapes
  sensor_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  urdf
)
target_link_libraries(top_open_box_geometry_publisher yaml-cpp)

ament_target_dependencies(
  top_open_center_reachability
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
)
target_link_libraries(top_open_center_reachability yaml-cpp)

ament_target_dependencies(
  grasp_height_geometry_audit
  rclcpp
  geometry_msgs
  geometric_shapes
  shape_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
)
target_link_libraries(grasp_height_geometry_audit yaml-cpp)

install(TARGETS single_case_extraction target_boundary_search differential_repeat_trials reference_trajectory_generator stage_constrained_reference_generator grasp_seeded_reference_generator stage_constrained_reference_generator_v2 stage_constrained_reference_generator_v3 stage_constrained_torso_recovery_v1 lift_actuated_extraction_baseline_v1 box_target_yaw_pitch_feasibility_pilot_v1 adaptive_target_boundary_search_v1 torso_axis_ablation_v1 paper_main_simulation_dataset_v1 top_open_box_geometry_publisher top_open_center_reachability grasp_height_geometry_audit DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/workspace_boundary_visualizer scripts/check_reference_v1_regression.py scripts/prepare_paper_main_simulation_dataset_v1.py scripts/finalize_paper_main_simulation_dataset_v1.py scripts/run_paper_main_simulation_dataset_v1.sh scripts/check_paper_dataset_progress.sh scripts/resume_paper_main_simulation_dataset_v1.sh DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
