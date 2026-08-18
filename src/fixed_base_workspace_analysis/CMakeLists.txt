cmake_minimum_required(VERSION 3.8)
project(fixed_base_workspace_analysis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2 REQUIRED)

add_executable(fixed_base_workspace src/fixed_base_workspace.cpp)
add_executable(fixed_base_workspace_fine src/fixed_base_workspace_fine.cpp)
add_executable(fixed_base_workspace_dof_ablation src/fixed_base_workspace_dof_ablation.cpp)
ament_target_dependencies(
  fixed_base_workspace
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  tf2
)
ament_target_dependencies(
  fixed_base_workspace_fine
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  tf2
)
ament_target_dependencies(
  fixed_base_workspace_dof_ablation
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  tf2
)

install(TARGETS fixed_base_workspace fixed_base_workspace_fine fixed_base_workspace_dof_ablation
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/fixed_base_workspace_postprocess
  scripts/fixed_base_workspace_fine_postprocess
  scripts/fixed_base_workspace_dof_ablation_postprocess
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
