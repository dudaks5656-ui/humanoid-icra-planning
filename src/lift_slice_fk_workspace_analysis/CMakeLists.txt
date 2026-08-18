cmake_minimum_required(VERSION 3.8)
project(lift_slice_fk_workspace_analysis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

add_executable(lift_slice_fk_workspace src/lift_slice_fk_workspace.cpp)
ament_target_dependencies(lift_slice_fk_workspace rclcpp moveit_core moveit_ros_planning)

install(TARGETS lift_slice_fk_workspace DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/postprocess_lift_slice_fk_workspace.py
  scripts/lift_slice_fk_workspace_rviz.py
  scripts/generate_lift_slice_fk_workspace_video.py
  scripts/finalize_lift_slice_fk_workspace.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
