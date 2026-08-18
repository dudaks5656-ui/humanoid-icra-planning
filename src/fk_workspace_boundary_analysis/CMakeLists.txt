cmake_minimum_required(VERSION 3.8)
project(fk_workspace_boundary_analysis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

add_executable(fk_workspace_boundary src/fk_workspace_boundary.cpp)
ament_target_dependencies(fk_workspace_boundary rclcpp moveit_core moveit_ros_planning)

install(TARGETS fk_workspace_boundary DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/postprocess_fk_workspace_boundary.py
  scripts/fk_workspace_boundary_rviz.py
  scripts/generate_fk_workspace_boundary_video.py
  scripts/finalize_fk_workspace_boundary.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
