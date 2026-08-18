cmake_minimum_required(VERSION 3.8)
project(fixed_base_workspace_envelope_demo)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(fixed_base_workspace_envelope_demo src/fixed_base_workspace_envelope_demo.cpp)
ament_target_dependencies(fixed_base_workspace_envelope_demo
  rclcpp geometry_msgs sensor_msgs std_msgs visualization_msgs
  moveit_core moveit_msgs moveit_ros_planning)
target_link_libraries(fixed_base_workspace_envelope_demo Eigen3::Eigen)
target_compile_features(fixed_base_workspace_envelope_demo PUBLIC cxx_std_17)

install(TARGETS fixed_base_workspace_envelope_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/fixed_base_workspace_envelope_overlay.py
  scripts/record_fixed_base_workspace_envelope_demo.py
  scripts/finalize_fixed_base_workspace_envelope_demo.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
