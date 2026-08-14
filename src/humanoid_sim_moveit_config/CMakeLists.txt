cmake_minimum_required(VERSION 3.8)
project(humanoid_sim_moveit_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometric_shapes REQUIRED)

add_executable(collision_state_checker src/collision_state_checker.cpp)
ament_target_dependencies(
  collision_state_checker
  rclcpp
  sensor_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
)

add_executable(self_collision_matrix_sampler src/self_collision_matrix_sampler.cpp)
ament_target_dependencies(
  self_collision_matrix_sampler
  rclcpp
  moveit_core
  moveit_ros_planning
)

add_executable(permanent_contact_analyzer src/permanent_contact_analyzer.cpp)
ament_target_dependencies(
  permanent_contact_analyzer
  rclcpp
  moveit_core
  moveit_ros_planning
  geometric_shapes
)

add_executable(contact_marker_publisher src/contact_marker_publisher.cpp)
ament_target_dependencies(
  contact_marker_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  moveit_core
  moveit_ros_planning
)

install(TARGETS
  collision_state_checker
  self_collision_matrix_sampler
  permanent_contact_analyzer
  contact_marker_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
