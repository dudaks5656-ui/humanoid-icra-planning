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
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(single_case_extraction src/single_case_extraction.cpp)
add_executable(target_boundary_search src/target_boundary_search.cpp)
add_executable(differential_repeat_trials src/differential_repeat_trials.cpp)
add_executable(reference_trajectory_generator src/reference_trajectory_generator.cpp)
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

install(TARGETS single_case_extraction target_boundary_search differential_repeat_trials reference_trajectory_generator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
