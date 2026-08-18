cmake_minimum_required(VERSION 3.8)
project(radial_workspace_validation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(radial_workspace_validation src/radial_workspace_validation.cpp)
ament_target_dependencies(radial_workspace_validation
  rclcpp geometry_msgs moveit_core moveit_ros_planning)
target_link_libraries(radial_workspace_validation Eigen3::Eigen)
target_compile_features(radial_workspace_validation PUBLIC cxx_std_17)

install(TARGETS radial_workspace_validation DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/radial_workspace_validation_demo.py
  scripts/radial_workspace_validation_overlay.py
  scripts/record_radial_workspace_validation_demo.py
  scripts/finalize_radial_workspace_validation.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
