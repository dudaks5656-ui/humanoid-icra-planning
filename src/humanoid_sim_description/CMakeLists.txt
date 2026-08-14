cmake_minimum_required(VERSION 3.8)
project(humanoid_sim_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf validation
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
