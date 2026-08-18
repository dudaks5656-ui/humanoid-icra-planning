cmake_minimum_required(VERSION 3.8)
project(boundary_sweep_workspace_differential_visualization)
find_package(ament_cmake REQUIRED)
install(PROGRAMS
  scripts/generate_boundary_sweep_differential.py
  scripts/boundary_sweep_differential_rviz.py
  scripts/generate_boundary_sweep_differential_video.py
  scripts/finalize_boundary_sweep_differential.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
