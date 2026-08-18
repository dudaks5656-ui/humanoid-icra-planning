cmake_minimum_required(VERSION 3.8)
project(workspace_projection_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/generate_workspace_projections.py
  scripts/workspace_projection_rviz.py
  scripts/generate_workspace_projection_video.py
  scripts/finalize_workspace_projection.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
