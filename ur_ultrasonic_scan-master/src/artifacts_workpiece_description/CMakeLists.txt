cmake_minimum_required(VERSION 3.8)
project(artifacts_workpiece_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()