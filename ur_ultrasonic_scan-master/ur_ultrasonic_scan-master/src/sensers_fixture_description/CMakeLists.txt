cmake_minimum_required(VERSION 3.8)
project(sensers_fixture_description)

find_package(ament_cmake REQUIRED)

# 安装 urdf 和 meshes 目录
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()