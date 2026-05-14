cmake_minimum_required(VERSION 3.8)
project(robot_arm_base_link)

find_package(ament_cmake REQUIRED)

# 安装 urdf 和 meshes 目录（如果没有 meshes，可以省略）
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()