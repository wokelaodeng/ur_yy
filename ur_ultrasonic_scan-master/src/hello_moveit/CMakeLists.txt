cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===============================
# 1️⃣ 基础依赖
# ===============================
find_package(ament_cmake REQUIRED)

# ROS2
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)   # ⭐【新增】你的PCL节点未来可能用到
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# MoveIt
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# 碰撞
find_package(shape_msgs REQUIRED)

# ===============================
# 2️⃣ ⭐ PCL + VTK（关键修改）
# ===============================
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(pcl_conversions REQUIRED)
find_package(X11 REQUIRED)

# ⭐【新增】解决 PCL 可视化窗口必须依赖 VTK
set(VTK_GROUP_ENABLE_Java NO)
find_package(VTK REQUIRED)

# 包含头文件
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}   # ⭐【新增】
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ===============================
# 3️⃣ 可执行文件
# ===============================
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(my_moveit_use src/my_moveit_use.cpp)
add_executable(my_collision_define src/my_collision_define.cpp)

# ⭐【新增】你的PCL窗口程序
add_executable(pcl_scan_workpiece_window src/pcl_scan_workpiece_window.cpp)

# ===============================
# 4️⃣ C++标准
# ===============================
target_compile_features(hello_moveit PUBLIC cxx_std_17)
target_compile_features(my_moveit_use PUBLIC cxx_std_17)
target_compile_features(my_collision_define PUBLIC cxx_std_17)
target_compile_features(pcl_scan_workpiece_window PUBLIC cxx_std_17)  # ⭐新增

# ===============================
# 5️⃣ include 目录
# ===============================
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(my_moveit_use PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(my_collision_define PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(pcl_scan_workpiece_window PUBLIC   # ⭐新增
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ===============================
# 6️⃣ ROS依赖绑定
# ===============================
ament_target_dependencies(hello_moveit
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  tf2_geometry_msgs
  Eigen3
  shape_msgs
  pcl_conversions
)

ament_target_dependencies(my_moveit_use
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs
  Eigen3
  pcl_conversions
)

ament_target_dependencies(my_collision_define
  rclcpp
  moveit_ros_planning_interface
  shape_msgs
)

# ⭐【新增】PCL窗口依赖
ament_target_dependencies(pcl_scan_workpiece_window
  rclcpp
  geometry_msgs
  std_msgs
  pcl_conversions
)

# ===============================
# 7️⃣ ⭐ 链接 PCL + VTK
# ===============================
target_link_libraries(my_moveit_use ${PCL_LIBRARIES})


# ⭐【关键新增】
target_link_libraries(pcl_scan_workpiece_window
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${X11_LIBRARIES}
)

# ===============================
# 8️⃣ 安装
# ===============================
install(TARGETS
  hello_moveit
  my_moveit_use
  my_collision_define
  pcl_scan_workpiece_window   # ⭐新增
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ===============================
# 9️⃣ 测试
# ===============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()