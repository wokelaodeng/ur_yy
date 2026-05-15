cmake_minimum_required(VERSION 3.8)
project(robot_camera_control_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================
# 1. 寻找所有必需的包
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# ⭐ 新增：寻找 TF2 相关坐标变换包
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# 寻找视觉相关库
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ==========================================
# 2. 编译各节点
# ==========================================
# 视觉提取
add_executable(d435i_camera_sim_use src/d435i_camera_sim_use.cpp)
target_include_directories(d435i_camera_sim_use PUBLIC ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
ament_target_dependencies(d435i_camera_sim_use rclcpp sensor_msgs cv_bridge pcl_conversions)
target_link_libraries(d435i_camera_sim_use ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(view_pcd_result src/view_pcd_result.cpp)
target_include_directories(view_pcd_result PUBLIC ${PCL_INCLUDE_DIRS})
ament_target_dependencies(view_pcd_result rclcpp)
target_link_libraries(view_pcd_result ${PCL_LIBRARIES})

add_executable(pcl_scan_workpiece_window src/pcl_scan_workpiece_window.cpp)
target_include_directories(pcl_scan_workpiece_window PUBLIC ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pcl_scan_workpiece_window rclcpp geometry_msgs sensor_msgs cv_bridge pcl_conversions)
target_link_libraries(pcl_scan_workpiece_window ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(my_collision_define src/my_collision_define.cpp)
ament_target_dependencies(my_collision_define rclcpp moveit_ros_planning_interface shape_msgs geometry_msgs pcl_conversions)
target_link_libraries(my_collision_define ${PCL_LIBRARIES})

# ⭐ 新增：编译坐标变换节点
add_executable(point_tf_position src/point_tf_position.cpp)
target_include_directories(point_tf_position PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(point_tf_position rclcpp tf2 tf2_ros tf2_eigen geometry_msgs pcl_conversions)
target_link_libraries(point_tf_position ${PCL_LIBRARIES})

# 轨迹规划
add_executable(trajectory_planner_sim src/Trajectory_Planner_Sim.cpp)
target_include_directories(trajectory_planner_sim PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(trajectory_planner_sim rclcpp geometry_msgs nav_msgs pcl_conversions)
target_link_libraries(trajectory_planner_sim ${PCL_LIBRARIES})

# 机器人控制
add_executable(robot_control_sim src/robot_control_sim.cpp)
ament_target_dependencies(robot_control_sim rclcpp geometry_msgs moveit_ros_planning_interface)

# ==========================================
# 3. 安装规则
# ==========================================
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  d435i_camera_sim_use
  view_pcd_result
  pcl_scan_workpiece_window
  my_collision_define
  point_tf_position       # <--- 加在这里
  trajectory_planner_sim
  robot_control_sim
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()