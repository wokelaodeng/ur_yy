cmake_minimum_required(VERSION 3.8)
project(camera_use)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================================================
# 1. 强制指定使用 PCL 1.13 (与 hello_moveit 保持一致)
# =========================================================
# 【核心修复补丁】解决 PCL 1.13 与 Ubuntu QHull 目标大小写冲突的 Bug
find_package(Qhull REQUIRED)
if(NOT TARGET QHULL::QHULL)
  add_library(QHULL::QHULL INTERFACE IMPORTED)
  # 自动检测系统提供的是哪种命名，并将其映射给全大写的 QHULL::QHULL
  if(TARGET Qhull::qhull_r)
    set_target_properties(QHULL::QHULL PROPERTIES INTERFACE_LINK_LIBRARIES Qhull::qhull_r)
  elseif(TARGET Qhull::qhull)
    set_target_properties(QHULL::QHULL PROPERTIES INTERFACE_LINK_LIBRARIES Qhull::qhull)
  else()
    set_target_properties(QHULL::QHULL PROPERTIES INTERFACE_LINK_LIBRARIES qhull)
  endif()
endif()

# 补丁打完后，再放心大胆地寻找 PCL
set(PCL_DIR "/usr/local/share/pcl-1.13") 
find_package(PCL 1.13 EXACT REQUIRED COMPONENTS common io visualization filters segmentation sample_consensus features surface)

# =========================================================
# 2. 引入 VTK 库 (PCL 1.13 的可视化窗口强依赖 VTK，防闪退)
# =========================================================
set(VTK_GROUP_ENABLE_Java NO)
find_package(VTK REQUIRED)

# =========================================================
# 3. 寻找所有 ROS 2 依赖与 OpenCV
# =========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(OpenCV REQUIRED)

# =========================================================
# 4. 包含头文件与库目录
# =========================================================
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# =========================================================
# 5. 添加可执行节点与依赖链接
# =========================================================
# --- [原有节点] ---
add_executable(my_camera_use src/my_camera_use.cpp)
target_include_directories(my_camera_use PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(my_camera_use PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(my_camera_use rclcpp sensor_msgs std_msgs cv_bridge image_transport pcl_conversions pcl_ros)
target_link_libraries(my_camera_use ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES})


# --- [新增节点 1] 真实物理相机单帧处理节点 ---
add_executable(my_camera_use_new src/my_camera_use_new.cpp)
target_include_directories(my_camera_use_new PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(my_camera_use_new PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(my_camera_use_new rclcpp sensor_msgs std_msgs cv_bridge image_transport pcl_conversions pcl_ros)
# 特别链接 stdc++fs 用于 C++17 的文件系统目录操作
target_link_libraries(my_camera_use_new ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs)


# --- [新增节点 2] 真实处理结果离线查看节点 ---
add_executable(real_point_cloud_deal_view src/real_point_cloud_deal_view.cpp)
target_include_directories(real_point_cloud_deal_view PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(real_point_cloud_deal_view PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(real_point_cloud_deal_view rclcpp pcl_conversions pcl_ros)
# 渲染窗口依赖 PCL 和 VTK
target_link_libraries(real_point_cloud_deal_view ${PCL_LIBRARIES} ${VTK_LIBRARIES})


# =========================================================
# 6. 安装规则声明 (必须将新节点注册到 lib 目录)
# =========================================================
install(TARGETS 
  my_camera_use
  my_camera_use_new
  real_point_cloud_deal_view
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()