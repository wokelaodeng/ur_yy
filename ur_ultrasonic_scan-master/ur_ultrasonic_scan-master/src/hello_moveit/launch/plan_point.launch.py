import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ========================================================================
    # 1. 启动 MoveIt 核心服务及 RViz
    # ========================================================================
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur7e_moveit_config'),
                         'launch', 'my_moveit_rviz.launch.py')
        )
    )

    # ========================================================================
    # 3. 启动自定义环境节点
    # ========================================================================
    # 碰撞定义节点
    collision_node = Node(
        package='hello_moveit',
        executable='my_collision_define',
        output='screen',
    )

    # 运动规划节点（通常手动执行，此处可配置是否跟随 launch 一起启动）
    moveit_node = Node(
        package='hello_moveit',
        executable='my_moveit_use',
        output='screen',
    )

    # PCL 实时渲染窗口节点
    pcl_scan_workpiece_node = Node(
        package='hello_moveit',
        executable='pcl_scan_workpiece_window',
        output='screen',
    )

    return LaunchDescription([
        moveit_launch,
        collision_node,   
        pcl_scan_workpiece_node,    
    ])