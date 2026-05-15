import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_camera_control_sim')

    # ==========================================
    # 1. 核心仿真与可视化环境 (第 0 秒启动)
    # ==========================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'gazebo.launch.py'))
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'my_moveit_rviz.launch.py'))
    )

    pcl_window_node = Node(
        package='robot_camera_control_sim',
        executable='pcl_scan_workpiece_window',
        output='screen'
    )

    # 启动相机感知节点，生成 target_cylinder_smoothed.pcd
    camera_perception_node = Node(
        package='robot_camera_control_sim',
        executable='d435i_camera_sim_use',
        output='screen'
    )

    # ==========================================
    # 2. 碰撞定义节点 (延时 5 秒，等待 MoveIt 就绪)
    # ==========================================
    collision_define_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='robot_camera_control_sim',
                executable='my_collision_define',
                output='screen'
            )
        ]
    )

    # ==========================================
    # 4. TF 坐标转换节点 (延时 12 秒)
    # 必须等待相机保存完初始 PCD，且 TF 树广播稳定后执行
    # ==========================================
    point_tf_position_node = Node(
        package='robot_camera_control_sim',
        executable='point_tf_position',
        output='screen'
    )

    delayed_tf_node = TimerAction(
        period=12.0,
        actions=[point_tf_position_node]
    )

    # ==========================================
    # 5. 轨迹规划节点 (事件触发：等待 TF 转换完成)
    # 必须等待 target_cylinder_base_link.pcd 生成后才能启动
    # ==========================================
    trajectory_planner_node = Node(
        package='robot_camera_control_sim',
        executable='trajectory_planner_sim',
        output='screen'
    )

    # 监听器：当 point_tf_position 节点运行结束退出时，自动启动规划节点
    start_planner_after_tf = RegisterEventHandler(
        OnProcessExit(
            target_action=point_tf_position_node,
            on_exit=[trajectory_planner_node]
        )
    )

    # ==========================================
    # 6. 将所有动作打包返回
    # ==========================================
    return LaunchDescription([
        gazebo_launch,
        moveit_rviz_launch,
        pcl_window_node,
        camera_perception_node,
        collision_define_node,
        delayed_tf_node,
        start_planner_after_tf
    ])
