import os
import re
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

import xacro

def remove_comments(text):
    pattern = r''
    return re.sub(pattern, '', text, flags=re.DOTALL)

# ====================================================================
# ⭐ 终极修复方案：拦截器函数
# 作用：将 package:// 替换为物理硬盘绝对路径 file://，彻底绕过 Gazebo 连网死锁
# ====================================================================
def resolve_package_path(match):
    pkg_name = match.group(1)
    rel_path = match.group(2)
    try:
        # 动态获取包在你电脑上的真实安装路径
        pkg_path = get_package_share_directory(pkg_name)
        return f"file://{pkg_path}/{rel_path}"
    except Exception:
        return match.group(0)

def generate_launch_description():
    robot_name_in_model = 'ur7e'
    package_name = 'mybot_description'
    urdf_name = "ur7e.urdf.xacro" # 确保后缀正确

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    table_xacro_path = os.path.join(pkg_share, 'urdf/table.xacro')

    table_urdf_path = '/tmp/table.urdf'

    generate_table_urdf = ExecuteProcess(
        cmd=['xacro', table_xacro_path, '-o', table_urdf_path],
        output='screen',
        name='generate_table_urdf'
    )

    spawn_table_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'workpieces', '-file', table_urdf_path,'-x', '0.4', '-y', '0.0', '-z', '0.05'],
        output='screen'
    )

    table_spawn_after_generate = RegisterEventHandler(
        OnProcessExit(
            target_action=generate_table_urdf,
            on_exit=[spawn_table_cmd]
        )
    )

    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 1. 解析 Xacro
    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    robot_description_content = remove_comments(doc.toxml())

    # ====================================================================
    # ⭐ 终极修复方案：执行拦截与清洗
    # ====================================================================
    robot_description_content = re.sub(
        r'package://([a-zA-Z0-9_]+)/([a-zA-Z0-9_./-]+)',
        resolve_package_path,
        robot_description_content
    )

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, robot_description, {"publish_frequency": 15.0}],
        output='screen'
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-topic', 'robot_description'],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur7e_manipulater_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    close_evt1 = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[load_joint_state_broadcaster],
        )
    )
    close_evt2 = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_trajectory_controller],
        )
    )

    ld = LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(generate_table_urdf)
    ld.add_action(table_spawn_after_generate)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(close_evt1)
    ld.add_action(close_evt2)

    return ld