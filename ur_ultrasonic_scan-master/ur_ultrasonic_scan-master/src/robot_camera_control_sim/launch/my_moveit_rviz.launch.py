from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils.launch_utils import add_debuggable_node, DeclareBooleanLaunchArg
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():

    # ===============================
    # MoveIt 配置（核心部分）
    # ===============================
    moveit_config = (
        MoveItConfigsBuilder("ur7e", package_name="ur7e_moveit_config")

        # 机器人 URDF
        .robot_description(file_path="config/ur7e.urdf.xacro")

        # SRDF（规划组定义）
        .robot_description_semantic(file_path="config/ur7e.srdf")

        # 控制器配置
        .trajectory_execution(file_path="config/moveit_controllers.yaml")

        # ⭐⭐⭐ 修复点：必须指定 default_planning_pipeline
        .planning_pipelines(
            pipelines=["ompl"],                # 使用 OMPL
            default_planning_pipeline="ompl"   # 默认规划器
        )

        .to_moveit_configs()
    )

    ld = LaunchDescription()

    # 启动 move_group（规划核心）
    my_generate_move_group_launch(ld, moveit_config)

    # 启动 RViz
    my_generate_moveit_rviz_launch(ld, moveit_config)

    return ld


# ===============================
# move_group 节点（规划核心）
# ===============================
def my_generate_move_group_launch(ld, moveit_config):

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))

    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    # ===============================
    # move_group 参数
    # ===============================
    move_group_configuration = {

        # URDF
        "robot_description": moveit_config.robot_description,

        # SRDF
        "robot_description_semantic": moveit_config.robot_description_semantic,

        "publish_robot_description_semantic": True,

        # 是否执行轨迹
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),

        # 插件能力
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),

        # 发布规划场景（RViz依赖）
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,

        "monitor_dynamics": False,
    }

    # ===============================
    # 加载 kinematics.yaml（关键）
    # ===============================
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("ur7e_moveit_config"),
        "config",
        "kinematics.yaml"
    )

    with open(kinematics_yaml_path, 'r') as f:
        kinematics_params = yaml.safe_load(f)

    # ===============================
    # MoveIt 参数
    # ===============================
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        kinematics_params,   # ⭐⭐⭐ IK配置在这里
        {"use_sim_time": True}
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=move_group_params,
        output="screen",
    )


# ===============================
# RViz 节点
# ===============================
def my_generate_moveit_rviz_launch(ld, moveit_config):

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))

    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    # RViz 需要这些参数
    rviz_parameters = [
        moveit_config.planning_pipelines,        # ⭐ 规划器
        moveit_config.robot_description_kinematics,  # ⭐ IK
        {"use_sim_time": True}
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        arguments=['-d', LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        output="log"
    )