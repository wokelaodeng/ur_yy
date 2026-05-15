#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 1. 初始化节点配置
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    // ===============================
    // ⭐ 核心修复：强制开启仿真时间匹配
    // ===============================
    node_options.append_parameter_override("use_sim_time", true);

    auto node = rclcpp::Node::make_shared("stable_scan_node", node_options);

    RCLCPP_INFO(node->get_logger(), "稳定扫描节点启动 (已开启仿真时间)");

    // 2. 启动后台线程处理 ROS 回调（接收关节状态）
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { 
        executor.spin(); 
    });

    // 3. MoveIt 初始化
    moveit::planning_interface::MoveGroupInterface move_group(node, "ur7e_manipulater");

    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    // 4. 获取当前位姿（安全起点）
    auto current_pose = move_group.getCurrentPose().pose;

    // 5. 构造扫描起点
    geometry_msgs::msg::Pose start_pose = current_pose;
    start_pose.position.x += 0.05;   // 前进 5cm
    start_pose.position.z -= 0.3;   // 向下 5cm
    start_pose.orientation = current_pose.orientation; // 姿态锁死

    // 6. 移动到扫描起点
    move_group.setPoseTarget(start_pose);

    if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "无法到达扫描起点！");
        rclcpp::shutdown();
        spinner.join();
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "已到达扫描起点，开始生成路径");

    // 7. 生成扫描路径（S型栅格）
    std::vector<geometry_msgs::msg::Pose> waypoints;
    double scan_length = 0.2;   // 扫描长度 20cm
    double line_spacing = 0.01; // 行间距 1cm
    int num_lines = 5;          // 扫描5条线

    for (int i = 0; i < num_lines; ++i)
    {
        for (double d = 0; d <= scan_length; d += 0.005)
        {
            geometry_msgs::msg::Pose p = start_pose;
            p.position.x += i * line_spacing; 

            if (i % 2 == 0) {
                p.position.y += d;                 
            } else {
                p.position.y += (scan_length - d);  
            }

            p.orientation = start_pose.orientation; 
            waypoints.push_back(p);
        }
    }

    RCLCPP_INFO(node->get_logger(), "生成路径点数量: %ld", waypoints.size());

    // 8. 计算 Cartesian 路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.002;
    double jump_threshold = 0.0;

    double fraction = move_group.computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        trajectory
    );

    RCLCPP_INFO(node->get_logger(), "路径覆盖率: %.2f%%", fraction * 100.0);

    if (fraction < 0.9)
    {
        RCLCPP_ERROR(node->get_logger(), "路径覆盖率太低，停止执行！");
        rclcpp::shutdown();
        spinner.join();
        return 0;
    }

    // 9. 执行轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "轨迹执行失败！");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "扫描完成");
    }

    // 清理与退出
    rclcpp::shutdown();
    spinner.join(); 
    return 0;
}