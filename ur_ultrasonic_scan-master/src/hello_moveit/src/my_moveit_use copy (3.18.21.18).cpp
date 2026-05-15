#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

// ===============================
// 四元数 → 欧拉角
// ===============================
void quaternionToRPY(const geometry_msgs::msg::Quaternion &q,
                     double &roll, double &pitch, double &yaw)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
}

// ===============================
// 欧拉角 → 四元数
// ===============================
geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);

    auto node = rclcpp::Node::make_shared("stable_scan_node", node_options);
    
    auto point_pub = node->create_publisher<geometry_msgs::msg::Point>("scan_point", 10);
    
    RCLCPP_INFO(node->get_logger(), "稳定扫描节点启动");

    // 多线程 spin
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "ur7e_manipulater");

    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    // ===============================
    // 1️⃣ 先回 home_pose_1
    // ===============================
    RCLCPP_INFO(node->get_logger(), "移动到 home_pose_1");

    move_group.setNamedTarget("home_pose_1");

    if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "无法到达 home_pose_1！");
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "已到达 home_pose_1");

    // ===============================
    // 2️⃣ 获取当前位姿
    // ===============================
    auto current_pose = move_group.getCurrentPose().pose;

    // ===============================
    // 3️⃣ 四元数 → 欧拉角
    // ===============================
    double roll, pitch, yaw;
    quaternionToRPY(current_pose.orientation, roll, pitch, yaw);

    RCLCPP_INFO(node->get_logger(), "当前姿态 RPY: %.3f %.3f %.3f", roll, pitch, yaw);

    // ===============================
    // ⭐ 可修改姿态（示例）
    // ===============================
    // 例如让探头稍微向下倾斜
    pitch = 0.0;

    current_pose.orientation = rpyToQuaternion(roll, pitch, yaw);

    // ===============================
    // 4️⃣ 构造扫描起点
    // ===============================
    geometry_msgs::msg::Pose start_pose = current_pose;

    start_pose.position.x += 0.05;
    start_pose.position.z -= 0.3;

    move_group.setPoseTarget(start_pose);

    if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "无法到达扫描起点！");
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "已到达扫描起点");

    // ===============================
    // 5️⃣ 扫描参数
    // scan_mode = 0 → 沿X轴扫描
    // scan_mode = 1 → 沿Y轴扫描
    // scan_mode = 2 → 自定义起点偏移 + 指定方向
    // ===============================
    int scan_mode = 1;  // ⭐⭐⭐ 修改这里控制扫描方式

    double scan_length = 0.2;
    double line_spacing = 0.01;
    int num_lines = 5;

    // 自定义偏移（scan_mode=2用）
    double offset_x = 0.00;
    double offset_y = 0.00;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    // ===============================
    // 6️⃣ 生成扫描路径
    // ===============================
    for (int i = 0; i < num_lines; ++i)
    {
        for (double d = 0; d <= scan_length; d += 0.005)
        {
            geometry_msgs::msg::Pose p = start_pose;

            // ===== 扫描模式选择 =====
            if (scan_mode == 0)
            {
                // 沿 X 扫描
                p.position.y += i * line_spacing;
                p.position.x += (i % 2 == 0) ? d : (scan_length - d);
            }
            else if (scan_mode == 1)
            {
                // 沿 Y 扫描（默认）
                p.position.x += i * line_spacing;
                p.position.y += (i % 2 == 0) ? d : (scan_length - d);
            }
            else if (scan_mode == 2)
            {
                // 自定义起点 + X扫描
                p.position.x += offset_x + i * line_spacing;
                p.position.y += offset_y + ((i % 2 == 0) ? d : (scan_length - d));
            }

            // 姿态：使用你调整后的姿态
            p.orientation = start_pose.orientation;

            waypoints.push_back(p);
            
            geometry_msgs::msg::Point p_msg;
            p_msg.x = p.position.x;
            p_msg.y = p.position.y;
            p_msg.z = p.position.z;

            point_pub->publish(p_msg);
        }
    }

    RCLCPP_INFO(node->get_logger(), "路径点数量: %ld", waypoints.size());

    // ===============================
    // 7️⃣ 计算路径
    // ===============================
    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = move_group.computeCartesianPath(
        waypoints,
        0.002,
        0.0,
        trajectory
    );

    RCLCPP_INFO(node->get_logger(), "路径覆盖率: %.2f%%", fraction * 100.0);

    if (fraction < 0.9)
    {
        RCLCPP_ERROR(node->get_logger(), "路径覆盖率太低！");
        return 0;
    }

    // ===============================
    // 8️⃣ 执行
    // ===============================
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    if (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "扫描完成");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "执行失败");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}