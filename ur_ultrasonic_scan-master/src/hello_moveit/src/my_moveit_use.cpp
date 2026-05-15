#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <chrono> 
#include <string>

using namespace std::chrono_literals; 

// ===============================
// 模块功能：四元数 → 欧拉角
// ===============================
void quaternionToRPY(const geometry_msgs::msg::Quaternion &q,
                     double &roll, double &pitch, double &yaw)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
}

// ===============================
// 模块功能：欧拉角 → 四元数
// ===============================
geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

// ===============================
// 模块功能：计算两点之间的空间直线距离
// ===============================
double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

// ===============================
// 模块功能：笛卡尔路径速度缩放器
// ===============================
void scaleCartesianTrajectorySpeed(moveit_msgs::msg::RobotTrajectory &trajectory, double scale) {
    if (scale <= 0.0 || scale >= 1.0) return; 
    
    for (auto &point : trajectory.joint_trajectory.points) {
        double time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
        time_from_start /= scale; 
        
        point.time_from_start.sec = static_cast<int32_t>(time_from_start);
        point.time_from_start.nanosec = static_cast<uint32_t>((time_from_start - point.time_from_start.sec) * 1e9);

        for (auto &v : point.velocities) v *= scale;
        for (auto &a : point.accelerations) a *= (scale * scale);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    srand(time(NULL)); 

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true); 

    auto node = rclcpp::Node::make_shared("stable_scan_node", node_options);
    auto point_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("scan_point", 100);
    
    RCLCPP_INFO(node->get_logger(), "稳定扫描节点启动");

    // ===============================
    // 后台 Spin 线程：只负责底层数据通信，绝不运行耗时回调
    // ===============================
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "ur7e_manipulater");

    move_group.startStateMonitor();
    std::this_thread::sleep_for(1s); 

    // 设置【空驶阶段】的速度：0.5
    move_group.setMaxVelocityScalingFactor(0.2); 
    move_group.setMaxAccelerationScalingFactor(0.2);

    auto print_current_pose = [&move_group, &node](const std::string& location_name) {
        try {
            auto curr_pose = move_group.getCurrentPose().pose;
            double r, p, y;
            quaternionToRPY(curr_pose.orientation, r, p, y);
            RCLCPP_INFO(node->get_logger(),
                        ">>> 📍 节点 [%s] 真实物理坐标 -> X: %.4f, Y: %.4f, Z: %.4f | 姿态(RPY): R: %.4f, P: %.4f, Y: %.4f",
                        location_name.c_str(),
                        curr_pose.position.x, curr_pose.position.y, curr_pose.position.z,
                        r, p, y);
        } catch (const std::exception& e) {
            RCLCPP_WARN(node->get_logger(), "无法获取 [%s] 的真实位姿信息！", location_name.c_str());
        }
    };

    // ===============================
    // 1️⃣ 移动到安全起始位 home_pose_1
    // ===============================
    RCLCPP_INFO(node->get_logger(), "准备移动到 home_pose_1，正在规划...");
    move_group.setNamedTarget("home_pose_1");
    
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if (move_group.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        if (move_group.execute(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            rclcpp::shutdown(); spinner.join(); return 0;
        }
        print_current_pose("home_pose_1");
    } else {
        rclcpp::shutdown(); spinner.join(); return 0;
    }

    // ===============================
    // 2️⃣ 自定义扫描起点与过渡点
    // ===============================
    double start_x = 0.20;  
    double start_y = 0.02;  
    double start_z = 0.15;  
    
    double start_roll = 0.0; 
    double start_pitch = 0.0;
    double start_yaw = 3.1415926535;

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = start_x;
    start_pose.position.y = start_y;
    start_pose.position.z = start_z;
    start_pose.orientation = rpyToQuaternion(start_roll, start_pitch, start_yaw);

    geometry_msgs::msg::Pose approach_pose = start_pose;
    approach_pose.position.z += 0.10; 

    // ===============================
    // 3️⃣ 先到过渡点，再垂直下降到起点
    // ===============================
    RCLCPP_INFO(node->get_logger(), "准备移动到高空过渡点 (Z:%.2f)...", approach_pose.position.z);
    move_group.setPoseTarget(approach_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    if (move_group.plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        if (move_group.execute(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            rclcpp::shutdown(); spinner.join(); return 0;
        }
        print_current_pose("高空过渡点");
    } else {
        rclcpp::shutdown(); spinner.join(); return 0;
    }

    RCLCPP_INFO(node->get_logger(), "准备垂直下降至扫描起点 (Z:%.2f)...", start_pose.position.z);
    move_group.setPoseTarget(start_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    if (move_group.plan(start_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        if (move_group.execute(start_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            rclcpp::shutdown(); spinner.join(); return 0;
        }
        print_current_pose("真实扫描起点");
    } else {
        rclcpp::shutdown(); spinner.join(); return 0;
    }
    
    RCLCPP_INFO(node->get_logger(), "已到达起点，准备生成扫描路径...");

    // ===============================
    // 4️⃣ 生成扫描路径
    // ===============================
    int scan_mode = 0;  
    double scan_length = 0.2;
    double line_spacing = 0.01;
    int num_lines = 5;
    double offset_x = 0.00;
    double offset_y = 0.00;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    for (int i = 0; i < num_lines; ++i)
    {
        for (double d = 0; d <= scan_length; d += 0.005)   
        {
            geometry_msgs::msg::Pose p = start_pose;       

            if (scan_mode == 0) {
                p.position.y += i * line_spacing;               
                p.position.x += (i % 2 == 0) ? d : (scan_length - d); 
            } else if (scan_mode == 1) {
                p.position.x += i * line_spacing;               
                p.position.y += (i % 2 == 0) ? d : (scan_length - d); 
            } else if (scan_mode == 2) {
                p.position.x += offset_x + i * line_spacing;
                p.position.y += offset_y + ((i % 2 == 0) ? d : (scan_length - d));
            }

            p.orientation = start_pose.orientation; 
            waypoints.push_back(p);
        }
    }

    // ===============================
    // 5️⃣ 计算路径并修改扫描速度进行异步执行
    // ===============================
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.002, 0.0, trajectory);
    RCLCPP_INFO(node->get_logger(), "路径覆盖率: %.2f%%", fraction * 100.0);

    if (fraction < 0.9) {
        RCLCPP_ERROR(node->get_logger(), "路径覆盖率太低！");
        rclcpp::shutdown(); spinner.join(); return 0;
    }

    // 强行将扫描速度降至原本的 0.01
    RCLCPP_INFO(node->get_logger(), "正在重写笛卡尔轨迹，将扫描速度降至 0.1...");
    scaleCartesianTrajectorySpeed(trajectory, 0.1);

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    RCLCPP_INFO(node->get_logger(), "开始极慢速平滑扫描，并启动实时采样...");
    move_group.asyncExecute(cartesian_plan); 

    // ===============================
    // 6️⃣ [核心重构] 主线程轮询采样 (安全解耦，防断联死锁)
    // ===============================
    geometry_msgs::msg::Pose last_pub_pose = start_pose;
    geometry_msgs::msg::Pose final_pose = waypoints.back();
    
    // 设置循环频率为 50Hz (20ms)。因为速度只有 0.01，50Hz 绰绰有余
    rclcpp::Rate loop_rate(50); 
    
    // 取消定时器，直接在主线程跑 while 循环！
    while (rclcpp::ok()) {
        
        geometry_msgs::msg::PoseStamped curr_pose_stamped;
        try {
            curr_pose_stamped = move_group.getCurrentPose();
            if (curr_pose_stamped.header.frame_id.empty()) {
                loop_rate.sleep();
                continue; 
            }
        } catch (const std::exception& e) {
            loop_rate.sleep();
            continue; 
        }

        auto curr_pose = curr_pose_stamped.pose;
        double dist_moved = calculateDistance(curr_pose.position, last_pub_pose.position);

        // 获取真实的传感器触发位置，不做任何捏造插值
        if (dist_moved >= 0.002) {
            last_pub_pose = curr_pose; 
            
            int health_status = (rand() % 100 < 15) ? 1 : 0;

            geometry_msgs::msg::PointStamped p_msg;
            p_msg.header.stamp = node->now();
            p_msg.header.frame_id = std::to_string(health_status); 
            p_msg.point = curr_pose.position; 
            
            point_pub->publish(p_msg);
        }

        // 到达终点则退出循环
        if (calculateDistance(curr_pose.position, final_pose.position) < 0.005) {
            RCLCPP_INFO(node->get_logger(), "已到达扫描终点，采样结束。");
            break; 
        }

        loop_rate.sleep(); // 休眠 20ms，让出 CPU
    }

    // 优雅关闭节点并等待 spinner 线程结束
    rclcpp::shutdown();
    spinner.join(); 
    return 0;
}