#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp> // ⭐ 新增：广播坐标数据
#include <visualization_msgs/msg/marker_array.hpp> // ⭐ 新增：RViz 缺陷可视化
#include <moveit/move_group_interface/move_group_interface.h>
#include <csignal>
#include <vector>
#include <mutex>

// =========================================================================
// ⭐ 终极控制节点：参数化速度 + 超声健康检测模拟 + 数据广播与可视化
// =========================================================================

std::atomic<bool> g_shutdown_requested(false);
void sigint_handler(int signum) { (void)signum; g_shutdown_requested.store(true); }

class RobotControlSim : public rclcpp::Node
{
public:
    RobotControlSim() : Node("robot_control_sim"), has_trajectory_(false), is_task_completed_(false)
    {
        RCLCPP_INFO(this->get_logger(), "🤖 机械臂执行控制节点已启动！数据分析与广播模块已就绪。");

        // --- 声明可调节参数 ---
        this->declare_parameter("approach_vel", 0.3); 
        this->declare_parameter("approach_acc", 0.2); 

        this->declare_parameter("scan_vel", 0.01);    // 1mm/s 模拟
        this->declare_parameter("scan_acc", 0.01);    

        this->declare_parameter("retract_vel", 0.2); 
        this->declare_parameter("retract_acc", 0.2);

        // ⭐ 新增：超声波数据广播器 (对接 PCL 窗口)
        scan_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("scan_point", 100);
        
        // ⭐ 新增：RViz2 缺陷地图可视化器
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("scan_results_markers", 10);

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/planned_poses", 10, std::bind(&RobotControlSim::trajectory_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&RobotControlSim::control_loop, this));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Pose> target_poses_;
    std::mutex data_mutex_;
    bool has_trajectory_;
    bool is_task_completed_;
    std::string trajectory_frame_id_;

    void trajectory_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (has_trajectory_ || is_task_completed_) return; 

        target_poses_ = msg->poses;
        trajectory_frame_id_ = msg->header.frame_id; 
        has_trajectory_ = true;
        RCLCPP_INFO(this->get_logger(), "✅ 接收到新轨迹，准备执行。");
    }

    void control_loop()
    {
        if (g_shutdown_requested.load()) { rclcpp::shutdown(); return; }
        if (!has_trajectory_ || is_task_completed_) return;

        static const std::string PLANNING_GROUP = "ur7e_manipulater";
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);

        if (!trajectory_frame_id_.empty()) move_group->setPoseReferenceFrame(trajectory_frame_id_);

        // 1. 初始回 Home
        set_speed_params(move_group, "approach");
        RCLCPP_INFO(this->get_logger(), "🏠 正在前往初始 Home 姿态...");
        move_group->setNamedTarget("home_pose_1"); 
        move_group->move();

        std::vector<geometry_msgs::msg::Pose> waypoints;
        const double JUMP_THRESHOLD = 0.05; 

        for (size_t i = 0; i < target_poses_.size(); ++i) {
            if (g_shutdown_requested.load()) break; 
            waypoints.push_back(target_poses_[i]);

            bool is_last_point = (i == target_poses_.size() - 1);
            bool is_jump = false;
            if (!is_last_point) {
                double dist = std::sqrt(std::pow(target_poses_[i].position.x - target_poses_[i+1].position.x, 2) +
                                        std::pow(target_poses_[i].position.y - target_poses_[i+1].position.y, 2) +
                                        std::pow(target_poses_[i].position.z - target_poses_[i+1].position.z, 2));
                if (dist > JUMP_THRESHOLD) is_jump = true;
            }

            if (is_jump || is_last_point) {
                execute_batch_optimized(move_group, waypoints);
                waypoints.clear(); 
            }
        }

        RCLCPP_INFO(this->get_logger(), "🎉 任务结束，机械臂已安全撤离。");
        is_task_completed_ = true; 
        has_trajectory_ = false; 
    }

    void execute_batch_optimized(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group, 
                                 const std::vector<geometry_msgs::msg::Pose>& waypoints)
    {
        if (waypoints.empty() || g_shutdown_requested.load()) return;

        auto safe_move = [&](const geometry_msgs::msg::Pose& target, const std::string& name) {
            move_group->setPoseTarget(target);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            RCLCPP_INFO(this->get_logger(), "-> %s", name.c_str());
            if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                move_group->execute(plan);
                return true;
            }
            return false;
        };

        // 阶段 A: 进场
        set_speed_params(move_group, "approach");
        geometry_msgs::msg::Pose p1 = waypoints[0]; p1.position.z += 0.15;
        safe_move(p1, "1. 飞向高空过渡点");

        geometry_msgs::msg::Pose p2 = waypoints[0]; p2.position.z += 0.05;
        safe_move(p2, "2. 下降至进场点");

        safe_move(waypoints[0], "3. 接触起始点");

        // 阶段 B: 扫描
        set_speed_params(move_group, "scan");
        RCLCPP_INFO(this->get_logger(), "-> 4. 开始笛卡尔轨迹扫描 (1mm/s 匀速模式)...");
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, false);

        if (fraction > 0.9) { 
            move_group->execute(trajectory);
            RCLCPP_INFO(this->get_logger(), "✔️ 扫描动作执行成功。开始分析超声数据...");
            
            // ==========================================
            // ⭐ 阶段 B.1: 模拟健康检测与数据广播
            // ==========================================
            simulate_and_broadcast_health(waypoints);
            
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 扫描规划覆盖率过低 (%.2f%%)", fraction * 100.0);
        }

        // 阶段 C: 撤离
        set_speed_params(move_group, "retract");
        RCLCPP_INFO(this->get_logger(), "-> 5. 扫描结束，执行安全撤离动作...");
        move_group->setNamedTarget("home_pose_1");
        moveit::planning_interface::MoveGroupInterface::Plan retract_plan;
        if (move_group->plan(retract_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group->execute(retract_plan);
            RCLCPP_INFO(this->get_logger(), "✔️ 机械臂已安全返回 Home 姿态。");
        }
    }

    // ⭐ 核心检测算法模拟函数
    void simulate_and_broadcast_health(const std::vector<geometry_msgs::msg::Pose>& waypoints)
    {
        visualization_msgs::msg::MarkerArray markers;
        int defect_count = 0;

        for (size_t i = 0; i < waypoints.size(); ++i) {
            const auto& pt = waypoints[i].position;

            // 1. 缺陷判定逻辑：假设在圆柱体的某个特定区域存在内部裂纹
            // 这里划定了一个 X: [0.35, 0.40], Y: [-0.03, 0.03] 的虚拟损伤区域
            bool is_defect = (pt.x > 0.35 && pt.x < 0.40 && pt.y > -0.03 && pt.y < 0.03);

            // 2. 广播 PointStamped 给旧的 PCL 窗口
            geometry_msgs::msg::PointStamped ps;
            ps.header.stamp = this->now();
            ps.header.frame_id = is_defect ? "defective" : "healthy"; // 将健康状态写入 frame_id
            ps.point = pt;
            scan_pub_->publish(ps);

            // 3. 构建 RViz 3D 可视化 Marker
            visualization_msgs::msg::Marker m;
            m.header.frame_id = trajectory_frame_id_.empty() ? "base_link" : trajectory_frame_id_;
            m.header.stamp = this->now();
            m.ns = "ultrasonic_scan_results";
            m.id = i;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position = pt;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.005; // 5mm 的小球
            m.scale.y = 0.005; 
            m.scale.z = 0.005;

            if (is_defect) {
                // 红色代表损伤
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0; 
                defect_count++;
                // 在终端高亮报警
                RCLCPP_WARN(this->get_logger(), "🚨 [检测到损伤] 坐标 -> X: %.3f, Y: %.3f, Z: %.3f", pt.x, pt.y, pt.z);
            } else {
                // 绿色代表健康
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.4; 
            }
            markers.markers.push_back(m);
        }

        // 一次性发布所有结果地图到 RViz
        marker_pub_->publish(markers);

        if (defect_count == 0) {
            RCLCPP_INFO(this->get_logger(), "🟢 本次扫描区域健康，未发现损伤。");
        } else {
            RCLCPP_ERROR(this->get_logger(), "🔴 警报！本次扫描共发现 %d 个损伤点云！", defect_count);
        }
    }

    void set_speed_params(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg, const std::string& phase)
    {
        double v = 0.1, a = 0.1;
        if (phase == "approach") {
            v = this->get_parameter("approach_vel").as_double();
            a = this->get_parameter("approach_acc").as_double();
        } else if (phase == "scan") {
            v = this->get_parameter("scan_vel").as_double();
            a = this->get_parameter("scan_acc").as_double();
        } else if (phase == "retract") {
            v = this->get_parameter("retract_vel").as_double();
            a = this->get_parameter("retract_acc").as_double();
        }
        mg->setMaxVelocityScalingFactor(v);
        mg->setMaxAccelerationScalingFactor(a);
    }
};

int main(int argc, char** argv)
{
    std::signal(SIGINT, sigint_handler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlSim>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !g_shutdown_requested.load()) {
        executor.spin_some(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();
    return 0;
}