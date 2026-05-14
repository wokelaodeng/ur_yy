#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <csignal>
#include <vector>
#include <mutex>
#include <thread>
#include <cmath>

std::atomic<bool> g_shutdown_requested(false);
void sigint_handler(int signum) { (void)signum; g_shutdown_requested.store(true); }

double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

class RobotControlSim : public rclcpp::Node
{
public:
    RobotControlSim(const rclcpp::NodeOptions& options) 
    : Node("robot_control_sim", options), has_trajectory_(false), is_task_completed_(false)
    {
        RCLCPP_INFO(this->get_logger(), "🤖 机械臂控制节点已启动！[包含退刀防串扰 & 瞬态持久化]");

        this->declare_parameter("approach_vel", 0.4); 
        this->declare_parameter("approach_acc", 0.3); 
        this->declare_parameter("scan_vel", 0.01);    
        this->declare_parameter("scan_acc", 0.01);    
        this->declare_parameter("retract_vel", 0.3);  
        this->declare_parameter("retract_acc", 0.3);

        scan_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("scan_point", 100);
        
        // ⭐ 修复2：为 Marker 启用 Transient Local QoS，确保 RViz 绝对不丢图！
        rclcpp::QoS marker_qos(10);
        marker_qos.transient_local();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("scan_results_markers", marker_qos);

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/planned_poses", 10, std::bind(&RobotControlSim::trajectory_callback, this, std::placeholders::_1));
    }

    void run_tasks()
    {
        while (rclcpp::ok() && !g_shutdown_requested.load()) {
            if (has_trajectory_ && !is_task_completed_) {
                execute_mission();
                is_task_completed_ = true; 
                has_trajectory_ = false; 
                RCLCPP_INFO(this->get_logger(), "💤 任务已全部结束，进入待机监听模式...");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

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
        RCLCPP_INFO(this->get_logger(), "✅ 接收到包含 %zu 个位姿的新轨迹！", target_poses_.size());
    }

    void execute_mission()
    {
        static const std::string PLANNING_GROUP = "ur7e_manipulator";
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
        if (!trajectory_frame_id_.empty()) move_group->setPoseReferenceFrame(trajectory_frame_id_);

        auto safe_move = [&](const std::string& name, auto target) {
            if (g_shutdown_requested.load()) return false;
            RCLCPP_INFO(this->get_logger(), "-> %s", name.c_str());
            
            if constexpr (std::is_same_v<decltype(target), std::string>) {
                move_group->setNamedTarget(target);
            } else {
                move_group->setPoseTarget(target);
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                move_group->execute(plan);
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 规划失败！步骤: %s", name.c_str());
                return false;
            }
        };

        set_speed_params(move_group, "approach");
        safe_move("🏠 前往初始 Home 姿态", std::string("home_pose_1"));

        std::vector<geometry_msgs::msg::Pose> waypoints;
        const double JUMP_THRESHOLD = 0.15; 

        for (size_t i = 0; i < target_poses_.size(); ++i) {
            if (g_shutdown_requested.load()) break; 
            waypoints.push_back(target_poses_[i]);

            bool is_last_point = (i == target_poses_.size() - 1);
            bool is_jump = false;
            if (!is_last_point) {
                double dist = calculateDistance(target_poses_[i].position, target_poses_[i+1].position);
                if (dist > JUMP_THRESHOLD) is_jump = true;
            }

            if (is_jump || is_last_point) {
                set_speed_params(move_group, "approach");
                geometry_msgs::msg::Pose p1 = waypoints[0]; p1.position.z += 0.15;
                safe_move("1. 飞向高空过渡点", p1);

                geometry_msgs::msg::Pose p2 = waypoints[0]; p2.position.z += 0.05;
                safe_move("2. 下降至进场点", p2);
                safe_move("3. 接触扫描起始点", waypoints[0]);

                set_speed_params(move_group, "scan");
                RCLCPP_INFO(this->get_logger(), "-> 4. 开始笛卡尔轨迹扫描 (异步，启动物理轮询)...");
                
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = move_group->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, false);

                if (fraction > 0.9) { 
                    move_group->asyncExecute(trajectory);
                    real_time_health_polling(move_group, waypoints.front(), waypoints.back());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ 扫描覆盖率低 (%.2f%%)，跳过。", fraction * 100.0);
                }

                // ⭐ 修复1：关键休眠！等待 asyncExecute 的机械臂彻底停止，再执行退刀！
                RCLCPP_INFO(this->get_logger(), "⏳ 等待机械臂伺服电机完全停稳...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));

                set_speed_params(move_group, "retract");
                RCLCPP_INFO(this->get_logger(), "-> 5. 扫描结束，开始执行标准工业退刀与安全撤离...");
                
                try {
                    auto current_stamped = move_group->getCurrentPose();
                    if (!current_stamped.header.frame_id.empty()) {
                        geometry_msgs::msg::Pose retract_pose = current_stamped.pose;
                        retract_pose.position.z += 0.10; 
                        
                        std::vector<geometry_msgs::msg::Pose> retract_waypoints;
                        retract_waypoints.push_back(retract_pose);
                        
                        moveit_msgs::msg::RobotTrajectory retract_traj;
                        double retract_frac = move_group->computeCartesianPath(retract_waypoints, 0.01, 0.0, retract_traj, false);
                        
                        if (retract_frac > 0.9) {
                            RCLCPP_INFO(this->get_logger(), "-> 5.1 执行垂直退刀 (Z+10cm)...");
                            move_group->execute(retract_traj);
                        }
                    }
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "⚠️ 无法获取位姿执行退刀。");
                }

                safe_move("5.2 撤离至安全 Home 姿态", std::string("home_pose_1"));

                waypoints.clear(); 
            }
        }
    }

    void real_time_health_polling(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group, 
                                  const geometry_msgs::msg::Pose& start_pose, 
                                  const geometry_msgs::msg::Pose& final_pose)
    {
        geometry_msgs::msg::Pose last_pub_pose = start_pose;
        rclcpp::Rate loop_rate(50); 
        
        int point_count = 0;
        int defect_count = 0;
        visualization_msgs::msg::MarkerArray markers;

        while (rclcpp::ok() && !g_shutdown_requested.load()) {
            geometry_msgs::msg::Pose curr_pose;
            try {
                auto stamped = move_group->getCurrentPose();
                if (stamped.header.frame_id.empty()) { loop_rate.sleep(); continue; }
                curr_pose = stamped.pose;
            } catch (...) {
                loop_rate.sleep(); continue;
            }

            if (curr_pose.position.x == 0 && curr_pose.position.y == 0 && curr_pose.position.z == 0) {
                loop_rate.sleep(); continue;
            }

            if (calculateDistance(curr_pose.position, last_pub_pose.position) >= 0.003) {
                last_pub_pose = curr_pose;
                point_count++;

                bool is_defect = (curr_pose.position.x > 0.35 && curr_pose.position.x < 0.40 && 
                                  curr_pose.position.y > -0.03 && curr_pose.position.y < 0.03);

                if (is_defect) {
                    printf("\033[1;31m🚨 [第%d点 损伤] X: %.3f, Y: %.3f, Z: %.3f\n\033[0m", point_count, curr_pose.position.x, curr_pose.position.y, curr_pose.position.z);
                    defect_count++;
                } else {
                    printf("\033[1;32m✅ [第%d点 健康] X: %.3f, Y: %.3f, Z: %.3f\n\033[0m", point_count, curr_pose.position.x, curr_pose.position.y, curr_pose.position.z);
                }

                geometry_msgs::msg::PointStamped ps;
                ps.header.stamp = this->now();
                ps.header.frame_id = is_defect ? "1" : "0"; 
                ps.point = curr_pose.position;
                scan_pub_->publish(ps);

                visualization_msgs::msg::Marker m;
                m.header.frame_id = trajectory_frame_id_.empty() ? "base_link" : trajectory_frame_id_;
                m.header.stamp = this->now();
                m.ns = "ultrasonic_scan";
                m.id = point_count;
                m.type = visualization_msgs::msg::Marker::SPHERE;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position = curr_pose.position;
                m.pose.orientation.w = 1.0;
                // ⭐ 适当调大一点点，让它在 RViz 里更显眼 (8mm)
                m.scale.x = 0.008; m.scale.y = 0.008; m.scale.z = 0.008;
                if (is_defect) {
                    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0; 
                } else {
                    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.4; 
                }
                markers.markers.push_back(m);
            }

            if (calculateDistance(curr_pose.position, final_pose.position) < 0.005) {
                break;
            }
            loop_rate.sleep();
        }

        marker_pub_->publish(markers); 
        
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "📊 本批次超声扫描作业报告汇总");
        RCLCPP_INFO(this->get_logger(), "------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "有效检测点: %d | 损伤点: %d", point_count, defect_count);
        RCLCPP_INFO(this->get_logger(), "==========================================");
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
    
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("use_sim_time", true);
    auto node = std::make_shared<RobotControlSim>(node_options);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    node->run_tasks();

    g_shutdown_requested.store(true);
    rclcpp::shutdown();
    spinner.join();
    return 0;
}