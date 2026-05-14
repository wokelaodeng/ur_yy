#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <thread>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({{"use_sim_time", true}});
    auto node = std::make_shared<rclcpp::Node>("my_moveit_use", options);
    auto logger = node->get_logger();

    RCLCPP_INFO(logger, "Node started, use_sim_time = %s",
                node->get_parameter("use_sim_time").as_bool() ? "true" : "false");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur7e_manipulater");

    move_group_interface.startStateMonitor();
    RCLCPP_INFO(logger, "State monitor started.");

    std::this_thread::sleep_for(500ms);

    RCLCPP_INFO(logger, "Waiting for current robot state...");
    moveit::core::RobotStatePtr current_state = nullptr;
    const int max_attempts = 20;
    for (int i = 0; i < max_attempts; ++i) {
        current_state = move_group_interface.getCurrentState(1.0);
        if (current_state) {
            RCLCPP_INFO(logger, "Robot state obtained on attempt %d.", i+1);
            break;
        }
        RCLCPP_WARN(logger, "Attempt %d/%d: still waiting for robot state...", i+1, max_attempts);
    }

    if (!current_state) {
        RCLCPP_ERROR(logger, "Failed to get robot state after %d seconds.", max_attempts);
        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();
        return 1;
    }

    auto current_pose = move_group_interface.getCurrentPose().pose;
    RCLCPP_INFO(logger, "Current end effector position: x=%.3f, y=%.3f, z=%.3f",
                current_pose.position.x, current_pose.position.y, current_pose.position.z);

    // 移动到 home_pose_1
    RCLCPP_INFO(logger, "Moving to home_pose_1...");
    move_group_interface.setNamedTarget("home_pose_1");
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    bool home_plan_success = (move_group_interface.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (home_plan_success) {
        RCLCPP_INFO(logger, "Home pose plan successful, executing...");
        auto home_execute_result = move_group_interface.execute(home_plan);
        if (home_execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger, "Successfully moved to home_pose_1");
            std::this_thread::sleep_for(1s);
            auto home_pose = move_group_interface.getCurrentPose().pose;
            RCLCPP_INFO(logger, "Home position reached: x=%.3f, y=%.3f, z=%.3f",
                        home_pose.position.x, home_pose.position.y, home_pose.position.z);
        } else {
            RCLCPP_ERROR(logger, "Failed to execute home pose motion");
            executor.cancel();
            spin_thread.join();
            rclcpp::shutdown();
            return 1;
        }
    } else {
        RCLCPP_ERROR(logger, "Failed to plan to home_pose_1. Check SRDF.");
    }

    auto start_pose = move_group_interface.getCurrentPose().pose;
    RCLCPP_INFO(logger, "Starting Cartesian path from: x=%.3f, y=%.3f, z=%.3f",
                start_pose.position.x, start_pose.position.y, start_pose.position.z);

    // ========== 定义路径点 ==========
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // [修正] 设置有效姿态：绕X轴180度，使工具Z轴垂直向下
    // 四元数 (x=1.0, y=0.0, z=0.0, w=0.0) 表示绕X轴旋转180度
    auto start_pose = move_group_interface.getCurrentPose().pose;
    geometry_msgs::msg::Pose waypoint1 = start_pose;
    waypoint1.position.x += 0.1;

    auto const waypoint2 = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        msg.position.x = 0.2;
        msg.position.y = 0.01;
        msg.position.z = 0.2;
        return msg;
    }();

    auto const waypoint3 = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        msg.position.x = 0.2;
        msg.position.y = 0.08;
        msg.position.z = 0.2;
        return msg;
    }();

    auto const waypoint4 = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        msg.position.x = 0.1;
        msg.position.y = 0.08;
        msg.position.z = 0.2;
        return msg;
    }();

    waypoints.push_back(waypoint1);
    waypoints.push_back(waypoint2);
    waypoints.push_back(waypoint3);
    waypoints.push_back(waypoint4);

    // ========== 规划笛卡尔路径 ==========
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory, true);

    RCLCPP_INFO(logger, "Cartesian path coverage: %.2f%%", fraction * 100.0);

    if (fraction >= 0.9) {
        // 时间参数化
        trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
        robot_trajectory::RobotTrajectory rt(move_group_interface.getRobotModel(), "ur7e_manipulater");
        rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory);

        if (!time_param.computeTimeStamps(rt)) {
            RCLCPP_ERROR(logger, "Time parameterization failed!");
            executor.cancel();
            spin_thread.join();
            rclcpp::shutdown();
            return 1;
        }

        rt.getRobotTrajectoryMsg(trajectory);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        auto result = move_group_interface.execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger, "Cartesian path execution succeeded");
        } else {
            RCLCPP_ERROR(logger, "Execution failed with error code: %d", result.val);
        }
    } else {
        RCLCPP_WARN(logger, "Cartesian path coverage too low: %.2f%%", fraction * 100.0);
    }

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}