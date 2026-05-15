#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

// PCL 相关
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

// Eigen 矩阵与数学库
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;
using PointT = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointT>;

struct PlannerConfig {
    float slice_step = 0.01f;        // X 轴切片步长 (每条扫描线的间距 1cm)
    float slice_thickness = 0.002f;  // 切片厚度
    float gap_threshold = 0.02f;     // 缺口打断阈值 (2cm)
    
    // ⭐ 新增：沿扫描线的轨迹点间距 (5mm)。
    // 保证机械臂收到的轨迹既不会太密导致卡顿，也不会太稀疏导致偏离曲面。
    float point_spacing = 0.005f;    
};

class TrajectoryPlannerNode : public rclcpp::Node
{
public:
    TrajectoryPlannerNode() : Node("trajectory_planner_sim")
    {
        RCLCPP_INFO(this->get_logger(), "🚀 轨迹规划节点已启动！正在读取平滑后的点云数据...");

        trajectory_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/planned_trajectory", 10);
        calculate_trajectory();
        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&TrajectoryPlannerNode::publish_loop, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    PlannerConfig config_;
    
    geometry_msgs::msg::PoseArray saved_pose_array_; 
    bool has_trajectory_ = false;

    static bool compareByY(const PointT& p1, const PointT& p2) {
        return p1.y > p2.y;
    }

    void calculate_trajectory()
    {
        PointCloudT::Ptr cloud(new PointCloudT);
        std::string file_name = "target_cylinder_smoothed.pcd";
        if (pcl::io::loadPCDFile<PointT>(file_name, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "读取失败！");
            return;
        }

        PointT min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);

        saved_pose_array_.header.frame_id = "camera_depth_optical_frame"; 
        int slice_index = 0; 

        for (float current_x = min_pt.x; current_x <= max_pt.x; current_x += config_.slice_step) {
            
            PointCloudT::Ptr slice_cloud(new PointCloudT);
            pcl::PassThrough<PointT> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(current_x - config_.slice_thickness, current_x + config_.slice_thickness); 
            pass.filter(*slice_cloud);

            if (slice_cloud->empty()) continue;

            std::vector<PointT> sorted_points(slice_cloud->points.begin(), slice_cloud->points.end());
            std::sort(sorted_points.begin(), sorted_points.end(), compareByY);

            std::vector<std::vector<PointT>> fragmented_lines;
            std::vector<PointT> current_line;
            
            current_line.push_back(sorted_points[0]);
            PointT last_added_point = sorted_points[0]; // ⭐ 记录上一个加入轨迹的点

            for (size_t i = 1; i < sorted_points.size(); ++i) {
                PointT p2 = sorted_points[i];
                
                // 1. 缺口检测 (计算与数组中上一个点的距离)
                PointT p1 = sorted_points[i - 1];
                float dist_to_prev = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));

                if (dist_to_prev > config_.gap_threshold) {
                    fragmented_lines.push_back(current_line);
                    current_line.clear(); 
                    current_line.push_back(p2);
                    last_added_point = p2;
                    continue;
                }

                // 2. ⭐ 沿线均匀降采样 (计算与上一个【被采纳】的轨迹点的距离)
                float dist_to_last_added = std::sqrt(std::pow(last_added_point.x - p2.x, 2) + 
                                                     std::pow(last_added_point.y - p2.y, 2) + 
                                                     std::pow(last_added_point.z - p2.z, 2));
                
                // 只有当距离超过 5mm 时，才算作一个有效的机械臂航点
                if (dist_to_last_added >= config_.point_spacing) {
                    current_line.push_back(p2);
                    last_added_point = p2;
                }
            }
            fragmented_lines.push_back(current_line); 

            for (auto& frag_line : fragmented_lines) {
                if (slice_index % 2 != 0) {
                    std::reverse(frag_line.begin(), frag_line.end());
                }

                for (const auto& pt : frag_line) {
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = pt.x;
                    pose.position.y = pt.y;
                    pose.position.z = pt.z;

                    Eigen::Vector3f surface_normal(pt.normal_x, pt.normal_y, pt.normal_z);
                    Eigen::Vector3f tool_z_axis = -surface_normal.normalized();
                    Eigen::Quaternionf q;
                    q.setFromTwoVectors(Eigen::Vector3f::UnitZ(), tool_z_axis);

                    pose.orientation.x = q.x();
                    pose.orientation.y = q.y();
                    pose.orientation.z = q.z();
                    pose.orientation.w = q.w();

                    saved_pose_array_.poses.push_back(pose);
                }
            }
            slice_index++; 
        }

        has_trajectory_ = true;
        RCLCPP_INFO(this->get_logger(), "🎉 轨迹规划完成！共生成 %zu 个稀疏化位姿，非常适合机械臂执行。", saved_pose_array_.poses.size());
    }

    void publish_loop()
    {
        if (has_trajectory_) {
            saved_pose_array_.header.stamp = this->now();
            trajectory_pub_->publish(saved_pose_array_);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}