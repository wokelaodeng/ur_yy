#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>       
#include <geometry_msgs/msg/pose_stamped.hpp> 

// PCL 与数学核心库 (Eigen)
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;
using PointT = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointT>;

// =========================================================================
// ⭐ 核心切片规划参数配置区 (工业轨迹成型面板)
// =========================================================================
struct PlannerConfig {
    // 1. 切片步进跨度 (单位：米) -> 决定扫描线的密集程度
    // 作用：相当于超声探头推着走的行距。
    // 调节效果：调小(如0.005)扫描线会非常密集，能查出微小裂纹，但耗时成倍增加。
    //          调大(如0.02)扫描线变稀疏，检测极快，但线与线之间可能产生漏检盲区。
    float slice_step = 0.01f;        
    
    // 2. 虚拟刀片厚度 (单位：米) -> 决定数据提取量
    // 作用：沿着 X 轴切片时，抓取厚度为 [-0.002, +0.002] 范围内的所有点构成一条线。
    // 调节效果：在点云稀疏的情况下，必须适当加厚刀片才能保证一条线里有足够多的点用于连线。
    float slice_thickness = 0.002f;
    
    // 3. 轨迹断层容忍度 (单位：米) (⭐ 方案3 核心参数：边缘全覆盖的关键)
    // 作用：程序在一条线上点与点之间画直线。如果两个点的距离大于该值，程序判定此处是物理断崖，会强行打断轨迹。
    // 调节效果：调小(如0.01)，在曲面边缘(点云拉伸变稀疏)的地方，极易触发误判，导致机械臂不敢走到边缘。
    //          调大(如0.038)，赋予规划器跨越稀疏区的胆量，强行用空间插补将点连到工件最边缘，实现 100% 满覆盖！
    float gap_threshold = 0.038f;     
    
    // 4. 轨迹采样点距 (单位：米) -> 决定最终输出的位姿数量
    // 作用：如果一段线上有 1000 个点，机械臂运算会卡死。我们设定每隔 5mm 抽取一个点作为 MoveIt 的 Target。
    // 调节效果：调小(如0.001)发出的姿态点极多，机械臂走得非常精确贴合，但容易报笛卡尔规划失败。调大(如0.01)则算得非常快。
    float point_spacing = 0.005f;    
};

class TrajectoryPlannerNode : public rclcpp::Node
{
public:
    TrajectoryPlannerNode() : Node("trajectory_planner_sim")
    {
        RCLCPP_INFO(this->get_logger(), "🚀 轨迹规划与可视化节点已启动！(全覆盖连线模式)");

        // 发布 PoseArray：在 RViz 中渲染成一排排指示法向量的【箭头】
        trajectory_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/planned_poses", 10);
        // 发布 Path：在 RViz 中渲染成首尾相连的【连续折线】
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        calculate_trajectory();

        // 1Hz 定时广播，确保刚打开的 RViz 或延迟启动的控制节点能稳定接收到数据
        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&TrajectoryPlannerNode::publish_loop, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::TimerBase::SharedPtr publish_timer_;
    PlannerConfig config_;
    
    geometry_msgs::msg::PoseArray saved_pose_array_; 
    nav_msgs::msg::Path saved_path_; 
    bool has_trajectory_ = false;

    // 自定义排序函数：确保一条线上的点按 Y 轴物理坐标严格顺序列队
    static bool compareByY(const PointT& p1, const PointT& p2) {
        return p1.y > p2.y;
    }

    void calculate_trajectory()
    {
        PointCloudT::Ptr cloud(new PointCloudT);
        // 【极其重要】读取的是经过 point_tf_position.cpp 转换后的，位于机械臂基座系下的点云！
        if (pcl::io::loadPCDFile<PointT>("target_cylinder_base_link.pcd", *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "读取失败！请先运行 point_tf_position 节点！");
            return;
        }

        // 获取点云在 X 轴上的绝对边界长度
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);

        // ⭐ 打上标签：告诉全系统这批姿态是相对于机器人世界坐标系(base_link)的
        saved_pose_array_.header.frame_id = "base_link"; 
        saved_path_.header.frame_id = "base_link";

        int slice_index = 0; 

        // ==========================================
        // ⭐ 核心算法：切片与蛇形序列化
        // ==========================================
        // 外层循环：虚拟刀片沿 X 轴步进，每次切出一道扫描线
        for (float current_x = min_pt.x; current_x <= max_pt.x; current_x += config_.slice_step) {
            
            PointCloudT::Ptr slice_cloud(new PointCloudT);
            pcl::PassThrough<PointT> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(current_x - config_.slice_thickness, current_x + config_.slice_thickness); 
            pass.filter(*slice_cloud);

            if (slice_cloud->empty()) continue;

            // 内层整理：将这一道切片里的点按 Y 轴空间顺序排队
            std::vector<PointT> sorted_points(slice_cloud->points.begin(), slice_cloud->points.end());
            std::sort(sorted_points.begin(), sorted_points.end(), compareByY);

            std::vector<std::vector<PointT>> fragmented_lines; // 用于存放打断的线段
            std::vector<PointT> current_line;
            current_line.push_back(sorted_points[0]);
            PointT last_added_point = sorted_points[0];

            // 遍历梳理：检查连续性并进行等距稀疏采样
            for (size_t i = 1; i < sorted_points.size(); ++i) {
                PointT p2 = sorted_points[i];
                PointT p1 = sorted_points[i - 1];
                
                // 1. 连续性检查：如果两点间隙大于 gap_threshold，强制割断当前线，开启新线段
                float dist_to_prev = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
                if (dist_to_prev > config_.gap_threshold) {
                    fragmented_lines.push_back(current_line);
                    current_line.clear(); 
                    current_line.push_back(p2);
                    last_added_point = p2;
                    continue;
                }

                // 2. 采样提取：只有当前点与上一个采纳点的距离大于 point_spacing，才录入为有效位姿
                float dist_to_last_added = std::sqrt(std::pow(last_added_point.x - p2.x, 2) + std::pow(last_added_point.y - p2.y, 2) + std::pow(last_added_point.z - p2.z, 2));
                if (dist_to_last_added >= config_.point_spacing) {
                    current_line.push_back(p2);
                    last_added_point = p2;
                }
            }
            fragmented_lines.push_back(current_line); 

            // ==========================================
            // ⭐ 姿态合成与逆序反转 (生成牛耕/蛇形图案)
            // ==========================================
            for (auto& frag_line : fragmented_lines) {
                
                // 工业奇技淫巧：如果是奇数行，将整条线点组顺序颠倒！
                // 这样上一行从左扫到右，下一行就从右扫到左，形成“S”型不断笔轨迹。
                if (slice_index % 2 != 0) {
                    std::reverse(frag_line.begin(), frag_line.end());
                }

                for (const auto& pt : frag_line) {
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = pt.x;
                    pose.position.y = pt.y;
                    pose.position.z = pt.z;

                    // 数学几何变换：将基于平面的“法线矢量”转换为机器人能理解的“四元数姿态”
                    // 逻辑：我们希望机械臂末端(Tool Z轴)反向对准表面的法向量(向里压)
                    Eigen::Vector3f surface_normal(pt.normal_x, pt.normal_y, pt.normal_z);
                    Eigen::Vector3f tool_z_axis = -surface_normal.normalized(); 
                    
                    Eigen::Quaternionf q;
                    // 计算从世界默认Z轴旋转到目标 tool_z_axis 的旋转量
                    q.setFromTwoVectors(Eigen::Vector3f::UnitZ(), tool_z_axis);

                    pose.orientation.x = q.x();
                    pose.orientation.y = q.y();
                    pose.orientation.z = q.z();
                    pose.orientation.w = q.w();

                    // 装填进发布器载体
                    saved_pose_array_.poses.push_back(pose);

                    geometry_msgs::msg::PoseStamped ps;
                    ps.header = saved_path_.header;
                    ps.pose = pose;
                    saved_path_.poses.push_back(ps);
                }
            }
            slice_index++; 
        }

        has_trajectory_ = true;
        RCLCPP_INFO(this->get_logger(), "🎉 轨迹与连线规划完成！");
    }

    void publish_loop()
    {
        if (has_trajectory_) {
            // 实时刷新时间戳，防止 RViz 报 TF Timeout
            auto now = this->now();
            saved_pose_array_.header.stamp = now;
            saved_path_.header.stamp = now;

            trajectory_pub_->publish(saved_pose_array_);
            path_pub_->publish(saved_path_); 
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