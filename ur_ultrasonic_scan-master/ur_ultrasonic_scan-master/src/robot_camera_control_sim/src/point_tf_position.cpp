#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp> // 用于将 TF 转换为 Eigen 矩阵

// PCL 相关
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <chrono>

using namespace std::chrono_literals;
using PointT = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointT>;

class PointTfPositionNode : public rclcpp::Node
{
public:
    PointTfPositionNode() : Node("point_tf_position")
    {
        RCLCPP_INFO(this->get_logger(), "🔄 点云 TF 坐标变换节点已启动，等待 TF 树就绪...");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 延迟 2 秒执行，确保能收到 Gazebo/MoveIt 广播的静态 TF
        timer_ = this->create_wall_timer(
            2s, std::bind(&PointTfPositionNode::transform_pointcloud, this));
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    void transform_pointcloud()
    {
        timer_->cancel(); // 仅执行一次

        std::string input_file = "target_cylinder_smoothed.pcd";
        std::string output_file = "target_cylinder_base_link.pcd"; // ⭐ 输出新文件
        
        std::string target_frame = "base_link";                  // 机械臂基坐标系
        std::string source_frame = "camera_depth_optical_frame"; // 相机坐标系

        PointCloudT::Ptr cloud(new PointCloudT);
        if (pcl::io::loadPCDFile<PointT>(input_file, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "❌ 读取文件 %s 失败！", input_file.c_str());
            rclcpp::shutdown();
            return;
        }

        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            // 查询从 source_frame 到 target_frame 的空间变换关系
            transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "❌ 无法获取 TF 变换: %s", ex.what());
            rclcpp::shutdown();
            return;
        }

        // 1. 将 ROS TF 转换为 Eigen 4x4 变换矩阵
        Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped);
        Eigen::Matrix4f transform_matrix = transform_eigen.matrix().cast<float>();

        // 2. ⭐ 核心：对点云的三维坐标和法向量同时进行刚体变换！
        PointCloudT::Ptr cloud_tf(new PointCloudT);
        pcl::transformPointCloudWithNormals(*cloud, *cloud_tf, transform_matrix);

        // 3. 保存到本地
        pcl::io::savePCDFileASCII(output_file, *cloud_tf);
        RCLCPP_INFO(this->get_logger(), "✅ 坐标变换完美成功！原点云已从相机视角转移至机械臂底座视角。");
        RCLCPP_INFO(this->get_logger(), "新点云已保存为: %s", output_file.c_str());

        // 任务完成，自动关闭节点
        rclcpp::shutdown();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointTfPositionNode>());
    return 0;
}