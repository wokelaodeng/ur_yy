/*
 * Copyright 2026 [Your Name/Lab]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * ...
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

// PCL 相关头文件
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

// [新增] RANSAC 平面分割与点云提取相关头文件
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// OpenCV 相关头文件
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <mutex>
#include <X11/Xlib.h>

class CameraUseNode : public rclcpp::Node
{
public:
    // ==========================================
    // 🎛️ 功能控制面板
    // ==========================================
    bool enable_3d_pointcloud_ = true; 
    bool enable_2d_color_img_  = true; 
    bool enable_2d_depth_img_  = true; 

    // --- 1. 直通滤波 (PassThrough) ---
    bool enable_passthrough_filter_ = true; 
    float pass_z_min_ = 0.1f;               
    float pass_z_max_ = 0.32f; // 【注意】为了更好地测试桌面，我建议把最大距离缩短到 0.8 米左右              

    // --- 2. 体素降采样 (VoxelGrid) ---
    bool enable_voxel_filter_  = true;      
    float voxel_size_ = 0.002f; // 5毫米的精度，足够保留手机的轮廓             

    // --- 3. [新增] RANSAC 平面剔除 (SACSegmentation) ---
    bool enable_ransac_filter_ = true;
    // 距离阈值：厚度为 1.5 厘米 (0.015m) 的“桌面切片”。如果手机底部被切了，就把这个值调小(如 0.01)
    float ransac_distance_threshold_ = 0.015f; 

    CameraUseNode() : Node("my_camera_use")
    {
        if (enable_3d_pointcloud_) {
            pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
                std::bind(&CameraUseNode::pointcloud_callback, this, std::placeholders::_1));
        }
        if (enable_2d_color_img_) {
            color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/camera/color/image_raw", 10,
                std::bind(&CameraUseNode::color_img_callback, this, std::placeholders::_1));
        }
        if (enable_2d_depth_img_) {
            depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/camera/aligned_depth_to_color/image_raw", 10,
                std::bind(&CameraUseNode::depth_img_callback, this, std::placeholders::_1));
        }
        RCLCPP_INFO(this->get_logger(), "视觉节点启动！当前流水线：直通 -> 体素 -> RANSAC桌面剔除");
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getLatestCloud() {
        std::lock_guard<std::mutex> lock(pc_mutex_);
        return latest_cloud_;
    }
    cv::Mat getLatestColorImg() {
        std::lock_guard<std::mutex> lock(color_mutex_);
        return latest_color_img_.clone();
    }
    cv::Mat getLatestDepthImg() {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        return latest_depth_img_.clone();
    }

private:
    std::mutex pc_mutex_, color_mutex_, depth_mutex_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_cloud_;
    cv::Mat latest_color_img_;
    cv::Mat latest_depth_img_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    // ==========================================
    // 📥 回调函数区：点云处理三重流水线
    // ==========================================
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *raw_cloud);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        // --- 工序 1：直通滤波 ---
        if (enable_passthrough_filter_) {
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud(raw_cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(pass_z_min_, pass_z_max_);
            pass.filter(*pass_cloud);
        } else {
            pass_cloud = raw_cloud;
        }

        // --- 工序 2：体素降采样 ---
        if (enable_voxel_filter_) {
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
            voxel.setInputCloud(pass_cloud);
            voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel.filter(*voxel_cloud);
        } else {
            voxel_cloud = pass_cloud;
        }

        // ==========================================
        // --- 工序 3：[核心新增] RANSAC 剔除最大平面 (桌面) ---
        // ==========================================
        if (enable_ransac_filter_ && voxel_cloud->points.size() > 50) {
            // coefficients 存储计算出的平面方程参数 (Ax + By + Cz + D = 0)
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            // inliers 存储所有属于桌面的点的索引号
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            
            // 创建分割器对象
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);       // 开启参数优化，更精准
            seg.setModelType(pcl::SACMODEL_PLANE);   // 告诉算法：我要找的是“平面”
            seg.setMethodType(pcl::SAC_RANSAC);      // 使用 RANSAC 方法
            seg.setMaxIterations(1000);              // 最多猜 1000 次
            seg.setDistanceThreshold(ransac_distance_threshold_); // 距离容忍度
            
            seg.setInputCloud(voxel_cloud);          // 注意：输入的是降采样后的点云，速度快百倍！
            seg.segment(*inliers, *coefficients);    // 执行算法，结果写入 inliers
            
            if (inliers->indices.size() == 0) {
                // 如果找不到平面（比如镜头对着空中），就跳过
                final_cloud = voxel_cloud;
            } else {
                // 创建提取器：根据 inliers 索引，对点云进行“动刀”
                pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                extract.setInputCloud(voxel_cloud);
                extract.setIndices(inliers);
                
                // 【绝杀参数】：setNegative 为 true，意思是“不要”这群点 (剔除桌面)
                // 如果改为 false，你提取出来的就是纯纯的一张桌子。
                extract.setNegative(true); 
                extract.filter(*final_cloud);
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "流水线 -> 原始: %zu | 裁剪: %zu | 降采样: %zu | 剔除桌面后剩余(工件): %zu", 
                raw_cloud->points.size(), pass_cloud->points.size(), 
                voxel_cloud->points.size(), final_cloud->points.size());
        } else {
            final_cloud = voxel_cloud;
        }
        
        // --- 安全更新渲染数据 ---
        std::lock_guard<std::mutex> lock(pc_mutex_);
        latest_cloud_ = final_cloud;
    }

    // ... (下面的 color_img_callback, depth_img_callback 和 main 函数与上个版本完全一致，为了节省版面不再赘述)
    // 请保留你之前代码的下面这些部分！
    void color_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat cv_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
            std::lock_guard<std::mutex> lock(color_mutex_);
            latest_color_img_ = cv_img;
        } catch (cv_bridge::Exception& e) { }
    }
    void depth_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat depth_16u = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            cv::Mat depth_8u, depth_color;
            depth_16u.convertTo(depth_8u, CV_8UC1, 255.0 / 2000.0); 
            cv::applyColorMap(depth_8u, depth_color, cv::COLORMAP_JET);
            std::lock_guard<std::mutex> lock(depth_mutex_);
            latest_depth_img_ = depth_color;
        } catch (cv_bridge::Exception& e) { }
    }
};

int main(int argc, char * argv[]) {
    XInitThreads(); 
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraUseNode>();
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if (node->enable_3d_pointcloud_) {
        viewer = std::make_shared<pcl::visualization::PCLVisualizer>("D435i Real-time 3D Viewer");
        viewer->setBackgroundColor(0.05, 0.05, 0.05); 
        viewer->addCoordinateSystem(0.2);             
        viewer->initCameraParameters();               
    }
    if (node->enable_2d_color_img_) cv::namedWindow("2D Color Stream", cv::WINDOW_AUTOSIZE);
    if (node->enable_2d_depth_img_) cv::namedWindow("2D Depth Stream (Colormap)", cv::WINDOW_AUTOSIZE);
    rclcpp::WallRate loop_rate(30);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->enable_3d_pointcloud_ && viewer && !viewer->wasStopped()) {
            auto cloud = node->getLatestCloud(); 
            if (cloud && !cloud->empty()) {
                if (!viewer->updatePointCloud(cloud, "d435i_cloud")) {
                    viewer->addPointCloud(cloud, "d435i_cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "d435i_cloud"); 
                }
            }
            viewer->spinOnce(10); 
        }
        if (node->enable_2d_color_img_) {
            cv::Mat color_img = node->getLatestColorImg();
            if (!color_img.empty()) cv::imshow("2D Color Stream", color_img); 
        }
        if (node->enable_2d_depth_img_) {
            cv::Mat depth_img = node->getLatestDepthImg();
            if (!depth_img.empty()) cv::imshow("2D Depth Stream (Colormap)", depth_img);
        }
        if (node->enable_2d_color_img_ || node->enable_2d_depth_img_) cv::waitKey(10); 
        if (viewer && viewer->wasStopped()) break; 
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}