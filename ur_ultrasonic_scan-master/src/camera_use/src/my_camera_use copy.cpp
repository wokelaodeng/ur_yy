/*
 * Copyright 2026 [Your Name/Lab]
 *
 * 本代码实现的功能：
 * 1. 订阅 D435i 点云，加入【智能降频机制】，每隔一定时间处理一帧。
 * 2. 针对 30cm 视距进行算法参数专门优化。
 * 3. 彻底解锁 MLS (移动最小二乘) 抛光，加入 NaN 强制清洗，防闪退。
 * 4. 运行 6 大工序：直通 -> 体素 -> RANSAC -> 聚类 -> MLS平滑(调试模式) -> PCA法线。
 * 5. 叠加真 3D 切片法预览，实时渲染红色蛇形路径。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

// PCL 基础与滤波模块
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>          
#include <pcl/filters/passthrough.h>         
#include <pcl/filters/extract_indices.h>     
#include <pcl/filters/filter.h>              // 【新增】用于 removeNaNFromPointCloud
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/search/kdtree.h>               
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/common/common.h>               

// PCL 曲面处理与特征提取模块
#include <pcl/surface/mls.h>                 // MLS 移动最小二乘平滑与上采样
#include <pcl/features/normal_3d.h>          // PCA 法线计算

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <X11/Xlib.h>

class CameraUseNode : public rclcpp::Node
{
public:
    // ==========================================
    // 🎛️ 核心控制面板
    // ==========================================
    bool enable_3d_pointcloud_ = true; 
    bool enable_2d_color_img_  = true; 
    bool enable_2d_depth_img_  = true; 

    // 保护机制：每 2.0 秒处理一帧，防止内存抢占和 CPU 爆满
    double process_interval_ = 5.0;

    // --- 前置滤波与聚类参数 (针对 30cm 视距调整) ---
    bool enable_passthrough_filter_ = true; 
    // 【调整】相机在30cm处，桌面Z值约为0.3。设置0.15~0.35即可完美囊括桌面和上方工件
    float pass_z_min_ = 0.15f;               
    float pass_z_max_ = 0.35f;              
    
    bool enable_voxel_filter_  = true;      
    // 【调整】30cm处点云很密，将体素缩小到 2mm (0.002)，保留更多物体细节
    float voxel_size_ = 0.002f;             
    
    bool enable_ransac_filter_ = true;
    // 【调整】近距离深度噪声小，将桌面容差阈值缩紧到 5mm，防止误删底部特征
    float ransac_distance_threshold_ = 0.005f; 
    
    bool enable_cluster_extraction_ = true;
    // 【调整】聚类容差设为 1cm
    float cluster_tolerance_ = 0.01f; 
    int min_cluster_size_ = 200;

    // --- 工序 5：MLS 平滑 (纯平滑调试版) ---
    bool enable_mls_smoothing_ = true;
    // 【调整】减小搜索半径至 8mm (0.008)。太大会导致边缘被严重磨平并拖慢速度
    float mls_search_radius_ = 0.008f; 
    
    // --- 工序 6：PCA 法线估计 ---
    bool enable_normal_estimation_ = true;
    // 法线搜索半径也可以匹配 MLS 的半径，或者稍微大一点 (1cm)
    float normal_search_radius_ = 0.01f;

    // --- 工序 7：真 3D 切片路径预览 ---
    bool enable_slicing_preview_ = true;
    float slice_step_ = 0.045f;       // 步进间距 45mm
    float slice_thickness_ = 0.002f;  // 切片刀厚度 2mm

    CameraUseNode() : Node("my_camera_use"), last_process_time_(0, 0, this->get_clock()->get_clock_type())
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
        RCLCPP_INFO(this->get_logger(), "视觉节点启动！已开启 %.1f 秒 智能降频保护，算法已针对 30cm 视距优化！", process_interval_);
    }

    void getLatestData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, 
                       pcl::PointCloud<pcl::Normal>::Ptr& normals,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sliced) {
        std::lock_guard<std::mutex> lock(pc_mutex_);
        cloud = latest_cloud_;
        normals = latest_normals_;
        sliced = latest_sliced_cloud_;
    }
    cv::Mat getLatestColorImg() { std::lock_guard<std::mutex> lock(color_mutex_); return latest_color_img_.clone(); }
    cv::Mat getLatestDepthImg() { std::lock_guard<std::mutex> lock(depth_mutex_); return latest_depth_img_.clone(); }

private:
    std::mutex pc_mutex_, color_mutex_, depth_mutex_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_cloud_;
    pcl::PointCloud<pcl::Normal>::Ptr latest_normals_; 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_sliced_cloud_; 
    cv::Mat latest_color_img_;
    cv::Mat latest_depth_img_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    
    rclcpp::Time last_process_time_;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto now = this->now();
        if ((now - last_process_time_).seconds() < process_interval_) {
            return; 
        }
        last_process_time_ = now; 

        RCLCPP_INFO(this->get_logger(), "--> 捕获新帧，开始执行重载计算...");

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *raw_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::Normal>::Ptr final_normals(new pcl::PointCloud<pcl::Normal>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        // 1. 直通滤波 (Z轴: 0.15m ~ 0.35m)
        if (enable_passthrough_filter_) {
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud(raw_cloud); pass.setFilterFieldName("z"); pass.setFilterLimits(pass_z_min_, pass_z_max_); pass.filter(*pass_cloud);
        } else pass_cloud = raw_cloud;

        // 2. 体素降采样 (30cm处，使用 2mm 网格保留细节)
        if (enable_voxel_filter_) {
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
            voxel.setInputCloud(pass_cloud); voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_); voxel.filter(*voxel_cloud);
        } else voxel_cloud = pass_cloud;

        // 3. RANSAC 桌面剔除 (容差 5mm)
        if (enable_ransac_filter_ && voxel_cloud->points.size() > 50) {
            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true); seg.setModelType(pcl::SACMODEL_PLANE); seg.setMethodType(pcl::SAC_RANSAC);      
            seg.setMaxIterations(1000); seg.setDistanceThreshold(ransac_distance_threshold_); 
            seg.setInputCloud(voxel_cloud); seg.segment(*inliers, *coeff);    
            if (inliers->indices.size() > 0) {
                pcl::ExtractIndices<pcl::PointXYZRGB> ext;
                ext.setInputCloud(voxel_cloud); ext.setIndices(inliers); ext.setNegative(true); ext.filter(*ransac_cloud);
            } else ransac_cloud = voxel_cloud;
        } else ransac_cloud = voxel_cloud;

        // 4. 欧式聚类目标提取
        if (enable_cluster_extraction_ && ransac_cloud->points.size() > 0) {
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud(ransac_cloud);
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            ec.setClusterTolerance(cluster_tolerance_); ec.setMinClusterSize(min_cluster_size_); ec.setMaxClusterSize(250000);
            ec.setSearchMethod(tree); ec.setInputCloud(ransac_cloud); ec.extract(cluster_indices);
            if (cluster_indices.size() > 0) {
                pcl::PointIndices::Ptr target_indices(new pcl::PointIndices(cluster_indices[0]));
                pcl::ExtractIndices<pcl::PointXYZRGB> ext;
                ext.setInputCloud(ransac_cloud); ext.setIndices(target_indices); ext.setNegative(false); ext.filter(*cluster_cloud);
            } else cluster_cloud = ransac_cloud;
        } else cluster_cloud = ransac_cloud;

        // ==========================================
        // 5. MLS 平滑 (调试重构版：清洗NaN、小半径、禁上采样)
        // ==========================================
        if (enable_mls_smoothing_ && cluster_cloud->points.size() > 50) {
            
            // 【新增】5.1 强制过滤 NaN 点，防止后续 KD树 或 MLS 计算抛出异常或内存闪退
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_nan(new pcl::PointCloud<pcl::PointXYZRGB>());
            std::vector<int> nan_indices;
            pcl::removeNaNFromPointCloud(*cluster_cloud, *cloud_no_nan, nan_indices);

            if (!cloud_no_nan->empty()) {
                pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
                mls.setInputCloud(cloud_no_nan);
                
                // 本质平滑即可，把法线计算交给下游更专业的 NormalEstimation，防止 Eigen 类型冲突
                mls.setComputeNormals(false); 
                mls.setPolynomialOrder(2);    
                
                // 保持单线程，PCL在ROS2多线程下Eigen容易出段错误
                mls.setNumberOfThreads(1);

                pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
                mls.setSearchMethod(tree);
                
                // 【调整】使用极小的搜索半径 (当前设为 8mm)
                mls.setSearchRadius(mls_search_radius_); 
                
                // 【调整】暂时禁用上采样，方便观察原始点的纯平滑效果，同时大幅降低计算量
                mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::NONE);

                mls.process(*mls_cloud); 
            } else {
                RCLCPP_WARN(this->get_logger(), "警告：当前帧过滤 NaN 后点云为空！");
                mls_cloud = cluster_cloud;
            }
        } else {
            mls_cloud = cluster_cloud;
        }

        // 6. PCA 法线估计
        if (enable_normal_estimation_ && mls_cloud->points.size() > 0) {
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
            ne.setInputCloud(mls_cloud);
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(normal_search_radius_);
            // 视点设为相机原点，保证法线朝向相机
            ne.setViewPoint(0.0, 0.0, 0.0);
            ne.compute(*final_normals);
        }

        // 7. 真 3D 切片法预览 (蛇形扫描可视化)
        if (enable_slicing_preview_ && mls_cloud->points.size() > 0) {
            pcl::PointXYZRGB min_pt, max_pt;
            pcl::getMinMax3D(*mls_cloud, min_pt, max_pt);

            for (float y = min_pt.y; y <= max_pt.y; y += slice_step_) {
                pcl::PassThrough<pcl::PointXYZRGB> slicer;
                slicer.setInputCloud(mls_cloud);
                slicer.setFilterFieldName("y"); 
                slicer.setFilterLimits(y - slice_thickness_, y + slice_thickness_);
                
                pcl::PointCloud<pcl::PointXYZRGB> current_slice;
                slicer.filter(current_slice);

                for (auto& p : current_slice.points) {
                    p.r = 255; p.g = 0; p.b = 0; 
                }
                *sliced_cloud += current_slice;
            }
        }

        RCLCPP_INFO(this->get_logger(), "<-- 计算完毕！工件提取: %zu 点 | MLS处理后: %zu 点", 
            cluster_cloud->points.size(), mls_cloud->points.size());

        std::lock_guard<std::mutex> lock(pc_mutex_);
        latest_cloud_ = mls_cloud;
        latest_normals_ = final_normals;
        latest_sliced_cloud_ = sliced_cloud;
    }

    void color_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try { cv::Mat cv_img = cv_bridge::toCvCopy(msg, "bgr8")->image; std::lock_guard<std::mutex> lock(color_mutex_); latest_color_img_ = cv_img; } catch (...) {}
    }
    void depth_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try { cv::Mat d_16 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image; cv::Mat d_8, d_c; d_16.convertTo(d_8, CV_8UC1, 255.0/2000.0); cv::applyColorMap(d_8, d_c, cv::COLORMAP_JET); std::lock_guard<std::mutex> lock(depth_mutex_); latest_depth_img_ = d_c; } catch (...) {}
    }
};

int main(int argc, char * argv[]) {
    XInitThreads(); rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraUseNode>();
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if (node->enable_3d_pointcloud_) {
        viewer = std::make_shared<pcl::visualization::PCLVisualizer>("D435i Real-time 3D Viewer");
        viewer->setBackgroundColor(0.05, 0.05, 0.05); viewer->addCoordinateSystem(0.2); viewer->initCameraParameters();               
    }
    if (node->enable_2d_color_img_) cv::namedWindow("2D Color Stream", cv::WINDOW_AUTOSIZE);
    if (node->enable_2d_depth_img_) cv::namedWindow("2D Depth Stream (Colormap)", cv::WINDOW_AUTOSIZE);
    rclcpp::WallRate loop_rate(30);
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->enable_3d_pointcloud_ && viewer && !viewer->wasStopped()) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
            pcl::PointCloud<pcl::Normal>::Ptr normals;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced;
            node->getLatestData(cloud, normals, sliced); 
            
            if (cloud && !cloud->empty()) {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> dark_color(cloud, 100, 100, 100);
                if (!viewer->updatePointCloud(cloud, dark_color, "d435i_cloud")) {
                    viewer->addPointCloud(cloud, dark_color, "d435i_cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "d435i_cloud"); 
                }
                
                if (normals && !normals->empty()) {
                    viewer->removePointCloud("normals"); 
                    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 20, 0.02, "normals");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.0, "normals"); 
                }

                if (sliced && !sliced->empty()) {
                    if (!viewer->updatePointCloud(sliced, "sliced_cloud")) {
                        viewer->addPointCloud(sliced, "sliced_cloud");
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sliced_cloud"); 
                    }
                }
            }
            viewer->spinOnce(10); 
        }
        if (node->enable_2d_color_img_) { cv::Mat color_img = node->getLatestColorImg(); if (!color_img.empty()) cv::imshow("2D Color Stream", color_img); }
        if (node->enable_2d_depth_img_) { cv::Mat depth_img = node->getLatestDepthImg(); if (!depth_img.empty()) cv::imshow("2D Depth Stream (Colormap)", depth_img); }
        if (node->enable_2d_color_img_ || node->enable_2d_depth_img_) cv::waitKey(10); 
        if (viewer && viewer->wasStopped()) break; 
        loop_rate.sleep();
    }
    rclcpp::shutdown(); cv::destroyAllWindows();
    return 0;
}