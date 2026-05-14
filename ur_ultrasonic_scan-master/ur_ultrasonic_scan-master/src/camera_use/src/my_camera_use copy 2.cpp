/*
 * Copyright 2026 [Your Name/Lab]
 *
 * 本代码实现的功能：
 * 1. 订阅 D435i 点云，加入【智能降频机制】，每隔 2.0 秒处理一帧。
 * 2. 针对 30cm 视距进行算法参数专门优化。
 * 3. 彻底弃用高耗能的 MLS 和仅支持 Intensity 的 Bilateral，采用 Voxel(几何均值) + SOR(统计去噪) + ROR(半径去噪) 组合。
 * 4. 完美兼顾算力与点云几何平滑度，杜绝内存闪退。
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
#include <pcl/filters/filter.h>              
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/search/kdtree.h>               
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/common/common.h>               

// 稳健去噪头文件
#include <pcl/filters/statistical_outlier_removal.h> // SOR 统计滤波
#include <pcl/filters/radius_outlier_removal.h>      // ROR 半径滤波

// PCL 特征提取模块
#include <pcl/features/normal_3d.h>          

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

    double process_interval_ = 2.0;

    // --- 前置滤波与聚类参数 (针对 30cm 视距) ---
    bool enable_passthrough_filter_ = true; 
    float pass_z_min_ = 0.10f;               
    float pass_z_max_ = 0.35f;              
    
    bool enable_voxel_filter_  = true;      
    // 体素网格大小。体素滤波会计算网格内的形心，这本身就是最稳健的几何平滑！
    float voxel_size_ = 0.003f; // 3mm 体素，提供恰到好处的均值平滑            
    
    bool enable_ransac_filter_ = true;
    float ransac_distance_threshold_ = 0.005f; 
    
    bool enable_cluster_extraction_ = true;
    float cluster_tolerance_ = 0.025f; 
    int min_cluster_size_ = 200;

    // --- 联合去噪流水线参数 ---
    bool enable_sor_filter_ = true;
    int sor_mean_k_ = 50;               
    float sor_stddev_ = 1.0f;           

    bool enable_ror_filter_ = true;
    float ror_radius_ = 0.008f;         // 考察半径 8mm
    int ror_min_neighbors_ = 10;        // 8mm内少于10个点将被视为悬空噪点剔除

    // --- 工序 6：PCA 法线估计 ---
    bool enable_normal_estimation_ = true;
    float normal_search_radius_ = 0.015f; // 扩大法线搜索半径，能让法线向量更加平滑一致

    // --- 工序 7：真 3D 切片路径预览 ---
    bool enable_slicing_preview_ = true;
    float slice_step_ = 0.045f;       
    float slice_thickness_ = 0.002f;  

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
        RCLCPP_INFO(this->get_logger(), "视觉节点启动！采用 Voxel + SOR + ROR 稳健联合滤波方案！");
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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::Normal>::Ptr final_normals(new pcl::PointCloud<pcl::Normal>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        // 1. 直通滤波
        if (enable_passthrough_filter_) {
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud(raw_cloud); pass.setFilterFieldName("z"); pass.setFilterLimits(pass_z_min_, pass_z_max_); pass.filter(*pass_cloud);
        } else pass_cloud = raw_cloud;

        // 2. 体素降采样 (自带几何平滑效果)
        if (enable_voxel_filter_) {
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
            voxel.setInputCloud(pass_cloud); voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_); voxel.filter(*voxel_cloud);
        } else voxel_cloud = pass_cloud;

        // 3. RANSAC 桌面剔除
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
        // 5. 极速稳健去噪流水线 (取代 MLS)
        // ==========================================
        if (cluster_cloud->points.size() > 50) {
            
            // 5.1 统计滤波 (SOR) - 过滤大范围飞点
            if (enable_sor_filter_) {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
                sor.setInputCloud(cluster_cloud);
                sor.setMeanK(sor_mean_k_);
                sor.setStddevMulThresh(sor_stddev_);
                sor.filter(*sor_cloud);
            } else {
                sor_cloud = cluster_cloud;
            }

            // 5.2 半径滤波 (ROR) - 精细清理边缘游离点
            if (enable_ror_filter_ && sor_cloud->points.size() > 0) {
                pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
                ror.setInputCloud(sor_cloud);
                ror.setRadiusSearch(ror_radius_);
                ror.setMinNeighborsInRadius(ror_min_neighbors_);
                ror.filter(*smoothed_cloud);
            } else {
                smoothed_cloud = sor_cloud;
            }
        } else {
            smoothed_cloud = cluster_cloud;
        }

        // 6. PCA 法线估计
        if (enable_normal_estimation_ && smoothed_cloud->points.size() > 0) {
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
            ne.setInputCloud(smoothed_cloud);
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(normal_search_radius_);
            ne.setViewPoint(0.0, 0.0, 0.0);
            ne.compute(*final_normals);
        }

        // 7. 真 3D 切片法预览 
        if (enable_slicing_preview_ && smoothed_cloud->points.size() > 0) {
            pcl::PointXYZRGB min_pt, max_pt;
            pcl::getMinMax3D(*smoothed_cloud, min_pt, max_pt);

            for (float y = min_pt.y; y <= max_pt.y; y += slice_step_) {
                pcl::PassThrough<pcl::PointXYZRGB> slicer;
                slicer.setInputCloud(smoothed_cloud); 
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

        RCLCPP_INFO(this->get_logger(), "<-- 计算完毕！有效提取: %zu 点 | 去噪输出: %zu 点", 
            cluster_cloud->points.size(), smoothed_cloud->points.size());

        std::lock_guard<std::mutex> lock(pc_mutex_);
        latest_cloud_ = smoothed_cloud; 
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