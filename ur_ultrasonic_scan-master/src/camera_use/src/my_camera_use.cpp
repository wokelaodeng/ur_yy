/*
 * Copyright 2026 [Your Name/Lab]
 *
 * 本代码实现的功能：
 * 1. 订阅 D435i 彩色对齐点云，带有性能评估模块。
 * 2. 针对【35cm视距】与【阶梯型(10mm/5mm/1mm)薄片工件】进行了极高精度参数重构。
 * 3. 彻底解决工件因 RANSAC 误杀导致的“断裂与解体”问题。
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
#include <pcl/filters/statistical_outlier_removal.h> 
#include <pcl/filters/radius_outlier_removal.h>      

// PCL 特征提取模块
#include <pcl/features/normal_3d.h>          

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <X11/Xlib.h>
#include <chrono> 

class CameraUseNode : public rclcpp::Node
{
public:
    // ==========================================
    // 🎛️ 核心控制面板 (针对 T型阶梯 PLA 工件极度优化)
    // ==========================================
    bool enable_3d_pointcloud_ = true; 
    bool enable_2d_color_img_  = true; 
    bool enable_2d_depth_img_  = true; 

    double process_interval_ = 2.0;

    // --- 1. 直通滤波 (空间裁剪) ---
    bool enable_passthrough_filter_ = true; 
    float pass_z_min_ = 0.26f; // 【修改】从0.20提至0.25。效果：更干净地切除机械臂在高空的挥舞干扰。
    float pass_z_max_ = 0.37f; // 【维持】包含35cm桌面。
    
    // --- 2. 体素降采样 (细化网格) ---
    bool enable_voxel_filter_  = true;      
    float voxel_size_ = 0.0005f; // 【修改】从2mm缩小至0.5mm。效果：提高点云分辨率，防止5mm凹槽的阶跃边缘被过度平滑成斜坡。
    
    // --- 3. RANSAC (精准剥离桌面) ---
    bool enable_ransac_filter_ = true;
    float ransac_distance_threshold_ = 0.0025f; // ⭐【核心修改】从4mm降至1.5mm！效果：这是解决工件“腰斩”的关键！由于相机存在1-2mm误差，原先的4mm极容易把5mm凹槽当成桌面吃掉。2.5mm能死死保住5mm的凹槽区。
    
    // --- 4. 欧式聚类 (提取目标) ---
    bool enable_cluster_extraction_ = true;
    float cluster_tolerance_ = 0.030f; // ⭐【核心修改】从1.5cm暴增至3cm！效果：即使5mm凹槽处由于反光出现了少许空洞（断层），3cm的广域容差也能把断开的“T”字上下两部分强行粘在同一个聚类里，彻底防止工件解体。
    int min_cluster_size_ = 30;        // 【修改】降低门槛，确保小工件不被丢弃。

    // --- 5. 联合去噪流水线 ---
    bool enable_sor_filter_ = true;
    int sor_mean_k_ = 50;               
    float sor_stddev_ = 2.0f;          // 【修改】从1.0放宽到2.0。效果：1.0太过苛刻，会把工件合法的物理边缘当做噪点杀掉。2.0能在去飞线的同时，完美保留几何边缘。

    bool enable_ror_filter_ = true;
    float ror_radius_ = 0.010f;        // 【修改】考察半径从6mm放大到10mm。
    int ror_min_neighbors_ = 4;        // 【修改】从8个点骤降到4个点。效果：极大地保护了最下方 1mm 薄板的存活率，不再将其误杀为“孤立噪点”。

    // --- 6. 法线估计 ---
    bool enable_normal_estimation_ = true;
    float normal_search_radius_ = 0.012f; // 【修改】略微加大搜索半径，在不平整的10mm表面获得更稳定的法线，方便超声探头贴合。

    // --- 7. 切片法预览 ---
    bool enable_slicing_preview_ = true;
    float slice_step_ = 0.020f;       
    float slice_thickness_ = 0.0015f;  // 【修改】配合体素变小，切片厚度也变薄，让红色轨迹更精细，不再臃肿。

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
        RCLCPP_INFO(this->get_logger(), "视觉节点启动！针对带凹槽的精细 PLA 工件参数重构完成。");
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

        RCLCPP_INFO(this->get_logger(), "\n=======================================================");
        RCLCPP_INFO(this->get_logger(), "--> 📷 捕获新帧，执行抗断裂点云流水线...");

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *raw_cloud);

        auto total_start = std::chrono::high_resolution_clock::now();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::Normal>::Ptr final_normals(new pcl::PointCloud<pcl::Normal>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        auto get_time_ms = [](auto start_time) {
            auto end_time = std::chrono::high_resolution_clock::now();
            return std::chrono::duration<double, std::milli>(end_time - start_time).count();
        };

        // 1. 直通滤波 
        auto t1 = std::chrono::high_resolution_clock::now();
        if (enable_passthrough_filter_) {
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud(raw_cloud); pass.setFilterFieldName("z"); pass.setFilterLimits(pass_z_min_, pass_z_max_); pass.filter(*pass_cloud);
        } else pass_cloud = raw_cloud;
        RCLCPP_INFO(this->get_logger(), "[1] 直通裁剪: 耗时 %.2f ms | 剩余点数: %zu", get_time_ms(t1), pass_cloud->size());

        // 2. 体素降采样 
        auto t2 = std::chrono::high_resolution_clock::now();
        if (enable_voxel_filter_) {
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
            voxel.setInputCloud(pass_cloud); voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_); voxel.filter(*voxel_cloud);
        } else voxel_cloud = pass_cloud;
        RCLCPP_INFO(this->get_logger(), "[2] 体素平滑: 耗时 %.2f ms | 剩余点数: %zu", get_time_ms(t2), voxel_cloud->size());

        // 3. RANSAC 桌面剔除
        auto t3 = std::chrono::high_resolution_clock::now();
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
        RCLCPP_INFO(this->get_logger(), "[3] 剥离桌面: 耗时 %.2f ms | 剩余点数: %zu", get_time_ms(t3), ransac_cloud->size());

        // 4. 欧式聚类目标提取
        auto t4 = std::chrono::high_resolution_clock::now();
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
        RCLCPP_INFO(this->get_logger(), "[4] 广域聚类: 耗时 %.2f ms | 有效目标点: %zu", get_time_ms(t4), cluster_cloud->size());

        // 5. 稳健去噪 (SOR+ROR)
        auto t5 = std::chrono::high_resolution_clock::now();
        if (cluster_cloud->points.size() > 50) {
            if (enable_sor_filter_) {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
                sor.setInputCloud(cluster_cloud);
                sor.setMeanK(sor_mean_k_);
                sor.setStddevMulThresh(sor_stddev_);
                sor.filter(*sor_cloud);
            } else sor_cloud = cluster_cloud;

            if (enable_ror_filter_ && sor_cloud->points.size() > 0) {
                pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
                ror.setInputCloud(sor_cloud);
                ror.setRadiusSearch(ror_radius_);
                ror.setMinNeighborsInRadius(ror_min_neighbors_);
                ror.filter(*smoothed_cloud);
            } else smoothed_cloud = sor_cloud;
        } else smoothed_cloud = cluster_cloud;
        RCLCPP_INFO(this->get_logger(), "[5] 保边去噪: 耗时 %.2f ms | 纯净点数: %zu", get_time_ms(t5), smoothed_cloud->size());

        // 6. PCA 法线估计
        auto t6 = std::chrono::high_resolution_clock::now();
        if (enable_normal_estimation_ && smoothed_cloud->points.size() > 0) {
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
            ne.setInputCloud(smoothed_cloud);
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(normal_search_radius_);
            ne.setViewPoint(0.0, 0.0, 0.0);
            ne.compute(*final_normals);
        }
        RCLCPP_INFO(this->get_logger(), "[6] 法线估计: 耗时 %.2f ms", get_time_ms(t6));

        // 7. 真 3D 切片法预览 (赋予切片醒目的红色)
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
                for (auto& p : current_slice.points) { p.r = 255; p.g = 0; p.b = 0; }
                *sliced_cloud += current_slice;
            }
        }

        RCLCPP_INFO(this->get_logger(), "✅ 流水线完成！总耗时: %.2f ms | 原始点: %zu -> 输出: %zu", 
                    get_time_ms(total_start), raw_cloud->size(), smoothed_cloud->size());
        RCLCPP_INFO(this->get_logger(), "=======================================================\n");

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
        viewer = std::make_shared<pcl::visualization::PCLVisualizer>("D435i Real-time 3D Viewer (RGBD)");
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
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_color(cloud);
                if (!viewer->updatePointCloud(cloud, rgb_color, "d435i_cloud")) {
                    viewer->addPointCloud(cloud, rgb_color, "d435i_cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "d435i_cloud"); 
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