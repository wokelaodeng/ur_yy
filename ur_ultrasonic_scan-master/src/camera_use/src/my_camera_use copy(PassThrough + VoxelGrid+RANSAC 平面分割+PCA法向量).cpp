/*
 * Copyright 2026 [Your Name/Lab]
 *
 * 本代码实现的功能：
 * 1. 订阅 D435i 相机的原始点云。
 * 2. 依次经过 5 道核心工序：直通 -> 体素 -> RANSAC -> 欧式聚类 -> PCA法线估计。
 * 3. 最终输出纯净的工件点云，以及每一个点对应的垂直法向量。
 * 4. 在 PCL 窗口中以绿色箭头实时渲染法向量。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

// PCL 基础与滤波模块
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>          // 体素滤波
#include <pcl/filters/passthrough.h>         // 直通滤波
#include <pcl/filters/extract_indices.h>     // 索引提取器
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> // RANSAC 平面分割
#include <pcl/search/kdtree.h>               // KD树搜索
#include <pcl/segmentation/extract_clusters.h> // 欧式聚类

// [新增] PCL 特征提取模块 (法线计算核心)
#include <pcl/features/normal_3d.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <X11/Xlib.h>

class CameraUseNode : public rclcpp::Node
{
public:
    // ==========================================
    // 🎛️ 核心控制面板与算法调参区 (保姆级注释)
    // ==========================================
    
    // [窗口显示开关]
    bool enable_3d_pointcloud_ = true; 
    bool enable_2d_color_img_  = true; 
    bool enable_2d_depth_img_  = true; 

    // --- 工序 1：直通滤波 (PassThrough) ---
    // 功能：像剪刀一样，切除视野极近处(盲区)和极远处(墙壁背景)的无关数据。
    bool enable_passthrough_filter_ = true; 
    // 变量：pass_z_min_ (最小深度保留阈值，单位：米)
    // 修改效果：调大(如0.3)会切除距镜头30cm内的所有物体，包括机械臂夹爪；调小(0.0)会引入镜头近处的反射噪点。
    float pass_z_min_ = 0.1f;               
    // 变量：pass_z_max_ (最大深度保留阈值，单位：米)
    // 修改效果：调小(如0.5)切除50cm外的数据。若比桌面距离还小，你的工件就会被切除。
    float pass_z_max_ = 0.8f;              

    // --- 工序 2：体素降采样 (VoxelGrid) ---
    // 功能：将密集的点云“马赛克化”，用极少量的点概括物体形状，大幅提速。
    bool enable_voxel_filter_  = true;      
    // 变量：voxel_size_ (体素叶子大小，单位：米)
    // 修改效果：调大(如0.02即2cm)，算得飞快，但工件边缘会变成锯齿状；调小(如0.001)，细节完美，但会导致后续聚类和法线计算严重卡顿。
    float voxel_size_ = 0.005f;             

    // --- 工序 3：RANSAC 桌面剔除 (SACSegmentation) ---
    // 功能：自动寻找画面中面积最大的纯平面(桌面)，并将其无情删除，使工件悬空。
    bool enable_ransac_filter_ = true;
    // 变量：ransac_distance_threshold_ (平面距离容忍度，单位：米)
    // 作用：距离理想平面多少米以内的点会被当成“桌面”。
    // 修改效果：若发现手机底部被当成桌子误删了，必须调小(如0.005)；若桌面删不干净(有残余色块)，适当调大(如0.02)。
    float ransac_distance_threshold_ = 0.015f; 

    // --- 工序 4：欧式聚类目标提取 (Euclidean Clustering) ---
    // 功能：在悬空的点云中寻找扎堆的群体，提取出最大的一块（工件），扔掉周围所有的飞点和线缆。
    bool enable_cluster_extraction_ = true;
    // 变量：cluster_tolerance_ (聚类容忍距离，单位：米)
    // 作用：设定“群聊”拉人的搜索半径。相距小于此值的点才算同一个物体。
    // 修改效果：为了消除手机周围的飞点噪点，应尽量调小(如0.01)；但【绝对不能】小于 voxel_size_，否则工件会因为降采样带来的缝隙被撕裂成碎片。
    float cluster_tolerance_ = 0.01f; 
    // 变量：min_cluster_size_ (最小聚类点数)
    // 修改效果：调大(如200)，只有几十个点的悬浮噪点群组会被直接无视、彻底抹除。
    int min_cluster_size_ = 200;

    // --- 工序 5：PCA 表面法线估计 (Normal Estimation) ---
    // 功能：为点云上的每一个点计算出一个垂直于该点表面的方向向量，作为超声探头的姿态指南针。
    bool enable_normal_estimation_ = true;
    // 变量：normal_search_radius_ (法线搜索半径，单位：米)
    // 作用：计算某个点的法线时，需要参考它周围多大范围内的邻居点来拟合“切平面”。
    // 修改效果：
    // - 调小(如0.005)：对细节极其敏感，但也对噪点极其敏感，法向量会像杂草一样剧烈抖动。
    // - 调大(如0.05)：法向量会极其整齐平滑，但在工件的“直角边缘”处，会把侧面和顶面的点混在一起，算出错误的45度倾斜法线（过度平滑）。
    // 推荐值：0.01 ~ 0.03
    float normal_search_radius_ = 0.02f;

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
        RCLCPP_INFO(this->get_logger(), "视觉节点启动！正在进行：直通->体素->RANSAC->聚类->法线估计");
    }

    // 安全获取最新数据的接口（包含点云和法线）
    void getLatestData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals) {
        std::lock_guard<std::mutex> lock(pc_mutex_);
        cloud = latest_cloud_;
        normals = latest_normals_;
    }
    cv::Mat getLatestColorImg() { std::lock_guard<std::mutex> lock(color_mutex_); return latest_color_img_.clone(); }
    cv::Mat getLatestDepthImg() { std::lock_guard<std::mutex> lock(depth_mutex_); return latest_depth_img_.clone(); }

private:
    std::mutex pc_mutex_, color_mutex_, depth_mutex_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_cloud_;
    pcl::PointCloud<pcl::Normal>::Ptr latest_normals_; // 存放计算出的法向量
    cv::Mat latest_color_img_;
    cv::Mat latest_depth_img_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *raw_cloud);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::Normal>::Ptr final_normals(new pcl::PointCloud<pcl::Normal>());

        // 1. 直通滤波 (空间裁剪)
        if (enable_passthrough_filter_) {
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud(raw_cloud); pass.setFilterFieldName("z"); pass.setFilterLimits(pass_z_min_, pass_z_max_); pass.filter(*pass_cloud);
        } else pass_cloud = raw_cloud;

        // 2. 体素降采样 (数据压缩)
        if (enable_voxel_filter_) {
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
            voxel.setInputCloud(pass_cloud); voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_); voxel.filter(*voxel_cloud);
        } else voxel_cloud = pass_cloud;

        // 3. RANSAC 桌面剔除 (剥离环境)
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

        // 4. 欧式聚类目标提取 (精准抠图)
        if (enable_cluster_extraction_ && ransac_cloud->points.size() > 0) {
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud(ransac_cloud);
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            ec.setClusterTolerance(cluster_tolerance_); ec.setMinClusterSize(min_cluster_size_); ec.setMaxClusterSize(250000);
            ec.setSearchMethod(tree); ec.setInputCloud(ransac_cloud); ec.extract(cluster_indices);
            
            // 获取点数最多的群体（即目标工件）
            if (cluster_indices.size() > 0) {
                pcl::PointIndices::Ptr target_indices(new pcl::PointIndices(cluster_indices[0]));
                pcl::ExtractIndices<pcl::PointXYZRGB> ext;
                ext.setInputCloud(ransac_cloud); ext.setIndices(target_indices); ext.setNegative(false); ext.filter(*final_cloud);
            } else final_cloud = ransac_cloud;
        } else final_cloud = ransac_cloud;

        // ==========================================
        // 5. PCA 表面法线估计 (提取姿态特征)
        // ==========================================
        if (enable_normal_estimation_ && final_cloud->points.size() > 0) {
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
            ne.setInputCloud(final_cloud);

            // 使用 KD树 加速邻居搜索
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
            ne.setSearchMethod(tree);
            // 设定邻居搜索范围
            ne.setRadiusSearch(normal_search_radius_);

            // 【极度致命的防撞机保护】: 强制法线朝向相机！
            // 作用：如果没有这句，PCA 算出的部分法线可能指向桌子内部。传给机械臂后，机械臂会试图从桌子底下往上撞穿工件。
            // 设定视点为 (0,0,0) 即相机光心，强制所有算出来的法线朝外指。
            ne.setViewPoint(0.0, 0.0, 0.0);

            // 运算并输出
            ne.compute(*final_normals);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "法线计算完毕！工件纯净点数: %zu, 成功生成姿态向量: %zu", 
                final_cloud->points.size(), final_normals->points.size());
        }

        // 线程安全写入缓存
        std::lock_guard<std::mutex> lock(pc_mutex_);
        latest_cloud_ = final_cloud;
        latest_normals_ = final_normals;
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
            node->getLatestData(cloud, normals); 
            
            if (cloud && !cloud->empty()) {
                if (!viewer->updatePointCloud(cloud, "d435i_cloud")) {
                    viewer->addPointCloud(cloud, "d435i_cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "d435i_cloud"); 
                }
                
                // ==========================================
                // 渲染引擎：将法向量画到屏幕上
                // ==========================================
                if (normals && !normals->empty()) {
                    viewer->removePointCloud("normals"); 
                    // 参数解释：
                    // - 10: 间隔渲染（每隔10个点画一根线，防止满屏绿线卡死电脑）
                    // - 0.02: 线的物理长度（2厘米）
                    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.02, "normals");
                    // 强制设定线的颜色为亮绿色 (R=0, G=1, B=0)
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "normals"); 
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