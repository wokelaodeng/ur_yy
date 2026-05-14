/*
 * Copyright 2026 [Your Name/Lab]
 *
 * 本代码实现的功能：
 * 1. 订阅 D435i 相机的原始点云，运行极其稳定的 5 道工序。
 * 2. 彻底移除不稳定的 MLS 模块，告别内存崩溃。
 * 3. 【全新加入】真 3D 切片路径预览功能！在 PCL 窗口中实时渲染 45mm 间距的红色扫描轨迹。
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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/search/kdtree.h>               
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/common/common.h>               // 用于获取点云的边界极值 (getMinMax3D)

// PCL 特征提取模块 (法线计算)
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

    // --- 前置稳定滤波工序 ---
    bool enable_passthrough_filter_ = true; 
    float pass_z_min_ = 0.1f;               
    float pass_z_max_ = 0.8f;              
    
    bool enable_voxel_filter_  = true;      
    // 既然去掉了上采样，我们将体素稍微调小一点，保证点云自身的致密性，填补空隙
    float voxel_size_ = 0.003f;             
    
    bool enable_ransac_filter_ = true;
    float ransac_distance_threshold_ = 0.015f; 
    
    bool enable_cluster_extraction_ = true;
    float cluster_tolerance_ = 0.015f; 
    int min_cluster_size_ = 200;

    // --- 工序 5：PCA 法线估计 ---
    bool enable_normal_estimation_ = true;
    float normal_search_radius_ = 0.02f;

    // --- 【全新功能】工序 6：真 3D 切片路径预览 ---
    // 功能：用数学平面模拟机械臂的扫描路径，从点云中“切”出实体轨迹线
    bool enable_slicing_preview_ = true;
    
    // 变量：slice_step_ (扫描步进间距，单位：米)
    // 作用：你的超声探头宽度假设是 50mm，为了保证覆盖，我们设定每隔 45mm 切一刀
    float slice_step_ = 0.045f; 
    
    // 变量：slice_thickness_ (切片容忍厚度，单位：米)
    // 作用：因为点云是离散的，绝对的数学平面切不到点，我们切一个带有厚度的“薄片”
    // 修改：调大(如0.005)线会变宽；调小(如0.001)线会变细但可能产生断点。推荐 0.002
    float slice_thickness_ = 0.002f;

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
        RCLCPP_INFO(this->get_logger(), "视觉节点启动！已抛弃不稳定算法，开启 3D 切片轨迹预览！");
    }

    // 暴露给显示线程的接口
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_sliced_cloud_; // [新增] 存放切片轨迹点
    cv::Mat latest_color_img_;
    cv::Mat latest_depth_img_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 准备所有的容器
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *raw_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::Normal>::Ptr final_normals(new pcl::PointCloud<pcl::Normal>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        // 1. 直通滤波
        if (enable_passthrough_filter_) {
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud(raw_cloud); pass.setFilterFieldName("z"); pass.setFilterLimits(pass_z_min_, pass_z_max_); pass.filter(*pass_cloud);
        } else pass_cloud = raw_cloud;

        // 2. 体素降采样
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
                ext.setInputCloud(ransac_cloud); ext.setIndices(target_indices); ext.setNegative(false); ext.filter(*final_cloud);
            } else final_cloud = ransac_cloud;
        } else final_cloud = ransac_cloud;

        // 5. PCA 法线估计 (我们完全信赖的老伙计)
        if (enable_normal_estimation_ && final_cloud->points.size() > 0) {
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
            ne.setInputCloud(final_cloud);
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(normal_search_radius_);
            ne.setViewPoint(0.0, 0.0, 0.0);
            ne.compute(*final_normals);
        }

        // ==========================================
        // 6. [核心新算法] 真 3D 切片法预览 (Slicing)
        // ==========================================
        if (enable_slicing_preview_ && final_cloud->points.size() > 0) {
            // 第 1 步：获取整个工件点云的空间三维极值（也就是包围盒边界）
            pcl::PointXYZRGB min_pt, max_pt;
            pcl::getMinMax3D(*final_cloud, min_pt, max_pt);

            // 第 2 步：沿着 Y 轴（假设这是工件宽度的方向）开始画线
            // 从最小的 Y 坐标开始，每次增加你设定的步距 (45mm)
            for (float y = min_pt.y; y <= max_pt.y; y += slice_step_) {
                
                // 第 3 步：使用一个极薄的直通滤波器充当“切片刀”
                pcl::PassThrough<pcl::PointXYZRGB> slicer;
                slicer.setInputCloud(final_cloud);
                slicer.setFilterFieldName("y"); // 沿着 Y 轴切
                // 设定刀片的厚度：[y - 2mm, y + 2mm]
                slicer.setFilterLimits(y - slice_thickness_, y + slice_thickness_);
                
                pcl::PointCloud<pcl::PointXYZRGB> current_slice;
                slicer.filter(current_slice);

                // 第 4 步：将切出来的这道轨迹强制染成亮红色，并加入总的轨迹云中
                for (auto& p : current_slice.points) {
                    p.r = 255; p.g = 0; p.b = 0; 
                }
                *sliced_cloud += current_slice;
            }
        }

        std::lock_guard<std::mutex> lock(pc_mutex_);
        latest_cloud_ = final_cloud;
        latest_normals_ = final_normals;
        latest_sliced_cloud_ = sliced_cloud; // 传给显示线程
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
                // 显示原始手机点云 (这里我强制把它变暗了一点，为了突出红色的切片线)
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> dark_color(cloud, 100, 100, 100);
                if (!viewer->updatePointCloud(cloud, dark_color, "d435i_cloud")) {
                    viewer->addPointCloud(cloud, dark_color, "d435i_cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "d435i_cloud"); 
                }
                
                // 显示绿色的法线
                if (normals && !normals->empty()) {
                    viewer->removePointCloud("normals"); 
                    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 20, 0.02, "normals");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.0, "normals"); 
                }

                // [新增] 显示鲜红色的 3D 切片扫描轨迹
                if (sliced && !sliced->empty()) {
                    if (!viewer->updatePointCloud(sliced, "sliced_cloud")) {
                        viewer->addPointCloud(sliced, "sliced_cloud");
                        // 强制放大红色点，使其看起来像连续的线
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