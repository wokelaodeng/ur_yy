#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// PCL 基础
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// PCL 滤波与分割算法
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>               

// ⭐ 新增：欧式聚类提取头文件
#include <pcl/segmentation/extract_clusters.h>

// PCL 特征提取
#include <pcl/features/normal_3d_omp.h>       
#include <pcl/search/kdtree.h>                

#include <mutex>
#include <memory>

using std::placeholders::_1;

// =========================================================================
// ⭐ 核心算法参数配置区 (Algorithm Configuration Zone)
// =========================================================================
struct FilterConfig {
    // 1. 体素降采样 (m)
    float voxel_leaf_size = 0.005f; 

    // 2. 直通滤波 Z 轴范围 (m)
    float passthrough_min_z = 0.8f;
    float passthrough_max_z = 1.25f;

    // 3. RANSAC 桌面判定阈值 (m)
    float ransac_distance_threshold = 0.01f;

    // ===================================================
    // 4. ⭐ 欧式聚类参数 (Euclidean Clustering)
    // ===================================================
    // 变量作用：判断两个点是否属于同一个物体的最大距离。
    // 单位：米 (m)。0.02f 代表 2cm。
    // 调节效果：如果设得太小（如 0.001），一个圆柱体会被切成无数块碎片；
    // 如果设得太大（如 0.1），靠近圆柱体的其他杂物会被错误地合并成一个物体。
    // 因为前面的降采样是 5mm，所以这里的容差必须大于 5mm，2cm(0.02) 是非常稳妥的值。
    float cluster_tolerance = 0.02f;
    
    // 变量作用：作为一个独立物体（Cluster）必须满足的最少点数。
    // 调节效果：极其有效地过滤掉空间中成群的“马赛克噪点”。100 个点起步。
    int cluster_min_size = 100;
    
    // 变量作用：一个物体的最大点数限制。
    int cluster_max_size = 25000;

    // 5. SOR 统计离群点滤波参数 (针对提取出的聚类)
    int sor_mean_k = 50;              
    float sor_stddev_thresh = 1.0f;   

    // 6. 法向量计算参数 (m)
    float normal_search_radius = 0.03f;
    int normal_display_step = 3; 
    float normal_display_scale = 0.02f;
};

// =========================================================================
// 主节点类定义
// =========================================================================
class CameraSimUseNode : public rclcpp::Node
{
public:
    CameraSimUseNode() : Node("d435i_camera_sim_use")
    {
        RCLCPP_INFO(this->get_logger(), "视觉感知节点已启动：正在执行带【欧式聚类实例分割】的 7 步感知流水线...");

        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&CameraSimUseNode::rgb_callback, this, _1));
        
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10, std::bind(&CameraSimUseNode::pc_callback, this, _1));

        viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("PCL Final Target with Normals");
        viewer_->setBackgroundColor(0.05, 0.05, 0.05); 
        viewer_->addCoordinateSystem(0.1);             

        cv::namedWindow("RGB Stream", cv::WINDOW_AUTOSIZE);

        ui_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&CameraSimUseNode::update_ui, this));
    }

    ~CameraSimUseNode()
    {
        cv::destroyAllWindows();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    
    cv::Mat current_rgb_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud_{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::Normal>::Ptr current_normals_{new pcl::PointCloud<pcl::Normal>};
    
    std::mutex data_mutex_; 
    FilterConfig config_;   

    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    rclcpp::TimerBase::SharedPtr ui_timer_;
    bool is_first_cloud_ = true;

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_rgb_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // =========================================================================
    // ⭐ 终极大脑：7步目标提取与特征管线
    // NaN -> Voxel -> PassThrough -> RANSAC -> Clustering -> SOR -> OMP
    // =========================================================================
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *raw_cloud);
        if (raw_cloud->empty()) return;

        // 【步骤 1：去除 NaN 点】 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*raw_cloud, *clean_cloud, indices);
        if (clean_cloud->empty()) return;

        // 【步骤 2：体素降采样】
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(clean_cloud); 
        vg.setLeafSize(config_.voxel_leaf_size, config_.voxel_leaf_size, config_.voxel_leaf_size); 
        vg.filter(*voxel_cloud);

        // 【步骤 3：直通滤波】 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pt;
        pt.setInputCloud(voxel_cloud);
        pt.setFilterFieldName("z"); 
        pt.setFilterLimits(config_.passthrough_min_z, config_.passthrough_max_z); 
        pt.filter(*pt_cloud); 
        if (pt_cloud->points.size() < 10) return; 

        // 【步骤 4：RANSAC 平面分割】 (剥离桌面)
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);     
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); 
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE); 
        seg.setMethodType(pcl::SAC_RANSAC);    
        seg.setMaxIterations(100);             
        seg.setDistanceThreshold(config_.ransac_distance_threshold);        
        seg.setInputCloud(pt_cloud);
        seg.segment(*inliers, *coefficients);  

        if (inliers->indices.size() == 0) return; 

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(pt_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true); // 留下非平面的物体群
        extract.filter(*objects_cloud);
        if (objects_cloud->points.size() < 10) return;

        // ==========================================
        // 【步骤 5：欧式聚类提取】 (锁定目标工件)
        // ==========================================
        // 为聚类创建一个 KD-Tree 搜索树
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr cluster_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        cluster_tree->setInputCloud(objects_cloud);

        std::vector<pcl::PointIndices> cluster_indices; // 存放所有被分离出来的聚类组
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(config_.cluster_tolerance); 
        ec.setMinClusterSize(config_.cluster_min_size);
        ec.setMaxClusterSize(config_.cluster_max_size);
        ec.setSearchMethod(cluster_tree);
        ec.setInputCloud(objects_cloud);
        ec.extract(cluster_indices); // 执行聚类

        if (cluster_indices.empty()) {
            // 没有找到符合大小要求的物体
            return; 
        }

        // ⭐ PCL 的聚类算法默认会按照点数从多到少对聚类进行排序。
        // 所以 cluster_indices[0] 绝对就是桌面上最大的那个物体（我们的圆柱体）！
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_workpiece_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*objects_cloud, cluster_indices[0], *target_workpiece_cloud);

        // ==========================================
        // 【步骤 6：SOR 统计离群点滤波】 (只针对提取出的最大工件)
        // ==========================================
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(target_workpiece_cloud); // 输入变为单纯的目标工件
        sor.setMeanK(config_.sor_mean_k);
        sor.setStddevMulThresh(config_.sor_stddev_thresh);
        sor.filter(*sor_cloud); 

        // 【步骤 7：OMP 多核法向量计算】
        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne.setSearchMethod(tree);
        ne.setInputCloud(sor_cloud);
        ne.setRadiusSearch(config_.normal_search_radius); 
        ne.compute(*current_normals_);

        pcl::copyPointCloud(*sor_cloud, *current_cloud_);
    }

    // =========================================================================
    // UI 渲染循环 
    // =========================================================================
    void update_ui()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!current_rgb_.empty()) {
            cv::imshow("RGB Stream", current_rgb_);
        }
        cv::waitKey(1); 

        if (!current_cloud_->empty() && !current_normals_->empty()) {
            if (is_first_cloud_) {
                viewer_->addPointCloud<pcl::PointXYZRGB>(current_cloud_, "cloud");
                viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
                
                viewer_->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(
                    current_cloud_, current_normals_, config_.normal_display_step, config_.normal_display_scale, "normals");
                
                viewer_->setCameraPosition(0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
                is_first_cloud_ = false;
            } else {
                viewer_->updatePointCloud<pcl::PointXYZRGB>(current_cloud_, "cloud");
                viewer_->removePointCloud("normals");
                viewer_->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(
                    current_cloud_, current_normals_, config_.normal_display_step, config_.normal_display_scale, "normals");
            }
        }
        viewer_->spinOnce(1, true); 
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSimUseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}