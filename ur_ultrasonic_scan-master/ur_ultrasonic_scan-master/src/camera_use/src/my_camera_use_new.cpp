/*
 * 模块：my_camera_use_new.cpp
 * 功能：真实 D435i 相机单帧截取 (Scan-and-Plan) 与 MLS 平滑重构
 * 修复：解决物理零点偏移导致的残缺问题，利用 17mm 厚度优势彻底消除桌边残留
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// OpenCV 库
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// PCL 基础与 IO
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h> 

// PCL 滤波与分割算法
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

// PCL MLS 平滑算法
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>                

#include <mutex>
#include <memory>
#include <filesystem> 

using std::placeholders::_1;

// =========================================================================
// ⭐ 碳化硅(SiC)专属参数配置区 (抗倾斜 & 强力剥离桌面版)
// =========================================================================
struct FilterConfig {
    // 1. 体素降采样大小
    // 1mm (0.001f)。对 17x11cm 的大工件足够精细，且能抹平硬件噪点。
    float voxel_leaf_size = 0.001f; 

    // 2. 直通滤波 (Z轴空间裁剪) 
    // 【核心修复】放弃危险的一刀切！放宽到 0.20m ~ 0.45m。
    // 让桌子和工件全部完整进来，把剥离桌子的任务交给 RANSAC。
    float passthrough_min_z = 0.20f;
    float passthrough_max_z = 0.45f;

    // 3. RANSAC 去桌面阈值 【⭐ 杀手锏】
    // 工件厚度为 17mm。我们将桌面删除阈值大胆调高至 8mm (0.008f)！
    // 效果：桌子及其上方 8mm 范围内的所有反光、噪波杂边会被完全摧毁！而 17mm 的工件本体绝对安全。
    float ransac_distance_threshold = 0.008f;
    
    // 4. 欧式聚类参数 【⭐ 物理隔离断链】
    // 缩小到 1cm (0.010f)。
    // 效果：即使远处还有一丝没删干净的桌子残留，因为距离工件大于 1cm，在这一步也会被判定为无关杂质直接丢弃。
    float cluster_tolerance = 0.010f;
    int cluster_min_size = 150;     // 提高门槛，彻底过滤悬浮的空气噪点
    int cluster_max_size = 250000;

    // 5. 统计滤波 (SOR)
    // 恢复到 1.0 标准差，用来仔细打磨工件周边的游离飞线。
    int sor_mean_k = 50;              
    float sor_stddev_thresh = 1.0f;   

    // 6. MLS (移动最小二乘法) 表面平滑
    // 1.5cm (0.015f) 搜索半径。既能拟合出碳化硅表面的完美平面，又防止边缘严重坍缩。
    float mls_search_radius = 0.015f; 
    int mls_polynomial_order = 2;

    // 7. RViz 显示参数
    int normal_display_step = 5; 
    float normal_display_scale = 0.01f;
};

class RealCameraScanNode : public rclcpp::Node
{
public:
    RealCameraScanNode() : Node("my_camera_use_new")
    {
        RCLCPP_INFO(this->get_logger(), "📷 真实 D435i 单帧扫描启动！(SiC抗倾斜强力去桌版)");

        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&RealCameraScanNode::rgb_callback, this, _1));
        
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(), std::bind(&RealCameraScanNode::pc_callback, this, _1));

        viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("Real D435i Static Surface");
        viewer_->setBackgroundColor(0.05, 0.05, 0.05); 
        viewer_->addCoordinateSystem(0.1);             

        cv::namedWindow("Real RGB Stream", cv::WINDOW_AUTOSIZE);

        ui_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&RealCameraScanNode::update_ui, this));
    }

    ~RealCameraScanNode() { cv::destroyAllWindows(); }

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
    bool is_scanned_ = false; 

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_rgb_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {}
    }

    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (is_scanned_) return; 

        std::lock_guard<std::mutex> lock(data_mutex_);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *raw_cloud);
        if (raw_cloud->empty()) return;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*raw_cloud, *clean_cloud, indices);
        if (clean_cloud->empty()) return;

        // 体素降采样
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(clean_cloud); 
        vg.setLeafSize(config_.voxel_leaf_size, config_.voxel_leaf_size, config_.voxel_leaf_size); 
        vg.filter(*voxel_cloud);

        // 直通滤波 (放宽范围)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pt;
        pt.setInputCloud(voxel_cloud); 
        pt.setFilterFieldName("z"); 
        pt.setFilterLimits(config_.passthrough_min_z, config_.passthrough_max_z); 
        pt.filter(*pt_cloud); 
        if (pt_cloud->points.size() < 50) return; 

        // RANSAC 剔除桌面 (8mm 强力剔除)
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);     
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); 
        seg.setOptimizeCoefficients(true); 
        seg.setModelType(pcl::SACMODEL_PLANE); 
        seg.setMethodType(pcl::SAC_RANSAC);    
        seg.setMaxIterations(1000); 
        seg.setDistanceThreshold(config_.ransac_distance_threshold);        
        seg.setInputCloud(pt_cloud); 
        seg.segment(*inliers, *coefficients);  

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (inliers->indices.size() > 0) {
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(pt_cloud); 
            extract.setIndices(inliers); 
            extract.setNegative(true); 
            extract.filter(*objects_cloud);
        } else {
            objects_cloud = pt_cloud;
        }
        if (objects_cloud->points.size() < 50) return;

        // 欧式聚类提取主体 (1cm 容差)
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr cluster_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        cluster_tree->setInputCloud(objects_cloud);
        std::vector<pcl::PointIndices> cluster_indices; 
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(config_.cluster_tolerance); 
        ec.setMinClusterSize(config_.cluster_min_size); 
        ec.setMaxClusterSize(config_.cluster_max_size);
        ec.setSearchMethod(cluster_tree); 
        ec.setInputCloud(objects_cloud); 
        ec.extract(cluster_indices); 

        if (cluster_indices.empty()) return; 

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*objects_cloud, cluster_indices[0], *target_cloud);

        // 统计滤波去噪
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(target_cloud); 
        sor.setMeanK(config_.sor_mean_k); 
        sor.setStddevMulThresh(config_.sor_stddev_thresh); 
        sor.filter(*sor_cloud); 
        if (sor_cloud->points.size() < 50) return;

        RCLCPP_INFO(this->get_logger(), "成功锁定碳化硅工件主体！正在执行 MLS 平滑重构...");

        // MLS 曲面平滑与法向估计
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        mls.setComputeNormals(true); 
        mls.setInputCloud(sor_cloud); 
        mls.setPolynomialOrder(config_.mls_polynomial_order); 
        mls.setSearchMethod(tree); 
        mls.setSearchRadius(config_.mls_search_radius); 
        mls.process(*mls_cloud);

        std::vector<int> dummy;
        pcl::removeNaNNormalsFromPointCloud(*mls_cloud, *mls_cloud, dummy);

        pcl::copyPointCloud(*mls_cloud, *current_cloud_);
        pcl::copyPointCloud(*mls_cloud, *current_normals_);

        // 保存点云
        std::string save_dir = "src/real_point_cloud_data";
        if (!std::filesystem::exists(save_dir)) {
            std::filesystem::create_directories(save_dir);
        }
        std::string file_path = save_dir + "/target_cylinder_smoothed.pcd";
        pcl::io::savePCDFileASCII(file_path, *mls_cloud);
        
        RCLCPP_INFO(this->get_logger(), "🎉 真实点云处理完成！文件已保存至: %s", file_path.c_str());
        is_scanned_ = true; 
    }

    void update_ui() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!current_rgb_.empty()) { cv::imshow("Real RGB Stream", current_rgb_); }
        cv::waitKey(1); 
        if (!current_cloud_->empty() && !current_normals_->empty()) {
            if (is_first_cloud_) {
                viewer_->addPointCloud<pcl::PointXYZRGB>(current_cloud_, "cloud");
                viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
                viewer_->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(current_cloud_, current_normals_, config_.normal_display_step, config_.normal_display_scale, "normals");
                viewer_->setCameraPosition(0.0, 0.0, -0.5, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
                is_first_cloud_ = false;
            } 
        }
        viewer_->spinOnce(1, true); 
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealCameraScanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}