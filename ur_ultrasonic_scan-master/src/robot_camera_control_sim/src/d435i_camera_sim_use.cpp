#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL 基础与 IO (保存 PCD 功能)
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 

// PCL 滤波与分割算法
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>               
#include <pcl/segmentation/extract_clusters.h>

// PCL MLS (移动最小二乘法) 平滑与法线估计
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>                

#include <memory>

using std::placeholders::_1;

// =========================================================================
// ⭐ 核心算法参数配置区 (工业调参面板)
// =========================================================================
struct FilterConfig {
    // 1. 体素降采样网格大小 (单位：米)
    // 作用：将空间划分为一个个小方块，每个方块内只保留一个点，用于压缩数据量。
    // 调节效果：调大(如0.01)会使点云变稀疏，计算极快，但会丢失极小特征（如1mm薄片）。
    //          调小(如0.001)会保留极其精细的特征，但计算量呈指数级爆炸。当前 0.005f 是速度与精度的完美平衡。
    float voxel_leaf_size = 0.005f; 

    // 2. 直通滤波 Z轴范围 (单位：米)
    // 作用：只保留距离相机镜头 [min_z, max_z] 范围内的点云，像切西瓜一样把背景和前景切掉。
    // 调节效果：min_z 调大可以切掉在镜头前挥舞的机械臂；max_z 调小可以切掉远处的地面或墙壁。
    float passthrough_min_z = 0.8f;
    float passthrough_max_z = 1.25f;

    // 3. RANSAC 桌面剔除距离阈值 (单位：米)
    // 作用：判断一个点到底算不算“桌面”的容忍距离。
    // 调节效果：调大(如0.05)会把离桌面 5cm 以内的所有东西都当成桌面删掉（致命：会把薄工件误杀）。
    //          调小(如0.01)要求极其严格，只删绝对平面，能完美保住底部的 1mm/5mm 薄片工件。
    float ransac_distance_threshold = 0.01f;
    
    // 4. 欧式聚类容差与大小限制
    // cluster_tolerance (单位：米): 两个点距离小于此值，就认为它们属于同一个物体。
    // 调节效果：如果工件表面反光导致点云中间有空洞（断层），必须调大此值(如0.03)，强行把断开的两半聚合成一个物体。调得太小会导致工件碎成好几块。
    float cluster_tolerance = 0.02f;
    // 限制聚类的最小和最大点数。过滤掉太小的飞线团块，和太大的异常背景。
    int cluster_min_size = 100;
    int cluster_max_size = 25000;

    // 5. 统计滤波 (SOR) 去除离群飞线
    // sor_mean_k: 考察周围多少个邻居点。
    // sor_stddev_thresh: 标准差倍数阈值。
    // 调节效果：sor_stddev_thresh 调小(如0.5)，去噪极其残暴，连工件合法的直角锐边也会被当成噪点啃掉（边缘坍缩）。
    //          调大(如2.0)，去噪温和，能保住直角边缘，但可能会残留少量微小飞线。
    int sor_mean_k = 50;              
    float sor_stddev_thresh = 1.0f;   

    // 6. MLS (移动最小二乘法) 表面平滑与法线计算 (⭐ 方案2 核心参数)
    // mls_search_radius (单位：米): 拟合平滑曲面时，参考周围多大半径内的点。
    // 调节效果：调大(如0.05)，表面会变得像镜面一样极致丝滑，但工件边缘会因为“周边点不够”而严重坍缩内缩。
    //          调小(如0.015)，贴合真实物理形状，完美保留边缘面积，防止点云缩水。
    float mls_search_radius = 0.015f; 
    int mls_polynomial_order = 2; // 拟合多项式的阶数，通常 2 阶即可拟合绝大多数工业曲面。

    // 7. RViz/PCL 可视化参数 (仅影响肉眼观看，不影响计算)
    int normal_display_step = 3;      // 每隔 3 个点画一条法线（防止法线太密看不清）
    float normal_display_scale = 0.02f; // 法线箭头的长度 (2cm)
};

class CameraSimUseNode : public rclcpp::Node
{
public:
    CameraSimUseNode() : Node("d435i_camera_sim_use")
    {
        RCLCPP_INFO(this->get_logger(), "视觉感知节点已启动：单帧处理保存模式 (无本地可视化窗口)...");

        // 只订阅对齐后的深度点云。可视化窗口由独立查看节点手动启动，避免 PCL/VTK 生命周期冲突。
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10, std::bind(&CameraSimUseNode::pc_callback, this, _1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    FilterConfig config_;   

    // ⭐ 单帧扫描锁：保证“拍照-处理-保存”这套沉重的动作只执行一次，解放 CPU 算力
    bool is_scanned_ = false; 

    // ⭐ 核心流水线：点云接收与处理回调
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 锁门：如果已经保存过点云了，直接 Return，从此相机节点只做 UI 渲染，不再消耗算力
        if (is_scanned_) return; 
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *raw_cloud);
        if (raw_cloud->empty()) return;

        // 【阶段 1：数据清洗】去除无效的 NaN 点 (由于深度相机红外吸收/反射导致的黑洞)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*raw_cloud, *clean_cloud, indices);
        if (clean_cloud->empty()) return;

        // 【阶段 2：体素网格降采样】使点云分布均匀，极大降低后续计算量
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(clean_cloud); 
        vg.setLeafSize(config_.voxel_leaf_size, config_.voxel_leaf_size, config_.voxel_leaf_size); 
        vg.filter(*voxel_cloud);

        // 【阶段 3：空间裁剪】一刀切除工作台以外的无关背景
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pt;
        pt.setInputCloud(voxel_cloud);
        pt.setFilterFieldName("z"); 
        pt.setFilterLimits(config_.passthrough_min_z, config_.passthrough_max_z); 
        pt.filter(*pt_cloud); 
        if (pt_cloud->points.size() < 10) return; 

        // 【阶段 4：RANSAC 桌面剥离】寻找最大平面(通常是桌面)，并将它从点云中剔除
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

        // 提取非桌面的其余部分 (即悬浮在桌上的所有物体)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(pt_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true); 
        extract.filter(*objects_cloud);
        if (objects_cloud->points.size() < 10) return;

        // 【阶段 5：欧式聚类】在空间中寻找互相连接的最大点云块 (真正锁定目标工件)
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

        // 默认体积最大的聚类块 (index 0) 就是我们的检测工件
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_workpiece_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*objects_cloud, cluster_indices[0], *target_workpiece_cloud);

        // 【阶段 6：SOR 统计去噪】像打磨砂纸一样，抹去工件周围零星的游离噪点
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(target_workpiece_cloud); 
        sor.setMeanK(config_.sor_mean_k);
        sor.setStddevMulThresh(config_.sor_stddev_thresh);
        sor.filter(*sor_cloud); 

        if (sor_cloud->points.size() < 50) return;

        RCLCPP_INFO(this->get_logger(), "成功锁定工件！正在执行防边缘坍缩 MLS 平滑计算...");

        // 【阶段 7：MLS 曲面重构与法向计算】超级核心！赋予每一个点空间矢量方向，指导机械臂姿态
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        
        mls.setComputeNormals(true); 
        mls.setInputCloud(sor_cloud);
        mls.setPolynomialOrder(config_.mls_polynomial_order); 
        mls.setSearchMethod(tree);
        mls.setSearchRadius(config_.mls_search_radius); 
        mls.process(*mls_cloud);

        // 防御性编程：MLS 有极小概率生成方向未知的 NaN 法线，必须剔除，否则 MoveIt 收到 NaN 位姿会直接崩溃！
        std::vector<int> dummy_indices;
        pcl::removeNaNNormalsFromPointCloud(*mls_cloud, *mls_cloud, dummy_indices);

        // 【阶段 8：固化成果】将带有 XYZ、RGB 和 法向向量(Normal) 的 9D 点云写入本地硬盘
        std::string file_name = "target_cylinder_smoothed.pcd";
        pcl::io::savePCDFileASCII(file_name, *mls_cloud);
        RCLCPP_INFO(this->get_logger(), "🎉 处理完成！点云已保存: %s", file_name.c_str());

        // 锁死门栓
        is_scanned_ = true; 
        rclcpp::shutdown();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSimUseNode>();
    rclcpp::spin(node);
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
