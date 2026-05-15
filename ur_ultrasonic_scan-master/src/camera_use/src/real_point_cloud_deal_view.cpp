/*
 * 模块：real_point_cloud_deal_view.cpp
 * 功能：读取并渲染已处理的真实 D435i 静态点云及其法向量
 */
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <X11/Xlib.h>

class RealPointCloudViewer : public rclcpp::Node
{
public:
    RealPointCloudViewer() : Node("real_point_cloud_deal_view")
    {
        RCLCPP_INFO(this->get_logger(), "👀 真实点云查看器已启动，正在读取数据...");
    }

    void run_viewer()
    {
        std::string file_path = "src/real_point_cloud_data/target_cylinder_smoothed.pcd";
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        // 读取 PCD 文件
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(file_path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "❌ 读取文件失败！请确认 %s 是否存在。", file_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ 成功读取真实点云，包含 %zu 个点。", cloud->size());

        // 初始化 PCL 查看器
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Real PointCloud & Normals Viewer"));
        viewer->setBackgroundColor(0.1, 0.1, 0.15); // 护眼深灰
        viewer->addCoordinateSystem(0.05);

        // 渲染彩色点云
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud, rgb, "real_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "real_cloud");

        // 渲染法向量 (每隔 5 个点画一根，长度 1cm)
        viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(cloud, cloud, 5, 0.01, "normals");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "normals"); // 绿色法线

        // 视角调整
        viewer->setCameraPosition(0.0, 0.0, -0.4, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0);

        // 渲染循环
        while (!viewer->wasStopped() && rclcpp::ok()) {
            viewer->spinOnce(100);
        }
    }
};

int main(int argc, char **argv)
{
    XInitThreads(); // 防止 PCL 在多线程环境下闪退
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RealPointCloudViewer>();
    node->run_viewer();
    
    rclcpp::shutdown();
    return 0;
}