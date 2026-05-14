#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

int main(int argc, char** argv)
{
    // 1. 明确要读取的文件名
    std::string file_name = "target_cylinder_smoothed.pcd";

    // 2. 声明点云容器
    // 注意：因为我们保存时用的是带有法向量的格式，所以这里必须用 PointXYZRGBNormal
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // 3. 读取文件
    std::cout << "正在读取文件: " << file_name << " ..." << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(file_name, *cloud) == -1)
    {
        PCL_ERROR("未能找到文件，请检查运行终端的当前路径下是否存在该 .pcd 文件！\n");
        return -1;
    }
    std::cout << "读取成功！共包含 " << cloud->points.size() << " 个数据点。" << std::endl;

    // 4. 初始化 PCL 可视化窗口
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Static PCD Viewer"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1); // 深灰色背景
    viewer->addCoordinateSystem(0.1);          // 添加长度为 0.1m 的 XYZ 坐标轴

    // 5. 将点云添加到窗口
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

    // 6. 添加法向量显示
    // 参数含义：(点云数据, 每隔3个点显示一条, 法向量长度为0.02m, 标识符)
    viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(cloud, cloud, 3, 0.02, "normals");

    // 设置初始相机视角 (摆正画面)
    viewer->setCameraPosition(0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);

    // 7. 保持窗口开启循环
    std::cout << "按 [Ctrl+C] 或直接关闭窗口退出程序。" << std::endl;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}