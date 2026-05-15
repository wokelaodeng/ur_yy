#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp> // 接收带状态的消息

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h> // 引入生成网格所需的头文件

#include <mutex>
#include <X11/Xlib.h> // 防止多线程闪退
#include <cmath> // 引入数学库计算圆的轨迹

// ===============================
// 全局变量区
// ===============================
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
std::mutex cloud_mutex;

// ===============================
// ROS 2 节点类：负责接收扫描数据
// ===============================
class ScanVisualizer : public rclcpp::Node
{
public:
    ScanVisualizer() : Node("pcl_scan_workpiece_window")
    {
        // 订阅 scan_point，接收 PointStamped 消息
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "scan_point", 100,
            std::bind(&ScanVisualizer::pointCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "PCL 显示窗口启动，等待扫描数据...");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;

    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex);

        pcl::PointXYZRGB p;
        p.x = msg->point.x;
        p.y = msg->point.y;
        p.z = msg->point.z;

        // 根据传输过来的帧 ID 解析健康状态
        // "0" 代表健康（白色），"1" 代表不健康（红色）
        if (msg->header.frame_id == "0")
        {
            p.r = 255; p.g = 255; p.b = 255; // 白色
        }
        else if (msg->header.frame_id == "1")
        {
            p.r = 255; p.g = 0; p.b = 0;     // 红色
        }
        else 
        {
            p.r = 0; p.g = 255; p.b = 0;     // 兜底色 (绿色)
        }

        cloud->points.push_back(p);
    }
};

// ===============================
// 主函数
// ===============================
int main(int argc, char **argv)
{
    // 初始化 X11 多线程机制，根治 PCL 段错误闪退
    XInitThreads();

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanVisualizer>();

    // ===============================
    // Viewer 必须在主线程创建和运行
    // ===============================
    pcl::visualization::PCLVisualizer viewer("实时 3D 扫描系统窗口");
    viewer.setBackgroundColor(0.1, 0.1, 0.15); // 护眼深灰背景

    // 调整相机初始视角
    viewer.setCameraPosition(0.8, -0.6, 0.5,   
                             0.30, 0.0, 0.05,  
                             0.0, 0.0, 1.0);  

    // ===============================
    // [终极重构] 生成并绘制精准尺寸的半透明圆柱体 Mesh (代替长方体)
    // ===============================
    // 1. 定义圆柱体参数 (对齐我们在 Gazebo 中的放置位置和尺寸)
    double radius = 0.15; // 半径 15cm
    double height = 0.10; // 高度 10cm
    double cx = 0.40;     // 物理世界中心 X
    double cy = 0.00;     // 物理世界中心 Y
    int segments = 48;    // 将圆切分为 48 段，使其足够平滑

    // 2. 创建一个空的 PointCloud，用于存放圆柱体的顶点
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    mesh_cloud->points.resize(segments * 2); // 顶面 segments 个点 + 底面 segments 个点

    for (int i = 0; i < segments; ++i)
    {
        double angle = i * 2.0 * M_PI / segments;
        // 计算圆周上的 X, Y 坐标
        double dx = radius * cos(angle);
        double dy = radius * sin(angle);

        // 底面点：X, Y 坐标 + Z=0
        mesh_cloud->points[i] = pcl::PointXYZ(cx + dx, cy + dy, 0.0);
        // 顶面点：X, Y 坐标 + Z=height
        mesh_cloud->points[i + segments] = pcl::PointXYZ(cx + dx, cy + dy, height);
    }

    // 3. 将顶点添加进视图，作为一个隐形的点云参考
    viewer.addPointCloud(mesh_cloud, "mesh_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, "mesh_cloud"); // 隐藏参考点

    // 4. 数学绘图法：手动绘制顶面圆、底面圆和侧面柱子
    RCLCPP_INFO(node->get_logger(), "正在为圆柱体骨架应用半透明木纹纹理...");
    for (int i = 0; i < segments; ++i)
    {
        // 获取当前顶点对和下一个顶点对的索引
        int current_idx = i;
        int next_idx = (i + 1) % segments; // 实现圆周首尾相连

        // 绘制侧面：连接底面圆弧和顶面圆弧 (addLine)
        viewer.addLine(mesh_cloud->points[current_idx], mesh_cloud->points[current_idx + segments], "side_line_" + std::to_string(i));
        // 绘制底面圆弧 (addLine)
        viewer.addLine(mesh_cloud->points[current_idx], mesh_cloud->points[next_idx], "bottom_line_" + std::to_string(i));
        // 绘制顶面圆弧 (addLine)
        viewer.addLine(mesh_cloud->points[current_idx + segments], mesh_cloud->points[next_idx + segments], "top_line_" + std::to_string(i));
    }

    // ===============================
    // [核心修改] 统一应用半透明木纹颜色 (Browns/Tan tones)
    // 彻底废除 addCube，改用 PCL 统一颜色接口
    // ===============================
    // 1. 获取所有以 "line_" 结尾的几何体 ID (包含侧面、顶面、底面线条)
    std::vector<std::string> shape_ids;
    for (int i = 0; i < segments; ++i) {
        shape_ids.push_back("side_line_" + std::to_string(i));
        shape_ids.push_back("bottom_line_" + std::to_string(i));
        shape_ids.push_back("top_line_" + std::to_string(i));
    }

    // 2. 批量应用颜色和半透明度
    for (const auto& id : shape_ids) {
        // 设置颜色：使用类似木材的褐色/黄褐色 (RGB: 205, 133, 63 -> Peru color)
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 205.0/255.0, 133.0/255.0, 63.0/255.0, id);
        // 设置半透明度：0.3 (30% 不透明)，使其足够透明能看清内部点云
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, id);
    }

    // 添加坐标系作为参考轴
    viewer.addCoordinateSystem(0.2);

    rclcpp::WallRate loop_rate(30);

    // ===============================
    // 渲染主循环
    // ===============================
    while (rclcpp::ok() && !viewer.wasStopped())
    {
        // 处理 ROS 回调，接收点云数据
        rclcpp::spin_some(node);

        {
            std::lock_guard<std::mutex> lock(cloud_mutex);
            if (!cloud->empty())
            {
                // 如果视图中已经有点云了，先移除老的
                if (viewer.contains("cloud"))
                    viewer.removePointCloud("cloud");

                // 添加包含 RGB 颜色信息的最新点云
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
                viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");

                // 设置较大的点尺寸，清晰展示红白轨迹
                viewer.setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud");
            }
        }

        // 刷新 PCL 窗口并短暂休眠
        viewer.spinOnce(10);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}