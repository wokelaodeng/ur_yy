#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.h>
#include <geometry_msgs/msg/pose.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("my_collision_define");
    auto logger = node->get_logger();

    // 等待 move_group 完全启动，确保规划场景接收器已就绪
    rclcpp::sleep_for(std::chrono::seconds(2));

    moveit::planning_interface::PlanningSceneInterface psi;

    // =======================================================
    // 仅添加动态工件：圆柱体 (Cylinder)
    // 地面和光学平台已在 URDF 中被原生支持，无需在此重复添加
    // =======================================================
    moveit_msgs::msg::CollisionObject cylinder;
    cylinder.header.frame_id = "world";
    cylinder.id = "cylinder_workpiece"; // 命名更加规范

    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = cylinder_primitive.CYLINDER;
    // 尺寸定义：高 0.10m，半径 0.15m (严格遵循先高度后半径的顺序)
    cylinder_primitive.dimensions = {0.10, 0.15};  
    cylinder.primitives.push_back(cylinder_primitive);

    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.orientation.w = 1.0;
    // 工件放置位置：相对于 world。MoveIt 圆柱 pose 是中心点。
    cylinder_pose.position.x = 0.15;
    cylinder_pose.position.y = -0.4;
    cylinder_pose.position.z = 0.90; // 平台表面 Z=0.85 + 圆柱半高 0.05
    cylinder.primitive_poses.push_back(cylinder_pose);

    cylinder.operation = cylinder.ADD;
    psi.applyCollisionObject(cylinder);
    
    RCLCPP_INFO(logger, "✅ 圆柱体待扫描工件已成功放置于光学平台上 (center: X=0.15, Y=-0.40, Z=0.90)");

    // 保持节点运行一小段时间，确保 ADD 消息被 MoveIt 成功接收
    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
    return 0;
}
