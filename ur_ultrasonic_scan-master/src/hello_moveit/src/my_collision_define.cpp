#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("my_collision_define");
  auto logger = node->get_logger();

  // 等待 move_group 完全启动（可选）
  rclcpp::sleep_for(std::chrono::seconds(2));

  moveit::planning_interface::PlanningSceneInterface psi;

  // 添加地面
  moveit_msgs::msg::CollisionObject ground;
  ground.header.frame_id = "world";
  ground.id = "ground";
  shape_msgs::msg::SolidPrimitive ground_primitive;
  ground_primitive.type = ground_primitive.BOX;
  ground_primitive.dimensions = {10.0, 10.0, 0.05};
  ground.primitives.push_back(ground_primitive);
  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.z = -0.025;  // 上表面在z=0
  ground.primitive_poses.push_back(ground_pose);
  ground.operation = ground.ADD;
  psi.applyCollisionObject(ground);
  RCLCPP_INFO(logger, "Ground added.");

  // 添加一个盒子
  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = "world";
  box.id = "cyliden_1";
  shape_msgs::msg::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.CYLINDER;
  box_primitive.dimensions = {0.15, 0.1};
  box.primitives.push_back(box_primitive);
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.05;
  box.primitive_poses.push_back(box_pose);
  box.operation = box.ADD;
  psi.applyCollisionObject(box);
  RCLCPP_INFO(logger, "Box added.");

  // 保持节点运行一小段时间，确保消息被处理
  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::shutdown();
  return 0;
}