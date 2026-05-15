#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/mesh.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur7e_manipulater");

    // Create collision object for the robot to avoid
    auto const collision_object = [frame_id =
                                    move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box_1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.2;
    primitive.dimensions[primitive.BOX_Y] = 0.2;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.25;
    box_pose.position.z = 0.05;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
    }();

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(collision_object);

        // [新增] 创建一个代表地面的碰撞对象
    auto const ground_object = [frame_id = move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "ground";  // 给地面一个唯一的ID

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        // 创建一个巨大且很薄的盒子作为地面，尺寸和位置可根据你的世界调整
        primitive.dimensions[primitive.BOX_X] = 10.0;  // 长 10米
        primitive.dimensions[primitive.BOX_Y] = 10.0;  // 宽 10米
        primitive.dimensions[primitive.BOX_Z] = 0.05;  // 高 (厚) 0.05米

        geometry_msgs::msg::Pose ground_pose;
        ground_pose.orientation.w = 1.0;
        // 将地面的上表面放在 z=0 处，这样机械臂的基座或其他部件如果z坐标小于0，就会被视为碰撞
        // 盒子中心在 z = -厚度/2 时，其上表面在z=0
        ground_pose.position.x = 0.0;
        ground_pose.position.y = 0.0;
        ground_pose.position.z = -0.025; // 因为厚度是0.05，中心在-0.025时，上表面在0

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(ground_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    // 应用地面对象到规划场景
    psi.applyCollisionObject(ground_object);
    RCLCPP_INFO(logger, "Ground collision object added to the planning scene.");




  // [修改点1] 移除重复的 waypoints 声明，只保留一个声明
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // 定义第一个目标点
  auto const waypoint1 = []{
      geometry_msgs::msg::Pose msg;
      // [修改点2] 设置正确的朝向，避免非法四元数（使用单位四元数表示无旋转）
      msg.orientation.w = 1.0;
      msg.orientation.x = 0.0;
      msg.orientation.y = 0.0;
      msg.orientation.z = 0.0;
      msg.position.x = 0.0;
      msg.position.y = 0.2;
      msg.position.z = 0.11;
      return msg;
  }();

  // 定义第二个目标点
  auto const waypoint2 = []{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;  // 保持相同朝向
      msg.orientation.x = 0.0;
      msg.orientation.y = 0.0;
      msg.orientation.z = 0.0;
      msg.position.x = 0.0;
      msg.position.y = 0.6;
      msg.position.z = 0.11;
      return msg;
  }();

    auto const waypoint3 = []{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;  // 保持相同朝向
      msg.orientation.x = 0.0;
      msg.orientation.y = 0.0;
      msg.orientation.z = 0.0;
      msg.position.x = 0.05;
      msg.position.y = 0.6;
      msg.position.z = 0.11;
      return msg;
  }();

      auto const waypoint4 = []{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;  // 保持相同朝向
      msg.orientation.x = 0.0;
      msg.orientation.y = 0.0;
      msg.orientation.z = 0.0;
      msg.position.x = 0.05;
      msg.position.y = 0.2;
      msg.position.z = 0.11;
      return msg;
  }();
  waypoints.push_back(waypoint1);
  waypoints.push_back(waypoint2);
  waypoints.push_back(waypoint3);
  waypoints.push_back(waypoint4);

  moveit_msgs::msg::RobotTrajectory trajectory;
  // computeCartesianPath 参数：路径点列表，步长（米），跳跃阈值（0.0禁用），返回的轨迹
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  if (fraction >= 0.9) {  // 90%以上路径成功
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      // 执行前最好先规划，但 computeCartesianPath 已经生成轨迹，可直接执行
      // 注意：需要确保 move_group 节点正在运行
      move_group_interface.execute(plan);
      RCLCPP_INFO(logger, "Execution successful");
  } else {
      RCLCPP_WARN(logger, "Cartesian path coverage too low: %.2f%%", fraction * 100.0);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}