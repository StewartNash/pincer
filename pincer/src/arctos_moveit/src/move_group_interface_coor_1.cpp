#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

class MoveGroupDemo : public rclcpp::Node
{
public:
  MoveGroupDemo() : Node("arm")
  {
    auto logger = this->get_logger();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());

    const std::string planning_group = "arm";

    MoveGroupInterface move_group(shared_from_this(), planning_group);
    PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState(10.0)->getJointModelGroup(planning_group);

    move_group.setEndEffectorLink("Link_6_1");

    auto current_pose = move_group.getCurrentPose();

    RCLCPP_INFO(logger, "x: %f", current_pose.pose.position.x);
    RCLCPP_INFO(logger, "y: %f", current_pose.pose.position.y);
    RCLCPP_INFO(logger, "z: %f", current_pose.pose.position.z);

    moveit_visual_tools::MoveItVisualTools visual_tools(shared_from_this());
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;

    visual_tools.publishText(text_pose, "MoveGroupInterface Moveo Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.trigger();

    RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.120679;
    target_pose.position.y = 0.072992;
    target_pose.position.z = 0.569166;
    target_pose.orientation.x = -0.386473;
    target_pose.orientation.y = -0.418023;
    target_pose.orientation.z = -0.760978;
    target_pose.orientation.w = 0.311139;

    move_group.setPoseTarget(target_pose);

    MoveGroupInterface::Plan my_plan;
    auto success = static_cast<bool>(move_group.plan(my_plan));

    RCLCPP_INFO(logger, "Planning %s", success ? "succeeded" : "FAILED");

    visual_tools.publishAxisLabeled(target_pose, "target_pose");
    visual_tools.publishText(text_pose, "Pose Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Execute trajectory");

    move_group.move();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto demo_node = std::make_shared<MoveGroupDemo>();
  rclcpp::spin(demo_node);
  rclcpp::shutdown();
  return 0;
}

