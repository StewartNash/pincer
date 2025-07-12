#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "arctos_moveit/msg/arm_joint_state.hpp"
#include <cmath>

using std::placeholders::_1;

class MoveitConvertNode : public rclcpp::Node
{
public:
  MoveitConvertNode()
  : Node("moveit_convert")
  {
    RCLCPP_INFO(this->get_logger(), "Starting moveit_convert node");

    steps_per_revolution_ = {32800, 18000, 72000, 3280, 14400, 0};
    prev_angle_.fill(0.0);
    init_angle_.fill(0.0);
    total_steps_.fill(0.0);

    subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/move_group/fake_controller_joint_states", 10,
      std::bind(&MoveitConvertNode::cmdCallback, this, _1)
    );

    publisher_ = this->create_publisher<arctos_moveit::msg::ArmJointState>("joint_steps", 10);
  }

private:
  void cmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!initialized_)
    {
      for (int i = 0; i < 6; ++i)
      {
        prev_angle_[i] = msg->position[i];
        init_angle_[i] = msg->position[i];
      }
      initialized_ = true;
    }

    arctos_moveit::msg::ArmJointState step_msg;

    step_msg.position1 = toSteps(0, msg->position[0] - prev_angle_[0]);
    step_msg.position2 = toSteps(1, msg->position[1] - prev_angle_[1]);
    step_msg.position3 = toSteps(2, msg->position[2] - prev_angle_[2]);
    step_msg.position4 = toSteps(3, msg->position[3] - prev_angle_[3]);
    step_msg.position5 = toSteps(4, msg->position[4] - prev_angle_[4]);
    step_msg.position6 = toSteps(5, msg->position[5] - prev_angle_[5]);

    // Optional: Keep track of running total
    total_msg_.position1 += step_msg.position1;
    total_msg_.position2 += step_msg.position2;
    total_msg_.position3 += step_msg.position3;
    total_msg_.position4 += step_msg.position4;
    total_msg_.position5 += step_msg.position5;

    publisher_->publish(total_msg_);

    for (int i = 0; i < 6; ++i)
      prev_angle_[i] = msg->position[i];

    RCLCPP_INFO(this->get_logger(), "Published joint step update.");
  }

  int toSteps(int joint_index, double delta_rad)
  {
    return static_cast<int>((delta_rad * steps_per_revolution_[joint_index]) / (2 * M_PI));
  }

  std::array<int, 6> steps_per_revolution_;
  std::array<double, 6> prev_angle_;
  std::array<double, 6> init_angle_;
  std::array<double, 6> total_steps_;
  bool initialized_ = false;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
  rclcpp::Publisher<arctos_moveit::msg::ArmJointState>::SharedPtr publisher_;
  arctos_moveit::msg::ArmJointState total_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveitConvertNode>());
  rclcpp::shutdown();
  return 0;
}

