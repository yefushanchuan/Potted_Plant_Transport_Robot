#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_ctrl/msg/robot_state.hpp"

namespace robot_ctrl
{

class PublishRobotState : public BT::SyncActionNode
{
public:
  PublishRobotState(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<robot_ctrl::msg::RobotState>::SharedPtr state_pub_;
};

}  // namespace robot_ctrl
