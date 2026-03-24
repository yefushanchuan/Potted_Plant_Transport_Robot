#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_ctrl/msg/robot_state.hpp"

namespace robot_ctrl
{

class SetRobotState : public BT::SyncActionNode
{
public:
  SetRobotState(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace robot_ctrl
