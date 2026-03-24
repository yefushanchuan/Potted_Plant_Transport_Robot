#pragma once

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace robot_ctrl
{

class PatrolAdvanceIndexAction : public BT::SyncActionNode
{
public:
  PatrolAdvanceIndexAction(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace robot_ctrl

