#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/condition_node.h"

namespace robot_ctrl
{

class IsRobotStarted : public BT::ConditionNode
{
public:
  IsRobotStarted(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("robot_started")};
  }

private:
  bool missing_input_reported_{false};
};

}  
