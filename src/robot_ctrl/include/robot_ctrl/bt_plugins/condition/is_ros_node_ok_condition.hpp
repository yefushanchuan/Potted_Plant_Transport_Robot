#pragma once

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"

namespace robot_ctrl
{

class IsRosNodeOk : public BT::ConditionNode
{
public:
  explicit IsRosNodeOk(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("node_name"),
             BT::InputPort<std::string>("topic_name") };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;   
};

} 
