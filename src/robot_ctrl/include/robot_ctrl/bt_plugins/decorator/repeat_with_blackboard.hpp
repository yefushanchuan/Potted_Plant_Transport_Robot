#pragma once

#include <behaviortree_cpp_v3/decorator_node.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_ctrl
{

class RepeatWithBlackboard : public BT::DecoratorNode
{
public:
  RepeatWithBlackboard(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  bool infinite_loop_;
  int repeat_count_;
  int current_count_;
};

}  // namespace robot_ctrl
