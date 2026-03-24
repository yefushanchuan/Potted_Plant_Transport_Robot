#pragma once

#include <behaviortree_cpp_v3/decorator_node.h>

namespace robot_ctrl
{

class RetryUntilSuccessful : public BT::DecoratorNode
{
public:
  RetryUntilSuccessful(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("retry", "Maximum retry attempts") };
  }

protected:
  BT::NodeStatus tick() override;

private:
  int remaining_attempts_;
  bool initialized_;
};

} 
