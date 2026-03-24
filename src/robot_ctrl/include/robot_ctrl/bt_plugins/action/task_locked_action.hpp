#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace robot_ctrl
{

class SetTaskLockedAction : public BT::SyncActionNode
{
public:
  SetTaskLockedAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<bool>("task_locked")  // 输入布尔值
    };
  }

  BT::NodeStatus tick() override
  {
    bool locked = false;
    if (!getInput("task_locked", locked))
    {
      throw BT::RuntimeError("Missing required input [task_locked]");
    }

    // 把值重新写回黑板，强制为 bool
    config().blackboard->set("task_locked", locked);

    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace robot_ctrl
