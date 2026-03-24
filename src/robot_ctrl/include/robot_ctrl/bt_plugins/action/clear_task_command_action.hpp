#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <string>

namespace robot_ctrl
{

class ClearTaskCommandNode : public BT::SyncActionNode
{
public:
    ClearTaskCommandNode(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

}  // namespace robot_ctrl
