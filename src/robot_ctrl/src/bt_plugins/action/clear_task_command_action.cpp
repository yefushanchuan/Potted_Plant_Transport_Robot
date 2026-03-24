#include "robot_ctrl/bt_plugins/action/clear_task_command_action.hpp"

#include <rclcpp/rclcpp.hpp>

namespace robot_ctrl
{

ClearTaskCommandNode::ClearTaskCommandNode(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList ClearTaskCommandNode::providedPorts()
{
    return {
        BT::OutputPort<std::string>("nav_source_room"),
        BT::OutputPort<std::string>("nav_target_room"),
        BT::OutputPort<std::string>("task_command"),
        BT::OutputPort<std::vector<std::string>>("target_pot_ids"),
        BT::OutputPort<std::vector<std::string>>("waypoint_rooms"),
        BT::OutputPort<int>("target_pot_num"),
        BT::OutputPort<std::string>("task_id"),
    };
}

BT::NodeStatus ClearTaskCommandNode::tick()
{
    RCLCPP_INFO(rclcpp::get_logger("ClearTaskCommandNode"), "Clearing task_command and nav_target_room");

    setOutput("task_command", "IDLE");
    setOutput("nav_source_room", "");
    setOutput("nav_target_room", "");
    setOutput("target_pot_ids", std::vector<std::string>{});
    setOutput("waypoint_rooms", std::vector<std::string>{});
    setOutput("target_pot_num", 0);
    setOutput("task_id", "");
    return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_ctrl
