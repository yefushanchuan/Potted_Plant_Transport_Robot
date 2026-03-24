#include "robot_ctrl/bt_plugins/condition/is_task_command_condition.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

IsCommand::IsCommand(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList IsCommand::providedPorts()
{
  return {
    BT::InputPort<std::string>("task_command"),
    BT::InputPort<std::string>("task_type")
  };
}

BT::NodeStatus IsCommand::tick()
{
  std::string cmd, type;

  if (!getInput("task_command", cmd)) {
    if (!missing_command_reported_)
    {
      ErrorLogQueue::instance().pushError(
        error_code::kInputMissing, "IsCommand: Missing input [task_command]");
      missing_command_reported_ = true;
    }
    return BT::NodeStatus::FAILURE;
  }
  missing_command_reported_ = false;

  if (!getInput("task_type", type)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[IsCommand] Missing input: task_type");
    if (!missing_type_reported_)
    {
      ErrorLogQueue::instance().pushError(
        error_code::kInputMissing, "IsCommand: Missing input [task_type]");
      missing_type_reported_ = true;
    }
    return BT::NodeStatus::FAILURE;
  }
  missing_type_reported_ = false;

  ErrorLogQueue::instance().clearLastErrorIfPrefix("IsCommand:");
  // RCLCPP_INFO(node_->get_logger(), "[IsCommand] Comparing '%s' == '%s'",
  //             cmd.c_str(), type.c_str());

  return (cmd == type) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace robot_ctrl
