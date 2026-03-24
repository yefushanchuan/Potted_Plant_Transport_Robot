#include "robot_ctrl/bt_plugins/action/set_robot_state_action.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"
#include "robot_ctrl/robot_state_utils.hpp"

namespace robot_ctrl
{

SetRobotState::SetRobotState(const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList SetRobotState::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_state", "Robot working state (e.g., IDLE, NAVIGATING, CHARGING)")
  };
}

BT::NodeStatus SetRobotState::tick()
{
  auto state = getInput<std::string>("robot_state");
  if (!state)
  {
    RCLCPP_ERROR(node_->get_logger(), "[BT] Missing input [robot_state]");
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "SetRobotState: Missing input [robot_state]");
    return BT::NodeStatus::FAILURE;
  }

  robot_ctrl::msg::RobotState state_msg;
  std::string error_label;
  if (!robot_state::fromString(state.value(), state_msg, &error_label))
  {
    RCLCPP_ERROR(node_->get_logger(), "[BT] Unknown robot_state %s", state->c_str());
    ErrorLogQueue::instance().pushError(
      error_code::kInvalidParam,
      "SetRobotState: Unknown robot_state " + state.value());
    return BT::NodeStatus::FAILURE;
  }

  config().blackboard->set("robot_state", state_msg.label);
  config().blackboard->set("robot_state_msg", state_msg);

  RCLCPP_INFO(node_->get_logger(), "[BT] Robot state set to: %s", state_msg.label.c_str());
  ErrorLogQueue::instance().clearLastErrorIfPrefix("SetRobotState:");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_ctrl
