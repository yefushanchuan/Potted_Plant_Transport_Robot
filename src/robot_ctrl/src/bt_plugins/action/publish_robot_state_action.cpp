#include "robot_ctrl/bt_plugins/action/publish_robot_state_action.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"
#include "robot_ctrl/robot_state_utils.hpp"

namespace robot_ctrl
{

PublishRobotState::PublishRobotState(const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  state_pub_ = node_->create_publisher<robot_ctrl::msg::RobotState>("/robot_state", 10);
  RCLCPP_INFO(node_->get_logger(), "[BT] PublishRobotState node initialized.");
}

BT::PortsList PublishRobotState::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_state", "Robot working state from blackboard")
  };
}

BT::NodeStatus PublishRobotState::tick()
{
  robot_ctrl::msg::RobotState state_msg;
  bool have_msg = config().blackboard->get("robot_state_msg", state_msg);

  if (!have_msg)
  {
    std::string state_value;
    if (config().blackboard->get("robot_state", state_value))
    {
      if (!robot_state::fromString(state_value, state_msg))
      {
        RCLCPP_ERROR(node_->get_logger(), "[BT] Unknown robot_state %s", state_value.c_str());
        ErrorLogQueue::instance().pushError(
          error_code::kInvalidParam,
          "PublishRobotState: Unknown robot_state " + state_value);
        return BT::NodeStatus::FAILURE;
      }
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "[BT] No robot_state found on blackboard.");
      ErrorLogQueue::instance().pushError(
        error_code::kInputMissing,
        "PublishRobotState: No robot_state found on blackboard");
      return BT::NodeStatus::FAILURE;
    }
  }

  state_pub_->publish(state_msg);
  ErrorLogQueue::instance().clearLastErrorIfPrefix("PublishRobotState:");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_ctrl
