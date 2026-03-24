#include "robot_ctrl/bt_plugins/condition/is_robot_init_success_condition.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

BT::NodeStatus IsRobotInitSuccess::tick()
{
  auto robot_init_success = getInput<bool>("robot_init_success");
  if (!robot_init_success.has_value())
  {
    RCLCPP_ERROR(rclcpp::get_logger("IsRobotInitSuccess"),
                "Failed to get [robot_init_success] from blackboard: %s",
                robot_init_success.error().c_str());
    if (!missing_input_reported_)
    {
      ErrorLogQueue::instance().pushError(
        error_code::kInputMissing,
        "IsRobotInitSuccess: Missing input [robot_init_success] (" +
        robot_init_success.error() + ")");
      missing_input_reported_ = true;
    }
    return BT::NodeStatus::FAILURE;
  }
  missing_input_reported_ = false;
  ErrorLogQueue::instance().clearLastErrorIfPrefix("IsRobotInitSuccess:");
  return robot_init_success.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace robot_ctrl
