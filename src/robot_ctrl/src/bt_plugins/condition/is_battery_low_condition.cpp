#include <functional>
#include "robot_ctrl/bt_plugins/condition/is_battery_low_condition.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

IsBatteryLow::IsBatteryLow(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery_state", 10,
    std::bind(&IsBatteryLow::batteryCallback, this, std::placeholders::_1));
}

void IsBatteryLow::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  is_battery_low_.store(
    msg->percentage < 0.2,
    std::memory_order_relaxed);

  has_msg_.store(true, std::memory_order_release);
  missing_msg_reported_.store(false, std::memory_order_relaxed);
  ErrorLogQueue::instance().clearLastErrorIfPrefix("IsBatteryLow:");

  // RCLCPP_INFO(node_->get_logger(), "received battery_state: %d",
  //             is_battery_low_.load());
}


BT::NodeStatus IsBatteryLow::tick()
{
  if (!has_msg_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(node_->get_logger(), "[BT] Battery status not received yet");
    if (!missing_msg_reported_.exchange(true, std::memory_order_relaxed))
    {
      ErrorLogQueue::instance().pushError(
        error_code::kDataMissing, "IsBatteryLow: Battery status not received yet");
    }
    return BT::NodeStatus::FAILURE;
  }
  missing_msg_reported_.store(false, std::memory_order_relaxed);

  return is_battery_low_.load(std::memory_order_relaxed)
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}


}  // namespace robot_ctrl
