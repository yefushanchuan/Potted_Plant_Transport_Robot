#include <functional>
#include "robot_ctrl/bt_plugins/condition/is_charging_condition.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

IsCharging::IsCharging(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery_state", 10,
    std::bind(&IsCharging::batteryCallback, this, std::placeholders::_1));
}

void IsCharging::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  is_battery_charging_.store(
    msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING,
    std::memory_order_relaxed);

  has_msg_.store(true, std::memory_order_release);
  missing_msg_reported_.store(false, std::memory_order_relaxed);
  ErrorLogQueue::instance().clearLastErrorIfPrefix("IsCharging:");

  // RCLCPP_INFO(node_->get_logger(), "received battery_state: %d",
  //             is_battery_charging_.load());
}


BT::NodeStatus IsCharging::tick()
{
  if (!has_msg_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(node_->get_logger(), "[BT] Battery status not received yet");
    if (!missing_msg_reported_.exchange(true, std::memory_order_relaxed))
    {
      ErrorLogQueue::instance().pushError(
        error_code::kDataMissing, "IsCharging: Battery status not received yet");
    }
    return BT::NodeStatus::FAILURE;
  }
  missing_msg_reported_.store(false, std::memory_order_relaxed);

  return is_battery_charging_.load(std::memory_order_relaxed)
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}


}  // namespace robot_ctrl
