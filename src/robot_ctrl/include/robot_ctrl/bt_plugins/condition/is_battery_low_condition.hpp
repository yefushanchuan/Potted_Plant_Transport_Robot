#pragma once

#include <string>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "behaviortree_cpp_v3/condition_node.h"

namespace robot_ctrl
{

class IsBatteryLow : public BT::ConditionNode
{
public:
  IsBatteryLow(const std::string & name, const BT::NodeConfiguration & config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  std::atomic_bool is_battery_low_{false};
  std::atomic<bool> has_msg_{false};
  std::atomic_bool missing_msg_reported_{false};
};

}  
