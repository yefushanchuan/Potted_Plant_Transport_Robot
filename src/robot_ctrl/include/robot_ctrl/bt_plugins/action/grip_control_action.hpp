#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "robotcar_base/srv/set_control_mode.hpp"
#include <robotcar_base/msg/car_info.hpp>
#include <mutex>

namespace robot_ctrl
{
class GripControlAction : public BT::StatefulActionNode
{
public:
  GripControlAction(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<robotcar_base::srv::SetControlMode>::SharedPtr client_;
  rclcpp::Subscription<robotcar_base::msg::CarInfo>::SharedPtr sub_;

  std::mutex mutex_;
  int current_hand_cap_{0};
  int target_mode2_{0};
  rclcpp::Time start_time_;
};
}