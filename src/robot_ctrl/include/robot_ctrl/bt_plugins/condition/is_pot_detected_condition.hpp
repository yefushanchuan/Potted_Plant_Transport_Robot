#pragma once

#include <atomic>
#include <mutex>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "pot_detection/msg/pots_info.hpp"

namespace robot_ctrl
{

class IsPotDetected : public BT::ConditionNode
{
public:
  IsPotDetected(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void potsInfoCallback(const pot_detection::msg::PotsInfo::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<pot_detection::msg::PotsInfo>::SharedPtr pots_sub_;

  std::mutex mutex_;
  pot_detection::msg::PotsInfo::SharedPtr latest_msg_;
  std::atomic_bool missing_msg_reported_{false};
};

}  // namespace robot_ctrl
