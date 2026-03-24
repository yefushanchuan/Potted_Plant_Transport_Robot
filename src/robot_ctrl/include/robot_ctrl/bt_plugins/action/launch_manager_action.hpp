#pragma once

#include <chrono>
#include <future>
#include <map>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_ctrl
{

class LaunchManagerAction : public BT::StatefulActionNode
{
public:
  LaunchManagerAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr getClient(const std::string & service_name);

  void resetPendingRequest();

  rclcpp::FutureReturnCode pollFuture();

  rclcpp::Node::SharedPtr node_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture pending_future_;
  std::string pending_service_;
  std::string pending_command_;
  std::chrono::steady_clock::time_point deadline_;
  std::chrono::milliseconds timeout_{5000};
};

}  // namespace robot_ctrl
