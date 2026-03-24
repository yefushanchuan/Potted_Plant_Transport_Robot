#pragma once
#include <string>
#include <memory>
#include <chrono>
#include <future>
#include <mutex>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"
#include "robot_charge/action/robot_dock.hpp"  
#include "rclcpp_action/rclcpp_action.hpp"

namespace robot_ctrl
{

class UndockRobotActionBT : public BT::StatefulActionNode
{
public:
  using UndockRobot = robot_charge::action::RobotDock;
  using GoalHandleUndockRobot = rclcpp_action::ClientGoalHandle<UndockRobot>;

  UndockRobotActionBT(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("charge_stop"),
      BT::InputPort<bool>("charge_enable"),
      BT::OutputPort<std::string>("action_feedback"),
      BT::OutputPort<double>("progress")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  UndockRobot::Goal goal_;
  rclcpp_action::Client<UndockRobot>::SharedPtr action_client_;
  GoalHandleUndockRobot::SharedPtr goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<UndockRobot>::WrappedResult> result_future_;
  int send_goal_timeout_;  // ms
  std::mutex feedback_mutex_;
  std::string last_feedback_;
  double last_progress_{0.0};
};

}  // namespace robot_ctrl
