#ifndef ROBOT_CTRL__BT_PLUGINS__ACTION__DOCK_ROBOT_ACTION_HPP_
#define ROBOT_CTRL__BT_PLUGINS__ACTION__DOCK_ROBOT_ACTION_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mutex>
#include <string>

#include "robot_charge/action/robot_dock.hpp"  

using DockRobot = robot_charge::action::RobotDock;
using GoalHandleDockRobot = rclcpp_action::ClientGoalHandle<DockRobot>;

namespace robot_ctrl
{

class DockRobotActionBT : public BT::StatefulActionNode
{
public:
  explicit DockRobotActionBT(const std::string & name,
                             const BT::NodeConfiguration & config);

  /* ---- 必须实现的静态接口 ---- */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("charge_enable"),
      BT::InputPort<bool>("charge_stop"),
      BT::OutputPort<std::string>("action_feedback"),
      BT::OutputPort<double>("progress")
    };
  }

  /* ---- StatefulActionNode 接口 ---- */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<DockRobot>::SharedPtr action_client_;
  DockRobot::Goal goal_;
  int send_goal_timeout_;
  GoalHandleDockRobot::SharedPtr goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<DockRobot>::WrappedResult> result_future_;
  std::mutex feedback_mutex_;
  std::string last_feedback_;
  double last_progress_{0.0};
};
}

#endif 
