#ifndef ROBOT_CTRL__SEND_ARM_GOAL_ACTION_HPP_
#define ROBOT_CTRL__SEND_ARM_GOAL_ACTION_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>
#include <mutex>
#include <chrono>

#include "arm_action/action/move_arm.hpp"

namespace robot_ctrl {

class SendArmGoalAction : public BT::StatefulActionNode {
public:
  using ArmCommand = arm_action::action::MoveArm;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ArmCommand>;

  SendArmGoalAction(const std::string& name, 
                   const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void processResult(const GoalHandle::WrappedResult& result);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ArmCommand>::SharedPtr client_;
  
  std::mutex result_mutex_;
  std::atomic<BT::NodeStatus> current_status_{BT::NodeStatus::IDLE};
  ArmCommand::Goal goal;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::Time start_time_;
  std::string last_error_;
};

} // namespace robot_ctrl

#endif // ROBOT_CTRL__SEND_ARM_GOAL_ACTION_HPP_