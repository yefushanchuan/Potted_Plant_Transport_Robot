#ifndef ROBOT_CTRL__BT_PLUGINS__ACTION__CANCEL_NAV_TO_POSE_ACTION_HPP_
#define ROBOT_CTRL__BT_PLUGINS__ACTION__CANCEL_NAV_TO_POSE_ACTION_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robot_ctrl
{

/// @brief 通知 Nav2 取消当前正在执行的 NavigateToPose 任务
class CancelNavToPose : public BT::SyncActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  CancelNavToPose(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

}  // namespace robot_ctrl

#endif  // ROBOT_CTRL__BT_PLUGINS__ACTION__CANCEL_NAV_TO_POSE_ACTION_HPP_
