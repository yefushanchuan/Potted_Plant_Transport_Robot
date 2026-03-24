#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_ctrl/srv/get_roompose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_ctrl
{
class GetNavGoalFromRoom : public BT::StatefulActionNode
{
public:
  GetNavGoalFromRoom(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  /* ===== 状态机接口 ===== */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void           onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<robot_ctrl::srv::GetRoompose>::SharedPtr client_;
  std::string room_name;

  /* 异步调用相关状态 */
  std::shared_future<robot_ctrl::srv::GetRoompose::Response::SharedPtr> future_;
};
}  // namespace robot_ctrl