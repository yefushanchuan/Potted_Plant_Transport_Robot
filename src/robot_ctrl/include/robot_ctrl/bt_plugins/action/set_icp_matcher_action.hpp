#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <std_srvs/srv/set_bool.hpp>
#include "rclcpp/rclcpp.hpp"

namespace robot_ctrl
{
class SetIcpActive : public BT::StatefulActionNode
{
public:
  SetIcpActive(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  /* ===== 状态机接口 ===== */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void           onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;

  /* 异步调用相关状态 */
  std::shared_future<std::shared_ptr<std_srvs::srv::SetBool::Response>> future_;
};
}  // namespace robot_ctrl