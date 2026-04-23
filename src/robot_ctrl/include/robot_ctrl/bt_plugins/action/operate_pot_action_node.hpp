#ifndef ROBOT_CTRL__OPERATE_POT_ACTION_NODE_HPP_
#define ROBOT_CTRL__OPERATE_POT_ACTION_NODE_HPP_

#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "robot_base/action/operate_pot.hpp"

namespace robot_ctrl
{

class OperatePotAction 
  : public nav2_behavior_tree::BtActionNode<robot_base::action::OperatePot>
{
public:
  using ActionT = robot_base::action::OperatePot;

  OperatePotAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<ActionT>(xml_tag_name, action_name, conf)
  {}

  // BT 输入输出端口定义
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<int>("action_type", "动作类型: 1(搬运), 2(卸载), 3(slave)"),
      BT::InputPort<int>("rack_index", "架子索引"),
      BT::OutputPort<std::string>("message", "底层返回的信息")
    });
  }

  // 发送 goal 前调用
  void on_tick() override
  {
    int action_type = 0;
    int rack_index = 0;

    if (!getInput("action_type", action_type)) {
      RCLCPP_WARN(node_->get_logger(), "Missing input [action_type], using default 0");
    }

    if (!getInput("rack_index", rack_index)) {
      RCLCPP_WARN(node_->get_logger(), "Missing input [rack_index], using default 0");
    }

    goal_.action_type = action_type;
    goal_.rack_index = rack_index;

    RCLCPP_INFO(node_->get_logger(),
                "OperatePot Goal: type=%d, rack=%d",
                action_type, rack_index);
  }

  // 成功回调
  BT::NodeStatus on_success() override
  {
    RCLCPP_INFO(node_->get_logger(), "OperatePot Action Success!");

    if (result_.result) {
      setOutput("message", result_.result->message);
    } else {
      RCLCPP_WARN(node_->get_logger(), "OperatePot result is null");
    }

    return BT::NodeStatus::SUCCESS;
  }

  // 失败回调
  BT::NodeStatus on_aborted() override
  {
    RCLCPP_ERROR(node_->get_logger(), "OperatePot Action Aborted!");
    return BT::NodeStatus::FAILURE;
  }

  // 取消回调
  BT::NodeStatus on_cancelled() override
  {
    RCLCPP_WARN(node_->get_logger(), "OperatePot Action Cancelled!");
    return BT::NodeStatus::SUCCESS;  // 可按需求改成 FAILURE
  }
};

}  // namespace robot_ctrl

#endif  // ROBOT_CTRL__OPERATE_POT_ACTION_NODE_HPP_
