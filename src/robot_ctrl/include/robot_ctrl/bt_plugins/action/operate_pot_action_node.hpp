#ifndef ROBOT_CTRL__OPERATE_POT_ACTION_NODE_HPP_
#define ROBOT_CTRL__OPERATE_POT_ACTION_NODE_HPP_

#include <string>
#include <memory>

// 引入 BehaviorTree 的基础类型 (PortsList, NodeConfiguration 等)
#include "behaviortree_cpp_v3/bt_factory.h" 
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
    const BT::NodeConfiguration & conf);

  // BT 输入输出端口声明
  static BT::PortsList providedPorts();

protected:
  // 生命周期回调声明
  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
};

}  // namespace robot_ctrl

#endif  // ROBOT_CTRL__OPERATE_POT_ACTION_NODE_HPP_
