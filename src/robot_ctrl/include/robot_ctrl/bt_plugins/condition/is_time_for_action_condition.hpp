#ifndef IS_TIME_FOR_ACTION_CONDITION_HPP_
#define IS_TIME_FOR_ACTION_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace robot_ctrl
{

/**
 * @brief 条件节点：在指定小时/分钟/间隔触发巡检
 *
 * 通过读取系统时间，判断当前是否处于设定的触发小时/分钟和秒段，
 * 同时保证距离上次触发时间超过最小间隔，从而避免重复触发。
 */
class IsTimeForAction : public BT::ConditionNode
{
public:
  IsTimeForAction(const std::string & name, const BT::NodeConfiguration & config);
  IsTimeForAction() = delete;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock system_clock_;
  rclcpp::Time last_trigger_time_;
  bool invalid_trigger_reported_{false};
  bool invalid_trigger_hour_reported_{false};
  bool time_error_reported_{false};
};

}  // namespace robot_ctrl

#endif  // ROBOT_CTRL_BT_PLUGINS_CONDITION_IS_TIME_FOR_PATROL_CONDITION_HPP_
