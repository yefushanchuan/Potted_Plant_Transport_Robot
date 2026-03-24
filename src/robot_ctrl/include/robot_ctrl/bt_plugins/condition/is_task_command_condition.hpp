#ifndef ROBOT_BT_NODES__IS_COMMAND_CONDITION_HPP_
#define ROBOT_BT_NODES__IS_COMMAND_CONDITION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace robot_ctrl
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when voice_task_command == task_type
 */
class IsCommand : public BT::ConditionNode
{
public:
  /**
   * @brief Constructor
   * @param condition_name Name of the condition node
   * @param conf BehaviorTree node configuration
   */
  IsCommand(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsCommand() = delete;

  /**
   * @brief The main tick function
   * @return SUCCESS if matches, otherwise FAILURE
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return list of ports
   */
  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  bool missing_command_reported_{false};
  bool missing_type_reported_{false};
};

}  // namespace robot_bt_nodes

#endif  // ROBOT_BT_NODES__IS_COMMAND_CONDITION_HPP_
