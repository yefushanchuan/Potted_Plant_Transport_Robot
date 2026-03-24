#ifndef ROBOT_CTRL__BT_PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_
#define ROBOT_CTRL__BT_PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_

#include <atomic>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace robot_ctrl
{

class IsGoalReached : public BT::ConditionNode
{
public:
  IsGoalReached(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool goal_input_missing_reported_{false};
  std::atomic_bool tf_missing_reported_{false};
};

}  // namespace robot_ctrl

#endif  // ROBOT_CTRL__BT_PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_
