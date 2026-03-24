#include "robot_ctrl/bt_plugins/condition/is_goal_reached_condition.hpp"

#include <cmath>

#include "robot_ctrl/bt_plugins/error_log_queue.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"

namespace robot_ctrl
{

IsGoalReached::IsGoalReached(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList IsGoalReached::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<double>("tolerance", 0.15, "允许的距离容差"),
    BT::InputPort<double>("yaw_tolerance", 0.35, "允许的偏航角容差(弧度)")
  };
}

BT::NodeStatus IsGoalReached::tick()
{
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal)
  {
    RCLCPP_ERROR(node_->get_logger(), "IsGoalReached: 无法从黑板读取 goal。");
    if (!goal_input_missing_reported_)
    {
      ErrorLogQueue::instance().pushError(
        error_code::kInputMissing, "IsGoalReached: Missing input [goal]");
      goal_input_missing_reported_ = true;
    }
    return BT::NodeStatus::FAILURE;
  }
  goal_input_missing_reported_ = false;

  double tolerance = 0.1;
  if (auto tol = getInput<double>("tolerance"))
  {
    tolerance = tol.value();
  }
  double yaw_tolerance = 0.35;
  if (auto yaw_tol = getInput<double>("yaw_tolerance"))
  {
    yaw_tolerance = std::abs(yaw_tol.value());
  }

  geometry_msgs::msg::TransformStamped map_to_base_tf;
  try
  {
    map_to_base_tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "IsGoalReached: 获取 TF map->base_footprint 失败: %s", ex.what());
    if (!tf_missing_reported_.exchange(true, std::memory_order_relaxed))
    {
      ErrorLogQueue::instance().pushError(
        error_code::kDataMissing, "IsGoalReached: TF map->base_footprint unavailable");
    }
    return BT::NodeStatus::FAILURE;
  }
  tf_missing_reported_.store(false, std::memory_order_relaxed);

  const auto & goal_pos = goal->pose.position;
  const auto & current_pos = map_to_base_tf.transform.translation;

  const double dx = goal_pos.x - current_pos.x;
  const double dy = goal_pos.y - current_pos.y;
  const double distance = std::hypot(dx, dy);
  const double goal_yaw = tf2::getYaw(goal->pose.orientation);
  const double current_yaw = tf2::getYaw(map_to_base_tf.transform.rotation);
  const double yaw_error = std::abs(std::atan2(
      std::sin(goal_yaw - current_yaw),
      std::cos(goal_yaw - current_yaw)));

  if (distance <= tolerance && yaw_error <= yaw_tolerance)
  {
    ErrorLogQueue::instance().clearLastErrorIfPrefix("IsGoalReached:");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
}  // namespace robot_ctrl
