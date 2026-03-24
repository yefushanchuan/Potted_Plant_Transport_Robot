#include "robot_ctrl/bt_plugins/condition/is_pot_detected_condition.hpp"

#include <cmath>
#include <functional>
#include <limits>

#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

IsPotDetected::IsPotDetected(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  pots_sub_ = node_->create_subscription<pot_detection::msg::PotsInfo>(
    "/pots_info", rclcpp::QoS(10),
    std::bind(&IsPotDetected::potsInfoCallback, this, std::placeholders::_1));
}

BT::PortsList IsPotDetected::providedPorts()
{
  return {
    BT::InputPort<int>("delta_threshold", 100, "允许的像素偏差阈值，单位：像素")
  };
}

void IsPotDetected::potsInfoCallback(const pot_detection::msg::PotsInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_msg_ = msg;
  missing_msg_reported_.store(false, std::memory_order_relaxed);
  ErrorLogQueue::instance().clearLastErrorIfPrefix("IsPotDetected:");
}

BT::NodeStatus IsPotDetected::tick()
{
  int threshold = 100;
  if (auto thr = getInput<int>("delta_threshold"))
  {
    threshold = thr.value();
  }

  pot_detection::msg::PotsInfo::SharedPtr pots_msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pots_msg = latest_msg_;
  }

  if (!pots_msg)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "IsPotDetected: 尚未收到 /pots_info 数据");
    if (!missing_msg_reported_.exchange(true, std::memory_order_relaxed))
    {
      ErrorLogQueue::instance().pushError(
        error_code::kDataMissing, "IsPotDetected: No /pots_info received");
    }
    return BT::NodeStatus::FAILURE;
  }
  missing_msg_reported_.store(false, std::memory_order_relaxed);

  const pot_detection::msg::Pot * closest_pot = nullptr;
  int closest_abs_delta = std::numeric_limits<int>::max();

  for (const auto & pot : pots_msg->pots)
  {
    if (pot.type != 1)
    {
      continue;
    }

    const int abs_delta = std::abs(pot.delta_x);
    if (abs_delta < closest_abs_delta)
    {
      closest_abs_delta = abs_delta;
      closest_pot = &pot;
    }
  }

  if (closest_pot && closest_abs_delta <= threshold)
  {
    RCLCPP_INFO(node_->get_logger(),
      "IsPotDetected: 最接近的type=1盆栽delta_x=%d", closest_pot->delta_x);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace robot_ctrl
