#include "robot_ctrl/bt_plugins/decorator/retry_decorator.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

RetryUntilSuccessful::RetryUntilSuccessful(const std::string& name, const BT::NodeConfiguration& config)
: BT::DecoratorNode(name, config), remaining_attempts_(0), initialized_(false)
{}

BT::NodeStatus RetryUntilSuccessful::tick()
{
  if (!child_node_)
  {
    throw BT::RuntimeError("RetryUntilSuccessful must have exactly one child");
  }

  // 读取 retry 次数（只在第一次 tick 时读取）
  if (!initialized_)
  {
    int retry_count = 0;
    if (!getInput<int>("retry", retry_count))
    {
      throw BT::RuntimeError("Missing required input [retry]");
    }
    remaining_attempts_ = retry_count;
    initialized_ = true;
  }

  // 执行子节点
  setStatus(BT::NodeStatus::RUNNING);
  BT::NodeStatus child_status = child_node_->executeTick();

  if (child_status == BT::NodeStatus::SUCCESS)
  {
    // 成功直接返回 SUCCESS
    initialized_ = false; // 下次重新读取 retry
    ErrorLogQueue::instance().clearLastErrorIfPrefix("RetryUntilSuccessful:");
    return BT::NodeStatus::SUCCESS;
  }
  else if (child_status == BT::NodeStatus::FAILURE)
  {
    // 如果还有重试次数，减少一次，返回 RUNNING（继续尝试）
    if (remaining_attempts_ > 0)
    {
      remaining_attempts_--;
      child_node_->halt(); // 重置子节点状态
      return BT::NodeStatus::RUNNING;
    }
    else
    {
      // 所有尝试用完，返回 FAILURE
      initialized_ = false; // 下次重新读取 retry
      ErrorLogQueue::instance().pushError(
        error_code::kRecoveryFailed,
        "RetryUntilSuccessful: all retry attempts exhausted after 3 retries");
      return BT::NodeStatus::FAILURE;
    }
  }

  return child_status; // 如果子节点是 RUNNING，保持 RUNNING
}

} 
