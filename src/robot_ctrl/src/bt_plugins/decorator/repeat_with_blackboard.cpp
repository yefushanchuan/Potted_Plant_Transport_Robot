#include "robot_ctrl/bt_plugins/decorator/repeat_with_blackboard.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

RepeatWithBlackboard::RepeatWithBlackboard(const std::string& name, const BT::NodeConfiguration& config)
  : BT::DecoratorNode(name, config),
    infinite_loop_(false),
    repeat_count_(0),
    current_count_(0)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList RepeatWithBlackboard::providedPorts()
{
  return {
    BT::InputPort<int>("target_pot_num"),
    BT::InputPort<int>("repeat_type")  // -1 表示无限循环
  };
}

BT::NodeStatus RepeatWithBlackboard::tick()
{
  if (!child_node_)
  {
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing, "RepeatWithBlackboard: missing child");
    throw BT::RuntimeError("RepeatWithBlackboard: missing child");
  }

  if (status() == BT::NodeStatus::IDLE)
  {
    int repeat_type = 0;
    if (!getInput("repeat_type", repeat_type))
    {
      repeat_type = 0; // 默认值：普通重复
    }

    if (repeat_type == -1)
    {
      infinite_loop_ = true;
      RCLCPP_INFO(node_->get_logger(), "RepeatWithBlackboard: infinite loop enabled");
    }
    else
    {
      infinite_loop_ = false;

      int target_pot_num = 0;
      if (!getInput("target_pot_num", target_pot_num))
      {
        ErrorLogQueue::instance().pushError(
          error_code::kInputMissing, "RepeatWithBlackboard: missing input [target_pot_num]");
        throw BT::RuntimeError("RepeatWithBlackboard: missing input [target_pot_num]");
      }
      if (target_pot_num <= 0)
      {
        ErrorLogQueue::instance().pushError(
          error_code::kInvalidParam, "RepeatWithBlackboard: target_pot_num must be positive");
        throw BT::RuntimeError("RepeatWithBlackboard: target_pot_num must be positive");
      }

      repeat_count_ = target_pot_num; // 输入多少次就重复多少次
      current_count_ = 0;
      RCLCPP_INFO(node_->get_logger(), "RepeatWithBlackboard: total repeats %d", repeat_count_);
    }
  }

  const BT::NodeStatus child_status = child_node_->executeTick();

  if (infinite_loop_)
  {
    // 子节点每次执行完就重启，永远 RUNNING
    if (child_status == BT::NodeStatus::SUCCESS || child_status == BT::NodeStatus::FAILURE)
    {
      child_node_->halt();
    }
    return BT::NodeStatus::RUNNING;
  }
  else
  {
    if (child_status == BT::NodeStatus::SUCCESS)
    {
      current_count_++;

      if (current_count_ < repeat_count_)
      {
        child_node_->halt();
        RCLCPP_INFO(node_->get_logger(), "RepeatWithBlackboard: repeat %d/%d",
                    current_count_, repeat_count_);
        return BT::NodeStatus::RUNNING;
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "RepeatWithBlackboard: all repeats completed");
        ErrorLogQueue::instance().clearLastErrorIfPrefix("RepeatWithBlackboard:");
        return BT::NodeStatus::SUCCESS;
      }
    }
    else if (child_status == BT::NodeStatus::FAILURE)
    {
      return BT::NodeStatus::FAILURE;
    }

    return child_status; // RUNNING 时直接传递
  }
}

void RepeatWithBlackboard::halt()
{
  DecoratorNode::halt();
  current_count_ = 0;
  infinite_loop_ = false;
}

}  // namespace robot_ctrl
