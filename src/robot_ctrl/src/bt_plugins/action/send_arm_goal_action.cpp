#include "robot_ctrl/bt_plugins/action/send_arm_goal_action.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl {

SendArmGoalAction::SendArmGoalAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      current_status_(BT::NodeStatus::IDLE)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    client_ = rclcpp_action::create_client<ArmCommand>(node_, "move_arm");
}

BT::PortsList SendArmGoalAction::providedPorts() {
    return { 
        BT::InputPort<int>("task_type")
    };
}

BT::NodeStatus SendArmGoalAction::onStart() {
    // 1. 获取输入
    auto task_type = getInput<int>("task_type");
    if (!task_type)
    {
        last_error_ = "Missing/invalid task_type input: " + task_type.error();
        RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
        ErrorLogQueue::instance().pushError(
            error_code::kInputMissing,
            "SendArmGoalAction: " + last_error_);
        return BT::NodeStatus::FAILURE;
    }

    if (task_type.value() < 0 || task_type.value() > std::numeric_limits<uint8_t>::max())
    {
        last_error_ = "task_type out of range: " + std::to_string(task_type.value());
        RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
        ErrorLogQueue::instance().pushError(
            error_code::kInvalidParam,
            "SendArmGoalAction: " + last_error_);
        return BT::NodeStatus::FAILURE;
    }

    goal.task_type = static_cast<uint8_t>(task_type.value());

    // 2. 等待服务
    if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
        last_error_ = "Action server not available";
        RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
        ErrorLogQueue::instance().pushError(
            error_code::kServiceUnavailable,
            "SendArmGoalAction: Action server not available");
        return BT::NodeStatus::FAILURE;
    }

    // 4. 配置回调
    auto send_goal_options = rclcpp_action::Client<ArmCommand>::SendGoalOptions();
    send_goal_options.result_callback = 
        [this](const GoalHandle::WrappedResult& result) {
            this->processResult(result);
        };

    // 5. 发送目标
    auto future = client_->async_send_goal(goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(1)) 
        != rclcpp::FutureReturnCode::SUCCESS) 
    {
        last_error_ = "Failed to send goal";
        RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
        ErrorLogQueue::instance().pushError(
            error_code::kActionFailure,
            "SendArmGoalAction: Failed to send goal");
        return BT::NodeStatus::FAILURE;
    }

    goal_handle_ = future.get();
    start_time_ = node_->now();
    current_status_ = BT::NodeStatus::RUNNING;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendArmGoalAction::onRunning() {
    // 1. 检查超时（30秒）
    if ((node_->now() - start_time_) > rclcpp::Duration(30, 0)) {
        last_error_ = "Action timed out";
        RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
        ErrorLogQueue::instance().pushError(
            error_code::kTimeout,
            "SendArmGoalAction: Action timed out");
        return BT::NodeStatus::FAILURE;
    }

    // 2. 返回当前状态
    auto status = current_status_.load();
    if (status == BT::NodeStatus::FAILURE) {
    }
    return status;
}

void SendArmGoalAction::onHalted() {
    if (goal_handle_ && rclcpp::ok()) {
        RCLCPP_INFO(node_->get_logger(), "Canceling goal...");
        client_->async_cancel_goal(goal_handle_);
    }
    current_status_ = BT::NodeStatus::IDLE;
}

// 专用结果处理函数
void SendArmGoalAction::processResult(const GoalHandle::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(result_mutex_);

    if (current_status_ != BT::NodeStatus::RUNNING) {
    RCLCPP_WARN(node_->get_logger(), "Received result after node was halted");
    return;
    }   

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), 
                       "Goal succeeded: %s", 
                       result.result->message.c_str());
            ErrorLogQueue::instance().clearLastErrorIfPrefix("SendArmGoalAction:");
            current_status_ = BT::NodeStatus::SUCCESS;
            break;
            
        case rclcpp_action::ResultCode::ABORTED:
            last_error_ = "Goal aborted: " + result.result->message;
            RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
            ErrorLogQueue::instance().pushError(
                error_code::kActionFailure,
                "SendArmGoalAction: Goal aborted");
            current_status_ = BT::NodeStatus::FAILURE;
            break;
            
        case rclcpp_action::ResultCode::CANCELED:
            last_error_ = "Goal canceled by user";
            RCLCPP_WARN(node_->get_logger(), "%s", last_error_.c_str());
            ErrorLogQueue::instance().pushError(
                error_code::kActionFailure,
                "SendArmGoalAction: Goal canceled by user");
            current_status_ = BT::NodeStatus::FAILURE;
            break;
            
        default:
            last_error_ = "Unknown result code";
            RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
            ErrorLogQueue::instance().pushError(
                error_code::kActionFailure,
                "SendArmGoalAction: Unknown result code");
            current_status_ = BT::NodeStatus::FAILURE;
    }
}

} // namespace robot_ctrl
