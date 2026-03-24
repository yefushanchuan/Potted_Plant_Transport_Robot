#include "robot_ctrl/bt_plugins/action/nav_to_pose_action.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <rclcpp_action/exceptions.hpp>
#include <sstream>

using namespace BT;
namespace robot_ctrl
{
  Nav2Pose::Nav2Pose(const std::string &name, const NodeConfiguration &config)
  : StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    node_->get_parameter_or("send_goal_timeout_ms", send_goal_timeout_, 1000);
  }

  NodeStatus Nav2Pose::onStart()
  {
    auto update_feedback = [this](const std::string & feedback, double progress)
    {
      if (config().blackboard)
      {
        config().blackboard->set("action_feedback", feedback);
        config().blackboard->set("progress", progress);
      }
      setOutput("action_feedback", feedback);
      setOutput("progress", progress);
    };

    goal_handle_.reset();
    result_future_ = {};
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      last_feedback_.clear();
      last_progress_ = 0.0;
      initial_distance_ = -1.0;
    }

    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal)
    {
      RCLCPP_ERROR(node_->get_logger(), "Goal is not set.");
      ErrorLogQueue::instance().pushError(
        error_code::kInputMissing,
        "Nav2Pose: Goal is not set");
      update_feedback("NavToPose: 未设置目标点", 0.0);
      return NodeStatus::FAILURE;
    }

    navigation_goal_.pose = goal.value();
    navigation_goal_.pose.header.frame_id = "map";
    navigation_goal_.pose.header.stamp = node_->now();

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [this](NavigateToPoseGoalHandle::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        if (!feedback)
        {
          return;
        }

        const double distance = feedback->distance_remaining;
        double progress = 0.0;

        std::lock_guard<std::mutex> lock(feedback_mutex_);
        if (std::isfinite(distance))
        {
          if (initial_distance_ < 0.0)
          {
            initial_distance_ = distance;
          }
          if (initial_distance_ > 0.0)
          {
            progress = std::clamp(1.0 - distance / initial_distance_, 0.0, 1.0);
          }
        }

        std::ostringstream oss;
        oss << "NavToPose: 剩余距离 " << std::fixed << std::setprecision(2) << distance;

        last_feedback_ = oss.str();
        last_progress_ = progress;
      };

    auto future_goal_handle = action_client_->async_send_goal(navigation_goal_, send_goal_options);
    RCLCPP_DEBUG(node_->get_logger(), "Sending goal with timeout %d ms", send_goal_timeout_);

    if (rclcpp::spin_until_future_complete(node_, future_goal_handle, std::chrono::milliseconds(send_goal_timeout_)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Send goal failed.");
      ErrorLogQueue::instance().pushError(
        error_code::kActionFailure,
        "Nav2Pose: Send goal failed");
      update_feedback("NavToPose: 发送目标失败", 0.0);
      return NodeStatus::FAILURE;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Goal handle is null.");
      ErrorLogQueue::instance().pushError(
        error_code::kActionFailure,
        "Nav2Pose: Goal handle is null");
      update_feedback("NavToPose: 目标句柄为空", 0.0);
      return NodeStatus::FAILURE;
    }

    result_future_ = action_client_->async_get_result(goal_handle_);
    update_feedback("NavToPose: 目标已发送", 0.0);

    RCLCPP_INFO(node_->get_logger(), "Navigating to pose [%.2f, %.2f, %.2f]",
                goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);

    return NodeStatus::RUNNING;
  }

  NodeStatus Nav2Pose::onRunning()
  {
    auto update_feedback = [this](const std::string & feedback, double progress)
    {
      if (config().blackboard)
      {
        config().blackboard->set("action_feedback", feedback);
        config().blackboard->set("progress", progress);
      }
      setOutput("action_feedback", feedback);
      setOutput("progress", progress);
    };

    if (!goal_handle_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Goal handle not initialized.");
      ErrorLogQueue::instance().pushError(
        error_code::kActionFailure,
        "Nav2Pose: Goal handle not initialized");
      update_feedback("NavToPose: 目标句柄未初始化", 0.0);
      return NodeStatus::FAILURE;
    }

    if (!result_future_.valid())
    {
      result_future_ = action_client_->async_get_result(goal_handle_);
    }

    auto status = result_future_.wait_for(std::chrono::milliseconds(10));

    if (status == std::future_status::ready)
    {
      auto wrapped_result = result_future_.get();

      switch (wrapped_result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "Navigation succeeded.");
          ErrorLogQueue::instance().clearLastErrorIfPrefix("Nav2Pose:");
          update_feedback("NavToPose: 到达目标", 1.0);
          goal_handle_.reset();
          result_future_ = {};
          return NodeStatus::SUCCESS;

        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(node_->get_logger(), "Navigation aborted.");
          ErrorLogQueue::instance().pushError(
            error_code::kActionFailure,
            "Nav2Pose: Navigation aborted");
          update_feedback("NavToPose: 导航中止", 0.0);
          goal_handle_.reset();
          result_future_ = {};
          return NodeStatus::FAILURE;

        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(node_->get_logger(), "Navigation canceled.");
          ErrorLogQueue::instance().pushError(
            error_code::kActionFailure,
            "Nav2Pose: Navigation canceled");
          update_feedback("NavToPose: 导航取消", 0.0);
          goal_handle_.reset();
          result_future_ = {};
          return NodeStatus::FAILURE;

        default:
          RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
          ErrorLogQueue::instance().pushError(
            error_code::kActionFailure,
            "Nav2Pose: Unknown result code");
          update_feedback("NavToPose: 未知结果码", 0.0);
          goal_handle_.reset();
          result_future_ = {};
          return NodeStatus::FAILURE;
      }
    }

    std::string feedback = "NavToPose: 执行中";
    double progress = 0.0;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      if (!last_feedback_.empty())
      {
        feedback = last_feedback_;
      }
      progress = last_progress_;
    }
    update_feedback(feedback, progress);
    return NodeStatus::RUNNING;
  }

  void Nav2Pose::onHalted()
  {
    RCLCPP_INFO(node_->get_logger(), "Goal halted.");
    if (!goal_handle_)
    {
      RCLCPP_DEBUG(node_->get_logger(), "No active goal handle to cancel.");
      result_future_ = {};
      return;
    }

    try
    {
      auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
      if (rclcpp::spin_until_future_complete(node_, cancel_future) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "Cancel goal failed.");
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "Goal canceled.");
      }
    }
    catch (const rclcpp_action::exceptions::UnknownGoalHandleError &e)
    {
      RCLCPP_WARN(node_->get_logger(), "Cancel goal skipped: %s", e.what());
    }

    goal_handle_.reset();
    result_future_ = {};

    if (config().blackboard)
    {
      config().blackboard->set("action_feedback", "NavToPose: 已终止");
      config().blackboard->set("progress", 0.0);
    }
    setOutput("action_feedback", "NavToPose: 已终止");
    setOutput("progress", 0.0);
  }

  PortsList Nav2Pose::providedPorts()
  {
    const char *description = "Goal pose to send to Nav2";
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", description),
      BT::OutputPort<std::string>("action_feedback"),
      BT::OutputPort<double>("progress")
    };
    
  }

} // namespace robot_ctrl
