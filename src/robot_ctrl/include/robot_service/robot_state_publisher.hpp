#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "robot_ctrl/msg/robot_state.hpp"
#include "robot_ctrl/robot_state_utils.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"
#include "behaviortree_cpp_v3/blackboard.h"

class RobotStatePublisher : public rclcpp::Node
{
public:
    RobotStatePublisher(BT::Blackboard::Ptr bb)
        : Node("robot_state_publisher"), blackboard_(bb)
    {
        pub_ = this->create_publisher<robot_ctrl::msg::RobotState>("/robot_state", 10);
        clear_error_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/robot_state/clear_error",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                ErrorLogQueue::instance().clearAllErrors();
                robot_ctrl::msg::RobotState state_msg;
                blackboard_->get("robot_state_msg", state_msg);
                robot_ctrl::robot_state::fromString("IDLE", state_msg);
                blackboard_->set("robot_state_msg", state_msg);
                blackboard_->set("robot_state", state_msg.label);
                blackboard_->set("task_id", std::string{});
                response->success = true;
                response->message = "cleared";
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RobotStatePublisher::on_timer, this)
        );

        RCLCPP_INFO(this->get_logger(), "RobotStatePublisher initialized");
    }

private:
    void on_timer()
    {
        robot_ctrl::msg::RobotState state_msg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!blackboard_->get("robot_state_msg", state_msg))
            {
                std::string state = "INIT";
                blackboard_->get("robot_state", state);
                if (!robot_ctrl::robot_state::fromString(state, state_msg))
                {
                    state_msg.state = robot_ctrl::msg::RobotState::STATE_UNKNOWN;
                    state_msg.label = state.empty() ? "UNKNOWN" : state;
                }
            }

            std::string task_command;
            if (blackboard_->get("task_command", task_command))
            {
                state_msg.task_command = task_command;
            }

            std::string task_id;
            if (blackboard_->get("task_id", task_id))
            {
                state_msg.task_id = task_id;
            }

            std::string action_feedback;
            if (blackboard_->get("action_feedback", action_feedback))
            {
                state_msg.action_feedback = action_feedback;
            }

            double progress = 0.0;
            if (blackboard_->get("progress", progress))
            {
                state_msg.progress = static_cast<float>(progress);
            }
        }

        state_msg.error_code = 0;
        state_msg.error_msg.clear();
        robot_ctrl::ErrorRecord last_error;
        if (ErrorLogQueue::instance().getLastError(last_error))
        {
            state_msg.error_code = last_error.code;
            state_msg.error_msg = last_error.msg;
        }

        state_msg.stamp = this->now();
        pub_->publish(state_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published robot_state: %s", state_msg.label.c_str());
    }

    rclcpp::Publisher<robot_ctrl::msg::RobotState>::SharedPtr pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_error_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Blackboard::Ptr blackboard_;
    std::mutex mtx_;
};
