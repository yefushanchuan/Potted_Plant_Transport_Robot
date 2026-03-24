#include "robot_ctrl/bt_plugins/action/interact_with_plc_action.hpp"
#include <chrono>
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

using namespace std::chrono_literals;
namespace robot_ctrl
{
InteractWithPLCAction::InteractWithPLCAction(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  plc_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
    "PLC_DATA", 10, std::bind(&InteractWithPLCAction::plcCallback, this, std::placeholders::_1));

  hand_pub_ = node_->create_publisher<std_msgs::msg::Int8MultiArray>("Hand_STATU", 10);
}

BT::PortsList InteractWithPLCAction::providedPorts()
{
  return { 
    BT::InputPort<std::string>("ask_plc")
  };
}

void InteractWithPLCAction::plcCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  plc_data_ = msg->data;
  // RCLCPP_INFO(node_->get_logger(), "received PLC_DATA == %d", plc_data_);
}

BT::NodeStatus InteractWithPLCAction::onStart()
{
  std::lock_guard<std::mutex> lock(mtx_);
  
  if (!getInput("ask_plc", current_ask_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing input [ask_plc]");
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "InteractWithPLCAction: Missing input [ask_plc]");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "InteractWithPLCAction: Received ask_plc = %s", current_ask_.c_str());

  if (current_ask_ == "ask_release") {
    publishHandStatus(1);  // Hand_STATU: 01请求放料
    expected_plc_value_ = 1;
    start_time_ = std::chrono::steady_clock::now();
  } 
  else if (current_ask_ == "release_over") {
    publishHandStatus(2);  // Hand_STATU: 02放料完成
    ErrorLogQueue::instance().clearLastErrorIfPrefix("InteractWithPLCAction:");
    return BT::NodeStatus::SUCCESS;
  } 
  else if (current_ask_ == "ask_catch") {
    publishHandStatus(3);  // Hand_STATU: 03请求取料
    expected_plc_value_ = 2;
    start_time_ = std::chrono::steady_clock::now();
  }
  else if (current_ask_ == "catch_over") {
    publishHandStatus(4);  // Hand_STATU: 04取料完成
    ErrorLogQueue::instance().clearLastErrorIfPrefix("InteractWithPLCAction:");
    return BT::NodeStatus::SUCCESS;
  } 
  else {
    RCLCPP_WARN(node_->get_logger(), "Unknown ask_plc value: %s", current_ask_.c_str());
    ErrorLogQueue::instance().pushError(
      error_code::kInvalidParam,
      "InteractWithPLCAction: Unknown ask_plc value");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus InteractWithPLCAction::onRunning()
{
  std::lock_guard<std::mutex> lock(mtx_);
  
  // 检查是否已经接收到期望的PLC值
  if (plc_data_ == expected_plc_value_) {
    ErrorLogQueue::instance().clearLastErrorIfPrefix("InteractWithPLCAction:");
    return BT::NodeStatus::SUCCESS;
  }
  
  // 检查是否超时
  auto now = std::chrono::steady_clock::now();
  if (now - start_time_ > 180s) {
    RCLCPP_WARN(node_->get_logger(), "Timeout waiting for PLC_DATA == %d", expected_plc_value_);
    ErrorLogQueue::instance().pushError(
      error_code::kTimeout,
      "InteractWithPLCAction: Timeout waiting for PLC_DATA");
    return BT::NodeStatus::FAILURE;
  }
  
  return BT::NodeStatus::RUNNING;
}

void InteractWithPLCAction::onHalted()
{
  // 清理工作（如果需要）
}

void InteractWithPLCAction::publishHandStatus(uint8_t value)
{
  std_msgs::msg::Int8MultiArray msg;
  msg.data.push_back(value);
  hand_pub_->publish(msg);
}
}
