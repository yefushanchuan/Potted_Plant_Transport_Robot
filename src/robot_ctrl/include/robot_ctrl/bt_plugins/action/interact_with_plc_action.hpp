#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <string>
#include <mutex>
#include <chrono>

namespace robot_ctrl
{
class InteractWithPLCAction : public BT::StatefulActionNode
{
public:
  InteractWithPLCAction(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  // 重写StatefulActionNode的三个关键方法
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr plc_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr hand_pub_;

  std::mutex mtx_;
  int plc_data_ = -1;
  
  // 新增成员变量用于状态跟踪
  std::string current_ask_;
  int expected_plc_value_ = -1;
  std::chrono::steady_clock::time_point start_time_;

  void plcCallback(const std_msgs::msg::Int8::SharedPtr msg);
  void publishHandStatus(uint8_t value);
};
}