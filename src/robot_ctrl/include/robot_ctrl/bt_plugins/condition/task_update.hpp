#pragma once

#include <string>
#include <mutex>
#include <cstdint>
#include <atomic>
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "robot_ctrl/msg/robot_state.hpp"
#include "robot_ctrl/srv/task_updata.hpp"

namespace robot_ctrl
{

class TaskUpdate : public BT::ConditionNode
{
public:
  TaskUpdate(
    const std::string & name,
    const BT::NodeConfiguration & config);

  TaskUpdate() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  struct TaskCommandData
  {
    std::string source_room;
    std::string target_room;
    std::string task_command;
    std::vector<std::string> pot_ids;
    int pot_num = 0;
    std::vector<std::string> waypoint_rooms;
  };

  void taskServiceCallback(
    const std::shared_ptr<robot_ctrl::srv::TaskUpdata::Request> request,
    std::shared_ptr<robot_ctrl::srv::TaskUpdata::Response> response);
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void storeCommand(
    const TaskCommandData & msg);
  robot_ctrl::msg::RobotState getRobotStateFromBlackboard() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<robot_ctrl::srv::TaskUpdata>::SharedPtr task_service_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  std::mutex mutex_;
  bool new_command_available_ = false;
  std::atomic<float> battery_percentage_{1.0f};
  std::atomic_bool has_battery_msg_{false};

  std::string last_target_room_;
  std::string current_target_room_;

  std::string last_task_command_;
  std::string current_task_command_;

  std::string last_source_room_;
  std::string current_source_room_;

  int last_pot_num_;
  int current_pot_num_;

  std::vector<std::string> last_pot_ids_;
  std::vector<std::string> current_pot_ids_;
  std::vector<std::string> last_waypoint_rooms_;
  std::vector<std::string> current_waypoint_rooms_;

  std::string last_task_id_;
  std::string current_task_id_;
  uint64_t task_seq_ = 0;

  void last_task_handler();

};

}  // namespace robot_ctrl
