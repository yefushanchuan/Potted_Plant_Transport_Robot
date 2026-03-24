#include "robot_ctrl/bt_plugins/condition/task_update.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"
#include <ctime>
#include <iomanip>
#include <sstream>
#include <functional>
namespace robot_ctrl
{

TaskUpdate::TaskUpdate(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery_state", 10,
    std::bind(&TaskUpdate::batteryCallback, this, std::placeholders::_1));

  task_service_ = node_->create_service<robot_ctrl::srv::TaskUpdata>(
    "/task_update",
    std::bind(&TaskUpdate::taskServiceCallback, this,
      std::placeholders::_1, std::placeholders::_2));
}

static std::string joinPotIds(const std::vector<std::string> & pot_ids)
{
  std::ostringstream oss;
  for (size_t i = 0; i < pot_ids.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << pot_ids[i];
  }
  return oss.str();
}

void TaskUpdate::last_task_handler()
{
  last_source_room_ = current_source_room_;
  last_target_room_ = current_target_room_;
  last_task_command_ = current_task_command_;
  last_task_id_ = current_task_id_;
  last_pot_ids_ = current_pot_ids_;
  last_pot_num_ = current_pot_num_;
  last_waypoint_rooms_ = current_waypoint_rooms_;
  new_command_available_ = false;
  // Placeholder for future task handling logic
}

BT::NodeStatus TaskUpdate::tick()
{
  bool task_locked = false;
  getInput("task_locked", task_locked);

  robot_ctrl::msg::RobotState robot_state_msg;
  if (!getInput("robot_state_msg", robot_state_msg))
  {
    robot_state_msg = getRobotStateFromBlackboard();
  }


  rclcpp::spin_some(node_);

  std::lock_guard<std::mutex> lock(mutex_);

  if (new_command_available_)
  {
      if (task_locked)//判断正在执行的任务是否被锁住
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "Task is locked, ignoring new command.");
        ErrorLogQueue::instance().pushError(
          error_code::kStateConflict,
          "TaskUpdate: Task is locked, ignoring new command");
        last_task_handler();
        return BT::NodeStatus::FAILURE;
      }

      setOutput("nav_source_room", current_source_room_);
      setOutput("nav_target_room", current_target_room_);
      setOutput("task_command", current_task_command_);
      setOutput("task_id", current_task_id_);
      setOutput("target_pot_ids", current_pot_ids_);
      setOutput("target_pot_num", current_pot_num_);
      setOutput("waypoint_rooms", current_waypoint_rooms_);

      std::string pot_ids_str = joinPotIds(current_pot_ids_);
      RCLCPP_INFO_STREAM(node_->get_logger(),
        "New command received: source_room='" << current_source_room_
        << "', target_room='" << current_target_room_
        << "', task_command='" << current_task_command_
        << "', pot_ids=[" << pot_ids_str
        << "], pot_num=" << current_pot_num_);

      last_task_handler();
      ErrorLogQueue::instance().clearLastErrorIfPrefix("TaskUpdate:");
      return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void TaskUpdate::taskServiceCallback(
  const std::shared_ptr<robot_ctrl::srv::TaskUpdata::Request> request,
  std::shared_ptr<robot_ctrl::srv::TaskUpdata::Response> response)
{
  TaskCommandData msg;
  msg.source_room = request->source_room;
  msg.target_room = request->target_room;
  msg.task_command = request->task_command;
  msg.pot_ids = request->pot_ids;
  msg.pot_num = request->pot_num;
  msg.waypoint_rooms = request->waypoint_rooms;

  auto robot_state_snapshot = getRobotStateFromBlackboard();
  const bool has_battery = has_battery_msg_.load(std::memory_order_acquire);
  const float battery_percentage = battery_percentage_.load(std::memory_order_relaxed);

  // if (has_battery && battery_percentage < 0.2F)
  // {
  //   const int battery_percent_int = static_cast<int>(battery_percentage * 100.0F);
  //   response->accepted = false;
  //   response->message =
  //     msg.task_command + " rejected! Battery low (" +
  //     std::to_string(battery_percent_int) + "% < 20%).";
  //   return;
  // }

  if(msg.task_command == "Charge" &&
         robot_state_snapshot.state == robot_ctrl::msg::RobotState::STATE_CHARGING)
  {
    response->accepted = false;
    response->message = msg.task_command + " rejected! Robot is already charging.";
  }
  else if(msg.task_command == "StopCharge" &&
         robot_state_snapshot.state != robot_ctrl::msg::RobotState::STATE_CHARGING)//非充电状态下，忽略停止充电命令
  {
    response->accepted = false;
    response->message = msg.task_command + " rejected! Robot is not charging.";
  }
  else if (msg.task_command == "Patrol" && msg.waypoint_rooms.empty())
  {
    response->accepted = false;
    response->message = "Patrol rejected! waypoint_rooms is empty.";
  }
  else
  {
    storeCommand(msg);
    response->accepted = true;
    response->message = msg.task_command + " received";
    response->task_id = current_task_id_;
  }
}

void TaskUpdate::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  battery_percentage_.store(msg->percentage, std::memory_order_relaxed);
  has_battery_msg_.store(true, std::memory_order_release);
}

void TaskUpdate::storeCommand(const TaskCommandData & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_source_room_ = msg.source_room;
  current_target_room_ = msg.target_room;
  current_task_command_ = msg.task_command;
  current_pot_ids_ = msg.pot_ids;
  current_pot_num_ = msg.pot_num;
  current_waypoint_rooms_ = msg.waypoint_rooms;
  ++task_seq_;
  const auto now_ns = node_ ? node_->now().nanoseconds() : 0;
  const auto now_sec = static_cast<std::time_t>(now_ns / 1000000000);
  std::tm tm{};
  localtime_r(&now_sec, &tm);
  std::ostringstream oss;
  if (!current_task_command_.empty())
  {
    oss << current_task_command_ << "-";
  }
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << "-" << task_seq_;
  current_task_id_ = oss.str();
  new_command_available_ = true;
}

BT::PortsList TaskUpdate::providedPorts()
{
  return {
    BT::OutputPort<std::string>("nav_source_room"),
    BT::OutputPort<std::string>("nav_target_room"),
    BT::OutputPort<std::string>("task_command"),
    BT::OutputPort<std::string>("task_id"),
    BT::OutputPort<std::vector<std::string>>("target_pot_ids"),
    BT::OutputPort<int>("target_pot_num"),
    BT::OutputPort<std::vector<std::string>>("waypoint_rooms"),
    BT::InputPort<bool>("task_locked"),
    BT::InputPort<robot_ctrl::msg::RobotState>("robot_state_msg")
  };
}

robot_ctrl::msg::RobotState TaskUpdate::getRobotStateFromBlackboard() const
{
  robot_ctrl::msg::RobotState state_msg;
  if (!config().blackboard->get("robot_state_msg", state_msg))
  {
    state_msg.state = robot_ctrl::msg::RobotState::STATE_UNKNOWN;
    state_msg.label = "UNKNOWN";
  }
  return state_msg;
}

}  // namespace robot_ctrl
