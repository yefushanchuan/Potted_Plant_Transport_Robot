#include "robot_ctrl/bt_plugins/action/get_goal_from_room_action.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

using namespace std::chrono_literals;

namespace robot_ctrl
{

GetNavGoalFromRoom::GetNavGoalFromRoom(const std::string& name,
                                       const BT::NodeConfiguration& config)
: StatefulActionNode(name, config)
{
  node_   = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<robot_ctrl::srv::GetRoompose>("/get_room_pose");
}

BT::PortsList GetNavGoalFromRoom::providedPorts()
{
  return {
    BT::InputPort<std::string>("nav_target_room"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
  };
}

/* =============== 状态机实现 =============== */

BT::NodeStatus GetNavGoalFromRoom::onStart()
{
  // 每次进入节点都重置 future
  future_ = {};

  auto res = getInput("nav_target_room", room_name);

  if (!res || room_name.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing input port [nav_target_room]");
    ErrorLogQueue::instance().pushError(
      error_code::kInputMissing,
      "GetNavGoalFromRoom: Missing input port [nav_target_room]");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_service(0s))   // 非阻塞检测
  {
    RCLCPP_ERROR(node_->get_logger(), "Service [/get_room_pose] not available");
    ErrorLogQueue::instance().pushError(
      error_code::kServiceUnavailable,
      "GetNavGoalFromRoom: Service [/get_room_pose] not available");
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<robot_ctrl::srv::GetRoompose::Request>();
  req->room_name = room_name;

  future_ = client_->async_send_request(req).future.share();
  return BT::NodeStatus::RUNNING;   // 转到 onRunning
}

BT::NodeStatus GetNavGoalFromRoom::onRunning()
{
  if (!future_.valid()){
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "GetNavGoalFromRoom: service not valid");
    return BT::NodeStatus::FAILURE;
  }

  // 非阻塞检查
  if (future_.wait_for(0s) != std::future_status::ready)
    return BT::NodeStatus::RUNNING;

  auto resp = future_.get();
  if (!resp){
    ErrorLogQueue::instance().pushError(
      error_code::kActionFailure,
      "GetNavGoalFromRoom: service no respence");
    return BT::NodeStatus::FAILURE;
  }

  if (!resp->success){
    RCLCPP_ERROR(node_->get_logger(), "GetNavGoalFromRoom: room not found");
    std::string error_msg = "GetNavGoalFromRoom: room '" + room_name + "' not found";
    ErrorLogQueue::instance().pushError(
      error_code::kDataMissing,
      error_msg);
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header.frame_id = "map";
  goal_msg.header.stamp    = node_->now();

  goal_msg.pose.position.x = resp->x;
  goal_msg.pose.position.y = resp->y;
  goal_msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, resp->yaw);
  goal_msg.pose.orientation = tf2::toMsg(q);

  setOutput("goal", goal_msg);
  ErrorLogQueue::instance().clearLastErrorIfPrefix("GetNavGoalFromRoom:");
  return BT::NodeStatus::SUCCESS;
}

void GetNavGoalFromRoom::onHalted()
{
  // 如果行为树打断本节点，可以在这里取消请求（ROS 2 client 无法直接取消，
  // 这里简单清理即可）。
  future_ = {};
}

}  // namespace robot_ctrl
