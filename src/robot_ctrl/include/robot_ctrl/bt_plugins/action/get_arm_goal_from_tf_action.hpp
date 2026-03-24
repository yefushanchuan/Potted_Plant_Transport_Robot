#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include <cmath>  
//机械臂零位mm
#define ARM_ZERO_X  200.0
#define ARM_ZERO_Y  -100.0
#define ARM_ZERO_Z  400.0
//机械臂初始位置：防碰撞
#define ARM_INIT_X  370.0
#define ARM_INIT_Y  -100.0
#define ARM_INIT_Z  250.0

//夹爪闭合中心到盆栽中心距离37.5mm 
#define HAND_RIGHT_LENGTH 37
//tag到机械臂末端中心的距离  ---- 40-62 = -22
#define HAND_LENGTH  0
//hand抓取pot的位置z：盆栽的固定高度153
#define HAND_POT_POSE_Z  153.0
//arm与tag之间的角度误差3°
#define ARM_TAG_ERROR_ANGLE 0.057 //rad
// y轴调节距离 20mm
#define HAND_Y_LENGTH 10

namespace robot_ctrl
{
class GetArmGoalFromTfAction : public BT::SyncActionNode
{
public:
  GetArmGoalFromTfAction(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal_type"),
      BT::OutputPort<geometry_msgs::msg::Pose>("arm_goal")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr arm_goal_pub_;
  geometry_msgs::msg::Pose tag_diff_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_current_pose_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_detection_sub_;
  std::atomic_bool tag_detected_ = false;  // 初始默认无检测到
  rclcpp::Time last_tag_seen_time_;
  geometry_msgs::msg::Pose arm_current_pose_;
  std::mutex arm_current_pose_mutex_;
};
}