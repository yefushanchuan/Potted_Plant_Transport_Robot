#include "robot_ctrl/bt_plugins/action/get_arm_goal_from_tf_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "robot_ctrl/bt_plugins/error_log_queue.hpp"

namespace robot_ctrl
{

GetArmGoalFromTfAction::GetArmGoalFromTfAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  arm_goal_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("/arm_goal", 10);
  arm_current_pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>("/arm_current_pose", 10,  
    [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(arm_current_pose_mutex_);
      arm_current_pose_ = *msg;
    });

  tag_detection_sub_ = node_->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
  "/apriltag/detections", 10,
  [this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    if (!msg->detections.empty()) {
      tag_detected_ = true;
    } else {
        tag_detected_ = false;
    }
  });
}

BT::NodeStatus GetArmGoalFromTfAction::tick()
{
  if (!tag_detected_) {
    RCLCPP_WARN(node_->get_logger(), "AprilTag lost! Aborting arm goal computation.");//优先检测是否存在tag，若不存在直接返回failure  
    ErrorLogQueue::instance().pushError(
      error_code::kDataMissing,
      "GetArmGoalFromTfAction: AprilTag lost! Aborting arm goal computation");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_buffer_->lookupTransform("arm_base_link", "Tag0", tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
    ErrorLogQueue::instance().pushError(
      error_code::kDataMissing, "GetArmGoalFromTfAction: arm TF lookup failed");
    return BT::NodeStatus::FAILURE;
  }
  //获取tag--arm的距离
  tag_diff_pose_.position.x = transform.transform.translation.x;
  tag_diff_pose_.position.y = transform.transform.translation.y;
  tag_diff_pose_.position.z = transform.transform.translation.z;
  // 提取 yaw
  tf2::Quaternion q_orig;
  tf2::fromMsg(transform.transform.rotation, q_orig);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);
  tf2::Quaternion q_yaw;

  q_yaw.setRPY(0.0, 0.0, yaw - M_PI/2 - ARM_TAG_ERROR_ANGLE);//获取的yaw-90°-角度误差2.5°
  q_yaw.normalize();
  tag_diff_pose_.orientation = tf2::toMsg(q_yaw);//转四元数得机械臂夹爪角度
      
  // goal_type 处理
  std::string goal_type;
  getInput("goal_type", goal_type);
  geometry_msgs::msg::Pose goal_pose;
  if (goal_type == "up")//提升动作
  {
    goal_pose.position.x = arm_current_pose_.position.x *1000.0;//机械臂当前x轴位置
    goal_pose.position.y = arm_current_pose_.position.y *1000.0;//机械臂当前y轴位置
    goal_pose.position.z = ARM_INIT_Z;  //机械臂初始位置z轴高度
    goal_pose.orientation.x = arm_current_pose_.orientation.x;
    goal_pose.orientation.y = arm_current_pose_.orientation.y;
    goal_pose.orientation.z = arm_current_pose_.orientation.z;
    goal_pose.orientation.w = arm_current_pose_.orientation.w; //机械臂末端当前角度
  }        
  else if (goal_type == "down") //下降动作
  {
    goal_pose.position.x = arm_current_pose_.position.x * 1000.0;//当前机械臂末端x轴位置
    goal_pose.position.y = arm_current_pose_.position.y * 1000.0;//机械臂当前y轴位置
    goal_pose.position.z = HAND_POT_POSE_Z;//目标点固定高度z轴位置
    goal_pose.orientation.x = tag_diff_pose_.orientation.x;
    goal_pose.orientation.y = tag_diff_pose_.orientation.y;
    goal_pose.orientation.z = tag_diff_pose_.orientation.z;
    goal_pose.orientation.w = tag_diff_pose_.orientation.w; // 目标点角度
  }
  else if (goal_type == "front")//前伸动作
  {
    goal_pose.position.x = tag_diff_pose_.position.x * 1000.0 - HAND_LENGTH;//目标点x轴位置
    goal_pose.position.y = tag_diff_pose_.position.y * 1000.0 + HAND_Y_LENGTH;//目标点y轴位置
    goal_pose.position.z = arm_current_pose_.position.z * 1000.0;//当前机械臂z轴位置
    goal_pose.orientation.x = tag_diff_pose_.orientation.x;
    goal_pose.orientation.y = tag_diff_pose_.orientation.y;
    goal_pose.orientation.z = tag_diff_pose_.orientation.z;
    goal_pose.orientation.w = tag_diff_pose_.orientation.w; //目标点角度
  }
  else if (goal_type == "back")//后退动作
  {
    goal_pose.position.x = ARM_INIT_X;//初始x轴位置
    goal_pose.position.y = arm_current_pose_.position.y *1000.0;//机械臂当前y轴位置
    goal_pose.position.z = arm_current_pose_.position.z *1000.0;//机械臂当前z轴位置
    goal_pose.orientation.x = arm_current_pose_.orientation.x;
    goal_pose.orientation.y = arm_current_pose_.orientation.y;
    goal_pose.orientation.z = arm_current_pose_.orientation.z;
    goal_pose.orientation.w = arm_current_pose_.orientation.w; //机械臂末端当前角度
  }
  else if (goal_type == "right")//右移微调动作
  {
    goal_pose.position.x = arm_current_pose_.position.x *1000.0;//机械臂当前x轴位置
    goal_pose.position.y = arm_current_pose_.position.y *1000.0 - HAND_RIGHT_LENGTH;//机械臂当前y轴位置 - 夹爪中心到盆栽中心距离
    goal_pose.position.z = arm_current_pose_.position.z *1000.0;//机械臂当前z轴位置
    goal_pose.orientation.x = arm_current_pose_.orientation.x;
    goal_pose.orientation.y = arm_current_pose_.orientation.y;
    goal_pose.orientation.z = arm_current_pose_.orientation.z;
    goal_pose.orientation.w = arm_current_pose_.orientation.w; //机械臂末端当前角度
  }
  else if (goal_type == "left")//左移微调动作
  {
    goal_pose.position.x = arm_current_pose_.position.x *1000.0;//机械臂当前x轴位置
    goal_pose.position.y = tag_diff_pose_.position.y * 1000.0 + HAND_RIGHT_LENGTH;//目标点y轴位置
    goal_pose.position.z = arm_current_pose_.position.z *1000.0;//机械臂当前z轴位置
    goal_pose.orientation.x = arm_current_pose_.orientation.x;
    goal_pose.orientation.y = arm_current_pose_.orientation.y;
    goal_pose.orientation.z = arm_current_pose_.orientation.z;
    goal_pose.orientation.w = arm_current_pose_.orientation.w; //机械臂末端当前角度
  }
  else if (goal_type == "init")//机械臂初始位置
  {
    goal_pose.position.x = ARM_INIT_X;
    goal_pose.position.y = ARM_INIT_Y;
    goal_pose.position.z = ARM_INIT_Z;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 1.0;
  }
  else if (goal_type == "zero")//机械臂原位
  {
    goal_pose.position.x = ARM_ZERO_X;
    goal_pose.position.y = ARM_ZERO_Y;
    goal_pose.position.z = ARM_ZERO_Z;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 1.0; 
  }
  else //未知类型，回到原位
  {
    goal_pose.position.x = ARM_ZERO_X;
    goal_pose.position.y = ARM_ZERO_Y;
    goal_pose.position.z = ARM_ZERO_Z;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 1.0; 
    RCLCPP_WARN(node_->get_logger(), "unknow goal_type: %s", goal_type.c_str());
  }
  
  if(goal_pose.position.x <= 150) goal_pose.position.x = 150;
  if(goal_pose.position.x >= 790) goal_pose.position.x = 790;
  if(goal_pose.position.y >= 300) goal_pose.position.y = 300;
  if(goal_pose.position.y <= -300) goal_pose.position.y = -300; 
  if(goal_pose.position.z >= 450) goal_pose.position.z = 450;
  if(goal_pose.position.z <= 32) goal_pose.position.z = 32;

  // 发布 arm_goal
  arm_goal_pub_->publish(goal_pose);

  // 输出到黑板
  setOutput("arm_goal", goal_pose);
  ErrorLogQueue::instance().clearLastErrorIfPrefix("GetArmGoalFromTfAction:");
  return BT::NodeStatus::SUCCESS;
}
}
