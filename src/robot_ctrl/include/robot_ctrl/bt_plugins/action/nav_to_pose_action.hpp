#ifndef RM_DECISION_NAV2POSE_HPP_
#define RM_DECISION_NAV2POSE_HPP_
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <action_msgs/msg/goal_status_array.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>

#include "rclcpp_action/rclcpp_action.hpp"



using namespace BT;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
namespace robot_ctrl
{
  class Nav2Pose : public StatefulActionNode
  {
  public:
    Nav2Pose(const std::string &name, const NodeConfiguration &config);

    // this function is invoked once at the beginning.
    NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    // action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    // goal handle
    NavigateToPoseGoalHandle::SharedPtr goal_handle_;
    // send goal timeout
    int send_goal_timeout_;
    nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult> result_future_;
    std::mutex feedback_mutex_;
    std::string last_feedback_;
    double last_progress_{0.0};
    double initial_distance_{-1.0};


  };
} // end namespace rm_decision

#endif
