#include <memory>
#include <string>

// BehaviorTree.CPP factory include
// Note: Depending on your ROS 2 distro (e.g., Humble vs Jazzy), 
// this might be "behaviortree_cpp_v3/bt_factory.h" or "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp_v3/bt_factory.h" 

// Replace this with the actual path to your header file
#include "robot_ctrl/bt_plugins/action/operate_pot_action_node.hpp"

BT_REGISTER_NODES(factory)
{
  // Create a builder lambda to inject the specific ROS 2 Action Server name
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      // "operate_pot" corresponds to the action server name initialized 
      // in the hardware interface (agrobot_hardware_interface.cpp)
      return std::make_unique<robot_ctrl::OperatePotAction>(
        name, "operate_pot", config);
    };

  // Register the builder with the factory. 
  // "OperatePot" is the XML tag you will use in your Behavior Tree (.xml) files.
  factory.registerBuilder<robot_ctrl::OperatePotAction>(
    "OperatePot", builder);
}
