#ifndef RECOVERY_NODE_HPP_
#define RECOVERY_NODE_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"

namespace robot_ctrl
{

class RecoveryNode : public BT::ControlNode
{
public:

  RecoveryNode(
    const std::string & name,
    const BT::NodeConfiguration & conf);


  ~RecoveryNode() override = default;


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("number_of_retries", 1, "Number of retries")
    };
  }

private:
  unsigned int current_child_idx_;
  unsigned int number_of_retries_;
  unsigned int retry_count_;


  BT::NodeStatus tick() override;


  void halt() override;
};

}  // namespace robot_ctrl

#endif   