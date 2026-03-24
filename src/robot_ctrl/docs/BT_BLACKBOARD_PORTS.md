# robot_ctrl 行为树节点黑板参数清单

日期：2026-01-26  
作者：Codex  

## 总览

- 节点数量（含 providedPorts 定义）：25
- 黑板参数总数（ports 口）：55
- 存在隐式黑板键的节点数：4

说明：本清单基于各节点的 `providedPorts()` 定义；另单独列出通过 `blackboard->get/set` 访问但未在 ports 中声明的键。

## 节点：CancelNavToPose (CancelNavToPose)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | timeout_ms | int | 等待取消指令完成的超时时间，单位毫秒 | src/robot_ctrl/src/bt_plugins/action/cancel_nav_to_pose_action.cpp |
| Input | brake_duration_ms | int | 取消导航后发布零速的持续时间，单位毫秒 | src/robot_ctrl/src/bt_plugins/action/cancel_nav_to_pose_action.cpp |
| Output | action_feedback | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/action/cancel_nav_to_pose_action.cpp |
| Output | progress | double | （无描述） | src/robot_ctrl/src/bt_plugins/action/cancel_nav_to_pose_action.cpp |

## 节点：ClearTaskCommand (ClearTaskCommandNode)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Output | nav_source_room | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/action/clear_task_command_action.cpp |
| Output | nav_target_room | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/action/clear_task_command_action.cpp |
| Output | task_command | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/action/clear_task_command_action.cpp |
| Output | target_pot_num | int | （无描述） | src/robot_ctrl/src/bt_plugins/action/clear_task_command_action.cpp |

## 节点：DockRobot (DockRobotActionBT)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | charge_enable | bool | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/action/dock_robot_action.hpp |
| Input | charge_stop | bool | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/action/dock_robot_action.hpp |
| Output | action_feedback | std::string | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/action/dock_robot_action.hpp |
| Output | progress | double | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/action/dock_robot_action.hpp |

## 节点：GetArmGoalFromTfAction

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | goal_type | std::string | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/action/get_arm_goal_from_tf_action.hpp |
| Output | arm_goal | geometry_msgs::msg::Pose | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/action/get_arm_goal_from_tf_action.hpp |

## 节点：GetGoalFromRoom (GetNavGoalFromRoom)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | nav_target_room | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/action/get_goal_from_room_action.cpp |
| Output | goal | geometry_msgs::msg::PoseStamped | （无描述） | src/robot_ctrl/src/bt_plugins/action/get_goal_from_room_action.cpp |

## 节点：GripControlAction

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | mode2 | int | Gripper control mode: 1=open, 2=close | src/robot_ctrl/src/bt_plugins/action/grip_control_action.cpp |

## 节点：InteractWithPLCAction

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | ask_plc | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/action/interact_with_plc_action.cpp |

## 节点：IsTaskCommand (IsCommand)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | task_command | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/condition/is_task_command_condition.cpp |
| Input | task_type | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/condition/is_task_command_condition.cpp |

## 节点：IsGoalReached (IsGoalReached)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | goal | geometry_msgs::msg::PoseStamped | （无描述） | src/robot_ctrl/src/bt_plugins/condition/is_goal_reached_condition.cpp |
| Input | tolerance | double | 允许的距离容差 | src/robot_ctrl/src/bt_plugins/condition/is_goal_reached_condition.cpp |

## 节点：IsPotDetected (IsPotDetected)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | delta_threshold | int | 允许的像素偏差阈值，单位：像素 | src/robot_ctrl/src/bt_plugins/condition/is_pot_detected_condition.cpp |

## 节点：IsRobotInitSuccess (IsRobotInitSuccess)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | robot_init_success | bool | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/condition/is_robot_init_success_condition.hpp |

## 节点：IsRobotStarted (IsRobotStarted)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | robot_started | bool | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/condition/is_robot_started_condition.hpp |

## 节点：IsRosNodeOk (IsRosNodeOk)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | node_name | std::string | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/condition/is_ros_node_ok_condition.hpp |
| Input | topic_name | std::string | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/condition/is_ros_node_ok_condition.hpp |

## 节点：IsTimeForAction (IsTimeForAction)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | trigger_hour | int | 触发小时[0-23]，-1 表示忽略小时 | src/robot_ctrl/src/bt_plugins/condition/is_time_for_action_condition.cpp |
| Input | trigger_minute | int | 在第几分钟触发[0-59] | src/robot_ctrl/src/bt_plugins/condition/is_time_for_action_condition.cpp |
| Input | window_seconds | int | 触发秒宽度，避免重复触发 | src/robot_ctrl/src/bt_plugins/condition/is_time_for_action_condition.cpp |
| Input | min_interval_sec | int | 两次触发的最小间隔 | src/robot_ctrl/src/bt_plugins/condition/is_time_for_action_condition.cpp |

## 节点：LaunchManager (LaunchManagerAction)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | command | std::string | start/stop/restart/start_mapping/start_navigation | src/robot_ctrl/src/bt_plugins/action/launch_manager_action.cpp |
| Input | timeout_ms | int | 等待服务响应的超时时间，单位毫秒 | src/robot_ctrl/src/bt_plugins/action/launch_manager_action.cpp |
| Output | result_message | std::string | launch_manager 返回的信息 | src/robot_ctrl/src/bt_plugins/action/launch_manager_action.cpp |

## 节点：NavToPose (Nav2Pose)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | goal | geometry_msgs::msg::PoseStamped | （无描述） | src/robot_ctrl/src/bt_plugins/action/nav_to_pose_action.cpp |
| Output | action_feedback | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/action/nav_to_pose_action.cpp |
| Output | progress | double | （无描述） | src/robot_ctrl/src/bt_plugins/action/nav_to_pose_action.cpp |

## 节点：PublishRobotState

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | robot_state | std::string | Robot working state from blackboard | src/robot_ctrl/src/bt_plugins/action/publish_robot_state_action.cpp |

**隐式黑板键（未在 ports 中声明）**

- `robot_state_msg`

## 节点：RecoveryNode (RecoveryNode)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | number_of_retries | int | Number of retries | src/robot_ctrl/include/robot_ctrl/bt_plugins/control/recovery_node.hpp |

## 节点：RepeatWithBlackboard (RepeatWithBlackboard)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | target_pot_num | int | （无描述） | src/robot_ctrl/src/bt_plugins/decorator/repeat_with_blackboard.cpp |

## 节点：Retry (RetryUntilSuccessful)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | retry | int | Maximum retry attempts | src/robot_ctrl/include/robot_ctrl/bt_plugins/decorator/retry_decorator.hpp |

## 节点：SendArmGoal (SendArmGoalAction)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | task_type | int | （无描述） | src/robot_ctrl/src/bt_plugins/action/send_arm_goal_action.cpp |

## 节点：SetIcpActive (SetIcpActive)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | icp_active | bool | （无描述） | src/robot_ctrl/src/bt_plugins/action/set_icp_matcher_action.cpp |

## 节点：SetRobotState (SetRobotState)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | robot_state | std::string | Robot working state (e.g., IDLE, NAVIGATING, CHARGING) | src/robot_ctrl/src/bt_plugins/action/set_robot_state_action.cpp |

**隐式黑板键（未在 ports 中声明）**

- `robot_state_msg`

## 节点：TaskUpdate (TaskUpdate)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Output | nav_source_room | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/condition/task_update.cpp |
| Output | nav_target_room | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/condition/task_update.cpp |
| Output | task_command | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/condition/task_update.cpp |
| Output | task_id | std::string | （无描述） | src/robot_ctrl/src/bt_plugins/condition/task_update.cpp |
| Output | target_pot_num | int | （无描述） | src/robot_ctrl/src/bt_plugins/condition/task_update.cpp |
| Input | task_locked | bool | （无描述） | src/robot_ctrl/src/bt_plugins/condition/task_update.cpp |
| Input | robot_state_msg | robot_ctrl::msg::RobotState | （无描述） | src/robot_ctrl/src/bt_plugins/condition/task_update.cpp |

## 节点：UndockRobot (UndockRobotActionBT)

| 方向 | 参数名 | 类型 | 作用/说明 | 来源文件 |
| --- | --- | --- | --- | --- |
| Input | charge_stop | bool | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/action/undock_robot_action.hpp |
| Input | charge_enable | bool | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/action/undock_robot_action.hpp |
| Output | action_feedback | std::string | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/action/undock_robot_action.hpp |
| Output | progress | double | （无描述） | src/robot_ctrl/include/robot_ctrl/bt_plugins/action/undock_robot_action.hpp |
