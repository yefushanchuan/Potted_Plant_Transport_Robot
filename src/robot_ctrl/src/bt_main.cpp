#include <memory>
#include <string>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "robot_service/robot_state_publisher.hpp" 
#include "robot_ctrl/robot_state_utils.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

// 自定义节点头文件
#include "robot_ctrl/bt_plugins/action/get_goal_from_room_action.hpp"
#include "robot_ctrl/bt_plugins/action/clear_task_command_action.hpp"
#include "robot_ctrl/bt_plugins/condition/is_task_command_condition.hpp"
#include "robot_ctrl/bt_plugins/condition/task_update.hpp"
#include "robot_ctrl/bt_plugins/action/nav_to_pose_action.hpp"
#include "robot_ctrl/bt_plugins/action/cancel_nav_to_pose_action.hpp"
#include "robot_ctrl/bt_plugins/condition/is_goal_reached_condition.hpp"
#include "robot_ctrl/bt_plugins/decorator/repeat_with_blackboard.hpp"
#include "robot_ctrl/bt_plugins/condition/log_error_condition.hpp"
#include "robot_ctrl/bt_plugins/decorator/retry_decorator.hpp"
#include "robot_ctrl/bt_plugins/action/task_locked_action.hpp"
#include "robot_ctrl/bt_plugins/condition/is_ros_node_ok_condition.hpp"
#include "robot_ctrl/bt_plugins/action/set_icp_matcher_action.hpp"
#include "robot_ctrl/bt_plugins/action/set_robot_state_action.hpp"
#include "robot_ctrl/bt_plugins/action/publish_robot_state_action.hpp"
#include "robot_ctrl/bt_plugins/condition/is_charging_condition.hpp"
#include "robot_ctrl/bt_plugins/condition/is_robot_init_success_condition.hpp"
#include "robot_ctrl/bt_plugins/condition/is_time_for_action_condition.hpp"
#include "robot_ctrl/bt_plugins/control/recovery_node.hpp"
#include "robot_ctrl/bt_plugins/action/launch_manager_action.hpp"
#include "robot_ctrl/bt_plugins/condition/is_robot_started_condition.hpp"
#include "robot_ctrl/bt_plugins/condition/is_battery_low_condition.hpp"
#include "robot_ctrl/bt_plugins/action/patrol_plan_init_action.hpp"
#include "robot_ctrl/bt_plugins/action/patrol_select_waypoint_action.hpp"
#include "robot_ctrl/bt_plugins/action/patrol_advance_index_action.hpp"
//充电子模块节点
// #include "robot_ctrl/bt_plugins/action/undock_robot_action.hpp"
// #include "robot_ctrl/bt_plugins/action/dock_robot_action.hpp"
//灌溉子模块节点
// #include "robot_ctrl/bt_plugins/condition/is_pot_detected_condition.hpp"
//表型子模块节点
#include "robot_ctrl/bt_plugins/action/camera_capture_action.hpp"
#include "robot_ctrl/bt_plugins/action/taskfile_upload_action.hpp"
#include "robot_ctrl/bt_plugins/action/record_failed_taskfile_action.hpp"
#include "robot_ctrl/bt_plugins/action/get_pending_taskfile_action.hpp"
#include "robot_ctrl/bt_plugins/action/ack_pending_taskfile_action.hpp"
#include "robot_ctrl/bt_plugins/action/taskfile_retry_queue_storage.hpp"


int main(int argc, char **argv)
{
    // 1️⃣ 初始化 ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_runner");

    // 2️⃣ 从参数读取行为树 XML 路径和黑板初始值
    node->declare_parameter<std::string>("bt_xml_file", "/home/user/agrobot_ws/src/robot_ctrl/config/behavior_trees/test_tree.xml");
    node->declare_parameter<std::string>("error_log_file", "src/robot_ctrl/logs/error_log.txt");
    node->declare_parameter<bool>("task_locked", false);
    node->declare_parameter<int>("target_pot_num", 0);
    node->declare_parameter<std::string>("error_message", "IDLE");
    node->declare_parameter<std::string>("robot_state", "IDLE");
    node->declare_parameter<int>("bt_loop_duration", 100);         // ms
    node->declare_parameter<int>("server_timeout", 1000);           // ms
    node->declare_parameter<int>("wait_for_service_timeout", 500);  // ms
    node->declare_parameter<int>("patrol_min_interval_sec", 3600);
    node->declare_parameter<int>("patrol_trigger_hour", -1);
    node->declare_parameter<int>("patrol_trigger_minute", 0);
    node->declare_parameter<int>("patrol_window_seconds", 5);
    node->declare_parameter<std::string>("taskfile_retry_queue_file", "src/robot_ctrl/logs/taskfile_retry_queue.txt");

    std::string xml_file;
    std::string error_log_file;
    bool task_locked;
    int target_pot_num;
    std::string error_message;
    std::string robot_state;
    int bt_loop_duration, server_timeout, wait_for_service_timeout;
    int patrol_min_interval_sec, patrol_trigger_hour, patrol_trigger_minute, patrol_window_seconds;
    std::string taskfile_retry_queue_file;

    node->get_parameter("bt_xml_file", xml_file);
    node->get_parameter("error_log_file", error_log_file);
    node->get_parameter("task_locked", task_locked);
    node->get_parameter("target_pot_num", target_pot_num);
    node->get_parameter("error_message", error_message);
    node->get_parameter("robot_state", robot_state);
    node->get_parameter("bt_loop_duration", bt_loop_duration);
    node->get_parameter("server_timeout", server_timeout);
    node->get_parameter("wait_for_service_timeout", wait_for_service_timeout);
    node->get_parameter("patrol_min_interval_sec", patrol_min_interval_sec);
    node->get_parameter("patrol_trigger_hour", patrol_trigger_hour);
    node->get_parameter("patrol_trigger_minute", patrol_trigger_minute);
    node->get_parameter("patrol_window_seconds", patrol_window_seconds);
    node->get_parameter("taskfile_retry_queue_file", taskfile_retry_queue_file);

    // 3️⃣ 创建行为树工厂
    BT::BehaviorTreeFactory factory;

    // 4️⃣ 注册自定义节点
    //actions
    factory.registerNodeType<robot_ctrl::GetNavGoalFromRoom>("GetGoalFromRoom");
    factory.registerNodeType<robot_ctrl::ClearTaskCommandNode>("ClearTaskCommand");
    factory.registerNodeType<robot_ctrl::Nav2Pose>("NavToPose");
    factory.registerNodeType<robot_ctrl::CancelNavToPose>("CancelNavToPose");
    factory.registerNodeType<robot_ctrl::SetTaskLockedAction>("SetTaskLocked");
    factory.registerNodeType<robot_ctrl::SetIcpActive>("SetIcpActive");
    factory.registerNodeType<robot_ctrl::SetRobotState>("SetRobotState");
    factory.registerNodeType<robot_ctrl::CameraCaptureAction>("CameraCapture");
    
    factory.registerNodeType<robot_ctrl::TaskfileUploadAction>("TaskfileUpload");
    factory.registerNodeType<robot_ctrl::RecordFailedTaskfileAction>("RecordFailedTaskfile");
    factory.registerNodeType<robot_ctrl::GetPendingTaskfileAction>("GetPendingTaskfile");
    factory.registerNodeType<robot_ctrl::AckPendingTaskfileAction>("AckPendingTaskfile");

    factory.registerNodeType<robot_ctrl::PatrolPlanInitAction>("PatrolPlanInit");
    factory.registerNodeType<robot_ctrl::PatrolSelectWaypointAction>("PatrolSelectWaypoint");
    factory.registerNodeType<robot_ctrl::PatrolAdvanceIndexAction>("PatrolAdvanceIndex");
    // factory.registerNodeType<robot_ctrl::DockRobotActionBT>("DockRobot");
    // factory.registerNodeType<robot_ctrl::UndockRobotActionBT>("UndockRobot");
    factory.registerNodeType<robot_ctrl::LaunchManagerAction>("LaunchManager");
    //conditions
    factory.registerNodeType<robot_ctrl::IsCommand>("IsTaskCommand");
    factory.registerNodeType<robot_ctrl::TaskUpdate>("TaskUpdate");
    factory.registerNodeType<robot_ctrl::IsRosNodeOk>("IsRosNodeOk");
    factory.registerNodeType<robot_ctrl::IsGoalReached>("IsGoalReached");
    // factory.registerNodeType<robot_ctrl::IsPotDetected>("IsPotDetected");
    factory.registerNodeType<robot_ctrl::IsCharging>("IsCharging");
    factory.registerNodeType<robot_ctrl::IsRobotInitSuccess>("IsRobotInitSuccess");
    factory.registerNodeType<robot_ctrl::IsRobotStarted>("IsRobotStarted");
    factory.registerNodeType<robot_ctrl::IsBatteryLow>("IsBatteryLow");
    factory.registerNodeType<robot_ctrl::IsTimeForAction>("IsTimeForAction");
    //decorators
    factory.registerNodeType<robot_ctrl::RepeatWithBlackboard>("RepeatWithBlackboard");
    factory.registerNodeType<robot_ctrl::LogErrorCondition>("LogError");
    factory.registerNodeType<robot_ctrl::RetryUntilSuccessful>("Retry");
    //control nodes
    factory.registerNodeType<robot_ctrl::RecoveryNode>("RecoveryNode");

    // 5️⃣ 创建黑板并初始化参数
    robot_ctrl::msg::RobotState initial_state;
    if (!robot_ctrl::robot_state::fromString(robot_state, initial_state))
    {
        RCLCPP_WARN(node->get_logger(), "Unknown initial robot_state %s, fallback to IDLE", robot_state.c_str());
        robot_ctrl::robot_state::fromString("IDLE", initial_state);
    }

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    blackboard->set("error_log_file", error_log_file);
    blackboard->set("task_locked", task_locked);
    blackboard->set("robot_init_success", "false");
    blackboard->set("taskfile_update_flag", "false");
    blackboard->set("robot_started", "false");
    blackboard->set("target_pot_num", target_pot_num);
    blackboard->set("error_message", error_message);
    blackboard->set("robot_state", initial_state.label);
    blackboard->set("robot_state_msg", initial_state);
    blackboard->set("task_command", "IDLE");
    blackboard->set("task_id", "");
    blackboard->set(robot_ctrl::taskfile_retry_queue::kRetryQueueFilePathKey, taskfile_retry_queue_file);
    blackboard->set(robot_ctrl::taskfile_retry_queue::kFailedTaskIdsKey, std::vector<std::string>{});
    blackboard->set("patrol_min_interval_sec", patrol_min_interval_sec);
    blackboard->set("patrol_trigger_hour", patrol_trigger_hour);
    blackboard->set("patrol_trigger_minute", patrol_trigger_minute);
    blackboard->set("patrol_window_seconds", patrol_window_seconds);
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(bt_loop_duration));
    blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(server_timeout));
    blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", std::chrono::milliseconds(wait_for_service_timeout));

    std::vector<std::string> persisted_failed_task_ids;
    std::string retry_queue_error;
    if (robot_ctrl::taskfile_retry_queue::loadFromFile(taskfile_retry_queue_file, persisted_failed_task_ids, retry_queue_error))
    {
        blackboard->set(robot_ctrl::taskfile_retry_queue::kFailedTaskIdsKey, persisted_failed_task_ids);
        if (!persisted_failed_task_ids.empty())
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Loaded %zu pending taskfile retry items from %s",
                persisted_failed_task_ids.size(),
                taskfile_retry_queue_file.c_str());
        }
    }
    else
    {
        RCLCPP_WARN(
            node->get_logger(),
            "Failed to load taskfile retry queue from %s: %s",
            taskfile_retry_queue_file.c_str(),
            retry_queue_error.c_str());
    }

    std::mutex blackboard_mutex;
    auto on_set_params = node->add_on_set_parameters_callback(
        [blackboard, &blackboard_mutex](const std::vector<rclcpp::Parameter> & params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "";
            std::lock_guard<std::mutex> lock(blackboard_mutex);
            for (const auto & param : params)
            {
                const auto & name = param.get_name();
                if (name == "patrol_min_interval_sec")
                {
                    if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
                    {
                        result.successful = false;
                        result.reason = "patrol_min_interval_sec must be int";
                        return result;
                    }
                    blackboard->set("patrol_min_interval_sec", param.as_int());
                }
                else if (name == "patrol_trigger_hour")
                {
                    if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
                    {
                        result.successful = false;
                        result.reason = "patrol_trigger_hour must be int";
                        return result;
                    }
                    blackboard->set("patrol_trigger_hour", param.as_int());
                }
                else if (name == "patrol_trigger_minute")
                {
                    if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
                    {
                        result.successful = false;
                        result.reason = "patrol_trigger_minute must be int";
                        return result;
                    }
                    blackboard->set("patrol_trigger_minute", param.as_int());
                }
                else if (name == "patrol_window_seconds")
                {
                    if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
                    {
                        result.successful = false;
                        result.reason = "patrol_window_seconds must be int";
                        return result;
                    }
                    blackboard->set("patrol_window_seconds", param.as_int());
                }
            }
            return result;
        });
    (void)on_set_params;

    // 6️⃣ 导出节点模型，方便 Groot 等工具加载
    try
    {
        namespace fs = std::filesystem;
        const fs::path package_share = ament_index_cpp::get_package_share_directory("robot_ctrl");
        fs::path workspace_root = package_share;
        for (int i = 0; i < 4 && workspace_root.has_parent_path(); ++i)
        {
            workspace_root = workspace_root.parent_path();
        }

        fs::path preferred_dir = workspace_root / "src/robot_ctrl/config/behavior_trees";
        bool use_source_dir = fs::exists(preferred_dir);
        if (!use_source_dir)
        {
            preferred_dir = package_share / "config/behavior_trees";
        }

        fs::create_directories(preferred_dir);
        const fs::path models_path = preferred_dir / "generated_bt_models.xml";

        std::ofstream out(models_path.string(), std::ios::trunc);
        if (out)
        {
            out << BT::writeTreeNodesModelXML(factory);
            RCLCPP_INFO(node->get_logger(), "Exported BT node models to %s", models_path.string().c_str());
            if (!use_source_dir)
            {
                RCLCPP_WARN(node->get_logger(),
                            "Source directory not found, models exported to install share directory instead.");
            }
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Failed to write BT node models to %s", models_path.string().c_str());
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(node->get_logger(), "Unable to export BT node models: %s", e.what());
    }

    // 7️⃣ 从 XML 创建行为树
    BT::Tree tree;
    try
    {
        tree = factory.createTreeFromFile(xml_file, blackboard);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to create BehaviorTree from '%s': %s",
                     xml_file.c_str(), e.what());
        rclcpp::shutdown();
        return 1;
    }

    BT::PublisherZMQ zmq_publisher(tree, 1000.0/bt_loop_duration, 1666, 1667);  // ZMQ 发布器

    auto publisher = std::make_shared<RobotStatePublisher>(blackboard);
    // 8️⃣ 控制台日志
    BT::StdCoutLogger logger(tree);

    // 只给 Publisher 单独开线程
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(publisher);          // 只塞 publisher

    std::thread bt_thread([&](){
        rclcpp::Rate rate(1000.0/bt_loop_duration);
        while(rclcpp::ok()){
            tree.tickRoot();           //  ticking 行为树
            spin_some(node);
            rate.sleep();
        }
    });

    exec.spin();    // 专门给 publisher 的 timer 用
    bt_thread.join();


    rclcpp::shutdown();
    return 0;
}
