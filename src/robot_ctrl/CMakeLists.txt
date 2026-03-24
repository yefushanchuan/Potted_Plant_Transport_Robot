cmake_minimum_required(VERSION 3.8)
project(robot_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(nlohmann_json REQUIRED)

find_package(pheno_module_controller REQUIRED)
# find_package(pot_detection REQUIRED)

# find_package(robot_base REQUIRED) 
# find_package(robot_charge REQUIRED)


include_directories(include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} 
  ${std_msgs_INCLUDE_DIRS}
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/BtTaskFileUpload.action"
  
  "msg/RobotState.msg"
  "msg/RoomposeConstants.msg"
  "srv/GetRoompose.srv"
  "srv/RecorderPose.srv"
  "srv/TaskUpdata.srv"
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

link_directories(/usr/local/lib)

install(DIRECTORY include/
  DESTINATION include
)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_ctrl/roompose_defs.hpp
  DESTINATION include/robot_ctrl)
#=============== 自定义服务节点 ==================================
#### 房间坐标加载节点
add_executable(roompose_loader src/robot_service/roompose_loader.cpp)
ament_target_dependencies(roompose_loader 
  rclcpp 
  std_msgs
  nlohmann_json
  )
target_link_libraries(roompose_loader ${cpp_typesupport_target})

#### 房间坐标记录节点
add_executable(roompose_recorder src/robot_service/roompose_recorder.cpp)
ament_target_dependencies(roompose_recorder 
  rclcpp 
  std_msgs
  nlohmann_json
  tf2
  tf2_ros
  tf2_geometry_msgs
  )
target_link_libraries(roompose_recorder ${cpp_typesupport_target})

install(TARGETS 
  roompose_loader
  roompose_recorder
  DESTINATION lib/${PROJECT_NAME})

# ================ 行为树插件 =============================
add_library(robot_ctrl_bt_plugins SHARED
  src/bt_plugins/condition/task_update.cpp
  src/bt_plugins/condition/is_task_command_condition.cpp
  src/bt_plugins/action/get_goal_from_room_action.cpp
  src/bt_plugins/action/clear_task_command_action.cpp
  src/bt_plugins/action/nav_to_pose_action.cpp
  src/bt_plugins/action/cancel_nav_to_pose_action.cpp
  src/bt_plugins/action/launch_manager_action.cpp
  src/bt_plugins/decorator/repeat_with_blackboard.cpp
  src/bt_plugins/condition/log_error_condition.cpp
  src/bt_plugins/decorator/retry_decorator.cpp
  src/bt_plugins/condition/is_ros_node_ok_condition.cpp
  src/bt_plugins/condition/is_goal_reached_condition.cpp
  src/bt_plugins/action/log_error_action.cpp
  src/bt_plugins/action/set_icp_matcher_action.cpp
  src/bt_plugins/action/set_robot_state_action.cpp
  src/bt_plugins/action/publish_robot_state_action.cpp
  src/bt_plugins/condition/is_charging_condition.cpp
  src/bt_plugins/condition/is_robot_init_success_condition.cpp
  src/bt_plugins/control/recovery_node.cpp
  src/bt_plugins/condition/is_robot_started_condition.cpp
  src/bt_plugins/condition/is_battery_low_condition.cpp
  src/bt_plugins/condition/is_time_for_action_condition.cpp

  ##自动充电子模块
  # src/bt_plugins/action/dock_robot_action.cpp
  # src/bt_plugins/action/undock_robot_action.cpp

  ##表型子模块
  src/bt_plugins/action/camera_capture_action.cpp
  src/bt_plugins/action/taskfile_upload_action.cpp
  src/bt_plugins/action/taskfile_retry_queue_storage.cpp
  src/bt_plugins/action/record_failed_taskfile_action.cpp
  src/bt_plugins/action/get_pending_taskfile_action.cpp
  src/bt_plugins/action/ack_pending_taskfile_action.cpp
  src/bt_plugins/action/patrol_plan_init_action.cpp
  src/bt_plugins/action/patrol_select_waypoint_action.cpp
  src/bt_plugins/action/patrol_advance_index_action.cpp
  
  ##灌溉子模块
  # src/bt_plugins/condition/is_pot_detected_condition.cpp
)

target_include_directories(robot_ctrl_bt_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(robot_ctrl_bt_plugins
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  std_srvs
  nav2_msgs
  rclcpp_action
  apriltag_msgs
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  pheno_module_controller
  # pot_detection
  # robot_charge
)

target_link_libraries(robot_ctrl_bt_plugins 
  ${cpp_typesupport_target}
  ${rclcpp_action_LIBRARIES}
  )

add_executable(bt_main src/bt_main.cpp)
ament_target_dependencies(bt_main rclcpp behaviortree_cpp_v3)
target_link_libraries(bt_main
  robot_ctrl_bt_plugins  
)

install(TARGETS bt_main DESTINATION lib/${PROJECT_NAME})


pluginlib_export_plugin_description_file(behaviortree_cpp_v3 config/behavior_trees/other/bt_plugins.xml)
install(TARGETS robot_ctrl_bt_plugins
  LIBRARY DESTINATION lib
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

install(
  DIRECTORY launch config roompose
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/launch_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
