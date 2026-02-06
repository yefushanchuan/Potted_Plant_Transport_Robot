cmake_minimum_required(VERSION 3.8)
project(robot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_path(LIBSERIAL_INCLUDE_DIR NAMES libserial/SerialPort.h)
find_library(LIBSERIAL_LIBRARY NAMES serial)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(livox_ros_driver2 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgrobotInfo.msg"
  "srv/SetControlMode.srv"
)

#=============  agrobot_hardware_interface node==================================
add_library(agrobot_hardware_interface SHARED
  src/agrobot_hardware_interface.cpp
)
target_include_directories(agrobot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${LIBSERIAL_INCLUDE_DIR}
)
ament_target_dependencies(agrobot_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_control
  tf2
  tf2_geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(agrobot_hardware_interface ${LIBSERIAL_LIBRARY} ${cpp_typesupport_target})

pluginlib_export_plugin_description_file(hardware_interface config/agrobot_hardware.xml)

# ================ laser filter node  ==================================
add_library(laser_filter_node SHARED
  src/laser_filter.cpp
)
target_include_directories(laser_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(laser_filter_node
  rclcpp
  sensor_msgs
  rclcpp_components
)
rclcpp_components_register_node(laser_filter_node PLUGIN "CLidarFilter" EXECUTABLE "laser_filter_node_exe")

# # ==========Build Tilt Compensator (CustomMsg only)=========================
# add_executable(tilt_compensator src/tilt_compensator.cpp)
# target_include_directories(tilt_compensator PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   ${Eigen3_INCLUDE_DIRS}
# )
# ament_target_dependencies(tilt_compensator
#   rclcpp
#   sensor_msgs
#   livox_ros_driver2
#   Eigen3
# )

install(TARGETS
    agrobot_hardware_interface
    laser_filter_node
    # tilt_compensator
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch config urdf rviz
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
        scripts/angular_calibration.py
        scripts/linear_calibration.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
