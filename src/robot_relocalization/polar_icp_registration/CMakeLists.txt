cmake_minimum_required(VERSION 3.8)
project(polar_icp_registration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(nav_msgs REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN polar_icp::PolarIcpNode
  EXECUTABLE ${PROJECT_NAME}_node
)

# 关键帧提取工具（纯 PCL，不依赖 ROS）
add_executable(kf_extract tools/kf_extract.cpp)
target_link_libraries(kf_extract ${PCL_LIBRARIES})
target_include_directories(kf_extract PRIVATE ${PCL_INCLUDE_DIRS})
install(TARGETS kf_extract DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
