cmake_minimum_required(VERSION 3.8)
project(dual_descriptor_relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(dual_descriptor_node
  src/main.cpp
  src/dual_descriptor_node.cpp
)

target_link_libraries(dual_descriptor_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(dual_descriptor_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS dual_descriptor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
