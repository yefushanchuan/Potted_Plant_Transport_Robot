cmake_minimum_required(VERSION 3.8)
project(plant_detector)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation kdtree search)

# Custom message (cluster result)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlantCluster.msg"
  "msg/PlantClusterArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# ── Include dirs ─────────────────────────────────────────────────────────────
target_include_directories(plant_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── Main node executable ─────────────────────────────────────────────────────
add_library(plant_detector_component SHARED
  src/plant_detector_node.cpp
  src/ground_remover.cpp
  src/euclidean_cluster.cpp
  src/plant_classifier.cpp
)

ament_target_dependencies(plant_detector_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(plant_detector_node
  ${PCL_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(plant_detector_node "${cpp_typesupport_target}")

rclcpp_components_register_node(plant_detector_component
  PLUGIN "plant_detector::PlantDetectorNode"
  EXECUTABLE plant_detector_node
)
# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS plant_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()