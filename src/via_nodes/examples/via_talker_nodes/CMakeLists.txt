cmake_minimum_required(VERSION 3.5)

### Build the nodes
project(via_talker_nodes)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(via_msgs REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_executable(via_talker_node
  "include/via_talker_nodes/via_talker_node.hpp"
  "src/via_talker_node.cpp")
ament_target_dependencies(via_talker_node rclcpp std_msgs)

ament_auto_add_executable(via_talker_2ints_node
  "include/via_talker_nodes/via_talker_2ints_node.hpp"
  "src/via_talker_2ints_node.cpp")
ament_target_dependencies(via_talker_2ints_node rclcpp via_msgs)

install(TARGETS
  via_talker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  via_talker_2ints_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()