cmake_minimum_required(VERSION 3.5)

### Build the nodes
project(via_listener_nodes)

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
ament_auto_add_executable(via_listener_node
  "include/via_listener_nodes/via_listener_node.hpp"
  "src/via_listener_node.cpp")

ament_auto_add_executable(via_listener_2ints_node
  "include/via_listener_nodes/via_listener_2ints_node.hpp"
  "src/via_listener_2ints_node.cpp")
ament_target_dependencies(via_listener_2ints_node rclcpp via_msgs)


ament_target_dependencies(via_listener_node rclcpp)

install(TARGETS
  via_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  via_listener_2ints_node
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