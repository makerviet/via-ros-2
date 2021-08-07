cmake_minimum_required(VERSION 3.5)

### Build the nodes
project(generic_camera_node)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(generic_camera_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(via_definitions REQUIRED)
find_package(via_converters REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_executable(generic_camera_node
  "src/generic_camera_node.cpp")

ament_target_dependencies(generic_camera_node
  std_srvs generic_camera_driver OpenCV rclcpp rclcpp_components
  camera_info_manager image_transport camera_calibration_parsers
  via_definitions via_converters)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_auto_package()