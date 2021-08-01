cmake_minimum_required(VERSION 3.5.0)

project(lane_line_detector_simple)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fconcepts)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(lane_line_detector_simple SHARED
  include/lane_line_detector_simple/lane_line_detector_simple.hpp
  src/lane_line_detector_simple.cpp)
ament_target_dependencies(lane_line_detector_simple
    "OpenCV"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS lane_line_detector_simple
  EXPORT lane_line_detector_simple
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_auto_package()
