cmake_minimum_required(VERSION 3.5.0)

project(traffic_sign_detector_simple)

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
find_package(via_common REQUIRED)
find_package(via_definitions REQUIRED)

include_directories(include/traffic_sign_detector_simple)
ament_auto_find_build_dependencies()
ament_auto_add_library(traffic_sign_detector_simple SHARED
  include/traffic_sign_detector_simple/traffic_sign_detector_simple.hpp
  src/traffic_sign_detector_simple.cpp)
ament_target_dependencies(traffic_sign_detector_simple
    "OpenCV"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS traffic_sign_detector_simple
  EXPORT traffic_sign_detector_simple
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_auto_package()
