cmake_minimum_required(VERSION 3.5)

project(via_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(via_definitions REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include ${cv_bridge_INCLUDE_DIRS})

add_library(via_common STATIC
  include/via_common/node_handler.hpp
  src/node_handler.cpp)

ament_target_dependencies(via_common sensor_msgs via_definitions)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS via_common
  EXPORT via_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(via_common)
ament_package()