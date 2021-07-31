cmake_minimum_required(VERSION 3.5.0)

project(via_simulation_bridge)

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
find_package(simple_websocket_server REQUIRED)
find_package(via_common REQUIRED)

include_directories(include/via_simulation_bridge) # tmp
ament_auto_find_build_dependencies()
ament_auto_add_library(via_simulation_bridge SHARED
  include/via_simulation_bridge/via_simulation_bridge.hpp
  include/via_simulation_bridge/base64.hpp
  src/via_simulation_bridge.cpp src/base64.cpp)
ament_target_dependencies(via_simulation_bridge
    "OpenCV"
    simple_websocket_server
)

# target_include_directories(via_simulation_bridge
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS via_simulation_bridge
  EXPORT via_simulation_bridge
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


ament_auto_package()
