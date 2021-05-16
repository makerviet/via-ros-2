cmake_minimum_required(VERSION 3.5.0)

project(generic_camera_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${cv_bridge_INCLUDE_DIRS})

add_library(generic_camera_driver SHARED
  include/generic_camera_driver/generic_camera_driver.hpp
  src/generic_camera_driver.cpp)
target_compile_definitions(generic_camera_driver
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(generic_camera_driver
    "OpenCV")

target_include_directories(generic_camera_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS generic_camera_driver
  EXPORT generic_camera_driver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(generic_camera_driver)

ament_package()
