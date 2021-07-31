cmake_minimum_required(VERSION 3.5.0)

project(traffic_sign_detector_yolox)

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

add_subdirectory(ncnn)
execute_process (
    COMMAND "cp ${CMAKE_SOURCE_DIR}/../../../../install/traffic_sign_detector_yolox/include/ncnn/*.h src/via_api/perception/traffic_sign_detector_yolox/include/built_ncnn_headers"
)

include_directories(include/traffic_sign_detector_yolox)
ament_auto_find_build_dependencies()
ament_auto_add_library(traffic_sign_detector_yolox SHARED
  include/traffic_sign_detector_yolox/traffic_sign_detector_yolox.hpp
  src/traffic_sign_detector_yolox.cpp src/utils.cpp)
target_link_libraries(traffic_sign_detector_yolox ncnn)
ament_target_dependencies(traffic_sign_detector_yolox
    "OpenCV"
)

install(
  DIRECTORY include/
  DESTINATION include
)

file(GLOB NCNN_HEADERS include/built_ncnn_headers/*.h*)
install(
  FILES ${NCNN_HEADERS}
  DESTINATION include/traffic_sign_detector_yolox
)

install(
  TARGETS traffic_sign_detector_yolox
  EXPORT traffic_sign_detector_yolox
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_auto_package()
