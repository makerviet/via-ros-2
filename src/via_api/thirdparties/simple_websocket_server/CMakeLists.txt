cmake_minimum_required(VERSION 3.0)

project(simple_websocket_server)

if(POLICY CMP0057)
cmake_policy(SET CMP0057 NEW)
endif()

include_directories("include/simple_websocket_server")

find_package(ament_cmake REQUIRED)

option(USE_STANDALONE_ASIO "set ON to use standalone Asio instead of Boost.Asio" OFF)
if(CMAKE_SOURCE_DIR STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    option(BUILD_TESTING "set ON to build library tests" ON)
else()
    option(BUILD_TESTING "set ON to build library tests" OFF)
endif()

add_library(simple_websocket_server INTERFACE)

target_include_directories(simple_websocket_server INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Threads REQUIRED)
target_link_libraries(simple_websocket_server INTERFACE ${CMAKE_THREAD_LIBS_INIT})

# TODO 2020 when Debian Jessie LTS ends:
# Remove Boost system, thread, regex components; use Boost::<component> aliases; remove Boost target_include_directories
if(USE_STANDALONE_ASIO)
    target_compile_definitions(simple_websocket_server INTERFACE ASIO_STANDALONE)
    find_path(ASIO_PATH asio.hpp)
    if(NOT ASIO_PATH)
        message(FATAL_ERROR "Standalone Asio not found")
    else()
        target_include_directories(simple_websocket_server INTERFACE ${ASIO_PATH})
    endif()
else()
    find_package(Boost 1.54.0 COMPONENTS system thread coroutine context REQUIRED)
    target_link_libraries(simple_websocket_server INTERFACE ${Boost_LIBRARIES})
    target_include_directories(simple_websocket_server INTERFACE ${Boost_INCLUDE_DIR})
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
        target_compile_definitions(simple_websocket_server INTERFACE USE_BOOST_REGEX)
        find_package(Boost 1.54.0 COMPONENTS regex REQUIRED)
        target_link_libraries(simple_websocket_server INTERFACE ${Boost_LIBRARIES})
        target_include_directories(simple_websocket_server INTERFACE ${Boost_INCLUDE_DIR})
    endif()
endif()
if(WIN32)
    target_link_libraries(simple_websocket_server INTERFACE ws2_32 wsock32)
endif()

if(APPLE)
    set(OPENSSL_ROOT_DIR "/usr/local/opt/openssl")
endif()
find_package(OpenSSL REQUIRED)
target_link_libraries(simple_websocket_server INTERFACE ${OPENSSL_LIBRARIES})
target_include_directories(simple_websocket_server INTERFACE ${OPENSSL_INCLUDE_DIR})

# install(
#   FILES asio_compatibility.hpp server_ws.hpp client_ws.hpp server_wss.hpp client_wss.hpp crypto.hpp utility.hpp status_code.hpp mutex.hpp DESTINATION
#   DESTINATION include/simple_websocket_server
# )

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS simple_websocket_server
  EXPORT simple_websocket_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS simple_websocket_server
  EXPORT simple_websocket_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
# ament_export_libraries(simple_websocket_server)
ament_package()
