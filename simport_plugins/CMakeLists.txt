cmake_minimum_required(VERSION 3.16)
project(simport_plugins)

# Optimize by default: colcon leaves the build type empty, which means no optimization, and the
# codecs then run several times slower (a lidar cloud: 16 ms instead of 6 ms to encode).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(simport_core REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/bandwidth.cpp
  src/binary_codec.cpp
  src/fixed_rate.cpp
  src/framers.cpp
  src/http_codec.cpp
  src/on_message.cpp
  src/serial_pty.cpp
  src/socket_can.cpp
  src/tcp_server.cpp
  src/text_codec.cpp
  src/udp.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC simport_core::simport_core
  PRIVATE pluginlib::pluginlib util
)

pluginlib_export_plugin_description_file(simport_core plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(simport_core)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cpplint misclassifies C++20 headers; see simport_core/CMakeLists.txt.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-build/include_order")

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_bandwidth.cpp
    test/test_binary_codec.cpp
    test/test_fixed_rate.cpp
    test/test_framers.cpp
    test/test_on_message.cpp
    test/test_serial_pty.cpp
    test/test_tcp_http.cpp
    test/test_text_codec.cpp
    test/test_udp.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_package()
