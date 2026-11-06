cmake_minimum_required(VERSION 3.16)
project(simport_ros)

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
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(simport_core REQUIRED)
find_package(simport_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Core ROS integration library: dynamic messages, ROS clock, the device node (also registered
# as a component).
add_library(${PROJECT_NAME} SHARED
  src/device.cpp
  src/device_node.cpp
  src/dynamic_message.cpp
  src/records.cpp
  src/ros_clock.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  simport_core::simport_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  rcpputils::rcpputils
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
  diagnostic_updater::diagnostic_updater
  ${diagnostic_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${simport_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "simport::DeviceNode"
  EXECUTABLE simport_device
)

# Source plugins, loaded through pluginlib only.
add_library(${PROJECT_NAME}_plugins SHARED
  src/point_cloud2_source.cpp
  src/ros_topic_source.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC ${PROJECT_NAME} ${sensor_msgs_TARGETS})
pluginlib_export_plugin_description_file(simport_ros ros_plugins.xml)

# Device file checker: validates a device and prints its packets.
add_executable(simport_check src/simport_check.cpp)
target_link_libraries(simport_check ${PROJECT_NAME})
install(TARGETS simport_check DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# `import simport`: helpers for Python nodes that serve a device's receive services.
ament_python_install_package(simport)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  simport_core simport_msgs pluginlib rclcpp rclcpp_components rclcpp_lifecycle rcpputils
  rosidl_typesupport_introspection_cpp lifecycle_msgs diagnostic_msgs diagnostic_updater
  sensor_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cpplint misclassifies C++20 headers; see simport_core/CMakeLists.txt.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-build/include_order")

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_dynamic_message.cpp
    test/test_ros_topic_source.cpp
    test/test_records.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${PROJECT_NAME}_plugins
    ${sensor_msgs_TARGETS})
endif()

ament_package()
