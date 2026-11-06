cmake_minimum_required(VERSION 3.16)
project(simport_examples)

# Optimize by default: colcon leaves the build type empty, which means no optimization.
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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simport_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Behaviour plugins of the devices that answer in process (devices/<name>/*.cpp).
add_library(${PROJECT_NAME}_behaviors SHARED devices/damiao/damiao_bus.cpp)
target_link_libraries(${PROJECT_NAME}_behaviors PUBLIC
  simport_ros::simport_ros pluginlib::pluginlib rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})
pluginlib_export_plugin_description_file(simport_ros behaviors.xml)
install(TARGETS ${PROJECT_NAME}_behaviors LIBRARY DESTINATION lib)

# The device folders: device files, behaviour scripts, data. Sources and tests stay here.
install(DIRECTORY devices DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS
  PATTERN "*.cpp" EXCLUDE PATTERN "*.hpp" EXCLUDE PATTERN "test_*" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cpplint misclassifies C++20 headers; see simport_core/CMakeLists.txt.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  # Header guards relative to this package, not to wherever the repository root happens to be.
  ament_cpplint(FILTERS "-build/include_order" ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_damiao devices/damiao/test_damiao.cpp)
  target_include_directories(test_damiao PRIVATE devices/damiao)
  target_link_libraries(test_damiao ${PROJECT_NAME}_behaviors)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_dynamixel devices/dynamixel/test_behavior.py)
endif()

ament_package()
