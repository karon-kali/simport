cmake_minimum_required(VERSION 3.16)
project(simport_core)

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
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/checksum.cpp
  src/clock.cpp
  src/config.cpp
  src/device_config.cpp
  src/input_channel.cpp
  src/logger.cpp
  src/mapping.cpp
  src/output_channel.cpp
  src/schema.cpp
  src/time_extrapolator.cpp
  src/value.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC yaml-cpp::yaml-cpp
  PRIVATE ament_index_cpp::ament_index_cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cpplint predates C++20 and classifies <span>, <format>, <stop_token> as C headers, so its
  # include-order check gives false positives. Run it ourselves with that check disabled.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-build/include_order")

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_checksum.cpp
    test/test_clock.cpp
    test/test_config.cpp
    test/test_device_config.cpp
    test/test_input_channel.cpp
    test/test_interfaces.cpp
    test/test_mapping.cpp
    test/test_output_channel.cpp
    test/test_schema.cpp
    test/test_time_extrapolator.cpp
    test/test_value.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_package()
