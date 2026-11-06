cmake_minimum_required(VERSION 3.16)
project(simport_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Frame.msg"
  "msg/Record.msg"
  "msg/RecordField.msg"
  "srv/Exchange.srv"
  DEPENDENCIES std_msgs diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
