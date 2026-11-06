cmake_minimum_required(VERSION 3.16)
project(my_simport_behavior)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(simport_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED src/my_behavior.cpp)
target_link_libraries(${PROJECT_NAME} simport_ros::simport_ros pluginlib::pluginlib)

# Makes the plugin known to simport by the names in plugins.xml.
pluginlib_export_plugin_description_file(simport_ros plugins.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(FILES example.yaml DESTINATION share/${PROJECT_NAME})

ament_package()
