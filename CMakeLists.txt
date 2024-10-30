cmake_minimum_required(VERSION 3.8)
project(layered_hardware_ign)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(include)

## Find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ign_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(layered_hardware REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
# yaml_cpp_vendor builds yaml-cpp if not installed
find_package(yaml_cpp_vendor REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(yaml-cpp)

## Declare a C++ library
add_library(
  layered_hardware_ign_hardware_plugins SHARED
  src/layered_hardware_ign_hardware_plugins.cpp)
target_include_directories(
  layered_hardware_ign_hardware_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  layered_hardware_ign_hardware_plugins
  controller_interface
  ign_ros2_control
  hardware_interface
  layered_hardware
  pluginlib
  rclcpp
  yaml_cpp_vendor)
target_link_libraries(
  layered_hardware_ign_hardware_plugins
  ignition-gazebo6::core
  ${YAML_CPP_LIBRARIES})

add_library(
  layered_hardware_ign_layer_plugins SHARED
  src/layered_hardware_ign_layer_plugins.cpp)
target_include_directories(
  layered_hardware_ign_layer_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  layered_hardware_ign_layer_plugins
  controller_interface
  ign_ros2_control
  hardware_interface
  layered_hardware
  pluginlib
  rclcpp
  yaml_cpp_vendor)
target_link_libraries(
  layered_hardware_ign_layer_plugins
  ignition-gazebo6::core
  ${YAML_CPP_LIBRARIES})

## Install resources
pluginlib_export_plugin_description_file(ign_ros2_control hardware_plugins.xml)
pluginlib_export_plugin_description_file(layered_hardware_ign layer_plugins.xml)
install(
  TARGETS layered_hardware_ign_hardware_plugins layered_hardware_ign_layer_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
  DIRECTORY examples
  DESTINATION share/layered_hardware_ign)

# TODO: declare tests here

ament_package()