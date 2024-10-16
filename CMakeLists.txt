cmake_minimum_required(VERSION 3.8)
project(layered_hardware_ign)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ign_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(layered_hardware REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(ignition-gazebo6 REQUIRED)

## Declare a C++ library
add_library(
  layered_hardware_ign_plugins SHARED
  src/layered_hardware_ign_plugins.cpp)
target_include_directories(layered_hardware_ign_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
  layered_hardware_ign_plugins
  ignition-gazebo6::core
)
ament_target_dependencies(
  layered_hardware_ign_plugins
  controller_interface
  ign_ros2_control
  hardware_interface
  layered_hardware
  pluginlib
  rclcpp
  yaml_cpp_vendor)

## Declare a C++ executable

pluginlib_export_plugin_description_file(ign_ros2_control plugins.xml)
pluginlib_export_plugin_description_file(layered_hardware_ign plugins.xml)

install(
  TARGETS layered_hardware_ign_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
    DIRECTORY config launch urdf
    DESTINATION share/layered_hardware_ign)

# TODO: declare tests here

ament_package()