#include <gz_ros2_control/gz_system_interface.hpp>
#include <layered_hardware_gz/layered_hardware_gz.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(layered_hardware_gz::LayeredHardwareGazeboSim,
                       gz_ros2_control::GazeboSimSystemInterface);