#include <gz_ros2_control/gz_system_interface.hpp>
#include <layered_hardware_ign/layered_hardware_ign.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(layered_hardware_ign::LayeredHardwareIgnition,
                       gz_ros2_control::GazeboSimSystemInterface);