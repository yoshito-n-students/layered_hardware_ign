#include <ign_ros2_control/ign_system_interface.hpp>
#include <layered_hardware_ign/ign_joint_layer.hpp>
#include <layered_hardware_ign/ign_layer_interface.hpp>
#include <layered_hardware_ign/layered_hardware_ign.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(layered_hardware_ign::LayeredHardwareIgnition,
                       ign_ros2_control::IgnitionSystemInterface)
PLUGINLIB_EXPORT_CLASS(layered_hardware_ign::IgnitionJointLayer,
                       layered_hardware_ign::IgnitionLayerInterface);