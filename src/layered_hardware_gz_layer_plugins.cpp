#include <layered_hardware_gz/gz_joint_layer.hpp>
#include <layered_hardware_gz/gz_layer_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(layered_hardware_gz::GazeboSimJointLayer,
                       layered_hardware_gz::GazeboSimLayerInterface);