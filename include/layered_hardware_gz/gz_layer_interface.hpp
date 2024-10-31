#ifndef LAYERED_HARDWARE_GZ_GZ_LAYER_INTERFACE_HPP
#define LAYERED_HARDWARE_GZ_GZ_LAYER_INTERFACE_HPP

#include <map>
#include <string>

#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware_gz/common_namespaces.hpp>
#include <layered_hardware_gz/logging_utils.hpp>
#include <rclcpp/node.hpp>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace layered_hardware_gz {

class GazeboSimLayerInterface : public lh::LayerInterface {
public:
  // initSim() for gazebo layers
  virtual bool initSim(const std::string &layer_name, rclcpp::Node::SharedPtr &model_nh,
                       std::map<std::string, gs::Entity> &joints,
                       const hi::HardwareInfo &hardware_info, gs::EntityComponentManager &ecm,
                       unsigned int update_rate) = 0;

protected:
  // disabled version of on_init() for non-gazebo layers
  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo & /*hardware_info*/) override final {
    LHG_ERROR("GazebosimLayerInterface::on_init(): \"%s\" initialized as a normal layer. "
              "Please initialize as an ignition layer using initSim()",
              layer_name.c_str());
    return CallbackReturn::ERROR;
  }
};

} // namespace layered_hardware_gz

#endif