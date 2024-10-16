#ifndef LAYERED_HARDWARE_IGN_IGN_LAYER_INTERFACE_HPP
#define LAYERED_HARDWARE_IGN_IGN_LAYER_INTERFACE_HPP

#include <memory>
#include <string>

#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware_ign/common_namespaces.hpp>

// #include <gazebo/physics/physics.hh>

namespace layered_hardware_ign {

class IgnitionLayerInterface : public lh::LayerInterface {
public:
  // initSim() for gazebo layers
  virtual bool initSim(const std::string &layer_name, rclcpp::Node::SharedPtr &model_nh,
                       std::map<std::string, ig::Entity> &joints,
                       const hi::HardwareInfo &hardware_info, ig::EntityComponentManager &ecm,
                       int &update_rate) = 0;

protected:
  // disabled version of on_init() for non-gazebo layers
  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo & /*hardware_info*/) override final {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware_ign"),
                        "IgnitionLayerInterface::on_init(): \""
                            << layer_name << "\" initialized as a normal layer. "
                            << "Please initialize as an ignition layer using initSim()");
    return CallbackReturn::ERROR;
  }
};

typedef std::shared_ptr<IgnitionLayerInterface> IgnitionLayerPtr;
typedef std::shared_ptr<const IgnitionLayerInterface> IgnitionLayerConstPtr;
} // namespace layered_hardware_ign

#endif