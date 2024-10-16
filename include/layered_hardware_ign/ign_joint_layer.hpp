#ifndef LAYERED_HARDWARE_IGN_IGN_JOINT_LAYER_HPP
#define LAYERED_HARDWARE_IGN_IGN_JOINT_LAYER_HPP

#include <map>
#include <memory>
#include <string>
#include <utility> // for std::move()
#include <vector>

#include <hardware_interface/handle.hpp> // for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp> // for hi::return_type
#include <layered_hardware/merge_utils.hpp>
#include <layered_hardware/string_registry.hpp>
#include <layered_hardware_ign/common_namespaces.hpp>
#include <layered_hardware_ign/ign_joint_driver.hpp>
#include <layered_hardware_ign/ign_layer_interface.hpp>
#include <layered_hardware_ign/logging_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware_ign {

class IgnitionJointLayer : public IgnitionLayerInterface {
public:
  virtual bool initSim(const std::string &layer_name, rclcpp::Node::SharedPtr &model_nh,
                       std::map<std::string, ig::Entity> &joint_entities,
                       const hi::HardwareInfo &hardware_info, ig::EntityComponentManager &ecm,
                       int &update_rate) override {
    // initialize the base class first
    if (!IgnitionLayerInterface::initSim(layer_name, model_nh, joint_entities, hardware_info, ecm,
                                         update_rate)) {
      return false;
    }

    // find parameter group for this layer
    const auto params_it = hardware_info.hardware_parameters.find(layer_name);
    if (params_it == hardware_info.hardware_parameters.end()) {
      LHI_ERROR("IgnitionJointLayer::initSim(): \"%s\" parameter is missing", layer_name.c_str());
      return false;
    }

    // parse parameters for this layer as yaml
    std::vector<std::string> joint_names;
    std::vector<YAML::Node> joint_params;
    try {
      const YAML::Node layer_params = YAML::Load(params_it->second);
      for (const auto &name_param_pair : layer_params["joints"]) {
        joint_names.emplace_back(name_param_pair.first.as<std::string>());
        joint_params.emplace_back(name_param_pair.second);
      }
    } catch (const YAML::Exception &error) {
      LHI_ERROR("IgnitionJointLayer::initSim(): %s (on parsing \"%s\" parameter)", //
                error.what(), layer_name.c_str());
      return false;
    }

    // find joints in simulator
    std::vector<ig::Joint> joints;
    for (const auto &name : joint_names) {
      // find joint entity by name
      const auto found_it = joint_entities.find(name);
      if (found_it == joint_entities.end()) {
        LHI_ERROR("IgnitionJointLayer::initSim(): \"%s\" joint is not a entity", name.c_str());
        return false;
      }

      // validate joint entity
      const ig::Joint joint(found_it->second);
      if (!joint.Valid(ecm)) {
        LHI_ERROR("IgnitionJointLayer::initSim(): \"%s\" joint is an invalid entity", name.c_str());
        return false;
      }

      joints.emplace_back(std::move(joint));
    }

    // init joint drivers
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      try {
        drivers_.emplace_back(
            new IgnitionJointDriver(joint_names[i], joint_params[i], joints[i], ecm));
      } catch (const std::runtime_error &error) {
        LHI_ERROR("IgnitionJointLayer::initSim(): Failed to create driver for \"%s\" joint: %s",
                  joint_names[i].c_str(), error.what());
        return false;
      }
      LHI_INFO("IgnitionJointLayer::initSim(): Initialized \"%s\"", joint_names[i].c_str());
    }

    return true;
  }

  virtual std::vector<hi::StateInterface> export_state_interfaces() override {
    // export reference to joint states owned by this layer
    std::vector<hi::StateInterface> ifaces;
    for (const auto &driver : drivers_) {
      ifaces = lh::merge(std::move(ifaces), driver->export_state_interfaces());
    }
    return ifaces;
  }

  virtual std::vector<hi::CommandInterface> export_command_interfaces() override {
    // export reference to joint commands owned by this layer
    std::vector<hi::CommandInterface> ifaces;
    for (const auto &driver : drivers_) {
      ifaces = lh::merge(std::move(ifaces), driver->export_command_interfaces());
    }
    return ifaces;
  }

  virtual ci::InterfaceConfiguration state_interface_configuration() const override {
    // any state interfaces required from other layers because this layer is "source"
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual ci::InterfaceConfiguration command_interface_configuration() const override {
    // any command interfaces required from other layers because this layer is "source"
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual void
  assign_interfaces(std::vector<hi::LoanedStateInterface> && /*state_interfaces*/,
                    std::vector<hi::LoanedCommandInterface> && /*command_interfaces*/) override {
    // any interfaces has to be imported from other layers because this layer is "source"
  }

  virtual hi::return_type
  prepare_command_mode_switch(const lh::StringRegistry &active_interfaces) override {
    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->prepare_command_mode_switch(active_interfaces));
    }
    return result;
  }

  virtual hi::return_type
  perform_command_mode_switch(const lh::StringRegistry &active_interfaces) override {
    // notify controller switching to all actuators
    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->perform_command_mode_switch(active_interfaces));
    }
    return result;
  }

  virtual hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // read from all drivers
    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->read(time, period));
    }
    return result;
  }

  virtual hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // write to all drivers
    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->write(time, period));
    }
    return result;
  }

private:
  std::vector<std::unique_ptr<IgnitionJointDriver>> drivers_;
};

} // namespace layered_hardware_ign

#endif