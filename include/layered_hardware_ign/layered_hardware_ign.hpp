#ifndef LAYERED_HARDWARE_IGN_LAYERED_HARDWARE_IGN_HPP
#define LAYERED_HARDWARE_IGN_LAYERED_HARDWARE_IGN_HPP

#include <string>
#include <vector>

#include <ign_ros2_control/ign_system_interface.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/layered_hardware.hpp>
#include <layered_hardware_ign/common_namespaces.hpp>
#include <layered_hardware_ign/ign_layer_interface.hpp>
#include <layered_hardware_ign/logging_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

#include <ignition/gazebo/System.hh>

namespace layered_hardware_ign {

class LayeredHardwareIgnition : public ign_ros2_control::IgnitionSystemInterface,
                                public lh::LayeredHardware {
public:
  LayeredHardwareIgnition()
      : ign_ros2_control::IgnitionSystemInterface(), lh::LayeredHardware(),
        ign_layer_loader_("layered_hardware_ign", "layered_hardware_ign::IgnitionLayerInterface") {}

  virtual ~LayeredHardwareIgnition() {
    // before destructing layer loader,
    // deallocate layers which were created by plugins or loader cannot unload plugins
    layers_.clear();
  }

  // LayeredHardwareGazebo will be initialized via initSim()

  virtual bool initSim(rclcpp::Node::SharedPtr &model_nh, std::map<std::string, ig::Entity> &joints,
                       const hi::HardwareInfo &hardware_info, ig::EntityComponentManager &_ecm,
                       int &update_rate) override {
    // this function is not defined in ign_ros2_control::IgnitionSystemInterface.
    // so we don't have to call it.

    // check if "layers" parameter is given
    const auto layers_param_it = hardware_info.hardware_parameters.find("layers");
    if (layers_param_it == hardware_info.hardware_parameters.end()) {
      LHI_ERROR("LayeredHardwareIgnition::initSim(): \"layers\" parameter is missing");
      return false;
    }

    // parse the "layers" parameter as yaml
    std::vector<std::string> layer_names, layer_types;
    try {
      const YAML::Node layers_param = YAML::Load(layers_param_it->second);
      for (const YAML::Node &layer_param : layers_param) {
        layer_names.push_back(layer_param["name"].as<std::string>());
        layer_types.push_back(layer_param["type"].as<std::string>());
      }
    } catch (const YAML::Exception &error) {
      LHI_ERROR("LayeredHardwareIgnition::initSim(): %s (on parsing \"layers\" parameter)",
                error.what());
      return false;
    }

    // load & initialize layers according to "layers" parameter
    for (std::size_t i = 0; i < layer_names.size(); ++i) {
      if (layer_loader_.isClassAvailable(layer_types[i])) {
        // load layer as a normal (non-ignition) layer
        const std::string layer_disp_name =
            "\"" + layer_names[i] + "\" non-ignition layer (" + layer_types[i] + ")";
        std::unique_ptr<lh::LayerInterface> layer;
        try {
          layer.reset(layer_loader_.createUnmanagedInstance(layer_types[i]));
        } catch (const pluginlib::PluginlibException &error) {
          LHI_ERROR("LayeredHardwareIgnition::initSim(): Failed to create %s: %s",
                    layer_disp_name.c_str(), error.what());
          return false;
        }
        // initialize layer in normal way
        if (layer->on_init(layer_names[i], hardware_info) != CallbackReturn::SUCCESS) {
          LHI_ERROR("LayeredHardwareIgnition::initSim(): Failed to initialize %s",
                    layer_disp_name.c_str());
          return false;
        }
        // store successfully-loaded layer
        layers_.push_back(std::move(layer));
        LHI_INFO("LayeredHardwareIgnition::initSim(): Loaded %s", layer_disp_name.c_str());
      } else if (ign_layer_loader_.isClassAvailable(layer_types[i])) {
        // load layer as an ignition layer
        const std::string layer_disp_name =
            "\"" + layer_names[i] + "\" ignition layer (" + layer_types[i] + ")";
        std::unique_ptr<IgnitionLayerInterface> layer;
        try {
          layer.reset(ign_layer_loader_.createUnmanagedInstance(layer_types[i]));
        } catch (const pluginlib::PluginlibException &error) {
          LHI_ERROR("LayeredHardwareIgnition::initSim(): Failed to create %s: %s",
                    layer_disp_name.c_str(), error.what());
          return false;
        }
        // initialize layer in ignition way
        if (!layer->initSim(layer_names[i], model_nh, joints, hardware_info, _ecm, update_rate)) {
          LHI_ERROR("LayeredHardwareIgnition::initSim(): Failed to initialize %s",
                    layer_disp_name.c_str());
          return false;
        }
        // store successfully-loaded layer
        layers_.push_back(std::move(layer));
        LHI_INFO("LayeredHardwareIgnition::initSim(): Loaded %s", layer_disp_name.c_str());
      } else {
        LHI_ERROR("LayeredHardwareIgnition::initSim(): "
                  "Failed to look up \"%s\" (%s) as neither normal nor ignition layers",
                  layer_names[i].c_str(), layer_types[i].c_str());
        return false;
      }
    }

    return true;
  }

  // for functions below, we just use implementation from lh::LayeredHardware

  virtual std::vector<hi::StateInterface> export_state_interfaces() override {
    return lh::LayeredHardware::export_state_interfaces();
  }

  virtual std::vector<hi::CommandInterface> export_command_interfaces() override {
    return lh::LayeredHardware::export_command_interfaces();
  }

  virtual hi::return_type
  prepare_command_mode_switch(const std::vector<std::string> &start_interfaces,
                              const std::vector<std::string> &stop_interfaces) override {
    return lh::LayeredHardware::prepare_command_mode_switch(start_interfaces, stop_interfaces);
  }

  virtual hi::return_type
  perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                              const std::vector<std::string> &stop_interfaces) override {
    return lh::LayeredHardware::perform_command_mode_switch(start_interfaces, stop_interfaces);
  }

  virtual hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    return lh::LayeredHardware::read(time, period);
  }

  virtual hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    return lh::LayeredHardware::write(time, period);
  }

protected:
  // hided version of on_init(), which will be never called.
  virtual CallbackReturn on_init(const hi::HardwareInfo &hardware_info) override final {
    return ign_ros2_control::IgnitionSystemInterface::on_init(hardware_info);
  }

protected:
  pluginlib::ClassLoader<IgnitionLayerInterface> ign_layer_loader_;
};

} // namespace layered_hardware_ign

#endif