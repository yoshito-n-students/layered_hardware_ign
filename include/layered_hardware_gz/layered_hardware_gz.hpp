#ifndef LAYERED_HARDWARE_GZ_LAYERED_HARDWARE_GZ_HPP
#define LAYERED_HARDWARE_GZ_LAYERED_HARDWARE_GZ_HPP

#include <string>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/layered_hardware.hpp>
#include <layered_hardware_gz/common_namespaces.hpp>
#include <layered_hardware_gz/gz_layer_interface.hpp>
#include <layered_hardware_gz/logging_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

#include <gz/sim/System.hh>

namespace layered_hardware_gz {

class LayeredHardwareGazeboSim : public gz_ros2_control::GazeboSimSystemInterface,
                                 public lh::LayeredHardware {
public:
  LayeredHardwareGazeboSim()
      : gz_ros2_control::GazeboSimSystemInterface(), lh::LayeredHardware(),
        gz_layer_loader_("layered_hardware_gz", "layered_hardware_gz::GazeboSimLayerInterface") {}

  virtual ~LayeredHardwareGazeboSim() {
    // before destructing layer loader,
    // deallocate layers which were created by plugins or loader cannot unload plugins
    layers_.clear();
  }

  // LayeredHardwareGazebo will be initialized via initSim()

  virtual bool initSim(rclcpp::Node::SharedPtr &model_nh, std::map<std::string, gs::Entity> &joints,
                       const hi::HardwareInfo &hardware_info, gs::EntityComponentManager &_ecm,
                       unsigned int update_rate) override {
    // this function is not defined in gz_ros2_control::GazeboSimSystemInterface.
    // so we don't have to call it.

    // check if "layers" parameter is given
    const auto layers_param_it = hardware_info.hardware_parameters.find("layers");
    if (layers_param_it == hardware_info.hardware_parameters.end()) {
      lhg_error("LayeredHardwareGazeboSim::initSim(): \"layers\" parameter is missing");
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
      lhg_error("LayeredHardwareGazeboSim::initSim(): %s (on parsing \"layers\" parameter)", error);
      return false;
    }

    // load & initialize layers according to "layers" parameter
    for (std::size_t i = 0; i < layer_names.size(); ++i) {
      if (layer_loader_.isClassAvailable(layer_types[i])) {
        // load layer as a normal (non-ignition) layer
        const std::string layer_disp_name =
            "\"" + layer_names[i] + "\" non-gazebo-sim layer (" + layer_types[i] + ")";
        lhg_info("LayeredHardwareGazeboSim::initSim(): Loading %s", layer_disp_name);
        std::unique_ptr<lh::LayerInterface> layer;
        try {
          layer.reset(layer_loader_.createUnmanagedInstance(layer_types[i]));
        } catch (const pluginlib::PluginlibException &error) {
          lhg_error("LayeredHardwareGazeboSim::initSim(): Failed to create %s: %s", //
                    layer_disp_name, error);
          return false;
        }
        // initialize layer in normal way
        if (layer->on_init(layer_names[i], hardware_info) != CallbackReturn::SUCCESS) {
          lhg_error("LayeredHardwareGazeboSim::initSim(): Failed to initialize %s",
                    layer_disp_name);
          return false;
        }
        // store successfully-loaded layer
        layers_.push_back(std::move(layer));
        lhg_info("LayeredHardwareGazeboSim::initSim(): Loaded %s", layer_disp_name);
      } else if (gz_layer_loader_.isClassAvailable(layer_types[i])) {
        // load layer as an ignition layer
        const std::string layer_disp_name =
            "\"" + layer_names[i] + "\" gazebo-sim layer (" + layer_types[i] + ")";
        lhg_info("LayeredHardwareGazeboSim::initSim(): Loading %s", layer_disp_name);
        std::unique_ptr<GazeboSimLayerInterface> layer;
        try {
          layer.reset(gz_layer_loader_.createUnmanagedInstance(layer_types[i]));
        } catch (const pluginlib::PluginlibException &error) {
          lhg_error("LayeredHardwareGazeboSim::initSim(): Failed to create %s: %s", //
                    layer_disp_name, error);
          return false;
        }
        // initialize layer in ignition way
        if (!layer->initSim(layer_names[i], model_nh, joints, hardware_info, _ecm, update_rate)) {
          lhg_error("LayeredHardwareGazeboSim::initSim(): Failed to initialize %s",
                    layer_disp_name);
          return false;
        }
        // store successfully-loaded layer
        layers_.push_back(std::move(layer));
        lhg_info("LayeredHardwareGazeboSim::initSim(): Loaded %s", layer_disp_name);
      } else {
        lhg_error("LayeredHardwareGazeboSim::initSim(): "
                  "Failed to look up \"%s\" (%s) as neither normal nor ignition layers",
                  layer_names[i], layer_types[i]);
        return false;
      }
    }

    // populate command & state interfaces from each layer
    // (these interfaces are referenced by layers,
    //  so they must be member variables to match the layers' lifespan)
    state_interfaces_ = export_state_interfaces();
    command_interfaces_ = export_command_interfaces();

    // assign state & command interfaces to each layer
    for (auto &layer : layers_) {
      layer->assign_interfaces(loan_interfaces<hi::LoanedStateInterface>(
                                   state_interfaces_, layer->state_interface_configuration()),
                               loan_interfaces<hi::LoanedCommandInterface>(
                                   command_interfaces_, layer->command_interface_configuration()));
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
    return gz_ros2_control::GazeboSimSystemInterface::on_init(hardware_info);
  }

protected:
  pluginlib::ClassLoader<GazeboSimLayerInterface> gz_layer_loader_;
};

} // namespace layered_hardware_gz

#endif