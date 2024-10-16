#ifndef LAYERED_HARDWARE_IGN_IGN_JOINT_DRIVER_HPP
#define LAYERED_HARDWARE_IGN_IGN_JOINT_DRIVER_HPP

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp> // for hi::return_type
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <layered_hardware/string_registry.hpp>
#include <layered_hardware_ign/common_namespaces.hpp>
#include <layered_hardware_ign/effort_mode.hpp>
#include <layered_hardware_ign/ign_joint_context.hpp>
#include <layered_hardware_ign/logging_utils.hpp>
#include <layered_hardware_ign/operation_mode_interface.hpp>
#include <layered_hardware_ign/position_mode.hpp>
#include <layered_hardware_ign/velocity_mode.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Joint.hh>

#include <yaml-cpp/yaml.h>

namespace layered_hardware_ign {

class IgnitionJointDriver {
public:
  IgnitionJointDriver(const std::string &name, const YAML::Node &params, const ig::Joint &joint,
                      ig::EntityComponentManager &ecm) {
    // parse parameters for this joint
    double initial_position;
    std::vector<std::string> mapped_mode_names;
    try {
      initial_position = params["initial_position"].as<double>(0.);
      for (const auto &iface_mode_name_pair : params["operation_mode_map"]) {
        bound_interfaces_.emplace_back(iface_mode_name_pair.first.as<std::string>());
        mapped_mode_names.emplace_back(iface_mode_name_pair.second.as<std::string>());
      }
    } catch (const YAML::Exception &error) {
      throw std::runtime_error("Failed to parse parameters for \"" + name +
                               "\" joint: " + error.what());
    }

    // set up the joint: enables state reports
    joint.EnablePositionCheck(ecm);
    joint.EnableVelocityCheck(ecm);
    joint.EnableTransmittedWrenchCheck(ecm);

    // TODO: set initial position for the joint
    (void)initial_position;

    // allocate context for joint
    context_.reset(new IgnitionJointContext{name, joint, ecm});

    // make operating mode map from ros-controller name to dynamixel's operating mode
    for (const auto &mode_name : mapped_mode_names) {
      try {
        mapped_modes_.emplace_back(make_operation_mode(mode_name));
      } catch (const std::runtime_error &error) {
        throw std::runtime_error("Failed to create \"" + mode_name + "\" operation mode for \"" +
                                 name + "\" joint:" + error.what());
      }
    }
  }

  virtual ~IgnitionJointDriver() {
    // stop present mode
    switch_operation_modes(/* new_mode = */ nullptr);
  }

  std::vector<hi::StateInterface> export_state_interfaces() {
    // export reference to joint states owned by this joint
    // TODO: export interfaces defined in hardware_info
    std::vector<hi::StateInterface> ifaces;
    ifaces.emplace_back(context_->name, hi::HW_IF_POSITION, &context_->pos);
    ifaces.emplace_back(context_->name, hi::HW_IF_VELOCITY, &context_->vel);
    ifaces.emplace_back(context_->name, hi::HW_IF_EFFORT, &context_->eff);
    return ifaces;
  }

  std::vector<hi::CommandInterface> export_command_interfaces() {
    // export reference to joint commands owned by this joint
    // TODO: export interfaces defined in hardware_info
    std::vector<hi::CommandInterface> ifaces;
    ifaces.emplace_back(context_->name, hi::HW_IF_POSITION, &context_->pos_cmd);
    ifaces.emplace_back(context_->name, hi::HW_IF_VELOCITY, &context_->vel_cmd);
    ifaces.emplace_back(context_->name, hi::HW_IF_EFFORT, &context_->eff_cmd);
    return ifaces;
  }

  hi::return_type prepare_command_mode_switch(const lh::StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() <= 1) {
      return hi::return_type::OK;
    } else { // active_bound_ifaces.size() >= 2
      LHI_ERROR("IgnitionJointDriver::prepare_command_mode_switch(): "
                "Reject mode switching of \"%s\" actuator "
                "because %zd bound interfaces are about to be active",
                context_->name.c_str(), active_bound_ifaces.size());
      return hi::return_type::ERROR;
    }
  }

  hi::return_type perform_command_mode_switch(const lh::StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() >= 2) {
      LHI_ERROR("IgnitionJointDriver::perform_command_mode_switch(): "
                "Could not switch mode of \"%s\" actuator "
                "because %zd bound interfaces are active",
                context_->name.c_str(), bound_interfaces_.size());
      return hi::return_type::ERROR;
    }

    // switch to actuator command mode associated with active bound interface
    if (!active_bound_ifaces.empty()) { // active_bound_ifaces.size() == 1
      switch_operation_modes(mapped_modes_[active_bound_ifaces.front()]);
    } else { // active_bound_ifaces.size() == 0
      switch_operation_modes(nullptr);
    }
    return hi::return_type::OK;
  }

  hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (present_mode_) {
      present_mode_->read(time, period);
    }
    return hi::return_type::OK; // TODO: return result of read
  }

  hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (present_mode_) {
      present_mode_->write(time, period);
    }
    return hi::return_type::OK; // TODO: return result of write
  }

private:
  std::shared_ptr<OperationModeInterface> make_operation_mode(const std::string &mode_str) {
    if (mode_str == "effort") {
      return std::make_shared<EffortMode>(context_);
    } else if (mode_str == "position") {
      return std::make_shared<PositionMode>(context_);
    } else if (mode_str == "velocity") {
      return std::make_shared<VelocityMode>(context_);
    } else {
      throw std::runtime_error("Unkown operation mode name \"" + mode_str + "\"");
    }
  }

  void switch_operation_modes(const std::shared_ptr<OperationModeInterface> &new_mode) {
    // do nothing if no mode switch is requested
    if (present_mode_ == new_mode) {
      return;
    }
    // stop present mode
    if (present_mode_) {
      LHI_INFO("IgnitionJointDriver::switch_operation_modes(): "
               "Stopping \"%s\" operation mode for \"%s\" joint",
               present_mode_->get_name().c_str(), context_->name.c_str());
      present_mode_->stopping();
      present_mode_.reset();
    }
    // start new mode
    if (new_mode) {
      LHI_INFO("IgnitionJointDriver::switch_operation_modes(): "
               "Starting \"%s\" operation mode for \"%s\" joint",
               new_mode->get_name().c_str(), context_->name.c_str());
      new_mode->starting();
      present_mode_ = new_mode;
    }
  }

private:
  std::shared_ptr<IgnitionJointContext> context_;

  // present operating mode
  std::shared_ptr<OperationModeInterface> present_mode_;
  // map from command interface to operating mode
  // (i.e. mapped_modes_[i] is associated with bound_interfaces_[i])
  std::vector<std::string> bound_interfaces_;
  std::vector<std::shared_ptr<OperationModeInterface>> mapped_modes_;
};

} // namespace layered_hardware_ign

#endif