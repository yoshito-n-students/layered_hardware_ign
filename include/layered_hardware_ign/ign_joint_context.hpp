#ifndef LAYERED_HARDWARE_IGN_IGN_JOINT_CONTEXT_HPP
#define LAYERED_HARDWARE_IGN_IGN_JOINT_CONTEXT_HPP

#include <limits>
#include <string>

#include <layered_hardware_ign/common_namespaces.hpp>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Joint.hh>

namespace layered_hardware_ign {

struct IgnitionJointContext {
  // handles
  const std::string name;
  ig::Joint joint;
  ig::EntityComponentManager &ecm;

  // states
  double pos = std::numeric_limits<double>::quiet_NaN(),
         vel = std::numeric_limits<double>::quiet_NaN(),
         eff = std::numeric_limits<double>::quiet_NaN();

  // commands
  double pos_cmd = std::numeric_limits<double>::quiet_NaN(),
         vel_cmd = std::numeric_limits<double>::quiet_NaN(),
         eff_cmd = std::numeric_limits<double>::quiet_NaN();
};

} // namespace layered_hardware_ign

#endif