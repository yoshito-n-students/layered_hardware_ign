#ifndef LAYERED_HARDWARE_GZ_GZ_JOINT_CONTEXT_HPP
#define LAYERED_HARDWARE_GZ_GZ_JOINT_CONTEXT_HPP

#include <limits>
#include <string>

#include <layered_hardware_gz/common_namespaces.hpp>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>

namespace layered_hardware_gz {

struct GazeboSimJointContext {
  // handles
  const std::string name;
  gs::Joint joint;
  gs::EntityComponentManager &ecm;

  // states
  double pos = std::numeric_limits<double>::quiet_NaN(),
         vel = std::numeric_limits<double>::quiet_NaN(),
         eff = std::numeric_limits<double>::quiet_NaN();

  // commands
  double pos_cmd = std::numeric_limits<double>::quiet_NaN(),
         vel_cmd = std::numeric_limits<double>::quiet_NaN(),
         eff_cmd = std::numeric_limits<double>::quiet_NaN();
};

} // namespace layered_hardware_gz

#endif