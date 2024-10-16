#ifndef LAYERED_HARDWARE_IGN_COMMON_NAMESPACES_HPP
#define LAYERED_HARDWARE_IGN_COMMON_NAMESPACES_HPP

/////////////////////
// common namespaces

namespace controller_interface {}

namespace hardware_interface {}

namespace ignition::gazebo {}

namespace layered_hardware {}

//////////////////////////////////////////
// ailias under 'layered_hardware_gazebo'

namespace layered_hardware_ign {
namespace ci = controller_interface;
namespace hi = hardware_interface;
namespace ig = ignition::gazebo;
namespace lh = layered_hardware;
} // namespace layered_hardware_ign

#endif