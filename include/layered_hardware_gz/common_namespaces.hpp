#ifndef LAYERED_HARDWARE_GZ_COMMON_NAMESPACES_HPP
#define LAYERED_HARDWARE_GZ_COMMON_NAMESPACES_HPP

/////////////////////
// common namespaces

namespace controller_interface {}

namespace gz::sim {}

namespace hardware_interface {}

namespace layered_hardware {}

//////////////////////////////////////////
// ailias under 'layered_hardware_gz'

namespace layered_hardware_gz {
namespace ci = controller_interface;
namespace gs = gz::sim;
namespace hi = hardware_interface;
namespace lh = layered_hardware;
} // namespace layered_hardware_gz

#endif