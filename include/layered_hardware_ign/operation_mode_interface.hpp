#ifndef LAYERED_HARDWARE_IGN_OPERATION_MODE_INTERFACE_HPP
#define LAYERED_HARDWARE_IGN_OPERATION_MODE_INTERFACE_HPP

#include <memory>
#include <string>

#include <layered_hardware_ign/ign_joint_context.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_ign {

class OperationModeInterface {
public:
  OperationModeInterface(const std::string &name, const std::shared_ptr<IgnitionJointContext> &context)
      : name_(name), context_(context) {}

  virtual ~OperationModeInterface() {}

  std::string get_name() const { return name_; }

  virtual void starting() = 0;

  virtual void read(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;

  virtual void write(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;

  virtual void stopping() = 0;

protected:
  const std::string name_;
  const std::shared_ptr<IgnitionJointContext> context_;
};

} // namespace layered_hardware_ign

#endif