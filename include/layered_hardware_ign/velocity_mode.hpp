#ifndef LAYERED_HARDWARE_IGN_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_IGN_VELOCITY_MODE_HPP

#include <cmath>
#include <memory>

#include <layered_hardware_ign/ign_joint_context.hpp>
#include <layered_hardware_ign/operation_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_ign {

class VelocityMode : public OperationModeInterface {
public:
  VelocityMode(const std::shared_ptr<IgnitionJointContext> &context)
      : OperationModeInterface("velocity", context) {}

  virtual ~VelocityMode() {}

  virtual void starting() override { context_->vel_cmd = 0.; }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    context_->pos = (*context_->joint.Position(context_->ecm))[0];
    context_->vel = (*context_->joint.Velocity(context_->ecm))[0];
    // context_->eff = (*context_->joint.TransmittedWrench(context_->ecm))[0].force().x();
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!std::isnan(context_->vel_cmd)) {
      context_->joint.SetVelocity(context_->ecm, {context_->vel_cmd});
    }
  }

  virtual void stopping() override { context_->joint.SetVelocity(context_->ecm, {0.}); }
};
} // namespace layered_hardware_ign

#endif