#ifndef LAYERED_HARDWARE_GZ_EFFORT_MODE_HPP
#define LAYERED_HARDWARE_GZ_EFFORT_MODE_HPP

#include <cmath>
#include <memory>

#include <layered_hardware_gz/gz_joint_context.hpp>
#include <layered_hardware_gz/operation_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_gz {

class EffortMode : public OperationModeInterface {
public:
  EffortMode(const std::shared_ptr<GazeboSimJointContext> &context)
      : OperationModeInterface("effort", context) {}

  virtual ~EffortMode() {}

  virtual void starting() override { context_->eff_cmd = 0.; }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    context_->pos = (*context_->joint.Position(context_->ecm))[0];
    context_->vel = (*context_->joint.Velocity(context_->ecm))[0];
    // context_->eff = (*context_->joint.TransmittedWrench(context_->ecm))[0].force().x();
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!std::isnan(context_->eff_cmd)) {
      context_->joint.SetForce(context_->ecm, {context_->eff_cmd});
    }
  }

  virtual void stopping() override { context_->joint.SetForce(context_->ecm, {0.}); }
};

} // namespace layered_hardware_gz

#endif