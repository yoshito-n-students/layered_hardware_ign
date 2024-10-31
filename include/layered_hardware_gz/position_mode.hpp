#ifndef LAYERED_HARDWARE_GZ_POSITION_MODE_HPP
#define LAYERED_HARDWARE_GZ_POSITION_MODE_HPP

#include <cmath>
#include <memory>

#include <layered_hardware_gz/gz_joint_context.hpp>
#include <layered_hardware_gz/operation_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_gz {

class PositionMode : public OperationModeInterface {
public:
  PositionMode(const std::shared_ptr<GazeboSimJointContext> &context)
      : OperationModeInterface("position", context) {}

  virtual ~PositionMode() {}

  virtual void starting() override {
    // the latest position may be useful in the starting procedure of a pos-based controller
    const double pos = (*context_->joint.Position(context_->ecm))[0];
    context_->pos = pos;
    context_->pos_cmd = pos;
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    context_->pos = (*context_->joint.Position(context_->ecm))[0];
    context_->vel = (*context_->joint.Velocity(context_->ecm))[0];
    // context_->eff = (*context_->joint.TransmittedWrench(context_->ecm))[0].force().x();
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!std::isnan(context_->pos_cmd)) {
      context_->joint.ResetPosition(context_->ecm, {context_->pos_cmd});
    }
  }

  virtual void stopping() {}
};
} // namespace layered_hardware_gz

#endif