#ifndef LAYERED_HARDWARE_IGN_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_IGN_VELOCITY_MODE_HPP

#include <cmath>
#include <memory>

#include <layered_hardware_ign/common_namespaces.hpp>
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

  virtual void starting() override {
    // enable ODE's joint motor function for effort-based velocity control
    // (TODO: specialization for other physics engines)
    // joint_->SetParam("fmax", 0, eff_lim_);

    context_->vel_cmd = 0.;
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    context_->pos = (*context_->joint.Position(context_->ecm))[0];
    context_->vel = (*context_->joint.Velocity(context_->ecm))[0];
    // data->effort = joint_->GetForce(0);
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!std::isnan(context_->vel_cmd)) {
      context_->joint.SetVelocity(context_->ecm, {context_->vel_cmd});
    }
  }

  virtual void stopping() override {
    // disable ODE's joint motor function and zero effort
    // joint_->SetParam("fmax", 0, 0.);
    // joint_->SetForce(0, 0.);
    context_->joint.SetVelocity(context_->ecm, {0.});
  }
};
} // namespace layered_hardware_ign

#endif