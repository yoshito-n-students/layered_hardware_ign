#ifndef LAYERED_HARDWARE_IGN_POSITION_MODE_HPP
#define LAYERED_HARDWARE_IGN_POSITION_MODE_HPP

#include <cmath>
#include <memory>

#include <layered_hardware_ign/common_namespaces.hpp>
#include <layered_hardware_ign/ign_joint_context.hpp>
#include <layered_hardware_ign/operation_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_ign {

class PositionMode : public OperationModeInterface {
public:
  PositionMode(const std::shared_ptr<IgnitionJointContext> &context)
      : OperationModeInterface("position", context) {}

  virtual ~PositionMode() {}

  virtual void starting() override {
    // enable ODE's joint motor function for effort-based position control
    // (TODO: specialization for other physics engines)
    // joint_->SetParam("fmax", 0, eff_lim_);

    // the latest position may be useful in the starting procedure of a pos-based controller
    const double pos = (*context_->joint.Position(context_->ecm))[0];
    context_->pos = pos;
    context_->pos_cmd = pos;
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    context_->pos = (*context_->joint.Position(context_->ecm))[0];
    context_->vel = (*context_->joint.Velocity(context_->ecm))[0];
    // data->effort = joint_->GetForce(0);
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!std::isnan(context_->pos_cmd)) {
      // context_->joint.ResetPosition(context_->ecm, {context_->pos_cmd});
      const double err = context_->pos_cmd - (*context_->joint.Position(context_->ecm))[0];
      if (err > 0.01) {
        context_->joint.SetVelocity(context_->ecm, {0.6});
      } else if (err < -0.01) {
        context_->joint.SetVelocity(context_->ecm, {-0.6});
      } else {
        context_->joint.SetVelocity(context_->ecm, {0.0});
      }
    }
  }

  virtual void stopping() {
    /*
    // disable ODE's joint motor function and zero effort
    joint_->SetParam("fmax", 0, 0.);
    joint_->SetForce(0, 0.);
    */
  }
};
} // namespace layered_hardware_ign

#endif