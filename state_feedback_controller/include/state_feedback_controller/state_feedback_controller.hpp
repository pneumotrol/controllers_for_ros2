#ifndef STATE_FEEDBACK_CONTROLLER_HPP_
#define STATE_FEEDBACK_CONTROLLER_HPP_

#include <string>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "state_feedback_controller_parameters.hpp"

namespace state_feedback_controller {

class StateFeedbackController : public controller_interface::ControllerInterface {
 public:
  StateFeedbackController();

  controller_interface::CallbackReturn
  on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &preiod) override;

  using TargetMsg = control_msgs::msg::MultiDOFCommand;

 protected:
  // parameters are defined in a yaml file,
  // and are libraryed by generate_parameter_library in CMakeLists.txt.
  std::shared_ptr<state_feedback_controller::ParamListener> param_listener_;
  state_feedback_controller::Params params_;

  // parameter
  realtime_tools::RealtimeBuffer<std::shared_ptr<TargetMsg>> target_;

  // target subscriber
  rclcpp::Subscription<TargetMsg>::SharedPtr target_subscriber_ = nullptr;

 private:
  // reference value subscribe callback
  void target_callback(const std::shared_ptr<TargetMsg> msg);
};

}  // namespace state_feedback_controller

#endif  // STATE_FEEDBACK_CONTROLLER_HPP_