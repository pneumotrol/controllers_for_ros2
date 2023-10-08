#include "state_feedback_controller/state_feedback_controller.hpp"

namespace state_feedback_controller {

StateFeedbackController::StateFeedbackController()
  : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn
StateFeedbackController::on_init() {
  // ----------------------------------------
  // parameter listener
  // ----------------------------------------
  // get an instance
  try {
    param_listener_ = std::make_shared<state_feedback_controller::ParamListener>(get_node());
  } catch (const std::exception &e) {
    fprintf(stderr,
            "Exception thrown during controller's init with message: %s \n",
            e.what());

    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
StateFeedbackController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;

  // request access to the interfaces specified in yaml (INDIVIDUAL)
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // define command interface (e.g. "joint1/effort")
  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto &joint : params_.joints) {
    command_interfaces_config.names.push_back(joint + "/" + params_.command_interface_name);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
StateFeedbackController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;

  // request access to the interfaces specified in yaml (INDIVIDUAL)
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // define state interface (e.g. "joint1/effort")
  state_interfaces_config.names.reserve(params_.joints.size());
  for (const auto &joint : params_.joints) {
    state_interfaces_config.names.push_back(joint + "/" + params_.state_interface_name);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn
StateFeedbackController::on_configure(const rclcpp_lifecycle::State &previous_state) {
  // unused
  static_cast<void>(previous_state);

  // ----------------------------------------
  // parameter listener
  // ----------------------------------------
  // get parameters from parameter listener
  // details are defined at yaml file.
  params_ = param_listener_->get_params();

  // ----------------------------------------
  // target buffer
  // ----------------------------------------
  // create target buffer
  target_.writeFromNonRT(std::make_unique<TargetMsg>());

  // initialize target buffer
  auto target = *(target_.readFromRT());
  target->dof_names = params_.joints;
  target->values.assign(params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  target->values_dot.assign(params_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // ----------------------------------------
  // QoS
  // ----------------------------------------
  auto qos = rclcpp::SystemDefaultsQoS();
  qos.keep_last(1);
  qos.best_effort();

  // ----------------------------------------
  // target subscriber
  // ----------------------------------------
  // create subscriber
  target_subscriber_ = get_node()->create_subscription<TargetMsg>(
    "~/target", qos,
    std::bind(&StateFeedbackController::target_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "configure successful.");

  return controller_interface::CallbackReturn::SUCCESS;
}

void StateFeedbackController::target_callback(const std::shared_ptr<TargetMsg> target) {
  // target_ is updated to the value of subscribed topic.
  if (target->dof_names.size() == params_.joints.size()) {
    target_.writeFromNonRT(target);
  } else {
    // command dimensions of the topic is invalid
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Received %zu, but expected %zu joints in command. Ignoring message.",
                 target->dof_names.size(), params_.joints.size());
  }
}

controller_interface::CallbackReturn
StateFeedbackController::on_activate(const rclcpp_lifecycle::State &previous_state) {
  // unused
  static_cast<void>(previous_state);

  // initialize target buffer
  auto target = *(target_.readFromRT());
  target->dof_names = params_.joints;
  target->values.assign(params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  target->values_dot.assign(params_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // initialize command interface
  for (auto &command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
StateFeedbackController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
  // unused
  static_cast<void>(previous_state);

  // initialize command interface
  for (auto &command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
StateFeedbackController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
  // unused
  static_cast<void>(time);
  static_cast<void>(period);

  auto targets = *(target_.readFromRT());

  auto Kp = 100.0;  // TODO: to params

  for (size_t i = 0; i < command_interfaces_.size(); i++) {
    if (!std::isnan(targets->values[i])) {
      // PID control
      // effort = target - current_state
      auto r = targets->values[i];
      auto y = state_interfaces_[i].get_value();
      auto u = Kp * (r - y);

      command_interfaces_[i].set_value(u);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace state_feedback_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(state_feedback_controller::StateFeedbackController,
                       controller_interface::ControllerInterface)