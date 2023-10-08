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
  // QoS
  // ----------------------------------------
  auto qos = rclcpp::SystemDefaultsQoS();
  qos.keep_last(1);
  qos.best_effort();

  // ----------------------------------------
  // state feedback gain
  // ----------------------------------------
  // create realtime buffer
  state_feedback_gain_.writeFromNonRT(std::make_unique<FeedbackGainMsg>());

  // initialize buffer
  auto state_feedback_gain = *(state_feedback_gain_.readFromRT());
  state_feedback_gain->data.assign(params_.state_interfaces.size() * params_.command_interfaces.size(), static_cast<double>(0.0));
  if (state_feedback_gain->data.size() == params_.state_feedback_gain.size()) {
    for (size_t i = 0; i < state_feedback_gain->data.size(); i++) {
      state_feedback_gain->data[i] = params_.state_feedback_gain[i];
    }
  }

  // ----------------------------------------
  // target
  // ----------------------------------------
  // create realtime buffer
  target_.writeFromNonRT(std::make_unique<TargetMsg>());

  // initialize buffer
  auto target = *(target_.readFromRT());
  target->data.assign(params_.command_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  // create subscriber
  target_subscriber_ = get_node()->create_subscription<TargetMsg>(
    "~/target", qos,
    std::bind(&StateFeedbackController::target_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "configure successful.");

  return controller_interface::CallbackReturn::SUCCESS;
}

void StateFeedbackController::target_callback(const std::shared_ptr<TargetMsg> target) {
  // target_ is updated to the value of subscribed topic.
  if (target->data.size() == params_.command_interfaces.size()) {
    target_.writeFromNonRT(target);
  } else {
    // target dimensions of the topic is invalid
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Received %zu, but expected %zu joints in command. Ignoring message.",
                 target->data.size(), params_.command_interfaces.size());
  }
}

controller_interface::InterfaceConfiguration
StateFeedbackController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;

  // request access to the interfaces specified in yaml (INDIVIDUAL)
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // define command interface (e.g. "joint1/effort")
  command_interfaces_config.names.reserve(params_.command_interfaces.size());
  for (const auto &command_interface : params_.command_interfaces) {
    command_interfaces_config.names.push_back(command_interface);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
StateFeedbackController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;

  // request access to the interfaces specified in yaml (INDIVIDUAL)
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // define state interface (e.g. "joint1/effort")
  state_interfaces_config.names.reserve(params_.state_interfaces.size());
  for (const auto &state_interface : params_.state_interfaces) {
    state_interfaces_config.names.push_back(state_interface);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn
StateFeedbackController::on_activate(const rclcpp_lifecycle::State &previous_state) {
  // unused
  static_cast<void>(previous_state);

  // initialize target buffer
  auto target = *(target_.readFromRT());
  target->data.assign(params_.command_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

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

  auto state_dim = state_interfaces_.size();
  auto command_dim = command_interfaces_.size();

  // ----------------------------------------
  // state feedback control
  // u = target - K * state
  // ----------------------------------------
  std::vector<double> u(command_dim, 0.0);
  auto target = *(target_.readFromRT());
  auto K = *(state_feedback_gain_.readFromRT());

  // u := target
  for (size_t i = 0; i < command_dim; i++) {
    if (!std::isnan(target->data[i])) {
      u[i] = target->data[i];
    }
  }

  // u := u - K * state
  for (size_t i = 0; i < state_dim; i++) {
    for (size_t j = 0; j < command_dim; j++) {
      u[j] -= K->data[i * command_dim + j] * state_interfaces_[i].get_value();
    }
  }

  // apply command
  for (size_t i = 0; i < command_dim; i++) {
    command_interfaces_[i].set_value(u[i]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace state_feedback_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(state_feedback_controller::StateFeedbackController,
                       controller_interface::ControllerInterface)