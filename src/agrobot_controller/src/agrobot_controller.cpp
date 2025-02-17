#include "agrobot_controller/agrobot_controller.hpp"
#include <chrono>
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <cstdio>
#include <exception>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agrobot_controller {

static const std::string kFrWheelParam = "front_right_wheel_name";
static const std::string kFlWheelParam = "front_left_wheel_name";
static const std::string kRrWheelParam = "back_right_wheel_name";
static const std::string kRlWheelParam = "back_left_wheel_name";
static const std::string kWheelBaseParam = "wheel_base";
static const std::string kAxesGapParam = "axes_gap";
static const std::string kWheelRadiusParam = "wheel_radius";
static const std::string kWheelHubRadiusParam = "wheel_hub_radius";
static const std::string kPublishRateParam = "publish_rate";
static const std::string kCmdVelTimeoutParam = "cmd_vel_timeout";

constexpr auto kVelCmdTopic = "~/cmd_vel";
//constexpr auto kOdomTopic = "/odom";
//constexpr auto kTfTopic = "/tf";

constexpr auto kVelHardwareInterfaceType = "velocity";
constexpr auto kPosHardwareInterfaceType = "position";

AgrobotController::AgrobotController()
    : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn AgrobotController::on_init() {
  try {
    // Interface names
    auto_declare<std::string>(kFlWheelParam, std::string());
    auto_declare<std::string>(kFrWheelParam, std::string());
    auto_declare<std::string>(kRlWheelParam, std::string());
    auto_declare<std::string>(kRrWheelParam, std::string());

    // Geometrical parameters
    auto_declare<double>(kWheelBaseParam, params_.wheel_base);
    auto_declare<double>(kAxesGapParam, params_.axes_gap);
    auto_declare<double>(kWheelRadiusParam, params_.wheel_radius);
    auto_declare<double>(kWheelHubRadiusParam, params_.wheel_hub_radius);

    // Node parameters
    auto_declare<double>(kPublishRateParam, 10.0);
    auto_declare<double>(kCmdVelTimeoutParam,
                         cmd_vel_timeout_.count() / 1000.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AgrobotController::on_configure(
    const rclcpp_lifecycle::State&) {
  auto node = this->get_node();
  auto logger = node->get_logger();

  RCLCPP_INFO(logger, "Parsing configuration...");

  try {
    fr_wheel_ = node->get_parameter(kFrWheelParam).as_string();
    fl_wheel_ = node->get_parameter(kFlWheelParam).as_string();
    rl_wheel_ = node->get_parameter(kRlWheelParam).as_string();
    rr_wheel_ = node->get_parameter(kRrWheelParam).as_string();

    if (fr_wheel_.empty() || fl_wheel_.empty() || rl_wheel_.empty() ||
        rr_wheel_.empty()) {
      RCLCPP_FATAL(
          logger, "Not all wheel joints specified. Got [%s], [%s], [%s], [%s],",
          fr_wheel_.c_str(), fl_wheel_.c_str(), rl_wheel_.c_str(),
          rr_wheel_.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    params_.wheel_base = node->get_parameter(kWheelBaseParam).as_double();
    params_.axes_gap = node->get_parameter(kAxesGapParam).as_double();
    params_.wheel_radius = node->get_parameter(kWheelBaseParam).as_double();
    params_.wheel_hub_radius =
        node->get_parameter(kWheelHubRadiusParam).as_double();

    cmd_vel_timeout_ = std::chrono::milliseconds(static_cast<int>(
        node->get_parameter(kCmdVelTimeoutParam).as_double() * 1000.0));
    publish_rate_ = node->get_parameter(kPublishRateParam).as_double();
    if (publish_rate_ <= 0) {
      RCLCPP_FATAL(logger,
                   "Got publish_rate parameter <= 0. It must be positive");
    }
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(logger, "Got error while setting configuration.");
    fprintf(stderr,
            "Exception thrown during configure stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Configuring vel_cmd subscriber
  last_cmd_vel_message_ptr_.set([](auto& ptr) {
    const auto empty_twist = geometry_msgs::msg::TwistStamped();
    ptr = std::make_shared<geometry_msgs::msg::TwistStamped>(empty_twist);
  });
  previous_timestamp_ = node->get_clock()->now();

  vel_cmd_subscriber_ =
      node->create_subscription<geometry_msgs::msg::TwistStamped>(
          kVelCmdTopic, rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg_ptr)
              -> void {
            if (!vel_cmd_subscriber_is_active_) {
              RCLCPP_WARN(this->get_node()->get_logger(),
                          "Can't accept new commands. Subscriber is inactive");
              return;
            }

            geometry_msgs::msg::TwistStamped::SharedPtr twist_ptr = msg_ptr;
            last_cmd_vel_message_ptr_.set(
                [&twist_ptr](auto& ptr) { ptr = twist_ptr; });
            previous_timestamp_ = this->get_node()->get_clock()->now();
          });

  RCLCPP_INFO(logger, "Configuring done");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AgrobotController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = std::vector<std::string>{
      fr_wheel_ + "/" + kVelHardwareInterfaceType,
      fl_wheel_ + "/" + kVelHardwareInterfaceType,
      rl_wheel_ + "/" + kVelHardwareInterfaceType,
      rr_wheel_ + "/" + kVelHardwareInterfaceType,
  };
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
AgrobotController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names = std::vector<std::string>{
      fr_wheel_ + "/" + kVelHardwareInterfaceType,
      fl_wheel_ + "/" + kVelHardwareInterfaceType,
      rl_wheel_ + "/" + kVelHardwareInterfaceType,
      rr_wheel_ + "/" + kVelHardwareInterfaceType,
      fr_wheel_ + "/" + kPosHardwareInterfaceType,
      fl_wheel_ + "/" + kPosHardwareInterfaceType,
      rl_wheel_ + "/" + kPosHardwareInterfaceType,
      rr_wheel_ + "/" + kPosHardwareInterfaceType,
  };
  return state_interfaces_config;
}

controller_interface::CallbackReturn AgrobotController::on_activate(
    const rclcpp_lifecycle::State&) {
  auto logger = this->get_node()->get_logger();
  RCLCPP_INFO(logger, "activation stage");

  try {
    const auto fr_wheel_result = configure_wheel(fr_wheel_, fr_wheel_handle_);
    const auto fl_wheel_result = configure_wheel(fl_wheel_, fl_wheel_handle_);
    const auto rr_wheel_result = configure_wheel(rr_wheel_, rr_wheel_handle_);
    const auto rl_wheel_result = configure_wheel(rl_wheel_, rl_wheel_handle_);

    if (fr_wheel_result == controller_interface::CallbackReturn::ERROR ||
        fl_wheel_result == controller_interface::CallbackReturn::ERROR ||
        rr_wheel_result == controller_interface::CallbackReturn::ERROR ||
        rl_wheel_result == controller_interface::CallbackReturn::ERROR) {
      return controller_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "activation error.");
    fprintf(stderr,
            "Exception thrown during activate stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  vel_cmd_subscriber_is_active_ = true;
  is_halted_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AgrobotController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  vel_cmd_subscriber_is_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AgrobotController::configure_wheel(
    const std::string& wheel_name, WheelHandle*& registered_handle) {
  auto logger = this->get_node()->get_logger();

  try {
    if (wheel_name.empty()) {
      RCLCPP_ERROR(logger, "No wheel name specified");
      return CallbackReturn::ERROR;
    }

    const auto state_handle =
        std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                     [&wheel_name](const auto& interface) {
                       return interface.get_name() ==
                              wheel_name + "/" + kVelHardwareInterfaceType;
                     });

    if (state_handle == state_interfaces_.cend()) {
      RCLCPP_ERROR(logger, "Unable to obtain joint");
      return CallbackReturn::ERROR;
    }

    const auto command_handle =
        std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                     [&wheel_name](const auto& interface) {
                       return interface.get_name() ==
                              wheel_name + "/" + kVelHardwareInterfaceType;
                     });

    if (command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s",
                   wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handle = new AgrobotController::WheelHandle{
        std::ref(*state_handle), std::ref(*command_handle)};

  } catch (const std::exception& e) {
    fprintf(stderr,
            "Exception thrown during configuring wheel with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AgrobotController::update(
    const rclcpp::Time& time, const rclcpp::Duration&) {
  auto logger = this->get_node()->get_logger();
  const auto current_time = time;

  const auto state = get_lifecycle_state().id();
  if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted_) {
      halt();
      is_halted_ = true;
    }
    return controller_interface::return_type::OK;
  }

  std::shared_ptr<geometry_msgs::msg::TwistStamped> last_command_msg;
  last_cmd_vel_message_ptr_.get(
      [&last_command_msg](const auto& ptr) { last_command_msg = ptr; });

  if (last_command_msg == nullptr) {
    RCLCPP_WARN(logger, "Velocity command message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command =
      current_time - last_command_msg->header.stamp;
  if (age_of_last_command > cmd_vel_timeout_) {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.linear.z = 0.0;
  }

  geometry_msgs::msg::TwistStamped command = *last_command_msg;
  auto v_x = command.twist.linear.x;
  auto v_y = command.twist.linear.y;
  auto w_z = command.twist.angular.z;

  std::array<double, 4> w;
  w[0] = fr_wheel_handle_->velocity_feedback.get().get_value();
  w[1] = fl_wheel_handle_->velocity_feedback.get().get_value();
  w[2] = rl_wheel_handle_->velocity_feedback.get().get_value();
  w[3] = rr_wheel_handle_->velocity_feedback.get().get_value();

  for (size_t i = 0; i < w.size(); ++i) {
    if (std::isnan(w[i])) {
      RCLCPP_ERROR(logger, "One of the wheel values is invalid.");
      return controller_interface::return_type::ERROR;
    }
  }

  // Place to do odometry

  const auto r = params_.wheel_radius;
  const auto H = params_.axes_gap;
  const auto B = params_.wheel_base;

  w[0] = 1 / r * (v_x + v_y + (H + B) / 2 * w_z);
  w[1] = 1 / r * (v_x - v_y - (H + B) / 2 * w_z);
  w[2] = 1 / r * (v_x + v_y - (H + B) / 2 * w_z);
  w[3] = 1 / r * (v_x - v_y + (H + B) / 2 * w_z);
  // RCLCPP_INFO(logger, "Setting control values: (%.2f, %.2f, %.2f, %.2f)", w[0],
  //             w[1], w[2], w[3]);

  if (!fr_wheel_handle_->velocity_cmd.get().set_value(w[0]) ||
      !fl_wheel_handle_->velocity_cmd.get().set_value(w[1]) ||
      !rl_wheel_handle_->velocity_cmd.get().set_value(w[2]) ||
      !rr_wheel_handle_->velocity_cmd.get().set_value(w[3])) {
    RCLCPP_ERROR(logger,
                 "failed when tried to write to wheel command interfaces");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

}  // namespace agrobot_controller

PLUGINLIB_EXPORT_CLASS(agrobot_controller::AgrobotController,
                       controller_interface::ControllerInterface);
