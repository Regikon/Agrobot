#include "agrobot_controller/agrobot_controller.hpp"
#include <chrono>
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include "geometry_msgs/msg/twist.hpp"

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
constexpr auto kOdomTopic = "/odom";
constexpr auto kTfTopic = "/tf";

constexpr auto kVelHardwareInterfaceType = "velocity";
constexpr auto kPosHardwareInterfaceType = "position";

AgrobotController::AgrobotController()
    : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn AgrobotController::on_init() {
  controller_interface::CallbackReturn parent_init_result =
      controller_interface::ControllerInterface::on_init();
  if (parent_init_result != controller_interface::CallbackReturn::SUCCESS) {
    return parent_init_result;
  }

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
    previous_timestamp_ = node->get_clock()->now();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(logger, "Got error while setting configuration.");
    fprintf(stderr,
            "Exception thrown during configure stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Configuring vel_cmd subscriber
  last_cmd_vel_message_ptr_.set(std::make_shared<geometry_msgs::msg::Twist>(
      new geometry_msgs::msg::Twist));
  previous_cmd_commands_.emplace(geometry_msgs::msg::Twist{});
  previous_cmd_commands_.emplace(geometry_msgs::msg::Twist{});

  vel_cmd_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
      kVelCmdTopic, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
        if (!vel_cmd_subscriber_is_active) {
          RCLCPP_WARN(this->get_node()->get_logger(),
                      "Can't accept new commands. Subscriber is inactive");
          return;
        }

        std::shared_ptr<geometry_msgs::msg::Twist> twist;
        last_cmd_vel_message_ptr_.get(twist);
        twist = msg;
      });
  previous_timestamp_ = node->get_clock()->now();

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

    const auto state_handle = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&wheel_name](const auto& interface) {
          return interface.get_name() == wheel_name &&
                 interface.get_interface_name() == kVelHardwareInterfaceType;
        });

    if (state_handle == state_interfaces_.cend()) {
      RCLCPP_ERROR(logger, "Unable to obtain joint ");
    }
  }
}

}  // namespace agrobot_controller
