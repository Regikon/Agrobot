#ifndef AGROBOT_CONTROLLER_HPP
#define AGROBOT_CONTROLLER_HPP

#include <controller_interface/controller_interface_base.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <queue>
#include <rclcpp/duration.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_box.hpp"

namespace agrobot_controller {

struct AgrobotControllerParams {
  double wheel_base;
  double axes_gap;
  double wheel_radius;
  double wheel_hub_radius;
};

class AgrobotController : public controller_interface::ControllerInterface {
 public:
  AgrobotController();

  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  std::string fl_wheel_;
  std::string fr_wheel_;
  std::string rl_wheel_;
  std::string rr_wheel_;
  AgrobotControllerParams params_;

  double publish_rate_;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_timestamp_;

  std::chrono::milliseconds cmd_vel_timeout_{200};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      vel_cmd_subscriber_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>>
      last_cmd_vel_message_ptr_{nullptr};
  std::queue<geometry_msgs::msg::TwistStamped> previous_cmd_commands_;
  bool vel_cmd_subscriber_is_active_ = false;

  bool is_halted_ = true;

  struct WheelHandle {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface>
        velocity_feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface>
        velocity_cmd;
  };
  WheelHandle* fl_wheel_handle_{nullptr};
  WheelHandle* fr_wheel_handle_{nullptr};
  WheelHandle* rl_wheel_handle_{nullptr};
  WheelHandle* rr_wheel_handle_{nullptr};

  controller_interface::CallbackReturn configure_wheel(
      const std::string& wheel_name, WheelHandle*& registered_handle);

  bool reset();
  void halt();
};

}  // namespace agrobot_controller

#endif
