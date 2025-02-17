#ifndef AGROBOT_CONTROLLER_ODOMETRY_HPP
#define AGROBOT_CONTROLLER_ODOMETRY_HPP

#include <cstddef>
#include <rclcpp/duration.hpp>
#include <tuple>
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/rolling_mean_accumulator.hpp"

namespace agrobot_controller {

class AgrobotOdometry {
 public:
  explicit AgrobotOdometry(size_t velocity_rolling_window_size = 3);

  void init(const rclcpp::Time& time);
  void update(double fl_vel, double fr_vel, double rl_vel,
                      double rr_vel, const rclcpp::Time& time);
  void resetPosition();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  std::tuple<double, double> getLinearVel() const { return {vel_x_, vel_y_}; }
  double getAngularVel() const { return angular_vel_; }

  void setWheelParams(double wheel_base, double axes_gap, double wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

 private:
  using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;

  void resetAccumulators();
  void integratePosition(const rclcpp::Duration& delta_t);

  // Current timestamp
  rclcpp::Time timestamp_;

  // Current pose
  double x_;
  double y_;
  double heading_;

  double vel_x_;
  double vel_y_;
  double angular_vel_;

  // Wheel kinematics
  double wheel_base_ = 0;
  double axes_gap_ = 0;
  double wheel_radius_ = 0;

  double left_wheel_old_pos_;
  double right_wheel_old_pos_;

  // Rolling mean accumulators for the linear and angular velocities
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator vel_x_accumulator_;
  RollingMeanAccumulator vel_y_accumulator_;
  RollingMeanAccumulator angular_vel_accumulator_;
};

}  // namespace agrobot_controller

#endif  // !AGROBOT_CONTROLLER_ODOMETRY_HPP
