#include "agrobot_controller/agrobot_odometry.hpp"
#include <cmath>
#include <rclcpp/duration.hpp>

namespace agrobot_controller {

AgrobotOdometry::AgrobotOdometry(size_t velocity_rolling_window_size)
    : vel_x_accumulator_(velocity_rolling_window_size),
      vel_y_accumulator_(velocity_rolling_window_size),
      angular_vel_accumulator_(velocity_rolling_window_size) {
  velocity_rolling_window_size_ = velocity_rolling_window_size;
}

void AgrobotOdometry::init(const rclcpp::Time& time) {
  this->resetAccumulators();
  timestamp_ = time;
}

void AgrobotOdometry::update(double fl_vel, double fr_vel, double rl_vel,
                             double rr_vel, const rclcpp::Time& time) {

  const auto delta_t = time.nanoseconds() - timestamp_.nanoseconds();

  std::array<double, 4> w{fr_vel, fl_vel, rl_vel, rr_vel};

  const double lin_vel_x = wheel_radius_ / 4 * (w[0] + w[1] + w[2] + w[3]);
  const double lin_vel_y = wheel_radius_ / 4 * (w[0] - w[1] + w[2] - w[3]);
  const double ang_vel_z = wheel_radius_ / (wheel_base_ + axes_gap_) *
                           (w[0] - w[1] - w[2] + w[3]) / 4 / M_PI;

  timestamp_ = time;

  vel_x_accumulator_.accumulate(lin_vel_x);
  vel_y_accumulator_.accumulate(lin_vel_y);
  angular_vel_accumulator_.accumulate(ang_vel_z);

  vel_x_ = vel_x_accumulator_.getRollingMean();
  vel_y_ = vel_y_accumulator_.getRollingMean();
  angular_vel_ = angular_vel_accumulator_.getRollingMean();

  integratePosition(rclcpp::Duration(0, delta_t));
}

void AgrobotOdometry::resetAccumulators() {
  for (size_t i = 0; i < velocity_rolling_window_size_; ++i) {
    vel_x_accumulator_.accumulate(0);
    vel_y_accumulator_.accumulate(0);
    angular_vel_accumulator_.accumulate(0);
  }
}

void AgrobotOdometry::integratePosition(const rclcpp::Duration& dt) {
  x_ += vel_x_ * dt.seconds();
  y_ += vel_y_ * dt.seconds();
  heading_ += angular_vel_ * dt.seconds();
}

void AgrobotOdometry::resetPosition() {
  x_ = 0;
  y_ = 0;
  heading_ = 0;
}

void AgrobotOdometry::setWheelParams(double wheel_base, double axes_gap,
                                     double wheel_radius) {
  wheel_base_ = wheel_base;
  axes_gap_ = axes_gap;
  wheel_radius_ = wheel_radius;
}

void AgrobotOdometry::setVelocityRollingWindowSize(size_t window_size) {
  velocity_rolling_window_size_ = window_size;
}

}  // namespace agrobot_controller
