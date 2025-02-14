#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadAgrobotController, load_controller) {
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
      executor, ros2_control_test_assets::minimal_robot_urdf, true,
      "test_controller_manager");

  ASSERT_NE(cm.load_controller("test_load_agrobot_controller",
                               "agrobot_controller/AgrobotController"),
            nullptr);

  rclcpp::shutdown();
}
