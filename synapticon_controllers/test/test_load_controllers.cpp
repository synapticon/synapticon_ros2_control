// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Checks that the controller manager can load every controller of the package through pluginlib.

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
class LoadController : public ::testing::TestWithParam<std::string>
{
};

TEST_P(LoadController, TheControllerManagerLoadsIt)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  controller_manager::ControllerManager cm(executor, ros2_control_test_assets::minimal_robot_urdf, true,
                                           "test_controller_manager");
  cm.set_parameter({ "test_controller.type", GetParam() });
  ASSERT_NE(cm.load_controller("test_controller"), nullptr) << GetParam();
}

INSTANTIATE_TEST_SUITE_P(SynapticonControllers, LoadController,
                         ::testing::Values("synapticon_controllers/DriveController",
                                           "synapticon_controllers/EthercatBusMonitorController",
                                           "synapticon_controllers/MotorCommandController"));
}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
