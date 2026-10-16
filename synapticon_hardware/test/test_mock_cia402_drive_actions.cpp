// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The drive action suite on MockCia402Hal: the hardware tests' logic, run on simulated drives.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/hal_config.hpp"
#include "synapticon_hardware/mock_cia402_hal.hpp"
#include "synapticon_hardware/testing/cia402_hal_tests.hpp"

namespace
{
using synapticon_hardware::Cia402Hal;
using synapticon_hardware::HalConfig;
using synapticon_hardware::JointSpec;
using synapticon_hardware::MockCia402Hal;
using synapticon_hardware::testing::BENCH_RATE_HZ;
using synapticon_hardware::testing::DriveSettings;
using synapticon_hardware::testing::FollowingError;

/// The drive settings as MockCia402Hal joint parameters.
JointSpec joint_spec(const std::string& name, const DriveSettings& settings)
{
  static const char* const FOLLOWING_ERROR[] = { "off", "report", "fault" };
  JointSpec spec{ name, {} };
  spec.parameters["mode_of_operation"] = std::to_string(settings.mode);
  spec.parameters["energize"] = settings.energize ? "true" : "false";
  spec.parameters["quick_stop_option_code"] = std::to_string(settings.quick_stop_option_code);
  if (std::isfinite(settings.quick_stop_deceleration))
  {
    spec.parameters["quick_stop_deceleration"] = std::to_string(settings.quick_stop_deceleration);
  }
  spec.parameters["following_error"] = FOLLOWING_ERROR[static_cast<int>(settings.following_error)];
  return spec;
}

/// Simulated drives, two of them whenever a test asks.
class MockCia402Bench : public synapticon_hardware::testing::HalBench
{
public:
  std::shared_ptr<Cia402Hal> open_drives(const std::vector<DriveSettings>& drives) override
  {
    HalConfig config;
    config.hardware_parameters["control_frequency"] = std::to_string(BENCH_RATE_HZ);
    config.hardware_parameters["quick_stop_hold"] = drives.at(0).quick_stop_hold ? "true" : "false";
    for (std::size_t j = 0; j < drives.size(); ++j)
    {
      config.joints.push_back(joint_spec("joint_" + std::to_string(j + 1), drives[j]));
    }
    auto hal = std::make_shared<MockCia402Hal>();
    if (!hal->setup(config) || !hal->connect() || !hal->start())
    {
      return nullptr;
    }
    return hal;
  }

  bool two_drives() const override
  {
    return true;
  }
};
}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  synapticon_hardware::testing::bench_instance() = std::make_shared<MockCia402Bench>();
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
