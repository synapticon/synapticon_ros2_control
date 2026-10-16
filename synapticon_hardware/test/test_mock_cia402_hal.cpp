// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The setup and call order of MockCia402Hal, and the state machine of its simulated drive.
///
/// The drive behaviour itself is tested by the drive action suite, which runs on this HAL.

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "mock/simulated_drive.hpp"
#include "synapticon_hardware/hal_config.hpp"
#include "synapticon_hardware/mock_cia402_hal.hpp"

namespace
{
using cia402::DeviceState;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::HalConfig;
using synapticon_hardware::MockCia402Hal;
using synapticon_hardware::SimulatedDrive;
using synapticon_hardware::SimulatedDriveSettings;

/// One joint with `joint_params`, under `hardware_params`.
HalConfig one_joint(const std::unordered_map<std::string, std::string>& joint_params = {},
                    const std::unordered_map<std::string, std::string>& hardware_params = {})
{
  HalConfig config;
  config.hardware_parameters = hardware_params;
  synapticon_hardware::JointSpec joint;
  joint.name = "joint_1";
  joint.parameters = joint_params;
  config.joints.push_back(joint);
  return config;
}

// --- Setup ------------------------------------------------------------------------

TEST(MockCia402Setup, AcceptsTheSupportedParameters)
{
  MockCia402Hal hal;
  EXPECT_TRUE(hal.setup(one_joint({ { "mode_of_operation", "-6" },
                                    { "following_error", "fault" },
                                    { "quick_stop_option_code", "2" },
                                    { "energize", "false" } },
                                  { { "control_frequency", "500" }, { "quick_stop_hold", "false" } })));
}

struct InvalidCase
{
  const char* name;
  std::unordered_map<std::string, std::string> joint_params;
  std::unordered_map<std::string, std::string> hardware_params;
};

class InvalidSetup : public ::testing::TestWithParam<InvalidCase>
{
};

TEST_P(InvalidSetup, IsRefused)
{
  MockCia402Hal hal;
  EXPECT_FALSE(hal.setup(one_joint(GetParam().joint_params, GetParam().hardware_params)));
  EXPECT_FALSE(hal.connect());
}

INSTANTIATE_TEST_SUITE_P(
    Cases, InvalidSetup,
    ::testing::Values(InvalidCase{ "UnknownFollowingError", { { "following_error", "sometimes" } }, {} },
                      InvalidCase{ "UnsupportedMode", { { "mode_of_operation", "10" } }, {} },
                      InvalidCase{ "NonNumericMode", { { "mode_of_operation", "csp" } }, {} },
                      InvalidCase{ "NonBooleanEnergize", { { "energize", "maybe" } }, {} },
                      InvalidCase{ "ZeroFrequency", {}, { { "control_frequency", "0" } } }),
    [](const auto& info) { return std::string(info.param.name); });

TEST(MockCia402Setup, NeedsAJoint)
{
  MockCia402Hal hal;
  EXPECT_FALSE(hal.setup(HalConfig{}));
}

// --- Call order -------------------------------------------------------------------

TEST(MockCia402Order, EachStepNeedsTheOneBefore)
{
  MockCia402Hal hal;
  EXPECT_FALSE(hal.connect());
  ASSERT_TRUE(hal.setup(one_joint()));
  EXPECT_FALSE(hal.start());
  ASSERT_TRUE(hal.connect());
  std::vector<DriveState> states;
  EXPECT_FALSE(hal.read(states));
  EXPECT_FALSE(hal.connect());
  EXPECT_FALSE(hal.setup(one_joint()));
  ASSERT_TRUE(hal.start());
  EXPECT_FALSE(hal.start());
  EXPECT_TRUE(hal.read(states));
  EXPECT_TRUE(hal.stop());
  EXPECT_TRUE(hal.disconnect());
}

// --- The simulated drive's state machine -------------------------------------------------

constexpr uint16_t DISABLE_VOLTAGE = 0x00;
constexpr uint16_t QUICK_STOP = 0x02;
constexpr uint16_t SHUTDOWN = 0x06;
constexpr uint16_t SWITCH_ON = 0x07;
constexpr uint16_t ENABLE_OPERATION = 0x0F;

/// A simulated drive walked from Switch On Disabled along `controlwords`, one step each.
SimulatedDrive walked(const std::vector<uint16_t>& controlwords)
{
  SimulatedDrive drive{ SimulatedDriveSettings{} };
  for (const uint16_t word : controlwords)
  {
    drive.command(word, DriveCommand{});
    drive.step(0.001);
  }
  return drive;
}

struct TransitionCase
{
  const char* name;
  std::vector<uint16_t> controlwords;
  DeviceState reached;
};

class Transition : public ::testing::TestWithParam<TransitionCase>
{
};

TEST_P(Transition, FollowsTheControlword)
{
  EXPECT_EQ(walked(GetParam().controlwords).device_state(), GetParam().reached);
}

INSTANTIATE_TEST_SUITE_P(
    Cases, Transition,
    ::testing::Values(
        TransitionCase{ "UpTheLadder", { SHUTDOWN, SWITCH_ON, ENABLE_OPERATION }, DeviceState::OPERATION_ENABLED },
        TransitionCase{ "SwitchedOnBackToReady", { SHUTDOWN, SWITCH_ON, SHUTDOWN }, DeviceState::READY_TO_SWITCH_ON },
        TransitionCase{ "ReadyDisableVoltage", { SHUTDOWN, DISABLE_VOLTAGE }, DeviceState::SWITCH_ON_DISABLED },
        TransitionCase{ "SwitchedOnQuickStop", { SHUTDOWN, SWITCH_ON, QUICK_STOP }, DeviceState::SWITCH_ON_DISABLED },
        TransitionCase{ "EnabledBackToSwitchedOn",
                        { SHUTDOWN, SWITCH_ON, ENABLE_OPERATION, SWITCH_ON },
                        DeviceState::SWITCHED_ON },
        TransitionCase{ "EnabledBackToReady",
                        { SHUTDOWN, SWITCH_ON, ENABLE_OPERATION, SHUTDOWN },
                        DeviceState::READY_TO_SWITCH_ON }),
    [](const auto& info) { return std::string(info.param.name); });

}  // namespace
