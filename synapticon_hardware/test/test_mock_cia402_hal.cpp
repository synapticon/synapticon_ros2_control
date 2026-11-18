// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The setup and call order of MockCia402Hal, and the state machine of its simulated drive.
///
/// The drive behaviour itself is tested by the drive action suite, which runs on this HAL.

#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <thread>
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
using synapticon_hardware::JointSpec;
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
                      InvalidCase{ "ModeWithTrailingText", { { "mode_of_operation", "8x" } }, {} },
                      InvalidCase{ "FractionalOptionCode", { { "quick_stop_option_code", "6.5" } }, {} },
                      InvalidCase{ "DecelerationWithUnit", { { "quick_stop_deceleration", "20rad" } }, {} },
                      InvalidCase{ "FrequencyWithUnit", {}, { { "control_frequency", "500Hz" } } },
                      InvalidCase{ "NonBooleanEnergize", { { "energize", "maybe" } }, {} },
                      InvalidCase{ "ZeroFrequency", {}, { { "control_frequency", "0" } } }),
    [](const auto& info) { return std::string(info.param.name); });

TEST(MockCia402Setup, NeedsAJoint)
{
  MockCia402Hal hal;
  EXPECT_FALSE(hal.setup(HalConfig{}));
}

/// joint_count() is known right after setup(), and a failed setup() keeps nothing of an earlier one.
TEST(MockCia402Setup, AFailedSetupKeepsNoConfiguration)
{
  MockCia402Hal hal;
  ASSERT_TRUE(hal.setup(one_joint()));
  EXPECT_EQ(hal.joint_count(), 1u);
  EXPECT_FALSE(hal.setup(one_joint({ { "following_error", "sometimes" } })));
  EXPECT_EQ(hal.joint_count(), 0u);
  EXPECT_FALSE(hal.connect());
}

// --- Call order -------------------------------------------------------------------

TEST(MockCia402Order, EachStepNeedsTheOneBefore)
{
  MockCia402Hal hal;
  EXPECT_FALSE(hal.connect());
  ASSERT_TRUE(hal.setup(one_joint()));
  EXPECT_FALSE(hal.start());
  EXPECT_FALSE(hal.enable());
  ASSERT_TRUE(hal.connect());
  std::vector<DriveState> states;
  EXPECT_FALSE(hal.read(states));
  EXPECT_FALSE(hal.enable());
  EXPECT_FALSE(hal.setup(one_joint()));
  ASSERT_TRUE(hal.start());
  EXPECT_TRUE(hal.read(states));
  EXPECT_TRUE(hal.stop());
  EXPECT_TRUE(hal.disconnect());
}

/// An up-call on the rung the HAL is on, and a down-call for a rung it is not on, succeed and change nothing.
TEST(MockCia402Order, CallsForTheRungItIsOnChangeNothing)
{
  MockCia402Hal hal;
  ASSERT_TRUE(hal.setup(one_joint()));
  EXPECT_TRUE(hal.disable());
  EXPECT_TRUE(hal.stop());
  EXPECT_TRUE(hal.disconnect());
  ASSERT_TRUE(hal.connect());
  EXPECT_TRUE(hal.connect());
  EXPECT_TRUE(hal.disable());
  EXPECT_TRUE(hal.stop());
  EXPECT_TRUE(hal.is_connected());
  EXPECT_FALSE(hal.is_streaming());
  ASSERT_TRUE(hal.start());
  EXPECT_TRUE(hal.start());
  EXPECT_TRUE(hal.disable());
  EXPECT_TRUE(hal.is_streaming());
  ASSERT_TRUE(hal.enable());
  EXPECT_TRUE(hal.enable());
  EXPECT_TRUE(hal.is_enabled());
  EXPECT_TRUE(hal.disconnect());
  EXPECT_FALSE(hal.is_connected());
  EXPECT_FALSE(hal.is_streaming());
  EXPECT_FALSE(hal.is_enabled());
}

/// Run `hal` for `seconds` with `commands` in every cycle. The states of the last read go to `states`. False when a
/// read or write failed.
bool run_for(MockCia402Hal& hal, double seconds, const std::vector<DriveCommand>& commands,
             std::vector<DriveState>& states)
{
  bool all_ok = true;
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < end)
  {
    all_ok = hal.read(states) && all_ok;
    all_ok = hal.write(commands) && all_ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return hal.read(states) && all_ok;
}

/// stop() takes the power off a drive first, also when only a drive action powered it.
TEST(MockCia402Order, StopSwitchesThePowerOff)
{
  MockCia402Hal hal;
  ASSERT_TRUE(hal.setup(one_joint()));
  ASSERT_TRUE(hal.connect());
  ASSERT_TRUE(hal.start());
  std::vector<DriveCommand> commands(1);
  commands[0].drive_action = synapticon_hardware::DriveAction::ENABLE;
  std::vector<DriveState> states;
  ASSERT_TRUE(run_for(hal, 0.1, commands, states));
  ASSERT_EQ(cia402::device_state(states[0].status_word), DeviceState::OPERATION_ENABLED);
  ASSERT_TRUE(hal.stop());
  EXPECT_FALSE(hal.is_streaming());
  ASSERT_TRUE(hal.start());
  ASSERT_TRUE(hal.read(states));
  EXPECT_EQ(cia402::device_state(states[0].status_word), DeviceState::SWITCH_ON_DISABLED);
  EXPECT_TRUE(hal.disconnect());
}

// --- Setpoint values --------------------------------------------------------------

/// Two velocity mode drives, connected, streaming and enabled.
class VelocityDrives : public ::testing::Test
{
protected:
  void SetUp() override
  {
    HalConfig config;
    for (const char* name : { "joint_1", "joint_2" })
    {
      config.joints.push_back(JointSpec{ name, { { "mode_of_operation", "9" } } });
    }
    ASSERT_TRUE(hal_.setup(config));
    ASSERT_TRUE(hal_.connect());
    ASSERT_TRUE(hal_.start());
    ASSERT_TRUE(hal_.enable());
  }
  void TearDown() override
  {
    hal_.disconnect();
  }

  /// Commands with velocity setpoints `first` and `second`.
  static std::vector<DriveCommand> velocities(double first, double second)
  {
    std::vector<DriveCommand> commands(2);
    commands[0].velocity = first;
    commands[1].velocity = second;
    return commands;
  }

  MockCia402Hal hal_;
  std::vector<DriveState> states_;
};

/// NaN keeps the value sent last. An infinite value fails the write and keeps the last value as well, while the
/// other joint's value still goes out.
TEST_F(VelocityDrives, NaNAndInfinityKeepTheValueSentLast)
{
  constexpr double NOT_SET = std::numeric_limits<double>::quiet_NaN();
  constexpr double INFINITE = std::numeric_limits<double>::infinity();
  ASSERT_TRUE(run_for(hal_, 0.2, velocities(1.0, 1.0), states_));
  EXPECT_NEAR(states_[0].velocity, 1.0, 0.05);

  ASSERT_TRUE(run_for(hal_, 0.2, velocities(NOT_SET, 0.5), states_));
  EXPECT_NEAR(states_[0].velocity, 1.0, 0.05);
  EXPECT_NEAR(states_[1].velocity, 0.5, 0.05);

  EXPECT_FALSE(run_for(hal_, 0.2, velocities(INFINITE, -0.5), states_));
  EXPECT_NEAR(states_[0].velocity, 1.0, 0.05);
  EXPECT_NEAR(states_[1].velocity, -0.5, 0.05);
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
