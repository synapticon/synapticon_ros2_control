// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The hold seed at activation and the NaN sanitising of commands.

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/command_shaping.hpp"

namespace
{
using synapticon_hardware::DriveAction;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::sanitise_commands;
using synapticon_hardware::seed_hold_commands;

constexpr double QUIET_NAN = std::numeric_limits<double>::quiet_NaN();

/// Two measured states with distinct positions.
std::vector<DriveState> two_states()
{
  std::vector<DriveState> states(2);
  states[0].position = 1.5;
  states[1].position = -0.25;
  return states;
}

/// The seed commands the measured position with zero motion, NaN gains and limit, and no action.
TEST(CommandShaping, TheSeedHoldsTheMeasuredPositionAndChangesNoGain)
{
  std::vector<DriveCommand> commands;
  seed_hold_commands(two_states(), commands);
  ASSERT_EQ(commands.size(), 2u);
  EXPECT_DOUBLE_EQ(commands[0].position, 1.5);
  EXPECT_DOUBLE_EQ(commands[1].position, -0.25);
  for (const auto& c : commands)
  {
    EXPECT_DOUBLE_EQ(c.velocity, 0.0);
    EXPECT_DOUBLE_EQ(c.effort, 0.0);
    EXPECT_TRUE(std::isnan(c.stiffness));
    EXPECT_TRUE(std::isnan(c.damping));
    EXPECT_TRUE(std::isnan(c.max_joint_torque));
    EXPECT_EQ(c.drive_action, DriveAction::NONE);
  }
}

/// NaN setpoints become the measured position and zero motion, NaN gains and limit pass through unchanged.
TEST(CommandShaping, NanSetpointsAreReplacedAndGainsPassThrough)
{
  std::vector<DriveCommand> commands(2);
  commands[0].position = QUIET_NAN;
  commands[0].velocity = QUIET_NAN;
  commands[0].effort = QUIET_NAN;
  commands[0].stiffness = QUIET_NAN;
  commands[0].max_joint_torque = QUIET_NAN;
  commands[1].position = 0.7;
  commands[1].velocity = 0.2;
  commands[1].effort = 0.3;
  commands[1].stiffness = 20.0;
  commands[1].drive_action = DriveAction::ENABLE;

  std::vector<DriveCommand> sanitised;
  sanitise_commands(commands, two_states(), sanitised);
  ASSERT_EQ(sanitised.size(), 2u);
  EXPECT_DOUBLE_EQ(sanitised[0].position, 1.5) << "a NaN position holds the measured one";
  EXPECT_DOUBLE_EQ(sanitised[0].velocity, 0.0);
  EXPECT_DOUBLE_EQ(sanitised[0].effort, 0.0);
  EXPECT_TRUE(std::isnan(sanitised[0].stiffness)) << "NaN gains mean no change and pass through";
  EXPECT_TRUE(std::isnan(sanitised[0].max_joint_torque));
  EXPECT_DOUBLE_EQ(sanitised[1].position, 0.7);
  EXPECT_DOUBLE_EQ(sanitised[1].velocity, 0.2);
  EXPECT_DOUBLE_EQ(sanitised[1].effort, 0.3);
  EXPECT_DOUBLE_EQ(sanitised[1].stiffness, 20.0);
  EXPECT_EQ(sanitised[1].drive_action, DriveAction::ENABLE);
}

/// Infinite setpoints are invalid, not missing: they pass through unchanged, so that the HAL refuses them.
TEST(CommandShaping, InfiniteSetpointsPassThroughForTheHalToRefuse)
{
  constexpr double INF = std::numeric_limits<double>::infinity();
  std::vector<DriveCommand> commands(2);
  commands[0].position = INF;
  commands[0].velocity = -INF;
  commands[1].effort = INF;

  std::vector<DriveCommand> sanitised;
  sanitise_commands(commands, two_states(), sanitised);
  ASSERT_EQ(sanitised.size(), 2u);
  EXPECT_EQ(sanitised[0].position, INF) << "an infinite position must not turn into a hold";
  EXPECT_EQ(sanitised[0].velocity, -INF);
  EXPECT_EQ(sanitised[1].effort, INF) << "an infinite effort must not turn into 0";
}

/// Sanitising writes to the output only.
TEST(CommandShaping, TheInputCommandsAreNotTouched)
{
  std::vector<DriveCommand> commands(1);
  commands[0].position = QUIET_NAN;
  std::vector<DriveCommand> sanitised;
  sanitise_commands(commands, two_states(), sanitised);
  EXPECT_TRUE(std::isnan(commands[0].position));
}

}  // namespace
