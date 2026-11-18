// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The command seed at activation, the hold reference and the NaN sanitising of commands.

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/command_shaping.hpp"

namespace
{
using synapticon_hardware::DriveAction;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::HoldReference;
using synapticon_hardware::not_given_command;
using synapticon_hardware::sanitise_commands;
using synapticon_hardware::seed_not_given;

constexpr double QUIET_NAN = std::numeric_limits<double>::quiet_NaN();

/// Two measured states with distinct positions.
std::vector<DriveState> two_states()
{
  std::vector<DriveState> states(2);
  states[0].position = 1.5;
  states[1].position = -0.25;
  return states;
}

/// The hold positions of two joints whose hold reference starts from two_states().
std::vector<double> two_holds()
{
  return { 1.5, -0.25 };
}

/// The seed gives nothing: every value NaN and no action, whatever the commands held before.
TEST(CommandShaping, TheSeedGivesNothing)
{
  std::vector<DriveCommand> commands(2);
  commands[0].position = 0.7;
  commands[1].drive_action = DriveAction::ENABLE;
  seed_not_given(commands);
  ASSERT_EQ(commands.size(), 2u);
  for (const auto& c : commands)
  {
    EXPECT_TRUE(std::isnan(c.position));
    EXPECT_TRUE(std::isnan(c.velocity));
    EXPECT_TRUE(std::isnan(c.effort));
    EXPECT_TRUE(std::isnan(c.stiffness));
    EXPECT_TRUE(std::isnan(c.damping));
    EXPECT_TRUE(std::isnan(c.max_joint_torque));
    EXPECT_EQ(c.drive_action, DriveAction::NONE);
  }
}

/// NaN setpoints become the hold position and zero motion, NaN gains and limit pass through unchanged.
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
  sanitise_commands(commands, two_holds(), sanitised);
  ASSERT_EQ(sanitised.size(), 2u);
  EXPECT_DOUBLE_EQ(sanitised[0].position, 1.5) << "a NaN position holds at the hold position";
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
  sanitise_commands(commands, two_holds(), sanitised);
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
  sanitise_commands(commands, two_holds(), sanitised);
  EXPECT_TRUE(std::isnan(commands[0].position));
}

/// With power, a joint without a position command holds the last valid target, however far it sags.
TEST(HoldReference, APoweredJointHoldsTheLastTargetWhileTheMeasurementMoves)
{
  HoldReference hold;
  std::vector<DriveState> states = two_states();
  hold.reset(states);
  const std::vector<uint8_t> powered{ 1u, 1u };
  std::vector<DriveCommand> commands(2, not_given_command());
  commands[0].position = 0.8;
  hold.update(commands, states, powered);
  commands[0].position = QUIET_NAN;
  for (int cycle = 0; cycle < 100; ++cycle)
  {
    states[0].position -= 0.01;
    states[1].position -= 0.01;
    hold.update(commands, states, powered);
  }
  EXPECT_DOUBLE_EQ(hold.positions()[0], 0.8) << "the last valid target";
  EXPECT_DOUBLE_EQ(hold.positions()[1], -0.25) << "the position at the start, never the sagging measurement";
}

/// Without power the hold position follows the measurement, so a joint moved by hand does not jump back when its
/// drive gets power. From that cycle on it stays.
TEST(HoldReference, AnUnpoweredJointFollowsTheMeasurementUntilItGetsPower)
{
  HoldReference hold;
  std::vector<DriveState> states = two_states();
  hold.reset(states);
  const std::vector<DriveCommand> commands(2, not_given_command());
  states[0].position = 0.3;  // moved by hand
  hold.update(commands, states, { 0u, 0u });
  EXPECT_DOUBLE_EQ(hold.positions()[0], 0.3);
  hold.update(commands, states, { 1u, 0u });  // gets power here
  states[0].position = 0.1;                   // and sags
  hold.update(commands, states, { 1u, 0u });
  EXPECT_DOUBLE_EQ(hold.positions()[0], 0.3);
}

/// A NaN or infinite position never becomes the hold position.
TEST(HoldReference, OnlyAFinitePositionMovesTheHold)
{
  HoldReference hold;
  const std::vector<DriveState> states = two_states();
  hold.reset(states);
  std::vector<DriveCommand> commands(2, not_given_command());
  commands[0].position = std::numeric_limits<double>::infinity();
  hold.update(commands, states, { 1u, 1u });
  EXPECT_DOUBLE_EQ(hold.positions()[0], 1.5);
}

}  // namespace
