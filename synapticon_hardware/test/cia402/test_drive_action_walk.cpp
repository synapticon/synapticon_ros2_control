// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief What each DriveAction means: its name, its CiA 402 target state, its descent route,
/// whether it may leave Fault, and whether the HAL drives it.

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/cia402/drive_action_walk.hpp"
#include "synapticon_control_msgs/srv/set_drive_action.hpp"

namespace
{
using cia402::DeviceState;
using cia402::WalkPath;
using synapticon_hardware::action_from_string;
using synapticon_hardware::action_to_string;
using synapticon_hardware::drive_walk;
using synapticon_hardware::DriveAction;
using synapticon_hardware::is_walk_action;
using synapticon_hardware::needs_energised_drive;

// Every action, so a test can be written out in full.
const std::vector<DriveAction> ALL_ACTIONS = {
  DriveAction::NONE,       DriveAction::ENABLE,  DriveAction::SWITCH_ON,       DriveAction::SHUTDOWN,
  DriveAction::QUICK_STOP, DriveAction::DISABLE, DriveAction::DISABLE_VOLTAGE, DriveAction::RESET_FAULT,
};

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------

/// Every action has a name that resolves back to it.
TEST(DriveActionNames, EveryActionRoundTripsThroughItsName)
{
  for (const DriveAction action : ALL_ACTIONS)
  {
    const std::string name = action_to_string(action);
    EXPECT_NE(name, "unknown") << static_cast<int>(action);
    const auto resolved = action_from_string(name);
    ASSERT_TRUE(resolved.has_value()) << name;
    EXPECT_EQ(*resolved, action) << name;
  }
}

/// The descent actions resolve from their configuration names.
TEST(DriveActionNames, TheDescentActionsAreNameableFromConfig)
{
  EXPECT_EQ(action_from_string("quick_stop"), DriveAction::QUICK_STOP);
  EXPECT_EQ(action_from_string("disable"), DriveAction::DISABLE);
  EXPECT_EQ(action_from_string("disable_voltage"), DriveAction::DISABLE_VOLTAGE);
}

/// Every SetDriveAction service constant resolves to its action.
TEST(DriveActionNames, EveryServiceConstantResolvesToItsAction)
{
  using Request = synapticon_control_msgs::srv::SetDriveAction::Request;
  EXPECT_EQ(action_from_string(Request::NONE), DriveAction::NONE);
  EXPECT_EQ(action_from_string(Request::ENABLE), DriveAction::ENABLE);
  EXPECT_EQ(action_from_string(Request::SWITCH_ON), DriveAction::SWITCH_ON);
  EXPECT_EQ(action_from_string(Request::SHUTDOWN), DriveAction::SHUTDOWN);
  EXPECT_EQ(action_from_string(Request::QUICK_STOP), DriveAction::QUICK_STOP);
  EXPECT_EQ(action_from_string(Request::DISABLE), DriveAction::DISABLE);
  EXPECT_EQ(action_from_string(Request::DISABLE_VOLTAGE), DriveAction::DISABLE_VOLTAGE);
  EXPECT_EQ(action_from_string(Request::RESET_FAULT), DriveAction::RESET_FAULT);
}

/// An unknown or empty name resolves to nothing.
TEST(DriveActionNames, AnUnknownNameResolvesToNothing)
{
  EXPECT_FALSE(action_from_string("quickstop").has_value());
  EXPECT_FALSE(action_from_string("").has_value());
}

// ---------------------------------------------------------------------------
// The walk each action asks for
// ---------------------------------------------------------------------------

/// enable, switch_on, shutdown and quick_stop walk to their namesake states.
TEST(DriveActionWalk, EachActionWalksToItsOwnState)
{
  EXPECT_EQ(drive_walk(DriveAction::ENABLE).target, DeviceState::OPERATION_ENABLED);
  EXPECT_EQ(drive_walk(DriveAction::SWITCH_ON).target, DeviceState::SWITCHED_ON);
  EXPECT_EQ(drive_walk(DriveAction::SHUTDOWN).target, DeviceState::READY_TO_SWITCH_ON);
  EXPECT_EQ(drive_walk(DriveAction::QUICK_STOP).target, DeviceState::QUICK_STOP_ACTIVE);
}

/// NONE has no target state.
TEST(DriveActionWalk, OnlyDoingNothingHasNoTarget)
{
  EXPECT_EQ(drive_walk(DriveAction::NONE).target, DeviceState::UNKNOWN);
}

/// reset_fault is a ramped walk to Switch On Disabled, retried until the drive leaves Fault.
TEST(DriveActionWalk, AFaultResetDescendsLikeADisable)
{
  EXPECT_EQ(drive_walk(DriveAction::RESET_FAULT).target, DeviceState::SWITCH_ON_DISABLED);
  EXPECT_EQ(drive_walk(DriveAction::RESET_FAULT).path, WalkPath::RAMPED);
}

/// Only reset_fault may leave Fault: a stop finds the drive de-energised, an enable needs a reset first.
TEST(DriveActionWalk, OnlyAFaultResetLeavesFault)
{
  for (const DriveAction action : ALL_ACTIONS)
  {
    const cia402::FaultExit expected =
        action == DriveAction::RESET_FAULT ? cia402::FaultExit::RESET : cia402::FaultExit::HOLD;
    EXPECT_EQ(drive_walk(action).fault_exit, expected) << action_to_string(action);
  }
}

/// disable and disable_voltage both end in Switch On Disabled, but only disable_voltage skips the ramp.
TEST(DriveActionWalk, DisableAndDisableVoltageShareATargetAndDifferInRoute)
{
  EXPECT_EQ(drive_walk(DriveAction::DISABLE).target, DeviceState::SWITCH_ON_DISABLED);
  EXPECT_EQ(drive_walk(DriveAction::DISABLE_VOLTAGE).target, DeviceState::SWITCH_ON_DISABLED);
  EXPECT_EQ(drive_walk(DriveAction::DISABLE).path, WalkPath::RAMPED);
  EXPECT_EQ(drive_walk(DriveAction::DISABLE_VOLTAGE).path, WalkPath::DIRECT);
}

/// Every action but disable_voltage walks the ramped route.
TEST(DriveActionWalk, OnlyDisableVoltageTakesTheDirectRoute)
{
  for (const DriveAction action : ALL_ACTIONS)
  {
    if (action == DriveAction::DISABLE_VOLTAGE)
    {
      continue;
    }
    EXPECT_EQ(drive_walk(action).path, WalkPath::RAMPED) << action_to_string(action);
  }
}

/// From every powered state, disable issues Quick stop first and disable_voltage issues Disable voltage at once.
TEST(DriveActionWalk, DisableStillRampsFromEveryPoweredState)
{
  constexpr uint16_t QUICK_STOP = 0x0002u;
  constexpr uint16_t DISABLE_VOLTAGE = 0x0000u;
  const auto ramped = drive_walk(DriveAction::DISABLE);
  const auto direct = drive_walk(DriveAction::DISABLE_VOLTAGE);
  for (const DeviceState state :
       { DeviceState::READY_TO_SWITCH_ON, DeviceState::SWITCHED_ON, DeviceState::OPERATION_ENABLED })
  {
    EXPECT_EQ(cia402::controlword_toward(state, ramped.target, 0x0000u, ramped.path), QUICK_STOP)
        << cia402::device_state_name(state);
    EXPECT_EQ(cia402::controlword_toward(state, direct.target, 0x0000u, direct.path), DISABLE_VOLTAGE)
        << cia402::device_state_name(state);
  }
}

/// Both disable routes leave a faulted drive in Fault and are done at once.
TEST(DriveActionWalk, NeitherDisableRouteResetsAFault)
{
  for (const DriveAction action : { DriveAction::DISABLE, DriveAction::DISABLE_VOLTAGE })
  {
    const auto walk = drive_walk(action);
    EXPECT_EQ(cia402::controlword_toward(DeviceState::FAULT, walk.target, 0x0000u, walk.path, walk.fault_exit), 0x0000u)
        << action_to_string(action);
    EXPECT_TRUE(cia402::reached(DeviceState::FAULT, walk.target, walk.fault_exit)) << action_to_string(action);
  }
}

/// reset_fault sends Fault Reset and is not done while the drive reports Fault.
TEST(DriveActionWalk, AResetFaultSendsFaultResetUntilTheDriveLeavesFault)
{
  constexpr uint16_t FAULT_RESET = 0x0080u;
  const auto walk = drive_walk(DriveAction::RESET_FAULT);
  EXPECT_EQ(cia402::controlword_toward(DeviceState::FAULT, walk.target, 0x0000u, walk.path, walk.fault_exit),
            FAULT_RESET);
  EXPECT_FALSE(cia402::reached(DeviceState::FAULT, walk.target, walk.fault_exit));
}

// ---------------------------------------------------------------------------
// Which actions the client drives
// ---------------------------------------------------------------------------

/// The walk actions are enable, the three stops and reset_fault. switch_on and shutdown are refused.
TEST(DriveActionWalk, TheWiredActionsAreTheFourDescentsAndEnable)
{
  EXPECT_TRUE(is_walk_action(DriveAction::ENABLE));
  EXPECT_TRUE(is_walk_action(DriveAction::DISABLE));
  EXPECT_TRUE(is_walk_action(DriveAction::QUICK_STOP));
  EXPECT_TRUE(is_walk_action(DriveAction::DISABLE_VOLTAGE));
  EXPECT_TRUE(is_walk_action(DriveAction::RESET_FAULT));

  EXPECT_FALSE(is_walk_action(DriveAction::SWITCH_ON));
  EXPECT_FALSE(is_walk_action(DriveAction::SHUTDOWN));
  EXPECT_FALSE(is_walk_action(DriveAction::NONE));
}

/// Every walk action has a target state.
TEST(DriveActionWalk, EveryWiredActionHasATarget)
{
  for (const DriveAction action : ALL_ACTIONS)
  {
    if (!is_walk_action(action))
    {
      continue;
    }
    EXPECT_NE(drive_walk(action).target, DeviceState::UNKNOWN) << action_to_string(action);
  }
}

/// enable and quick_stop need a joint that may energise, the descents and reset_fault apply to any joint.
TEST(DriveActionWalk, OnlyTheActionsNeedingPowerAreRefusedOnAMonitoredJoint)
{
  EXPECT_TRUE(needs_energised_drive(DriveAction::ENABLE));
  EXPECT_TRUE(needs_energised_drive(DriveAction::QUICK_STOP));
  EXPECT_FALSE(needs_energised_drive(DriveAction::DISABLE));
  EXPECT_FALSE(needs_energised_drive(DriveAction::DISABLE_VOLTAGE));
  EXPECT_FALSE(needs_energised_drive(DriveAction::RESET_FAULT));
  EXPECT_FALSE(needs_energised_drive(DriveAction::NONE));
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
