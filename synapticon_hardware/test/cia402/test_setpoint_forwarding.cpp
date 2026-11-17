// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Whether a motion controller's setpoints reach a drive, across drive actions and
/// reported drive states.

#include <gtest/gtest.h>

#include "synapticon_hardware/cia402/state_machine.hpp"
#include "synapticon_hardware/cia402/setpoint_forwarding.hpp"

namespace
{

using cia402::DeviceState;
using synapticon_hardware::DriveAction;
using synapticon_hardware::is_fault_state;
using synapticon_hardware::is_stop_action;
using synapticon_hardware::SetpointForwarding;

constexpr DriveAction STOP_ACTIONS[] = { DriveAction::QUICK_STOP, DriveAction::DISABLE, DriveAction::DISABLE_VOLTAGE };
constexpr DriveAction OTHER_ACTIONS[] = { DriveAction::NONE, DriveAction::SWITCH_ON, DriveAction::SHUTDOWN,
                                          DriveAction::RESET_FAULT };

/// A fresh joint forwards.
TEST(SetpointForwarding, StartsForwarding)
{
  EXPECT_TRUE(SetpointForwarding{}.forwarding);
}

/// quick_stop, disable and disable_voltage are the stop actions, nothing else is.
TEST(SetpointForwarding, OnlyTheThreeStopActionsStop)
{
  for (const DriveAction action : STOP_ACTIONS)
  {
    EXPECT_TRUE(is_stop_action(action));
  }
  for (const DriveAction action : OTHER_ACTIONS)
  {
    EXPECT_FALSE(is_stop_action(action));
  }
  EXPECT_FALSE(is_stop_action(DriveAction::ENABLE));
}

/// A stop action stops forwarding, and a repeated stop is not a change.
TEST(SetpointForwarding, AStopActionStartsIgnoring)
{
  for (const DriveAction action : STOP_ACTIONS)
  {
    SetpointForwarding joint;
    EXPECT_TRUE(joint.on_action(action));
    EXPECT_FALSE(joint.forwarding);
    EXPECT_FALSE(joint.on_action(action)) << "a repeated stop is not a change";
  }
}

/// An enable resumes forwarding, and an enable while forwarding is not a change.
TEST(SetpointForwarding, EnableResumesForwarding)
{
  SetpointForwarding joint;
  joint.on_action(DriveAction::QUICK_STOP);
  EXPECT_TRUE(joint.on_action(DriveAction::ENABLE));
  EXPECT_TRUE(joint.forwarding);
  EXPECT_FALSE(joint.on_action(DriveAction::ENABLE)) << "enable while forwarding is not a change";
}

/// NONE, switch_on, shutdown and reset_fault change nothing either way.
TEST(SetpointForwarding, OtherActionsLeaveTheStateAlone)
{
  for (const bool forwarding : { true, false })
  {
    for (const DriveAction action : OTHER_ACTIONS)
    {
      SetpointForwarding joint{ forwarding };
      EXPECT_FALSE(joint.on_action(action));
      EXPECT_EQ(joint.forwarding, forwarding);
    }
  }
}

/// Entering Fault or Fault Reaction Active stops forwarding, staying there is not a change.
TEST(SetpointForwarding, EnteringAFaultStateStartsIgnoring)
{
  for (const DeviceState state : { DeviceState::FAULT, DeviceState::FAULT_REACTION_ACTIVE })
  {
    EXPECT_TRUE(is_fault_state(state));
    SetpointForwarding joint;
    joint.on_state(DeviceState::OPERATION_ENABLED);
    EXPECT_TRUE(joint.on_state(state));
    EXPECT_FALSE(joint.forwarding);
    EXPECT_FALSE(joint.on_state(state)) << "a drive staying in Fault is not a change";
  }
}

/// A drive first seen in Fault is ignored from the start.
TEST(SetpointForwarding, ADriveAlreadyInFaultWhenFirstSeenIsIgnored)
{
  SetpointForwarding joint;
  EXPECT_TRUE(joint.on_state(DeviceState::FAULT));
  EXPECT_FALSE(joint.forwarding);
}

/// A drive still reporting Fault after an enable is no new fault, so the enable survives it.
TEST(SetpointForwarding, StayingInFaultDoesNotUndoAnEnable)
{
  SetpointForwarding joint;
  joint.on_state(DeviceState::FAULT);
  joint.on_action(DriveAction::ENABLE);
  EXPECT_FALSE(joint.on_state(DeviceState::FAULT));
  EXPECT_TRUE(joint.forwarding);
  joint.on_state(DeviceState::SWITCH_ON_DISABLED);
  joint.on_state(DeviceState::OPERATION_ENABLED);
  EXPECT_TRUE(joint.forwarding);
}

/// With the state applied before the action, a fresh fault is seen and the enable requested now still wins.
TEST(SetpointForwarding, AFreshFaultThenAnEnableInTheSameCycleForwards)
{
  SetpointForwarding joint;
  joint.on_state(DeviceState::OPERATION_ENABLED);
  EXPECT_TRUE(joint.on_state(DeviceState::FAULT));
  EXPECT_TRUE(joint.on_action(DriveAction::ENABLE));
  EXPECT_TRUE(joint.forwarding);
}

/// A new fault after an enable stops forwarding again, and Fault Reaction Active to Fault is one fault.
TEST(SetpointForwarding, FaultingAgainIgnoresAgain)
{
  SetpointForwarding joint;
  joint.on_state(DeviceState::FAULT);
  joint.on_action(DriveAction::ENABLE);
  joint.on_state(DeviceState::OPERATION_ENABLED);
  EXPECT_TRUE(joint.on_state(DeviceState::FAULT_REACTION_ACTIVE));
  EXPECT_FALSE(joint.forwarding);
  EXPECT_FALSE(joint.on_state(DeviceState::FAULT)) << "Fault Reaction Active to Fault is one fault";
}

/// No reported state resumes forwarding, only the enable action does.
TEST(SetpointForwarding, NoReportedStateResumesForwarding)
{
  for (const DeviceState state :
       { DeviceState::SWITCH_ON_DISABLED, DeviceState::READY_TO_SWITCH_ON, DeviceState::SWITCHED_ON,
         DeviceState::OPERATION_ENABLED, DeviceState::QUICK_STOP_ACTIVE, DeviceState::NOT_READY_TO_SWITCH_ON,
         DeviceState::UNKNOWN })
  {
    EXPECT_FALSE(is_fault_state(state));
    SetpointForwarding ignoring{ false, DeviceState::FAULT };
    EXPECT_FALSE(ignoring.on_state(state));
    EXPECT_FALSE(ignoring.forwarding);
    SetpointForwarding forwarding{ true, DeviceState::OPERATION_ENABLED };
    EXPECT_FALSE(forwarding.on_state(state));
    EXPECT_TRUE(forwarding.forwarding);
  }
}

/// A reset_fault leaves the joint ignoring until an enable.
TEST(SetpointForwarding, AResetFaultDoesNotResumeAfterAFault)
{
  SetpointForwarding joint;
  joint.on_state(DeviceState::FAULT);
  joint.on_action(DriveAction::RESET_FAULT);
  joint.on_state(DeviceState::SWITCH_ON_DISABLED);
  EXPECT_FALSE(joint.forwarding);
  joint.on_action(DriveAction::ENABLE);
  EXPECT_TRUE(joint.forwarding);
}

}  // namespace
