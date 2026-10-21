// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Drive action suite: Two drives: an action on one leaves the other alone. Skipped without a second drive.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <gtest/gtest.h>

#include "synapticon_hardware/testing/hal_bench.hpp"

namespace synapticon_hardware::testing
{

using cia402::DeviceState;

// ---------------------------------------------------------------------------
// Two drives: an action on one leaves the other alone
// ---------------------------------------------------------------------------

/// The lifecycle enable() and disable() walk both drives.
TEST_F(DrivePair, BothComeUpAndGoDownTogether)
{
  ASSERT_TRUE(start());
  ASSERT_TRUE(drive_->enable());
  sample();
  EXPECT_EQ(drive_state(0), DeviceState::OPERATION_ENABLED);
  EXPECT_EQ(drive_state(1), DeviceState::OPERATION_ENABLED);
  ASSERT_TRUE(drive_->disable());
  sample();
  EXPECT_EQ(drive_state(0), DeviceState::SWITCH_ON_DISABLED);
  EXPECT_EQ(drive_state(1), DeviceState::SWITCH_ON_DISABLED);
}

/// A quick stop of joint_1 leaves joint_2 enabled and following its motion.
TEST_F(DrivePair, AQuickStopOfOneLeavesTheOtherEnabledAndMoving)
{
  ASSERT_TRUE(start());
  energize_both();
  set_off(1, 0.3);
  const double p_before = readings_[1].position;
  act({ 0 }, DriveAction::QUICK_STOP);
  ASSERT_GE(tick_until([this] { return drive_state(0) == DeviceState::QUICK_STOP_ACTIVE; }, 0.05), 0.0);
  tick(cycles(1.0));
  EXPECT_EQ(drive_state(0), DeviceState::QUICK_STOP_ACTIVE);
  EXPECT_EQ(drive_state(1), DeviceState::OPERATION_ENABLED) << "the other drive was touched";
  EXPECT_EQ(drive_->active_drive_action(1), DriveAction::NONE);
  EXPECT_NEAR(std::abs(readings_[1].position - p_before), 0.3, 0.05) << "the other drive stopped moving";
  EXPECT_FALSE(drive_->forwarding_setpoints(0));
  EXPECT_TRUE(drive_->forwarding_setpoints(1)) << "the other drive lost its setpoints";
}

/// A quick stop of both in one cycle reaches both, then disable_voltage of joint_2 leaves joint_1 quick stopped.
TEST_F(DrivePair, ADisableVoltageOfOneLeavesTheOtherInQuickStop)
{
  ASSERT_TRUE(start());
  energize_both();
  act({ 0, 1 }, DriveAction::QUICK_STOP);  // both in the same cycle
  ASSERT_GE(tick_until(
                [this] {
                  return drive_state(0) == DeviceState::QUICK_STOP_ACTIVE &&
                         drive_state(1) == DeviceState::QUICK_STOP_ACTIVE;
                },
                0.05),
            0.0)
      << "a quick stop for both did not reach both";
  act({ 1 }, DriveAction::DISABLE_VOLTAGE);
  EXPECT_GE(tick_until([this] { return drive_state(1) == DeviceState::SWITCH_ON_DISABLED; },
                       0.05 + bench().disable_delay(1)),
            0.0);
  tick(cycles(0.5));
  EXPECT_EQ(drive_state(0), DeviceState::QUICK_STOP_ACTIVE) << "the other drive was touched";
  EXPECT_EQ(drive_state(1), DeviceState::SWITCH_ON_DISABLED);
}

/// A fault on joint_1, and its reset, leave joint_2 enabled.
TEST_F(DrivePair, AFaultOnOneLeavesTheOtherRunning)
{
  // Only joint_1 faults on a following error. The watchdog is wider for the drive's own turn after
  // the Fault Reset.
  ASSERT_TRUE(start({ .following_error = FollowingError::FAULT }));
  max_lag_ = 1.2;
  energize_both();
  set_off(1, 0.3);
  setpoints_[0].position += 0.02;  // a jump joint_1 cannot follow within the window
  ASSERT_GE(tick_until([this] { return drive_state(0) == DeviceState::FAULT; }, 0.5), 0.0)
      << "the step did not fault joint_1";
  tick(cycles(0.5));
  EXPECT_EQ(drive_state(1), DeviceState::OPERATION_ENABLED) << "the fault spread to the other drive";

  act({ 0 }, DriveAction::QUICK_STOP);
  tick(2);
  EXPECT_EQ(drive_->active_drive_action(0), DriveAction::NONE) << "should complete at once";
  setpoints_[0].position = readings_[0].position;
  tick(cycles(0.02));
  act({ 0 }, DriveAction::RESET_FAULT);
  EXPECT_GE(tick_until([this] { return drive_state(0) == DeviceState::SWITCH_ON_DISABLED; }, 1.0), 0.0);
  EXPECT_EQ(drive_state(1), DeviceState::OPERATION_ENABLED);
}

}  // namespace synapticon_hardware::testing
