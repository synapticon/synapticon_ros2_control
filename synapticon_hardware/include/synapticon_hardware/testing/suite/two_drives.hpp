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
  ASSERT_NO_FATAL_FAILURE(sample());
  EXPECT_EQ(drive_state(0), DeviceState::OPERATION_ENABLED);
  EXPECT_EQ(drive_state(1), DeviceState::OPERATION_ENABLED);
  ASSERT_TRUE(drive_->disable());
  ASSERT_NO_FATAL_FAILURE(sample());
  EXPECT_EQ(drive_state(0), DeviceState::SWITCH_ON_DISABLED);
  EXPECT_EQ(drive_state(1), DeviceState::SWITCH_ON_DISABLED);
}

/// A quick stop of joint_1 leaves joint_2 enabled and following its motion.
TEST_F(DrivePair, AQuickStopOfOneLeavesTheOtherEnabledAndMoving)
{
  ASSERT_TRUE(start());
  ASSERT_NO_FATAL_FAILURE(energize_both());
  ASSERT_NO_FATAL_FAILURE(set_off(1, 0.3));
  const double p_before = readings_[1].position;
  ASSERT_NO_FATAL_FAILURE(act({ 0 }, DriveAction::QUICK_STOP));
  ASSERT_NO_FATAL_FAILURE(reach([this] { return drive_state(0) == DeviceState::QUICK_STOP_ACTIVE; }, 0.05,
                                "joint_1 did not reach Quick Stop Active"));
  ASSERT_NO_FATAL_FAILURE(tick(cycles(1.0)));
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
  ASSERT_NO_FATAL_FAILURE(energize_both());
  ASSERT_NO_FATAL_FAILURE(act({ 0, 1 }, DriveAction::QUICK_STOP));  // both in the same cycle
  ASSERT_NO_FATAL_FAILURE(reach(
      [this] {
        return drive_state(0) == DeviceState::QUICK_STOP_ACTIVE && drive_state(1) == DeviceState::QUICK_STOP_ACTIVE;
      },
      0.05, "a quick stop for both did not reach both"));
  ASSERT_NO_FATAL_FAILURE(act({ 1 }, DriveAction::DISABLE_VOLTAGE));
  ASSERT_NO_FATAL_FAILURE(reach([this] { return drive_state(1) == DeviceState::SWITCH_ON_DISABLED; },
                                0.05 + bench().disable_delay(1), "joint_2 did not reach Switch On Disabled"));
  ASSERT_NO_FATAL_FAILURE(tick(cycles(0.5)));
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
  ASSERT_NO_FATAL_FAILURE(energize_both());
  ASSERT_NO_FATAL_FAILURE(set_off(1, 0.3));
  setpoints_[0].position += 0.02;  // a jump joint_1 cannot follow within the window
  ASSERT_NO_FATAL_FAILURE(
      reach([this] { return drive_state(0) == DeviceState::FAULT; }, 0.5, "the step did not fault joint_1"));
  ASSERT_NO_FATAL_FAILURE(tick(cycles(0.5)));
  EXPECT_EQ(drive_state(1), DeviceState::OPERATION_ENABLED) << "the fault spread to the other drive";

  ASSERT_NO_FATAL_FAILURE(act({ 0 }, DriveAction::QUICK_STOP));
  ASSERT_NO_FATAL_FAILURE(tick(2));
  EXPECT_EQ(drive_->active_drive_action(0), DriveAction::NONE) << "should complete at once";
  setpoints_[0].position = readings_[0].position;
  ASSERT_NO_FATAL_FAILURE(tick(cycles(0.02)));
  ASSERT_NO_FATAL_FAILURE(act({ 0 }, DriveAction::RESET_FAULT));
  ASSERT_NO_FATAL_FAILURE(reach([this] { return drive_state(0) == DeviceState::SWITCH_ON_DISABLED; }, 1.0,
                                "joint_1 did not reach Switch On Disabled"));
  EXPECT_EQ(drive_state(1), DeviceState::OPERATION_ENABLED);
}

}  // namespace synapticon_hardware::testing
