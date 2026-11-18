// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Drive action suite: Quick stop: availability, the ramp, the hold and the two option codes.

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
// Bring-up
// ---------------------------------------------------------------------------

/// Quick stop counts as available for 0x605A = 6 or 2, and not for any other option code.
TEST_F(SingleDrive, QuickStopAvailabilityFollowsTheOptionCode)
{
  ASSERT_TRUE(start({}));
  EXPECT_TRUE(drive_->quick_stop_available(0));
  release_drives();

  ASSERT_TRUE(start({ .quick_stop_option_code = 2 }));
  EXPECT_TRUE(drive_->quick_stop_available(0));
  release_drives();

  ASSERT_TRUE(start({ .quick_stop_option_code = 0 }));
  EXPECT_FALSE(drive_->quick_stop_available(0));
}

// ---------------------------------------------------------------------------
// Quick stop
// ---------------------------------------------------------------------------

/// A quick stop reaches Quick Stop Active and stays there, holding position.
TEST_F(SingleDrive, QuickStopHoldsAStandingDrive)
{
  ASSERT_TRUE(start({}));
  ASSERT_NO_FATAL_FAILURE(energize());
  const double p0 = reading_.position;
  limit_travel(0.05);
  ASSERT_NO_FATAL_FAILURE(act(DriveAction::QUICK_STOP));
  ASSERT_NO_FATAL_FAILURE(reach([this] { return drive_state() == DeviceState::QUICK_STOP_ACTIVE; }, 0.05,
                                "did not reach Quick Stop Active"));
  ASSERT_NO_FATAL_FAILURE(tick(cycles(2.0)));
  EXPECT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE) << "must stay, not drop to Switch On Disabled";
  EXPECT_NEAR(reading_.position, p0, 0.01);
}

/// A quick stop from motion ramps down at the rate 0x6085 sets.
TEST_F(SingleDrive, QuickStopRampsAtTheConfiguredDeceleration)
{
  // The ramp starts from the drive's velocity demand at the request, which varies with frame
  // timing on a host without real time, and can be zero. Retry until a ramp is seen.
  ASSERT_TRUE(start({ .quick_stop_deceleration = 2.0 }));
  double slope = QUIET_NAN;
  for (int attempt = 0; attempt < 3 && std::isnan(slope); ++attempt)
  {
    ASSERT_NO_FATAL_FAILURE(energize());
    limit_travel(0.6);
    ASSERT_NO_FATAL_FAILURE(run_at(0.8, 0.3));
    limit_travel(1.2);
    ASSERT_NO_FATAL_FAILURE(act(DriveAction::QUICK_STOP));
    const Trace samples = trace(3.0, at_rest);
    ASSERT_FALSE(HasFatalFailure());
    ASSERT_TRUE(at_rest(samples)) << "did not stop";
    EXPECT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE);
    slope = fitted_deceleration(samples);
  }
  ASSERT_FALSE(std::isnan(slope)) << "the drive stopped without a ramp in three attempts";
  EXPECT_NEAR(slope, 2.0, 0.5);
}

/// After a quick stop the joint stays put although the controller streams on: the drive ignores its stale target.
TEST_F(SingleDrive, QuickStopHoldsWhileTheMotionControllerKeepsStreaming)
{
  // The ramp starts from the drive's velocity demand, which varies on a host without real time, so
  // retry until the ramp carried the joint clearly past the last forwarded target.
  ASSERT_TRUE(start({ .quick_stop_deceleration = 2.0 }));
  bool tested = false;
  for (int attempt = 0; attempt < 3 && !tested; ++attempt)
  {
    ASSERT_NO_FATAL_FAILURE(energize());
    limit_travel(0.6);
    ASSERT_NO_FATAL_FAILURE(run_at(0.8, 0.3));
    const double last_forwarded = setpoint_.position;  // the request's own cycle is not forwarded
    limit_travel(1.2);
    ASSERT_NO_FATAL_FAILURE(act(DriveAction::QUICK_STOP));
    const Trace samples = trace(3.0, at_rest);
    ASSERT_FALSE(HasFatalFailure());
    ASSERT_TRUE(at_rest(samples)) << "did not stop";
    const double p_stopped = reading_.position;
    if (std::abs(p_stopped - last_forwarded) < 0.1)
    {
      continue;
    }  // no real ramp: not conclusive
    ASSERT_NO_FATAL_FAILURE(carry_on(2.0));  // the controller does not know about the quick stop and carries on
    EXPECT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE);
    EXPECT_NEAR(reading_.position, p_stopped, 0.03) << "the drive moved back toward its last target";
    EXPECT_GT(std::abs(setpoint_.position - reading_.position), 0.5) << "the stream did not move away";
    tested = true;
  }
  EXPECT_TRUE(tested) << "the drive stopped without a ramp in three attempts";
}

/// A joint that ramped past the commanded target does not return to it.
TEST_F(SingleDrive, QuickStopDoesNotSnapBackToAHeldTarget)
{
  ASSERT_NO_FATAL_FAILURE(expect_no_return_after_quick_stop());
}

/// Enabling from Quick Stop Active moves nothing.
TEST_F(SingleDrive, ReEnablingFromQuickStopActiveDoesNotJump)
{
  ASSERT_TRUE(start({}));
  ASSERT_NO_FATAL_FAILURE(energize());
  ASSERT_NO_FATAL_FAILURE(act(DriveAction::QUICK_STOP));
  ASSERT_NO_FATAL_FAILURE(tick(cycles(0.2)));
  ASSERT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE);
  const double p = reading_.position;
  limit_travel(0.05);
  ASSERT_NO_FATAL_FAILURE(energize());
  ASSERT_NO_FATAL_FAILURE(tick(cycles(0.5)));
  EXPECT_NEAR(reading_.position, p, 0.005);
}

/// With an unsupported 0x605A (here 0), a quick stop performs a disable instead.
TEST_F(SingleDrive, QuickStopFallsBackToDisableForAnUnsupportedOptionCode)
{
  ASSERT_TRUE(start({ .quick_stop_option_code = 0 }));
  ASSERT_NO_FATAL_FAILURE(energize());
  ASSERT_NO_FATAL_FAILURE(act(DriveAction::QUICK_STOP));
  EXPECT_EQ(drive_->active_drive_action(0), DriveAction::DISABLE);
  ASSERT_NO_FATAL_FAILURE(reach([this] { return drive_state() == DeviceState::SWITCH_ON_DISABLED; }, 1.0,
                                "did not reach Switch On Disabled"));
}

/// With 0x605A = 2, a quick stop from motion ramps the joint down, then de-energises the drive.
TEST_F(SingleDrive, QuickStopWithOptionCodeTwoStopsThenDeEnergises)
{
  ASSERT_TRUE(start({ .quick_stop_option_code = 2, .quick_stop_deceleration = 2.0 }));
  ASSERT_NO_FATAL_FAILURE(energize());
  limit_travel(0.5);
  ASSERT_NO_FATAL_FAILURE(run_at(0.5, 0.3));
  const double p_request = reading_.position;
  limit_travel(0.6);
  ASSERT_NO_FATAL_FAILURE(act(DriveAction::QUICK_STOP));
  EXPECT_NE(drive_->active_drive_action(0), DriveAction::DISABLE) << "should not fall back";
  bool passed_quick_stop = false;
  const double t = tick_until(
      [this, &passed_quick_stop] {
        passed_quick_stop |= drive_state() == DeviceState::QUICK_STOP_ACTIVE;
        return drive_state() == DeviceState::SWITCH_ON_DISABLED;
      },
      2.0);
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_GE(t, 0.0) << "did not de-energise after the stop";
  EXPECT_TRUE(passed_quick_stop) << "never reported Quick Stop Active";
  ASSERT_NO_FATAL_FAILURE(tick(cycles(0.3)));
  // 0.5 rad/s at 2 rad/s^2 ramps over about 0.06 rad. Far less means no ramp, far more a coast.
  EXPECT_LT(std::abs(reading_.position - p_request), 0.2);
}

}  // namespace synapticon_hardware::testing
