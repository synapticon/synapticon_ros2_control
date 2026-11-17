// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Drive action suite: The ramped and direct descents to Switch On Disabled, and requests that complete at once.

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
// The descents
// ---------------------------------------------------------------------------

/// A disable from motion cuts the voltage only once the joint has stopped, and it does not coast.
TEST_F(SingleDrive, TheRampedDisableCutsOnlyOnceStopped)
{
  ASSERT_TRUE(start({ .quick_stop_deceleration = 2.0 }));
  ASSERT_NO_FATAL_FAILURE(energize());
  limit_travel(0.6);
  run_at(0.8, 0.3);
  limit_travel(1.2);
  act(DriveAction::DISABLE);
  const Trace samples = trace(4.0, [](const Trace& s) { return s.back().state == DeviceState::SWITCH_ON_DISABLED; });
  ASSERT_EQ(drive_state(), DeviceState::SWITCH_ON_DISABLED) << "never reached Switch On Disabled";
  // The cut was commanded one cycle before the drive reported it. By then the joint had been
  // slower than the stop threshold (0.1 rad/s, plus velocity noise) for the stop time (50 ms).
  ASSERT_GE(samples.size(), 2u);
  EXPECT_TRUE(stayed_below(samples, 0.04, 0.15, samples.size() - 2))
      << "the voltage was cut while the joint was still moving";
  const double p_cut = reading_.position;
  tick(cycles(0.3));
  EXPECT_LT(std::abs(reading_.position - p_cut), 0.02) << "the joint coasted after the cut";
}

/// The same for the blocking lifecycle disable().
TEST_F(SingleDrive, TheLifecycleDisableCutsOnlyOnceStopped)
{
  ASSERT_TRUE(start({ .quick_stop_deceleration = 2.0 }));
  ASSERT_NO_FATAL_FAILURE(energize());
  limit_travel(0.6);
  run_at(0.8, 0.3);
  EXPECT_TRUE(drive_->disable());
  sample();
  EXPECT_EQ(drive_state(), DeviceState::SWITCH_ON_DISABLED);
  EXPECT_LT(joint_speed(), 0.15) << "the voltage was cut while the joint was still moving";
}

/// disable() right after a disable_voltage cuts at once instead of ramping.
TEST_F(SingleDrive, TheLifecycleDisableKeepsADisableVoltageInProgress)
{
  // A disable_voltage whose frame is followed within the same cycle by the lifecycle disable():
  // the drive only sees the later frame, so disable() must keep the direct route, not ramp.
  ASSERT_TRUE(start({ .quick_stop_deceleration = 2.0 }));
  ASSERT_NO_FATAL_FAILURE(energize());
  limit_travel(0.5);
  run_at(0.5, 0.3);
  limit_travel(0.5);
  setpoint_.drive_action = DriveAction::DISABLE_VOLTAGE;
  setpoint_.position += setpoint_.velocity * DT;
  ASSERT_TRUE(drive_->write({ setpoint_ }));
  const auto start = Clock::now();
  EXPECT_TRUE(drive_->disable());
  const double took = std::chrono::duration<double>(Clock::now() - start).count();
  sample();
  EXPECT_EQ(drive_state(), DeviceState::SWITCH_ON_DISABLED);
  // A ramp from 0.5 rad/s at 2 rad/s^2 would take 0.25 s, plus the time to count as stopped.
  EXPECT_LT(took, 0.1 + bench().disable_delay(0)) << "disable() ramped instead of keeping the disable_voltage";
}

/// disable_voltage from motion goes straight to Switch On Disabled, without a ramp.
TEST_F(SingleDrive, DisableVoltageCutsAtOnce)
{
  ASSERT_TRUE(start({ .quick_stop_deceleration = 2.0 }));
  ASSERT_NO_FATAL_FAILURE(energize());
  limit_travel(0.5);
  run_at(0.5, 0.3);
  limit_travel(0.3);
  act(DriveAction::DISABLE_VOLTAGE);
  bool passed_quick_stop = false;
  const double t = tick_until(
      [this, &passed_quick_stop] {
        passed_quick_stop |= drive_state() == DeviceState::QUICK_STOP_ACTIVE;
        return drive_state() == DeviceState::SWITCH_ON_DISABLED;
      },
      0.1 + bench().disable_delay(0));
  EXPECT_GE(t, 0.0);
  EXPECT_LT(t, 0.02 + bench().disable_delay(0));
  EXPECT_FALSE(passed_quick_stop) << "the direct route must not ramp";
}

/// disable_voltage from Quick Stop Active goes straight to Switch On Disabled.
TEST_F(SingleDrive, DisableVoltageFromQuickStopActiveIsDirect)
{
  ASSERT_TRUE(start({}));
  ASSERT_NO_FATAL_FAILURE(energize());
  act(DriveAction::QUICK_STOP);
  tick(cycles(0.2));
  ASSERT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE);
  act(DriveAction::DISABLE_VOLTAGE);
  const double t =
      tick_until([this] { return drive_state() == DeviceState::SWITCH_ON_DISABLED; }, 0.1 + bench().disable_delay(0));
  EXPECT_GE(t, 0.0);
  EXPECT_LT(t, 0.02 + bench().disable_delay(0));
}

// ---------------------------------------------------------------------------
// Requests that complete at once or are refused
// ---------------------------------------------------------------------------

/// A quick stop of a drive that is not energised completes at once, without waiting for a timeout.
TEST_F(SingleDrive, AQuickStopOfADeEnergisedDriveCompletesAtOnce)
{
  ASSERT_TRUE(start({}));
  tick(cycles(0.01));
  ASSERT_EQ(drive_state(), DeviceState::SWITCH_ON_DISABLED);
  act(DriveAction::QUICK_STOP);
  tick(2);
  EXPECT_EQ(drive_->active_drive_action(0), DriveAction::NONE);
  EXPECT_EQ(drive_state(), DeviceState::SWITCH_ON_DISABLED);
}

/// A quick stop of a faulted drive completes at once, and reset_fault then recovers the drive.
TEST_F(SingleDrive, AQuickStopOfAFaultedDriveCompletesAndAResetRecovers)
{
  // The drive's own following error reaction quick-stops on 0x6085, so make that ramp short. After
  // the Fault Reset the drive may turn the motor about 0.6 rad on its own while still reporting
  // Fault (about one electrical revolution), hence the wider watchdog.
  ASSERT_TRUE(start({ .quick_stop_deceleration = 1000.0, .following_error = FollowingError::FAULT }));
  ASSERT_NO_FATAL_FAILURE(energize());
  limit_travel(1.2);
  // A jump the drive cannot follow within the window: a following error. The position loop answers
  // it with a velocity spike before the fault reaction stops it, so the jump is kept small.
  setpoint_.position += 0.02 * sign_;
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::FAULT; }, 0.5), 0.0)
      << "the step did not fault the drive";

  act(DriveAction::QUICK_STOP);
  tick(2);
  EXPECT_EQ(drive_->active_drive_action(0), DriveAction::NONE) << "should complete at once";
  EXPECT_EQ(drive_state(), DeviceState::FAULT) << "a quick stop does not reset a fault";

  hold_here();  // the drive does not leave Fault while the fault condition is commanded
  tick(cycles(0.02));
  act(DriveAction::RESET_FAULT);
  EXPECT_GE(tick_until([this] { return drive_state() == DeviceState::SWITCH_ON_DISABLED; }, 1.0), 0.0);
}

}  // namespace synapticon_hardware::testing
