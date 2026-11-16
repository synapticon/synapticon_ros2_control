// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Drive action suite: Impedance mode: the quick stop hold, the ramped stop and the descents.

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
// Impedance mode (skipped where the drive has no torque sensor configured)
// ---------------------------------------------------------------------------

/// Impedance mode, standing still: a quick stop holds, and disable_voltage then cuts at once.
TEST_F(SingleDrive, ImpedanceQuickStopHoldsAndDisableVoltageCuts)
{
  if (!start({ .mode = IMPEDANCE_MODE }))
  {
    SKIP_OR_FAIL("impedance bring-up failed, most likely no torque sensor is configured");
  }
  ASSERT_NO_FATAL_FAILURE(energize());
  const double p0 = reading_.position;
  limit_travel(0.05);
  act(DriveAction::QUICK_STOP);
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::QUICK_STOP_ACTIVE; }, 0.05), 0.0);
  tick(cycles(1.0));
  EXPECT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE);
  EXPECT_NEAR(reading_.position, p0, 0.01);
  act(DriveAction::DISABLE_VOLTAGE);
  EXPECT_GE(tick_until([this] { return drive_state() == DeviceState::SWITCH_ON_DISABLED; },
                       0.1 + bench().disable_delay(0)),
            0.0);
}

/// Impedance mode: a quick stop from motion stops the joint and keeps it there while new targets keep coming.
TEST_F(SingleDrive, ImpedanceQuickStopFromMotionStops)
{
  // The drive does not ramp in impedance mode, so the stop comes from the hold.
  if (!start({ .mode = IMPEDANCE_MODE }))
  {
    SKIP_OR_FAIL("impedance bring-up failed, most likely no torque sensor is configured");
  }
  setpoint_.stiffness = test_gains(0).stiffness;
  setpoint_.damping = test_gains(0).damping;
  ASSERT_NO_FATAL_FAILURE(energize());
  run_at(0.5, 0.8);
  const double p_request = reading_.position;
  limit_travel(0.6);
  act(DriveAction::QUICK_STOP);
  carry_on(0.3);
  const double p_stopped = reading_.position;
  carry_on(1.0);
  EXPECT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE);
  EXPECT_LT(std::abs(p_stopped - p_request), 0.2) << "the joint ran on after the quick stop";
  EXPECT_NEAR(reading_.position, p_stopped, 0.02) << "the joint did not stay where it stopped";
}

/// Impedance mode without the hold: the joint stops at the last forwarded target while the controller streams on.
TEST_F(SingleDrive, ImpedanceWithoutTheHoldStopsAtTheLastForwardedTarget)
{
  if (!start({ .mode = IMPEDANCE_MODE, .quick_stop_hold = false }))
  {
    SKIP_OR_FAIL("impedance bring-up failed, most likely no torque sensor is configured");
  }
  setpoint_.stiffness = test_gains(0).stiffness;
  setpoint_.damping = test_gains(0).damping;
  ASSERT_NO_FATAL_FAILURE(energize());
  run_at(0.5, 0.8);
  const double last_forwarded = setpoint_.position;  // the request's own cycle is not forwarded
  limit_travel(0.6);
  act(DriveAction::QUICK_STOP);
  carry_on(1.5);
  EXPECT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE);
  EXPECT_GT(std::abs(setpoint_.position - last_forwarded), 0.5) << "the controller did not run on";
  EXPECT_NEAR(reading_.position, last_forwarded, 0.05) << "the joint did not stop at the last target";
}

/// Impedance mode: a disable from motion stops the joint before the voltage is cut.
TEST_F(SingleDrive, ImpedanceRampedDisableFromMotionStops)
{
  if (!start({ .mode = IMPEDANCE_MODE }))
  {
    SKIP_OR_FAIL("impedance bring-up failed, most likely no torque sensor is configured");
  }
  setpoint_.stiffness = test_gains(0).stiffness;
  setpoint_.damping = test_gains(0).damping;
  ASSERT_NO_FATAL_FAILURE(energize());
  run_at(0.5, 0.8);
  const double p_request = reading_.position;
  limit_travel(0.6);
  act(DriveAction::DISABLE);
  bool reached = false;
  for (int i = 0; i < cycles(1.0) && !reached; ++i)
  {
    setpoint_.position += setpoint_.velocity * DT;
    tick();
    reached = drive_state() == DeviceState::SWITCH_ON_DISABLED;
  }
  EXPECT_TRUE(reached) << "did not reach Switch On Disabled within 1 s";
  EXPECT_LT(std::abs(reading_.position - p_request), 0.2) << "the joint ran on during the descent";
}

/// Impedance mode with 0x605A = 2: a quick stop from motion stops the joint, then the drive de-energises.
TEST_F(SingleDrive, ImpedanceQuickStopWithOptionCodeTwoStopsThenDeEnergises)
{
  if (!start({ .mode = IMPEDANCE_MODE, .quick_stop_option_code = 2 }))
  {
    SKIP_OR_FAIL("impedance bring-up failed, most likely no torque sensor is configured");
  }
  setpoint_.stiffness = test_gains(0).stiffness;
  setpoint_.damping = test_gains(0).damping;
  ASSERT_NO_FATAL_FAILURE(energize());
  run_at(0.5, 0.8);
  const double p_request = reading_.position;
  limit_travel(0.6);
  act(DriveAction::QUICK_STOP);
  bool reached = false;
  for (int i = 0; i < cycles(1.5) && !reached; ++i)
  {
    setpoint_.position += setpoint_.velocity * DT;  // the motion controller carries on
    tick();
    reached = drive_state() == DeviceState::SWITCH_ON_DISABLED;
  }
  EXPECT_TRUE(reached) << "did not de-energise within 1.5 s";
  EXPECT_LT(std::abs(reading_.position - p_request), 0.2) << "the joint ran on before it was stopped";
}

}  // namespace synapticon_hardware::testing
