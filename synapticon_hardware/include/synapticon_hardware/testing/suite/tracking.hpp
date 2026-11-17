// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Drive action suite: Link health, position tracking and the effort path.

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
// Link health, position tracking and the effort path
// ---------------------------------------------------------------------------

/// The link health reports the drive online in OP and the link up once the bus streams. Other slaves on the bus may be
/// in another state, so the bus-wide states only need to include OP.
TEST_F(SingleDrive, LinkHealthReportsTheDriveOnlineInOp)
{
  ASSERT_TRUE(start({}));
  tick(cycles(0.02));
  synapticon_hardware::LinkHealth health;
  drive_->link_health(health);
  ASSERT_EQ(health.drives.size(), 1u);
  EXPECT_EQ(health.drives[0].al_state, synapticon_hardware::al_state::OP);
  EXPECT_TRUE(health.drives[0].online);
  EXPECT_TRUE(health.bus.link_up);
  EXPECT_GE(health.bus.slaves_responding, 1u);
  EXPECT_TRUE(health.bus.al_states & synapticon_hardware::al_state::OP);
  EXPECT_EQ(health.bus.domain_wc_state, synapticon_hardware::wc_state::COMPLETE);
}

/// The readiness query follows the lifecycle: connected after bring-up, streaming after start,
/// enabled between enable() and disable(), and ready only while all three hold.
TEST_F(SingleDrive, TheReadinessQueryFollowsTheLifecycle)
{
  ASSERT_TRUE(start({}));
  EXPECT_TRUE(drive_->is_connected());
  EXPECT_TRUE(drive_->is_streaming());
  EXPECT_FALSE(drive_->is_enabled());
  EXPECT_FALSE(drive_->is_ready());
  hold_here();
  ASSERT_TRUE(drive_->write({ setpoint_ }));
  ASSERT_TRUE(drive_->enable());
  EXPECT_TRUE(drive_->is_enabled());
  EXPECT_TRUE(drive_->is_ready());
  ASSERT_TRUE(drive_->disable());
  EXPECT_FALSE(drive_->is_enabled());
  EXPECT_TRUE(drive_->is_streaming());
  ASSERT_TRUE(drive_->stop());
  EXPECT_FALSE(drive_->is_streaming());
  EXPECT_TRUE(drive_->is_connected());
  drive_->disconnect();
  EXPECT_FALSE(drive_->is_connected());
  drive_.reset();
  bench().released();
}

/// CSP: a small streamed move is tracked, and the joint settles on the final target. At constant speed the measured
/// velocity matches the streamed one, so the drive scales velocities with the gear ratio as the HAL assumes.
TEST_F(SingleDrive, CspTracksASmallMove)
{
  ASSERT_TRUE(start({}));
  ASSERT_NO_FATAL_FAILURE(energize());
  limit_travel(0.8);
  double max_error = 0.0;
  const double v = 0.5 * sign_;
  double sum = 0.0;
  int summed = 0;
  for (int i = 0; i < cycles(0.5); ++i)
  {
    setpoint_.velocity = v * std::min(1.0, (i + 1) / (0.2 * BENCH_RATE_HZ));
    setpoint_.position += setpoint_.velocity * DT;
    tick();
    max_error = std::max(max_error, std::abs(reading_.position - setpoint_.position));
    if (i >= cycles(0.3))  // 0.1 s after the ramp, at constant speed
    {
      sum += reading_.velocity;
      ++summed;
    }
  }
  setpoint_.velocity = 0.0;
  tick(cycles(0.5));
  EXPECT_LT(max_error, 0.1) << "the drive lagged the streamed target";
  EXPECT_NEAR(reading_.position, setpoint_.position, 0.02) << "the joint did not settle on the target";
  EXPECT_NEAR(sum / summed, v, 0.15 * std::abs(v) + bench().velocity_step(0))
      << "the measured velocity is not the streamed one: is the gear ratio applied to velocities?";
}

/// CSV: a small streamed velocity is followed, and the joint stops once the target returns to zero.
TEST_F(SingleDrive, CsvTracksASmallMove)
{
  ASSERT_TRUE(start({ .mode = CSV_MODE }));
  ASSERT_NO_FATAL_FAILURE(energize());
  limit_travel(0.8);
  const double v = 0.5 * sign_;
  double sum = 0.0;
  int summed = 0;
  for (int i = 0; i < cycles(0.6); ++i)
  {
    setpoint_.velocity = v * std::min(1.0, (i + 1) / (0.2 * BENCH_RATE_HZ));
    tick();
    if (i >= cycles(0.3))
    {
      sum += reading_.velocity;
      ++summed;
    }
  }
  setpoint_.velocity = 0.0;
  tick(cycles(0.5));
  // A drive with a coarse velocity unit gets the target cut to a whole unit.
  EXPECT_NEAR(sum / summed, v, 0.1 + bench().velocity_step(0)) << "the drive did not follow the streamed velocity";
  EXPECT_LT(joint_speed(), 0.05) << "the joint did not stop";
}

/// CSV: a quick stop from motion brakes the joint, and it stays stopped while velocity targets keep coming.
TEST_F(SingleDrive, CsvQuickStopFromMotionStops)
{
  ASSERT_TRUE(start({ .mode = CSV_MODE }));
  ASSERT_NO_FATAL_FAILURE(energize());
  limit_travel(0.6);
  const double v = 0.5 * sign_;
  for (int i = 0; i < cycles(0.4); ++i)
  {
    setpoint_.velocity = v * std::min(1.0, (i + 1) / (0.2 * BENCH_RATE_HZ));
    tick();
  }
  const double p_request = reading_.position;
  act(DriveAction::QUICK_STOP);
  tick(cycles(0.3));
  const double p_stopped = reading_.position;
  tick(cycles(1.0));
  EXPECT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE);
  EXPECT_LT(std::abs(p_stopped - p_request), 0.2) << "the joint ran on after the quick stop";
  EXPECT_NEAR(reading_.position, p_stopped, 0.02) << "the joint did not stay where it stopped";
}

/// Impedance with both gains at zero: a small effort command is the joint torque demand.
TEST_F(SingleDrive, ImpedanceEffortWithZeroGainsIsTheJointTorqueDemand)
{
  if (!start({ .mode = IMPEDANCE_MODE }))
  {
    SKIP_OR_FAIL("impedance bring-up failed, most likely no torque sensor is configured");
  }
  ASSERT_NO_FATAL_FAILURE(energize());
  setpoint_.stiffness = 0.0;
  setpoint_.damping = 0.0;
  tick(cycles(0.05));
  EXPECT_NEAR(reading_.joint_torque_demand, 0.0, 0.02) << "no command, no demand";

  // Small enough for friction to hold a free joint, and the watchdog covers the case it does not.
  limit_travel(0.5);
  const double effort = 0.05 * sign_;
  setpoint_.effort = effort;
  double sum = 0.0;
  const int samples = cycles(0.2);
  for (int i = 0; i < samples; ++i)
  {
    tick();
    sum += reading_.joint_torque_demand;
  }
  setpoint_.effort = 0.0;
  tick(cycles(0.05));
  EXPECT_NEAR(sum / samples, effort, 0.02) << "the effort did not reach the joint torque demand";
}

}  // namespace synapticon_hardware::testing
