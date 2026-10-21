// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Drive action suite: Following errors, setpoints during a reaction, and fault resets that happen only on request.

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
// The drive's own following error reaction
// ---------------------------------------------------------------------------

/// Measures the travel after the drive's own following error reaction to a step, asserting only a final Fault.
TEST_F(SingleDrive, AFollowingErrorFaultFromAStepIsMeasured)
{
  constexpr double DECEL = 200.0;  // rad/s^2: braking would be 5 to 6 times shorter than coasting
  ASSERT_TRUE(start({ .quick_stop_deceleration = DECEL, .following_error = FollowingError::FAULT }));
  energize();
  limit_travel(6.0);  // a coast cannot be cut short: the drive is already de-energised
  const double p_step = reading_.position;
  setpoint_.position += 0.2 * sign_;  // small, since the joint coasts in Fault after the reaction
  Snapshot reaction{ QUIET_NAN, QUIET_NAN, QUIET_NAN, DeviceState::UNKNOWN };
  Snapshot fault{ QUIET_NAN, QUIET_NAN, QUIET_NAN, DeviceState::UNKNOWN };
  trace(4.0, [&](const Trace& s) {
    const Snapshot& now = s.back();
    if (std::isnan(reaction.t) && (now.state == DeviceState::FAULT_REACTION_ACTIVE || now.state == DeviceState::FAULT))
    {
      reaction = now;
    }
    if (std::isnan(fault.t) && now.state == DeviceState::FAULT)
    {
      fault = now;
    }
    return !std::isnan(fault.t) && now.t > fault.t + 0.3 && at_rest(s);
  });
  ASSERT_FALSE(std::isnan(reaction.t)) << "the step did not fault the drive";
  const double travel = std::abs(reading_.position - reaction.position);
  const double braking = reaction.speed * reaction.speed / (2.0 * DECEL);
  // Coasting assumes friction alone, about 36 rad/s^2 for a free joint.
  const double coasting = reaction.speed * reaction.speed / (2.0 * 36.0);
  std::printf("FOLLOWING ERROR FAULT: reaction seen %.0f ms after the step at %.2f rad/s, Fault %.0f ms "
              "later at %.2f rad/s, travelled %.3f rad after the reaction (braking at %.0f rad/s^2 would be "
              "%.3f, coasting %.3f), %.3f rad in total from the step. Verdict: %s.\n",
              reaction.t * 1e3, reaction.speed, (fault.t - reaction.t) * 1e3, fault.speed, travel, DECEL, braking,
              coasting, std::abs(reading_.position - p_step),
              travel < 2.0 * braking + 0.02 ? "braked on the ramp" :
              fault.speed > reaction.speed  ? "still accelerating in Fault Reaction Active, then coasted" :
                                              "coasted");
  EXPECT_EQ(drive_state(), DeviceState::FAULT);
}

/// A client quick stop on a reported following error (0x2017:1 = 0) stops the joint, which then holds in Quick
/// Stop Active. Measures the delays and the travel for two ramps.
TEST_F(SingleDrive, AClientQuickStopOnAReportedFollowingErrorStopsAndHolds)
{
  struct Variant
  {
    const char* name;
    double decel;
    bool recommand;
  };
  const Variant variants[] = {
    { "200 rad/s^2", 200.0, false },
    { "1000 rad/s^2", 1000.0, false },
  };
  double sign = sign_;
  for (const Variant& variant : variants)
  {
    ASSERT_TRUE(start({ .quick_stop_deceleration = variant.decel, .following_error = FollowingError::REPORT }));
    energize();
    limit_travel(3.0);
    const double p_step = reading_.position;
    setpoint_.position += 1.0 * sign;
    Snapshot reported{ QUIET_NAN, QUIET_NAN, QUIET_NAN, DeviceState::UNKNOWN };
    Snapshot stopping{ QUIET_NAN, QUIET_NAN, QUIET_NAN, DeviceState::UNKNOWN };
    Snapshot after{ QUIET_NAN, QUIET_NAN, QUIET_NAN, DeviceState::UNKNOWN };  // the first state after Quick Stop Active
    uint16_t error_code = 0;
    char error_report[9] = {};
    int cycles_since_report = -1;
    trace(3.0, [&](const Trace& s) {
      const Snapshot& now = s.back();
      setpoint_.drive_action = DriveAction::NONE;
      if (!std::isnan(stopping.t) && std::isnan(after.t) && now.state != DeviceState::QUICK_STOP_ACTIVE)
      {
        after = now;
        error_code = reading_.error_code;
        std::memcpy(error_report, &reading_.error_report, 8);
      }
      if (std::isnan(reported.t) && (reading_.status_word & 0x2000u) != 0u)
      {
        reported = now;
        cycles_since_report = 0;
      }
      else if (cycles_since_report >= 0)
      {
        ++cycles_since_report;
      }
      // Request the quick stop one cycle after the report, as a fault reaction would. The recommand
      // variant first forwards the actual position for one cycle.
      if (cycles_since_report == 0 && variant.recommand)
      {
        setpoint_.position = reading_.position;
      }
      if (cycles_since_report == (variant.recommand ? 1 : 0))
      {
        setpoint_.drive_action = DriveAction::QUICK_STOP;
      }
      if (std::isnan(stopping.t) && now.state == DeviceState::QUICK_STOP_ACTIVE)
      {
        stopping = now;
      }
      return !std::isnan(stopping.t) && now.t > stopping.t + 0.3 && at_rest(s);
    });
    setpoint_.drive_action = DriveAction::NONE;
    ASSERT_FALSE(std::isnan(reported.t)) << variant.name << ": no following error was reported";
    ASSERT_FALSE(std::isnan(stopping.t)) << variant.name << ": no Quick Stop Active";
    const double travel = std::abs(reading_.position - stopping.position);
    const double braking = stopping.speed * stopping.speed / (2.0 * variant.decel);
    std::printf("REPORTED FOLLOWING ERROR + CLIENT QUICK STOP [%s]: reported %.0f ms after the step at "
                "%.2f rad/s, Quick Stop Active %.0f ms later at %.2f rad/s, travelled %.3f rad after that "
                "(v^2/2a would be %.3f), %.3f rad in total from the step, final state %s%s\n",
                variant.name, reported.t * 1e3, reported.speed, (stopping.t - reported.t) * 1e3, stopping.speed, travel,
                braking, std::abs(reading_.position - p_step), cia402::device_state_name(drive_state()),
                std::isnan(after.t) ? "" :
                                      (", left Quick Stop Active after " +
                                       std::to_string(static_cast<int>((after.t - stopping.t) * 1e3)) + " ms for " +
                                       cia402::device_state_name(after.state) + " at " + std::to_string(after.speed) +
                                       " rad/s, error 0x" +
                                       [](uint16_t e) {
                                         char b[8];
                                         std::snprintf(b, sizeof(b), "%04X", e);
                                         return std::string(b);
                                       }(error_code) +
                                       " '" + error_report + "'")
                                          .c_str());
    EXPECT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE) << variant.name << ": must end up holding";
    EXPECT_LT(std::abs(reading_.position - p_step), 2.5) << variant.name << ": ran away";
    const double p_held = reading_.position;
    tick(cycles(0.5));
    EXPECT_NEAR(reading_.position, p_held, 0.02) << variant.name << ": did not hold";
    release_drives();
    sign = -sign;
  }
}

/// With 0x2017:1 = 0 the drive only reports a following error, keeps running, and the report is visible.
TEST_F(SingleDrive, AReportedFollowingErrorKeepsTheDriveRunningAndIsVisible)
{
  ASSERT_TRUE(start({ .following_error = FollowingError::REPORT }));  // the window of the fault tests
  energize();
  limit_travel(1.0);
  const double target = reading_.position + 0.5 * sign_;
  setpoint_.position = target;
  bool bit13 = false;
  uint16_t error_code = 0;
  const Trace samples = trace(2.0, [&](const Trace& s) {
    bit13 |= (reading_.status_word & 0x2000u) != 0u;  // CSP: statusword bit 13, following error
    if (reading_.error_code != 0u)
    {
      error_code = reading_.error_code;
    }
    return s.back().t > 0.3 && at_rest(s);
  });
  std::printf("REPORTED FOLLOWING ERROR: statusword bit 13 %s, error code 0x%04X, state %s, "
              "%.3f rad from the target\n",
              bit13 ? "seen" : "never seen", error_code, cia402::device_state_name(drive_state()),
              std::abs(reading_.position - target));
  EXPECT_EQ(drive_state(), DeviceState::OPERATION_ENABLED) << "report only must not fault";
  EXPECT_NEAR(reading_.position, target, 0.02) << "the drive did not keep running to the target";
  EXPECT_TRUE(bit13 || error_code == 0x8611u) << "the following error was not reported";
}

// ---------------------------------------------------------------------------
// Setpoints under reaction: a stopped or faulted drive gets none until it is enabled again
// ---------------------------------------------------------------------------

/// A quick stop starts ignoring setpoints at once, and an enable resumes them with the one streamed at that moment.
TEST_F(SingleDrive, AQuickStoppedDriveIgnoresSetpointsUntilEnabled)
{
  ASSERT_TRUE(start({ .quick_stop_deceleration = 20.0 }));
  EXPECT_TRUE(drive_->forwarding_setpoints(0));
  energize();
  EXPECT_TRUE(drive_->forwarding_setpoints(0));
  run_at(0.5, 0.3);
  limit_travel(0.5);
  act(DriveAction::QUICK_STOP);
  EXPECT_FALSE(drive_->forwarding_setpoints(0)) << "a stop action must start ignoring at once";
  carry_on(1.0);  // the controller carries on, its target ends 0.5 rad from the joint
  ASSERT_EQ(drive_state(), DeviceState::QUICK_STOP_ACTIVE);
  EXPECT_FALSE(drive_->forwarding_setpoints(0));

  // The recovery re-commands the controller, here to the joint's own position, then enables.
  hold_here();
  act(DriveAction::ENABLE);
  EXPECT_TRUE(drive_->forwarding_setpoints(0)) << "enable must resume forwarding at once";
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::OPERATION_ENABLED; }, 2.0), 0.0);
  const double p_enabled = reading_.position;
  tick(cycles(0.2));
  EXPECT_NEAR(reading_.position, p_enabled, 0.01) << "a stale target reached the drive";

  // What the controller streams now is what the drive follows.
  const double target = p_enabled + 0.05 * sign_;
  setpoint_.position = target;
  tick(cycles(0.5));
  EXPECT_NEAR(reading_.position, target, 0.01) << "the streamed target did not reach the drive";
}

/// The target streamed during the enable walk is the first the drive follows, and nothing moves before the enable.
TEST_F(SingleDrive, TheTargetStreamedDuringTheEnableWalkIsTheFirstTheDriveFollows)
{
  ASSERT_TRUE(start({}));
  energize();
  act(DriveAction::DISABLE);
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::SWITCH_ON_DISABLED; }, 1.0), 0.0);
  tick(cycles(0.1));
  // A target a little away from the joint, streamed from before the enable request on.
  const double target = reading_.position + 0.05 * sign_;
  setpoint_.position = target;
  setpoint_.velocity = 0.0;
  limit_travel(0.3);
  act(DriveAction::ENABLE);
  bool moved_before_enabled = false;
  const double p_walk = reading_.position;
  ASSERT_GE(tick_until(
                [&] {
                  moved_before_enabled |= std::abs(reading_.position - p_walk) > 0.01;
                  return drive_state() == DeviceState::OPERATION_ENABLED;
                },
                2.0),
            0.0)
      << "did not reach Operation Enabled";
  EXPECT_FALSE(moved_before_enabled) << "the joint moved while the drive was not yet enabled";
  tick(cycles(0.5));
  EXPECT_NEAR(reading_.position, target, 0.01) << "the drive did not follow the streamed target";
}

/// A disable_voltage starts ignoring too, and the lifecycle enable() resumes forwarding.
TEST_F(SingleDrive, TheLifecycleEnableResumesForwarding)
{
  ASSERT_TRUE(start({}));
  energize();
  act(DriveAction::DISABLE_VOLTAGE);
  EXPECT_FALSE(drive_->forwarding_setpoints(0));
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::SWITCH_ON_DISABLED; },
                       0.1 + bench().disable_delay(0)),
            0.0);
  hold_here();
  ASSERT_TRUE(drive_->write({ setpoint_ }));
  ASSERT_TRUE(drive_->enable());
  EXPECT_TRUE(drive_->forwarding_setpoints(0));
  ASSERT_TRUE(drive_->disable());
  EXPECT_FALSE(drive_->forwarding_setpoints(0)) << "the lifecycle disable() is a stop action";
}

/// A drive that faults ignores setpoints from the fault through the reset, until it is enabled.
TEST_F(SingleDrive, AFaultedDriveIgnoresSetpointsThroughTheResetUntilEnabled)
{
  ASSERT_TRUE(start({ .quick_stop_deceleration = 1000.0, .following_error = FollowingError::FAULT }));
  energize();
  limit_travel(1.2);
  setpoint_.position += 0.02 * sign_;
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::FAULT; }, 0.5), 0.0)
      << "the step did not fault the drive";
  tick(1);
  EXPECT_FALSE(drive_->forwarding_setpoints(0)) << "a reported Fault must start ignoring";

  hold_here();
  tick(cycles(0.02));
  act(DriveAction::RESET_FAULT);
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::SWITCH_ON_DISABLED; }, 1.0), 0.0);
  EXPECT_FALSE(drive_->forwarding_setpoints(0)) << "a reset is not a decision to run again";

  hold_here();
  act(DriveAction::ENABLE);
  EXPECT_TRUE(drive_->forwarding_setpoints(0));
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::OPERATION_ENABLED; }, 2.0), 0.0);
  EXPECT_TRUE(drive_->forwarding_setpoints(0)) << "staying in Fault during the walk undid the enable";
}

// ---------------------------------------------------------------------------
// A fault is reset only on request: no stop action and no enable resets it
// ---------------------------------------------------------------------------

/// A disable of a faulted drive leaves it in Fault, with its error code: it is de-energised already.
TEST_F(SingleDrive, ADisableLeavesAFaultAlone)
{
  provoke_step_fault(1000.0, 0.02);
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::FAULT; }, 0.5), 0.0);
  const uint16_t error_code = reading_.error_code;
  act(DriveAction::DISABLE);
  tick(cycles(0.3));
  EXPECT_EQ(drive_->active_drive_action(0), DriveAction::NONE) << "should complete at once";
  EXPECT_EQ(drive_state(), DeviceState::FAULT) << "a disable reset the fault";
  EXPECT_EQ(reading_.error_code, error_code) << "the error code was lost";
}

/// The same for disable_voltage.
TEST_F(SingleDrive, ADisableVoltageLeavesAFaultAlone)
{
  provoke_step_fault(1000.0, 0.02);
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::FAULT; }, 0.5), 0.0);
  const uint16_t error_code = reading_.error_code;
  act(DriveAction::DISABLE_VOLTAGE);
  tick(cycles(0.3));
  EXPECT_EQ(drive_->active_drive_action(0), DriveAction::NONE) << "should complete at once";
  EXPECT_EQ(drive_state(), DeviceState::FAULT) << "a disable_voltage reset the fault";
  EXPECT_EQ(reading_.error_code, error_code) << "the error code was lost";
}

/// A disable_voltage requested during the drive's own fault reaction does not reset the fault once it ends.
TEST_F(SingleDrive, AStopRequestedDuringTheFaultReactionDoesNotResetTheFault)
{
  // 200 rad/s^2 keeps Fault Reaction Active long enough to request the stop within it (about 30 ms
  // from a 0.05 rad step), and the reaction brakes the joint within about 0.15 rad.
  provoke_step_fault(200.0, 0.05);
  if (drive_state() != DeviceState::FAULT_REACTION_ACTIVE)
  {
    GTEST_SKIP() << "the drive went to Fault before a request could reach Fault Reaction Active";
  }
  act(DriveAction::DISABLE_VOLTAGE);
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::FAULT; }, 0.5), 0.0);
  const uint16_t error_code = reading_.error_code;
  tick(cycles(0.3));
  EXPECT_EQ(drive_state(), DeviceState::FAULT) << "the pending disable_voltage reset the fault";
  EXPECT_EQ(reading_.error_code, error_code) << "the error code was lost";
}

/// An enable of a faulted drive is refused until a reset_fault, after which it succeeds.
TEST_F(SingleDrive, AnEnableOfAFaultedDriveIsRefusedUntilAResetFault)
{
  provoke_step_fault(1000.0, 0.02);
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::FAULT; }, 0.5), 0.0);
  hold_here();
  tick(cycles(0.02));
  act(DriveAction::ENABLE);
  EXPECT_EQ(drive_->active_drive_action(0), DriveAction::NONE) << "the enable was not refused";
  EXPECT_FALSE(drive_->forwarding_setpoints(0)) << "a refused enable must not resume forwarding";
  tick(cycles(0.3));
  EXPECT_EQ(drive_state(), DeviceState::FAULT) << "the enable reset the fault";

  act(DriveAction::RESET_FAULT);
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::SWITCH_ON_DISABLED; }, 1.0), 0.0);
  hold_here();
  act(DriveAction::ENABLE);
  EXPECT_GE(tick_until([this] { return drive_state() == DeviceState::OPERATION_ENABLED; }, 2.0), 0.0)
      << "the enable after the reset failed";
}

/// The lifecycle disable() leaves a faulted drive in Fault and still succeeds.
TEST_F(SingleDrive, TheLifecycleDisableLeavesAFaultAlone)
{
  provoke_step_fault(1000.0, 0.02);
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::FAULT; }, 0.5), 0.0);
  const uint16_t error_code = reading_.error_code;
  EXPECT_TRUE(drive_->disable());
  sample();
  EXPECT_EQ(drive_state(), DeviceState::FAULT) << "disable() reset the fault";
  EXPECT_EQ(reading_.error_code, error_code) << "the error code was lost";
}

/// The lifecycle enable() refuses a faulted drive instead of resetting it.
TEST_F(SingleDrive, TheLifecycleEnableRefusesAFaultedDrive)
{
  provoke_step_fault(1000.0, 0.02);
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::FAULT; }, 0.5), 0.0);
  hold_here();
  tick(cycles(0.02));
  ASSERT_TRUE(drive_->write({ setpoint_ }));
  EXPECT_FALSE(drive_->enable()) << "enable() went ahead with a faulted drive";
  sample();
  EXPECT_EQ(drive_state(), DeviceState::FAULT) << "enable() reset the fault";
}

/// The lifecycle enable() also refuses a drive that faulted after an earlier enable() succeeded.
TEST_F(SingleDrive, TheLifecycleEnableRefusesADriveThatFaultedAfterAnEarlierEnable)
{
  ASSERT_TRUE(start({ .quick_stop_deceleration = 1000.0, .following_error = FollowingError::FAULT }));
  hold_here();
  ASSERT_TRUE(drive_->write({ setpoint_ }));
  ASSERT_TRUE(drive_->enable());
  sample();
  hold_here();
  tick(cycles(0.05));
  limit_travel(1.2);
  setpoint_.position += 0.02 * sign_;
  ASSERT_GE(tick_until([this] { return drive_state() == DeviceState::FAULT; }, 0.5), 0.0)
      << "the step did not fault the drive";
  hold_here();
  tick(cycles(0.02));
  ASSERT_TRUE(drive_->write({ setpoint_ }));
  EXPECT_FALSE(drive_->enable()) << "enable() reported success for a faulted drive";
  sample();
  EXPECT_EQ(drive_state(), DeviceState::FAULT) << "enable() reset the fault";
}

/// A joint configured energize: false refuses enable and quick_stop.
TEST_F(SingleDrive, AMonitoredJointRefusesEnergisingRequests)
{
  ASSERT_TRUE(start({ .energize = false }));
  for (const DriveAction action : { DriveAction::ENABLE, DriveAction::QUICK_STOP })
  {
    act(action);
    EXPECT_EQ(drive_->active_drive_action(0), DriveAction::NONE);
    tick(cycles(0.2));
    EXPECT_EQ(drive_state(), DeviceState::SWITCH_ON_DISABLED);
  }
}

}  // namespace synapticon_hardware::testing
