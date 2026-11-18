// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief How Cia402Hal handles calls out of order, a failing bus, and drives that fault or never arrive.
///
/// The drives here do not follow the controlword unless a test asks for it: each reports the statusword a test gives
/// it, which is how a stuck drive looks to the HAL. The timeout tests take as long as the timeouts, about five seconds
/// each.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/cia402/cia402_hal.hpp"
#include "../log_capture.hpp"

namespace
{
using synapticon_hardware::Cia402Hal;
using synapticon_hardware::DriveAction;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::HalConfig;
using Clock = std::chrono::steady_clock;

constexpr uint16_t NOT_READY_TO_SWITCH_ON = 0x00;
constexpr uint16_t SWITCH_ON_DISABLED = 0x40;
constexpr uint16_t READY_TO_SWITCH_ON = 0x21;
constexpr uint16_t SWITCHED_ON = 0x23;
constexpr uint16_t OPERATION_ENABLED = 0x27;
constexpr uint16_t ENABLE_OPERATION_BIT = 0x08;
constexpr uint16_t QUICK_STOP_ACTIVE = 0x07;
constexpr uint16_t FAULT_REACTION_ACTIVE = 0x0F;
constexpr uint16_t FAULT = 0x08;
constexpr uint16_t FAULT_RESET_BIT = 0x80;
constexpr uint16_t UNDECODABLE = 0x01;   ///< a statusword that matches no CiA 402 state
constexpr uint16_t COMMAND_BITS = 0x8F;  ///< the controlword bits that carry the command
constexpr uint16_t SHUTDOWN = 0x06;
constexpr uint16_t SWITCH_ON = 0x07;
constexpr uint16_t ENABLE_OPERATION = 0x0F;
constexpr uint16_t DISABLE_VOLTAGE = 0x00;
constexpr uint16_t QUICK_STOP = 0x02;

/// The statusword a drive in `statusword` reports after it took the command of `controlword`. Covers the
/// transitions a walk asks for. A quick stop stays in Quick Stop Active.
uint16_t respond(uint16_t statusword, uint16_t controlword)
{
  const bool powered = statusword == OPERATION_ENABLED || statusword == QUICK_STOP_ACTIVE;
  switch (controlword & COMMAND_BITS)
  {
    case SHUTDOWN:
      return statusword == SWITCH_ON_DISABLED || statusword == SWITCHED_ON || statusword == OPERATION_ENABLED ?
                 READY_TO_SWITCH_ON :
                 statusword;
    case SWITCH_ON:
      return statusword == READY_TO_SWITCH_ON || statusword == OPERATION_ENABLED ? SWITCHED_ON : statusword;
    case ENABLE_OPERATION:
      return statusword == SWITCHED_ON || powered ? OPERATION_ENABLED : statusword;
    case QUICK_STOP:
      return powered ? QUICK_STOP_ACTIVE : SWITCH_ON_DISABLED;
    case DISABLE_VOLTAGE:
      return SWITCH_ON_DISABLED;
    default:
      return statusword;
  }
}

/// The walk timeout of Cia402Hal, with a margin.
constexpr auto WALK_TIMEOUT = std::chrono::milliseconds(5200);

/// @brief A Cia402Hal whose drives report the statusword a test sets, whatever the controlword says.
class ScriptedDrives : public Cia402Hal
{
public:
  /// One joint per entry of `energize`, every drive in Switch On Disabled and at rest.
  explicit ScriptedDrives(const std::vector<bool>& energize) : Cia402Hal("test_cia402_hal_failures")
  {
    std::vector<JointSettings> joints;
    for (std::size_t j = 0; j < energize.size(); ++j)
    {
      joints.push_back({ "joint_" + std::to_string(j + 1), energize[j], false });
    }
    statuswords_.assign(joints.size(), SWITCH_ON_DISABLED);
    controlwords_.assign(joints.size(), 0);
    sent_.resize(joints.size());
    following_.assign(joints.size(), false);
    in_transit_.resize(joints.size());
    arrived_.resize(joints.size());
    positions_.assign(joints.size(), 0.0);
    holds_.resize(joints.size());
    init_joints(std::move(joints), 1000.0);
  }

  bool setup(const HalConfig&) override
  {
    return true;
  }
  bool connect() override
  {
    return true;
  }
  bool start() override
  {
    streaming_ = true;
    return true;
  }
  bool stop() override
  {
    streaming_ = false;
    return true;
  }
  bool disconnect() override
  {
    return true;
  }
  bool read(std::vector<DriveState>&) override
  {
    return true;
  }
  bool is_connected() const override
  {
    return true;
  }
  bool quick_stop_available(std::size_t) const override
  {
    return true;
  }

  /// The statusword every drive reports from the next read on.
  void report(uint16_t statusword)
  {
    statuswords_.assign(statuswords_.size(), statusword);
  }
  /// The statusword `joint` reports from the next read on.
  void report(std::size_t joint, uint16_t statusword)
  {
    statuswords_[joint] = statusword;
  }
  /// Report Fault from the next read on, after a first read in the present state. Only on `joint` when given.
  void fault_after_one_read(std::optional<std::size_t> joint = std::nullopt)
  {
    fault_after_reads_ = reads_ + 1;
    faulting_joint_ = joint;
  }
  void fail_reads()
  {
    reads_fail_ = true;
  }
  void fail_writes()
  {
    writes_fail_ = true;
  }
  void move_at(double speed)
  {
    speed_ = speed;
  }
  /// From now on, a drive that is sent Quick Stop reports Switch On Disabled, as a drive without power would.
  void settle_on_quick_stop()
  {
    settle_on_quick_stop_ = true;
  }
  /// From now on, `joint` follows the controlword one cycle late: a read reports its answer to the controlword sent
  /// in the exchange before the last one.
  void follow_controlword(std::size_t joint)
  {
    following_[joint] = true;
  }
  /// The measured position of `joint`.
  void place(std::size_t joint, double position)
  {
    positions_[joint] = position;
  }
  /// The hold positions written to `joint` so far, oldest first.
  const std::vector<double>& holds(std::size_t joint) const
  {
    return holds_[joint];
  }

  /// True when a controlword written to `joint` so far carried `command`.
  bool sent(std::size_t joint, uint16_t command) const
  {
    return std::any_of(sent_[joint].begin(), sent_[joint].end(),
                       [command](uint16_t word) { return (word & COMMAND_BITS) == command; });
  }

  /// The controlword last written to `joint`.
  uint16_t controlword(std::size_t joint) const
  {
    return controlwords_[joint];
  }
  /// The statusword `joint` reports.
  uint16_t reported(std::size_t joint) const
  {
    return statuswords_[joint];
  }

  /// One cycle with `action` requested on every joint.
  bool cycle(DriveAction action)
  {
    std::vector<DriveCommand> commands(statuswords_.size());
    for (auto& command : commands)
    {
      command.drive_action = action;
    }
    return exchange_read() && write(commands);
  }

protected:
  bool exchange_read() override
  {
    ++reads_;
    if (fault_after_reads_ > 0 && reads_ > fault_after_reads_)
    {
      if (faulting_joint_)
      {
        report(*faulting_joint_, FAULT);
      }
      else
      {
        report(FAULT);
      }
    }
    for (std::size_t j = 0; j < statuswords_.size(); ++j)
    {
      if (following_[j] && arrived_[j])
      {
        statuswords_[j] = respond(statuswords_[j], *arrived_[j]);
      }
    }
    return !reads_fail_;
  }
  bool exchange_write() override
  {
    if (writes_fail_)
    {
      return false;
    }
    for (std::size_t j = 0; j < controlwords_.size(); ++j)
    {
      arrived_[j] = in_transit_[j];
      in_transit_[j] = controlwords_[j];
    }
    return true;
  }
  uint16_t statusword(std::size_t joint) const override
  {
    return statuswords_[joint];
  }
  uint16_t error_code(std::size_t joint) const override
  {
    return statuswords_[joint] == FAULT ? 0x7300 : 0;
  }
  uint16_t last_controlword(std::size_t joint) const override
  {
    return controlwords_[joint];
  }
  double speed(std::size_t) const override
  {
    return speed_;
  }
  double transport_position(std::size_t joint) const override
  {
    return positions_[joint];
  }
  void write_controlword(std::size_t joint, uint16_t controlword) override
  {
    controlwords_[joint] = controlword;
    sent_[joint].push_back(controlword);
    if (settle_on_quick_stop_ && (controlword & COMMAND_BITS) == QUICK_STOP)
    {
      statuswords_[joint] = SWITCH_ON_DISABLED;
    }
  }
  void write_setpoints(std::size_t, const DriveCommand&) override
  {
  }
  void write_hold(std::size_t joint, double position) override
  {
    holds_[joint].push_back(position);
  }

private:
  std::vector<uint16_t> statuswords_;
  std::vector<uint16_t> controlwords_;
  std::vector<std::vector<uint16_t>> sent_;
  bool settle_on_quick_stop_{ false };
  std::vector<bool> following_;
  std::vector<std::optional<uint16_t>> in_transit_;  ///< sent in the last exchange
  std::vector<std::optional<uint16_t>> arrived_;     ///< sent in the exchange before, answered by the next read
  std::vector<double> positions_;
  std::vector<std::vector<double>> holds_;
  int reads_{ 0 };
  int fault_after_reads_{ 0 };
  std::optional<std::size_t> faulting_joint_;
  bool reads_fail_{ false };
  bool writes_fail_{ false };
  double speed_{ 0.0 };
};

/// Seconds since `start`.
double since(Clock::time_point start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

// --- Calls out of order --------------------------------------------------------

TEST(Cia402HalFailures, EnableAndWritesAreRefusedBeforeStart)
{
  ScriptedDrives drives({ true });
  EXPECT_FALSE(drives.enable());
  EXPECT_FALSE(drives.write({ DriveCommand{} }));
  EXPECT_FALSE(drives.is_enabled());
}

TEST(Cia402HalFailures, DisableBeforeStartHasNothingToWindDown)
{
  ScriptedDrives drives({ true });
  EXPECT_TRUE(drives.disable());
  EXPECT_FALSE(drives.is_enabled());
}

TEST(Cia402HalFailures, WritesWithTheWrongNumberOfCommandsAreRefused)
{
  ScriptedDrives drives({ true, true });
  ASSERT_TRUE(drives.start());
  EXPECT_FALSE(drives.write({ DriveCommand{} }));
  EXPECT_TRUE(drives.write({ DriveCommand{}, DriveCommand{} }));
}

/// An infinite setpoint fails the write, while NaN is a valid value that keeps the last one.
TEST(Cia402HalFailures, AnInfiniteSetpointFailsTheWrite)
{
  ScriptedDrives drives({ true, true });
  ASSERT_TRUE(drives.start());
  std::vector<DriveCommand> commands(2);
  commands[1].velocity = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(drives.write(commands));
  EXPECT_FALSE(drives.write(commands));
  commands[1].velocity = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(drives.write(commands));
}

/// A negative torque ceiling, stiffness or damping fails the write like an infinite value. Negative position, velocity
/// and torque offset are valid.
TEST(Cia402HalFailures, ANegativeCeilingOrGainFailsTheWrite)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  for (const auto field : { &DriveCommand::max_joint_torque, &DriveCommand::stiffness, &DriveCommand::damping })
  {
    DriveCommand command;
    command.*field = -1.0;
    EXPECT_FALSE(drives.write({ command }));
  }
  EXPECT_TRUE(drives.write({ DriveCommand{ -1.0, -1.0, -1.0, 0.0, 0.0, 0.0 } }));
  EXPECT_TRUE(drives.write({ DriveCommand{} }));
}

/// An action code the HAL does not walk, such as an undeclared vendor action, is ignored.
TEST(Cia402HalFailures, AnUnsupportedActionIsIgnored)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  EXPECT_TRUE(drives.cycle(static_cast<DriveAction>(200)));
  EXPECT_EQ(drives.active_drive_action(0), DriveAction::NONE);
}

// --- A failing bus -------------------------------------------------------------

TEST(Cia402HalFailures, EnableFailsWhenTheBusReadFails)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.fail_reads();
  EXPECT_FALSE(drives.enable());
  EXPECT_FALSE(drives.is_enabled());
}

/// A failed read does not keep disable() from writing. The drive gets the controlwords of the way down on the states
/// it still reports, as with a short working counter, and disable() returns false, since the reads failed.
TEST(Cia402HalFailures, DisableWritesTheWayDownWhileTheReadsFail)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(OPERATION_ENABLED);
  drives.follow_controlword(0);
  ASSERT_TRUE(drives.enable());
  drives.fail_reads();
  const auto start = Clock::now();
  EXPECT_FALSE(drives.disable());
  EXPECT_LT(since(start), 1.0);  // once the drive reports Switch On Disabled, not at the timeout
  EXPECT_TRUE(drives.sent(0, QUICK_STOP));
  EXPECT_EQ(drives.controlword(0) & COMMAND_BITS, DISABLE_VOLTAGE);
  EXPECT_EQ(drives.reported(0), SWITCH_ON_DISABLED);
}

/// When enable() fails on a read, the disable() that follows still writes the way down on the states received last,
/// until its timeout.
TEST(Cia402HalFailures, AnEnableThatFailsOnAReadStillTakesTheDrivesDown)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(OPERATION_ENABLED);
  drives.fail_reads();
  const auto start = Clock::now();
  EXPECT_FALSE(drives.enable());
  EXPECT_GE(since(start), 4.9);
  EXPECT_EQ(drives.controlword(0) & COMMAND_BITS, QUICK_STOP);
  EXPECT_FALSE(drives.is_enabled());
}

TEST(Cia402HalFailures, EnableFailsWhenTheBusWriteFails)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.fail_writes();
  EXPECT_FALSE(drives.enable());
  EXPECT_FALSE(drives.is_enabled());
}

// --- Drives that fault or never arrive -------------------------------------------

/// With every joint monitored only, enable has nothing to energise and succeeds at once.
TEST(Cia402HalFailures, EnableWithOnlyMonitoredJointsSucceedsWithoutEnergising)
{
  ScriptedDrives drives({ false, false });
  ASSERT_TRUE(drives.start());
  EXPECT_TRUE(drives.enable());
}

TEST(Cia402HalFailures, EnableFailsWhenADriveFaultsOnTheWay)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.fault_after_one_read();
  const auto start = Clock::now();
  EXPECT_FALSE(drives.enable());
  EXPECT_LT(since(start), 1.0);  // at once, not at the timeout
  EXPECT_FALSE(drives.is_enabled());
}

/// A drive that already got power is taken down again when another one faults during enable(). It stays in
/// Operation Enabled here, so the way down ends at the timeout.
TEST(Cia402HalFailures, AFailedEnableTakesThePoweredDrivesDownAgain)
{
  ScriptedDrives drives({ true, true });
  ASSERT_TRUE(drives.start());
  drives.report(0, OPERATION_ENABLED);
  drives.fault_after_one_read(1);
  EXPECT_FALSE(drives.enable());
  EXPECT_EQ(drives.controlword(0) & ENABLE_OPERATION_BIT, 0);
  EXPECT_FALSE(drives.forwarding_setpoints(0));
  EXPECT_FALSE(drives.is_enabled());
}

TEST(Cia402HalFailures, EnableTimesOutOnADriveThatNeverArrives)
{
  ScriptedDrives drives({ true, false });
  ASSERT_TRUE(drives.start());
  const auto start = Clock::now();
  EXPECT_FALSE(drives.enable());
  EXPECT_GE(since(start), 4.9);
  EXPECT_FALSE(drives.is_enabled());
}

/// An undecodable statusword gets no Shutdown on the way up, which would take a powered drive down without a ramp.
/// The walk waits for its timeout, and the way down then starts with a Quick Stop.
TEST(Cia402HalFailures, EnableSendsNoShutdownToAnUndecodableState)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(UNDECODABLE);
  drives.settle_on_quick_stop();
  const auto start = Clock::now();
  EXPECT_FALSE(drives.enable());
  EXPECT_GE(since(start), 4.9);
  EXPECT_FALSE(drives.sent(0, SHUTDOWN));
  EXPECT_TRUE(drives.sent(0, QUICK_STOP));
  EXPECT_FALSE(drives.is_enabled());
}

/// A drive in Not Ready to Switch On has no power, so disable() does not wait for it.
TEST(Cia402HalFailures, DisableDoesNotWaitForADriveThatIsNotReadyToSwitchOn)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(NOT_READY_TO_SWITCH_ON);
  const auto start = Clock::now();
  EXPECT_TRUE(drives.disable());
  EXPECT_LT(since(start), 1.0);
}

/// A drive that stays in Quick Stop Active and keeps moving gets its voltage cut after the ramp wait, and disable
/// gives up at the walk timeout.
TEST(Cia402HalFailures, DisableTimesOutOnADriveThatKeepsMovingInQuickStop)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(QUICK_STOP_ACTIVE);
  drives.move_at(1.0);
  const auto start = Clock::now();
  EXPECT_FALSE(drives.disable());
  EXPECT_GE(since(start), 4.9);
  EXPECT_FALSE(drives.is_enabled());
  EXPECT_EQ(drives.controlword(0) & 0x02, 0);  // Enable voltage cleared
}

/// A NaN speed does not count as standing still: a disable action in Quick Stop Active cuts the voltage only at rest.
TEST(Cia402HalFailures, ANaNSpeedDoesNotCountAsAtRest)
{
  for (const double speed : { 0.0, std::numeric_limits<double>::quiet_NaN() })
  {
    ScriptedDrives drives({ true });
    ASSERT_TRUE(drives.start());
    drives.report(QUICK_STOP_ACTIVE);
    drives.move_at(speed);
    const auto start = Clock::now();
    while (since(start) < 0.3)
    {
      ASSERT_TRUE(drives.cycle(DriveAction::DISABLE));
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(drives.controlword(0) & COMMAND_BITS, speed == 0.0 ? DISABLE_VOLTAGE : QUICK_STOP) << "speed " << speed;
  }
}

/// A drive that waits in Switched On after a timed-out enable() is not left with Enable Operation in its
/// controlword, which it would follow on its own later.
TEST(Cia402HalFailures, ATimedOutEnableWithdrawsEnableOperation)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(SWITCHED_ON);
  EXPECT_FALSE(drives.enable());
  EXPECT_EQ(drives.controlword(0) & ENABLE_OPERATION_BIT, 0);
  EXPECT_FALSE(drives.forwarding_setpoints(0));
}

/// A drive_action walk does not block: it ends at its deadline while the cycles go on, and a timed-out enable turns
/// into a disable that withdraws Enable Operation and the setpoints.
TEST(Cia402HalFailures, AnEnableActionTurnsIntoADisableAtItsDeadline)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(SWITCHED_ON);
  ASSERT_TRUE(drives.cycle(DriveAction::ENABLE));
  ASSERT_EQ(drives.active_drive_action(0), DriveAction::ENABLE);
  EXPECT_NE(drives.controlword(0) & ENABLE_OPERATION_BIT, 0);
  EXPECT_TRUE(drives.forwarding_setpoints(0));
  const auto start = Clock::now();
  while (drives.active_drive_action(0) == DriveAction::ENABLE && Clock::now() - start < 2 * WALK_TIMEOUT)
  {
    ASSERT_TRUE(drives.cycle(DriveAction::ENABLE));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GE(since(start), 4.5);
  EXPECT_EQ(drives.active_drive_action(0), DriveAction::DISABLE);
  EXPECT_EQ(drives.controlword(0) & ENABLE_OPERATION_BIT, 0);
  EXPECT_FALSE(drives.forwarding_setpoints(0));
}

TEST(Cia402HalFailures, AnEnableActionEndsWhenTheDriveFaults)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  ASSERT_TRUE(drives.cycle(DriveAction::ENABLE));
  ASSERT_EQ(drives.active_drive_action(0), DriveAction::ENABLE);
  drives.report(FAULT);
  ASSERT_TRUE(drives.cycle(DriveAction::ENABLE));
  EXPECT_EQ(drives.active_drive_action(0), DriveAction::NONE);
}

TEST(Cia402HalFailures, EnableIsRefusedWhileADriveIsInFault)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(FAULT);
  EXPECT_FALSE(drives.enable());
  ASSERT_TRUE(drives.cycle(DriveAction::ENABLE));
  EXPECT_EQ(drives.active_drive_action(0), DriveAction::NONE);
}

/// A reset_fault for a drive that is not in Fault is ignored.
TEST(Cia402HalFailures, AFaultResetOutsideFaultIsIgnored)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  ASSERT_TRUE(drives.cycle(DriveAction::RESET_FAULT));
  EXPECT_EQ(drives.active_drive_action(0), DriveAction::NONE);
}

/// A reset_fault that arrives during the fault reaction is kept, and sends the reset once the drive is in Fault.
TEST(Cia402HalFailures, AFaultResetDuringTheFaultReactionWaitsForFault)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(FAULT_REACTION_ACTIVE);
  ASSERT_TRUE(drives.cycle(DriveAction::RESET_FAULT));
  EXPECT_EQ(drives.active_drive_action(0), DriveAction::RESET_FAULT);
  EXPECT_EQ(drives.controlword(0) & FAULT_RESET_BIT, 0);
  ASSERT_TRUE(drives.cycle(DriveAction::RESET_FAULT));
  EXPECT_EQ(drives.controlword(0) & FAULT_RESET_BIT, 0);
  drives.report(FAULT);
  ASSERT_TRUE(drives.cycle(DriveAction::RESET_FAULT));
  EXPECT_NE(drives.controlword(0) & FAULT_RESET_BIT, 0);
  drives.report(SWITCH_ON_DISABLED);
  ASSERT_TRUE(drives.cycle(DriveAction::RESET_FAULT));
  EXPECT_EQ(drives.active_drive_action(0), DriveAction::NONE);
}

/// is_enabled() follows the drives: it turns false when a powered drive leaves Operation Enabled by itself, and
/// a monitored drive does not count.
TEST(Cia402HalFailures, IsEnabledFollowsTheDrives)
{
  ScriptedDrives drives({ true, false });
  ASSERT_TRUE(drives.start());
  drives.report(0, OPERATION_ENABLED);
  ASSERT_TRUE(drives.enable());
  EXPECT_TRUE(drives.is_enabled());
  drives.report(0, FAULT);
  ASSERT_TRUE(drives.cycle(DriveAction::NONE));
  EXPECT_FALSE(drives.is_enabled());
  drives.report(0, OPERATION_ENABLED);
  ASSERT_TRUE(drives.cycle(DriveAction::NONE));
  EXPECT_TRUE(drives.is_enabled());
}

/// disable() says once that the drives have no power: when it took one down, not again when all were down already.
TEST(Cia402HalFailures, DisableLogsTheEndOfTheWayDownOnlyWhenItTookADriveDown)
{
  const synapticon_hardware::test::LogCapture log;
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(OPERATION_ENABLED);
  ASSERT_TRUE(drives.enable());
  drives.follow_controlword(0);
  ASSERT_TRUE(drives.disable());
  EXPECT_FALSE(drives.is_enabled());
  EXPECT_EQ(log.count("have no power now"), 1u);
  EXPECT_TRUE(drives.disable());
  EXPECT_TRUE(drives.disable());
  EXPECT_EQ(log.count("have no power now"), 1u);
}

/// A repeated enable() walks the drives again: it cancels a running stop and lets the setpoints through again.
TEST(Cia402HalFailures, ARepeatedEnableWalksTheDrivesAgain)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(OPERATION_ENABLED);
  ASSERT_TRUE(drives.enable());
  ASSERT_TRUE(drives.cycle(DriveAction::DISABLE));
  ASSERT_EQ(drives.active_drive_action(0), DriveAction::DISABLE);
  ASSERT_FALSE(drives.forwarding_setpoints(0));
  EXPECT_TRUE(drives.enable());
  EXPECT_EQ(drives.active_drive_action(0), DriveAction::NONE);
  EXPECT_TRUE(drives.forwarding_setpoints(0));
  EXPECT_TRUE(drives.is_enabled());
  EXPECT_EQ(drives.controlword(0) & COMMAND_BITS, ENABLE_OPERATION);
}

/// A disable action writes Quick Stop, and enable() comes before the lagging drive reports a change. The repeated
/// enable() asserts Enable Operation again, so the drive comes back to Operation Enabled after a short quick stop.
TEST(Cia402HalFailures, ARepeatedEnableReplacesTheQuickStopOfADisableAction)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  drives.report(OPERATION_ENABLED);
  drives.follow_controlword(0);
  ASSERT_TRUE(drives.enable());
  ASSERT_TRUE(drives.cycle(DriveAction::DISABLE));
  ASSERT_EQ(drives.controlword(0) & COMMAND_BITS, QUICK_STOP);
  ASSERT_EQ(drives.reported(0), OPERATION_ENABLED);
  EXPECT_TRUE(drives.enable());
  EXPECT_EQ(drives.controlword(0) & COMMAND_BITS, ENABLE_OPERATION);
  for (int i = 0; i < 5; ++i)
  {
    ASSERT_TRUE(drives.cycle(DriveAction::NONE));
  }
  EXPECT_EQ(drives.controlword(0) & COMMAND_BITS, ENABLE_OPERATION);
  EXPECT_EQ(drives.reported(0), OPERATION_ENABLED);
  EXPECT_TRUE(drives.is_enabled());
}

/// While enable() waits for a drive to arrive, a joint already in Operation Enabled keeps its target. A joint below
/// Operation Enabled is held at its measured position.
TEST(Cia402HalFailures, EnableHoldsOnlyTheJointsBelowOperationEnabled)
{
  ScriptedDrives drives({ true, true });
  ASSERT_TRUE(drives.start());
  drives.report(0, OPERATION_ENABLED);
  drives.place(0, 1.0);
  drives.follow_controlword(1);
  drives.place(1, 2.0);
  ASSERT_TRUE(drives.enable());
  EXPECT_EQ(drives.reported(1), OPERATION_ENABLED);
  EXPECT_TRUE(drives.holds(0).empty());
  ASSERT_FALSE(drives.holds(1).empty());
  for (const double hold : drives.holds(1))
  {
    EXPECT_EQ(hold, 2.0);
  }
}

/// disable() cancels an enable action still walking, so it does not resume afterwards.
TEST(Cia402HalFailures, DisableCancelsAnEnableActionInProgress)
{
  ScriptedDrives drives({ true });
  ASSERT_TRUE(drives.start());
  ASSERT_TRUE(drives.cycle(DriveAction::ENABLE));
  ASSERT_EQ(drives.active_drive_action(0), DriveAction::ENABLE);
  EXPECT_TRUE(drives.disable());
  EXPECT_EQ(drives.active_drive_action(0), DriveAction::NONE);
}

}  // namespace
