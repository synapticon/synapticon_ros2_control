// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief How Cia402Hal handles calls out of order, a failing bus, and drives that fault or never arrive.
///
/// The drives here do not follow the controlword: each reports the statusword a test gives it, which is how a stuck
/// drive looks to the HAL. The timeout tests take as long as the timeouts, about five seconds each.

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/cia402/cia402_hal.hpp"

namespace
{
using synapticon_hardware::Cia402Hal;
using synapticon_hardware::DriveAction;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::HalConfig;
using Clock = std::chrono::steady_clock;

constexpr uint16_t SWITCH_ON_DISABLED = 0x40;
constexpr uint16_t SWITCHED_ON = 0x23;
constexpr uint16_t OPERATION_ENABLED = 0x27;
constexpr uint16_t ENABLE_OPERATION_BIT = 0x08;
constexpr uint16_t QUICK_STOP_ACTIVE = 0x07;
constexpr uint16_t FAULT = 0x08;

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

  /// The controlword last written to `joint`.
  uint16_t controlword(std::size_t joint) const
  {
    return controlwords_[joint];
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
    return !reads_fail_;
  }
  bool exchange_write() override
  {
    return !writes_fail_;
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
  double transport_position(std::size_t) const override
  {
    return 0.0;
  }
  void write_controlword(std::size_t joint, uint16_t controlword) override
  {
    controlwords_[joint] = controlword;
  }
  void write_setpoints(std::size_t, const DriveCommand&) override
  {
  }
  void write_hold(std::size_t, double) override
  {
  }

private:
  std::vector<uint16_t> statuswords_;
  std::vector<uint16_t> controlwords_;
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
