// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Tests for the CiA 402 statusword decode and the controlword walk toward a target state.
///
/// Each pair of target and current state is checked against the CiA 402 state diagram, each transition for the bits
/// it must leave alone, and the Fault Reset bit for being sent as a pulse.

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/cia402/state_machine.hpp"

namespace
{
using cia402::DeviceState;
using cia402::FaultExit;
using cia402::WalkPath;
using enum cia402::DeviceState;

// The command patterns over the controlword bits the state machine owns, and KEEP for a controlword left alone.
constexpr int SHUT = 0x06;
constexpr int SWON = 0x07;
constexpr int OPER = 0x0F;
constexpr int CUTV = 0x00;
constexpr int QSTP = 0x02;
constexpr int RSET = 0x80;
constexpr int KEEP = -1;

/// Every state, in DeviceState order, as the columns of the walk rows.
const std::vector<DeviceState> STATES = { START,
                                          NOT_READY_TO_SWITCH_ON,
                                          SWITCH_ON_DISABLED,
                                          READY_TO_SWITCH_ON,
                                          SWITCHED_ON,
                                          OPERATION_ENABLED,
                                          QUICK_STOP_ACTIVE,
                                          FAULT_REACTION_ACTIVE,
                                          FAULT,
                                          UNKNOWN };
const std::vector<DeviceState> POWER_LADDER = { SWITCH_ON_DISABLED, READY_TO_SWITCH_ON, SWITCHED_ON, OPERATION_ENABLED };

std::string name(DeviceState state)
{
  return cia402::device_state_name(state);
}

std::string route(DeviceState from, DeviceState to)
{
  return "from " + name(from) + " toward " + name(to);
}

/// The controlword of one step from `from` toward `to`, from the held word `held`.
uint16_t step(DeviceState from, DeviceState to, uint16_t held = 0, WalkPath path = WalkPath::RAMPED,
              FaultExit exit = FaultExit::HOLD)
{
  return cia402::controlword_toward(from, to, held, path, exit);
}

// Statusword decode.

/// A statusword of each state with an encoding. START has none.
const std::vector<std::pair<DeviceState, uint16_t>> STATUSWORDS = {
  { NOT_READY_TO_SWITCH_ON, 0x00 },
  { SWITCH_ON_DISABLED, 0x40 },
  { READY_TO_SWITCH_ON, 0x21 },
  { SWITCHED_ON, 0x23 },
  { OPERATION_ENABLED, 0x27 },
  { QUICK_STOP_ACTIVE, 0x07 },
  { FAULT_REACTION_ACTIVE, 0x0F },
  { FAULT, 0x08 },
  { UNKNOWN, 0x01 },
};

TEST(StatuswordDecode, EachStateDecodesFromItsStatusword)
{
  for (const auto& [state, statusword] : STATUSWORDS)
  {
    EXPECT_EQ(cia402::device_state(statusword), state) << name(state);
  }
}

TEST(StatuswordDecode, WarningRemoteTargetAndVendorBitsAreIgnored)
{
  for (const auto& [state, statusword] : STATUSWORDS)
  {
    EXPECT_EQ(cia402::device_state(static_cast<uint16_t>(statusword | 0xF980)), state) << name(state);
  }
}

TEST(StatuswordDecode, EachStateHasAName)
{
  for (const DeviceState state : STATES)
  {
    EXPECT_FALSE(name(state).empty());
  }
  EXPECT_EQ(name(OPERATION_ENABLED), "Operation Enabled");
  EXPECT_EQ(name(UNKNOWN), "Unknown");
}

// The walk: the command from each current state toward a target.

struct WalkRow
{
  const char* label;
  DeviceState target;
  WalkPath path;
  std::vector<int> commands;  ///< Per current state, in STATES order.
};

class Walk : public ::testing::TestWithParam<WalkRow>
{
};

TEST_P(Walk, IssuesTheDiagramsCommandFromEachState)
{
  const WalkRow& row = GetParam();
  ASSERT_EQ(row.commands.size(), STATES.size());
  for (std::size_t i = 0; i < STATES.size(); ++i)
  {
    const uint16_t expected = row.commands[i] == KEEP ? 0 : static_cast<uint16_t>(row.commands[i]);
    EXPECT_EQ(step(STATES[i], row.target, 0, row.path), expected) << route(STATES[i], row.target);
  }
}

// On the way up, an undecodable state (UN) gets no new command: it may have power, and a Shutdown would take it.
//                              ST    NR    SD    RS    SO    OE    QS    FR    FA    UN
INSTANTIATE_TEST_SUITE_P(
    Targets, Walk,
    ::testing::Values(
        // Up by shutdown, switch on and enable operation. Quick Stop Active is left by transition 16.
        WalkRow{ "OperationEnabled",
                 OPERATION_ENABLED,
                 WalkPath::RAMPED,
                 { SHUT, SHUT, SHUT, SWON, OPER, OPER, OPER, SHUT, KEEP, KEEP } },
        // One rung lower, transition 5 from Operation Enabled. Quick Stop Active is left downward.
        WalkRow{ "SwitchedOn",
                 SWITCHED_ON,
                 WalkPath::RAMPED,
                 { SHUT, SHUT, SHUT, SWON, SWON, SWON, CUTV, SHUT, KEEP, KEEP } },
        // Shutdown from every rung (transitions 2, 6 and 8).
        WalkRow{ "ReadyToSwitchOn",
                 READY_TO_SWITCH_ON,
                 WalkPath::RAMPED,
                 { SHUT, SHUT, SHUT, SHUT, SHUT, SHUT, CUTV, SHUT, KEEP, KEEP } },
        // Ramped: a quick stop first, the voltage cut only from Quick Stop Active.
        WalkRow{ "SwitchOnDisabled",
                 SWITCH_ON_DISABLED,
                 WalkPath::RAMPED,
                 { QSTP, QSTP, CUTV, QSTP, QSTP, QSTP, CUTV, QSTP, KEEP, QSTP } },
        // Direct: the voltage cut in one step from everywhere.
        WalkRow{ "SwitchOnDisabledDirectly",
                 SWITCH_ON_DISABLED,
                 WalkPath::DIRECT,
                 { CUTV, CUTV, CUTV, CUTV, CUTV, CUTV, CUTV, CUTV, KEEP, CUTV } },
        // Only transition 11 enters Quick Stop Active, so the walk does not climb to reach it. On the way up it sends
        // Quick Stop, which takes the drive to Switch On Disabled (transitions 7 and 10).
        WalkRow{ "QuickStopActive",
                 QUICK_STOP_ACTIVE,
                 WalkPath::RAMPED,
                 { KEEP, KEEP, KEEP, QSTP, QSTP, QSTP, QSTP, KEEP, KEEP, KEEP } },
        // The states the drive enters by itself, Fault and Unknown cannot be walked to.
        WalkRow{ "Start", START, WalkPath::RAMPED, std::vector<int>(10, KEEP) },
        WalkRow{ "NotReady", NOT_READY_TO_SWITCH_ON, WalkPath::RAMPED, std::vector<int>(10, KEEP) },
        WalkRow{ "FaultReaction", FAULT_REACTION_ACTIVE, WalkPath::RAMPED, std::vector<int>(10, KEEP) },
        WalkRow{ "Fault", FAULT, WalkPath::RAMPED, std::vector<int>(10, KEEP) },
        WalkRow{ "Unknown", UNKNOWN, WalkPath::RAMPED, std::vector<int>(10, KEEP) }),
    [](const auto& info) { return std::string(info.param.label); });

TEST(WalkRoutes, TheDirectRouteChangesOnlyTheWayDownToSwitchOnDisabled)
{
  for (const DeviceState to : STATES)
  {
    for (const DeviceState from : STATES)
    {
      if (to != SWITCH_ON_DISABLED)
      {
        EXPECT_EQ(step(from, to, 0, WalkPath::DIRECT), step(from, to, 0, WalkPath::RAMPED)) << route(from, to);
      }
    }
  }
}

TEST(WalkRoutes, WithoutARouteTheWayDownIsRamped)
{
  EXPECT_EQ(cia402::controlword_toward(OPERATION_ENABLED, SWITCH_ON_DISABLED, 0), QSTP);
}

TEST(WalkRoutes, ATargetReachedKeepsGettingItsCommand)
{
  const std::vector<std::pair<DeviceState, int>> held = {
    { OPERATION_ENABLED, OPER },  { SWITCHED_ON, SWON },       { READY_TO_SWITCH_ON, SHUT },
    { SWITCH_ON_DISABLED, CUTV }, { QUICK_STOP_ACTIVE, QSTP },
  };
  for (const auto& [state, command] : held)
  {
    EXPECT_EQ(step(state, state), command) << name(state);
  }
}

// Waiting in Quick Stop Active for the ramp.

TEST(QuickStopRamp, EachRampedStepOutOfQuickStopActiveWaitsForTheRamp)
{
  for (const DeviceState to : { SWITCH_ON_DISABLED, READY_TO_SWITCH_ON, SWITCHED_ON })
  {
    EXPECT_TRUE(cia402::awaits_ramp(QUICK_STOP_ACTIVE, to, WalkPath::RAMPED)) << name(to);
  }
}

TEST(QuickStopRamp, NoOtherStepWaitsForTheRamp)
{
  for (const DeviceState from : STATES)
  {
    for (const DeviceState to : STATES)
    {
      EXPECT_FALSE(cia402::awaits_ramp(from, to, WalkPath::DIRECT)) << route(from, to);
      if (from != QUICK_STOP_ACTIVE)
      {
        EXPECT_FALSE(cia402::awaits_ramp(from, to, WalkPath::RAMPED)) << route(from, to);
      }
    }
  }
  // Steps out of Quick Stop Active that keep the torque on.
  for (const DeviceState to : { QUICK_STOP_ACTIVE, OPERATION_ENABLED, UNKNOWN })
  {
    EXPECT_FALSE(cia402::awaits_ramp(QUICK_STOP_ACTIVE, to, WalkPath::RAMPED)) << name(to);
  }
}

TEST(QuickStopRamp, WhileWaitingTheQuickStopCommandIsHeld)
{
  EXPECT_EQ(step(QUICK_STOP_ACTIVE, QUICK_STOP_ACTIVE), QSTP);
  EXPECT_EQ(step(QUICK_STOP_ACTIVE, SWITCH_ON_DISABLED), CUTV);
}

// Arrival.

TEST(Arrival, ReachedMeansArrivedExceptForAQuickStopAndTheWayDownFromFault)
{
  for (const DeviceState from : STATES)
  {
    for (const DeviceState to : STATES)
    {
      if (to == QUICK_STOP_ACTIVE || (from == FAULT && to == SWITCH_ON_DISABLED))
      {
        continue;
      }
      EXPECT_EQ(cia402::reached(from, to), from == to) << route(from, to);
      EXPECT_EQ(cia402::reached(from, to, FaultExit::RESET), from == to) << route(from, to);
    }
  }
}

TEST(Arrival, ADriveWithoutPowerHasQuickStoppedAlready)
{
  for (const DeviceState from : { START, NOT_READY_TO_SWITCH_ON, SWITCH_ON_DISABLED, FAULT })
  {
    EXPECT_TRUE(cia402::reached(from, QUICK_STOP_ACTIVE)) << name(from);
  }
}

/// Ready to Switch On and Switched On have no power, but an earlier Enable Operation could still power the drive.
TEST(Arrival, ADriveThatMayStillDriveOrIsOnItsWayUpHasNotQuickStopped)
{
  for (const DeviceState from : { READY_TO_SWITCH_ON, SWITCHED_ON, OPERATION_ENABLED, FAULT_REACTION_ACTIVE, UNKNOWN })
  {
    EXPECT_FALSE(cia402::reached(from, QUICK_STOP_ACTIVE)) << name(from);
  }
}

TEST(Arrival, OnlyThePoweredStatesAndUnknownMayBeEnergised)
{
  for (const DeviceState state : STATES)
  {
    const bool powered =
        state == OPERATION_ENABLED || state == QUICK_STOP_ACTIVE || state == FAULT_REACTION_ACTIVE || state == UNKNOWN;
    EXPECT_EQ(cia402::may_be_energised(state), powered) << name(state);
  }
}

// Fault, left only on an explicit reset.

TEST(FaultExits, AWalkAloneNeverLeavesFault)
{
  constexpr uint16_t VENDOR_AND_HALT = 0x9100;
  for (const WalkPath path : { WalkPath::RAMPED, WalkPath::DIRECT })
  {
    for (const DeviceState to : STATES)
    {
      EXPECT_EQ(step(FAULT, to, VENDOR_AND_HALT, path), VENDOR_AND_HALT) << name(to);
      EXPECT_EQ(cia402::next_controlword(VENDOR_AND_HALT, FAULT, to, path), VENDOR_AND_HALT) << name(to);
    }
  }
}

TEST(FaultExits, AResetLeavesFaultTowardTheLadder)
{
  for (const WalkPath path : { WalkPath::RAMPED, WalkPath::DIRECT })
  {
    for (const DeviceState to : POWER_LADDER)
    {
      EXPECT_EQ(step(FAULT, to, 0, path, FaultExit::RESET), RSET) << name(to);
    }
  }
}

TEST(FaultExits, AResetIsNotSentTowardTargetsOffTheLadder)
{
  for (const DeviceState to :
       { START, NOT_READY_TO_SWITCH_ON, QUICK_STOP_ACTIVE, FAULT_REACTION_ACTIVE, FAULT, UNKNOWN })
  {
    EXPECT_EQ(step(FAULT, to, 0, WalkPath::RAMPED, FaultExit::RESET), 0) << name(to);
  }
}

TEST(FaultExits, TheWayDownFromFaultIsDoneAtOnceWithoutAResetAndGoesOnWithOne)
{
  EXPECT_TRUE(cia402::reached(FAULT, SWITCH_ON_DISABLED));
  EXPECT_FALSE(cia402::reached(FAULT, SWITCH_ON_DISABLED, FaultExit::RESET));
}

TEST(FaultExits, TheResetBitDropsEverySecondCycle)
{
  const auto next = [](uint16_t word) {
    return cia402::next_controlword(word, FAULT, SWITCH_ON_DISABLED, WalkPath::RAMPED, FaultExit::RESET);
  };
  const uint16_t first = next(0);
  EXPECT_EQ(first & cia402::FAULT_RESET_BIT, cia402::FAULT_RESET_BIT);
  EXPECT_EQ(next(first) & cia402::FAULT_RESET_BIT, 0);
}

/// A drive that follows the CiA 402 diagram. The states it only leaves by itself settle in Switch On Disabled, and it
/// leaves Fault on rising edges of the Fault Reset bit only, `edges_needed` of them.
class DiagramDrive
{
public:
  explicit DiagramDrive(DeviceState state = SWITCH_ON_DISABLED, int edges_needed = 1)
    : state_(state), edges_needed_(edges_needed)
  {
  }

  void fault()
  {
    state_ = FAULT;
    edges_ = 0;
  }

  DeviceState state() const
  {
    return state_;
  }

  bool faulted() const
  {
    return state_ == FAULT;
  }

  /// Take one cycle's controlword.
  void receive(uint16_t word)
  {
    const bool rising = (word & ~previous_ & cia402::FAULT_RESET_BIT) != 0;
    previous_ = word;
    if (state_ != FAULT)
    {
      state_ = after(word & cia402::CONTROLWORD_COMMAND_BITS);
    }
    else if (rising && ++edges_ >= edges_needed_)
    {
      state_ = SWITCH_ON_DISABLED;
    }
  }

private:
  DeviceState after(int command) const
  {
    const bool on_ladder = std::find(POWER_LADDER.begin(), POWER_LADDER.end(), state_) != POWER_LADDER.end();
    switch (command)
    {
      case SHUT:
        return on_ladder ? READY_TO_SWITCH_ON : SWITCH_ON_DISABLED;
      case SWON:
        return SWITCHED_ON;
      case OPER:
        return OPERATION_ENABLED;
      case QSTP:
        return state_ == OPERATION_ENABLED ? QUICK_STOP_ACTIVE : SWITCH_ON_DISABLED;
      default:
        return SWITCH_ON_DISABLED;  // disable voltage, and a reset outside Fault
    }
  }

  DeviceState state_;
  int edges_needed_;
  int edges_ = 0;
  uint16_t previous_ = 0;
};

TEST(WalkAgainstTheDiagram, EveryWalkWithResetArrivesWithinEightSteps)
{
  for (const WalkPath path : { WalkPath::RAMPED, WalkPath::DIRECT })
  {
    for (const DeviceState to : POWER_LADDER)
    {
      for (const DeviceState from : STATES)
      {
        DiagramDrive drive(from);
        uint16_t word = 0;
        for (int k = 0; k < 8 && !cia402::reached(drive.state(), to, FaultExit::RESET); ++k)
        {
          word = step(drive.state(), to, word, path, FaultExit::RESET);
          drive.receive(word);
        }
        EXPECT_TRUE(cia402::reached(drive.state(), to, FaultExit::RESET))
            << route(from, to) << ", stuck in " << name(drive.state());
      }
    }
  }
}

TEST(WalkAgainstTheDiagram, WithoutResetAFaultedDriveStaysFaulted)
{
  for (const WalkPath path : { WalkPath::RAMPED, WalkPath::DIRECT })
  {
    for (const DeviceState to : POWER_LADDER)
    {
      DiagramDrive drive(FAULT);
      uint16_t word = 0;
      for (int k = 0; k < 8; ++k)
      {
        word = cia402::next_controlword(word, drive.state(), to, path);
        drive.receive(word);
      }
      EXPECT_EQ(drive.state(), FAULT) << name(to);
    }
  }
}

// Bits outside the state machine.

TEST(ForeignBits, TheStateMachineOwnsBitsZeroToThreeAndSeven)
{
  EXPECT_EQ(cia402::CONTROLWORD_COMMAND_BITS, 0x008F);
}

TEST(ForeignBits, HaltModeAndVendorBitsSurviveAndDoNotChangeTheCommand)
{
  constexpr uint16_t FOREIGN = 0xFB70;  // bits 15 to 11, 9, 8 and 6 to 4
  for (const DeviceState to : STATES)
  {
    for (const DeviceState from : STATES)
    {
      const uint16_t word = step(from, to, FOREIGN);
      EXPECT_EQ(word & ~cia402::CONTROLWORD_COMMAND_BITS, FOREIGN) << route(from, to);
      EXPECT_EQ(word & cia402::CONTROLWORD_COMMAND_BITS, step(from, to) & cia402::CONTROLWORD_COMMAND_BITS)
          << route(from, to);
    }
  }
}

TEST(ForeignBits, AStaleResetBitIsOverwrittenByTheNextCommand)
{
  EXPECT_EQ(step(SWITCH_ON_DISABLED, OPERATION_ENABLED, RSET | 0x0100), SHUT | 0x0100);
}

// Vendor actions.

TEST(VendorActions, SetAndClearTouchOnlyTheirBits)
{
  const cia402::VendorAction action{ 0x0800, 0x1000, 0x2000, 0x2000 };
  EXPECT_EQ(cia402::apply_vendor_action(action, 0x100F), 0x080F);
  EXPECT_EQ(cia402::apply_vendor_action(action, 0x0000), 0x0800);
}

TEST(VendorActions, CompletionComparesTheMaskedStatusword)
{
  const cia402::VendorAction action{ 0x0800, 0x0000, 0x3000, 0x1000 };
  EXPECT_TRUE(cia402::vendor_action_complete(action, 0x1027));
  EXPECT_FALSE(cia402::vendor_action_complete(action, 0x3027));
  EXPECT_FALSE(cia402::vendor_action_complete(action, 0x0027));
}

TEST(VendorActions, AnActionSurvivesTheWalkUp)
{
  const cia402::VendorAction action{ 0x0800, 0x0000, 0x0000, 0x0000 };
  uint16_t word = cia402::apply_vendor_action(action, 0);
  for (const DeviceState from : POWER_LADDER)
  {
    word = step(from, OPERATION_ENABLED, word);
    EXPECT_EQ(word & 0x0800, 0x0800);
  }
  EXPECT_EQ(word, 0x0800 | OPER);
}

// The Fault Reset bit, edge-triggered, along the HAL's controlword path.

/// One cycle as the HAL runs it: a reset_fault walks toward Switch On Disabled and may leave Fault, any other walk
/// heads for its target without, and an idle joint heads for UNKNOWN, which holds the word.
uint16_t hal_cycle(uint16_t held, DiagramDrive& drive, bool reset_fault, DeviceState to = UNKNOWN)
{
  const DeviceState target = reset_fault ? SWITCH_ON_DISABLED : to;
  const FaultExit exit = reset_fault ? FaultExit::RESET : FaultExit::HOLD;
  const uint16_t word = cia402::next_controlword(held, drive.state(), target, WalkPath::RAMPED, exit);
  drive.receive(word);
  return word;
}

bool reset_bit(uint16_t word)
{
  return (word & cia402::FAULT_RESET_BIT) != 0;
}

TEST(FaultResetPulse, AResetIsHighForOneCycle)
{
  DiagramDrive drive;
  drive.fault();
  uint16_t word = hal_cycle(0, drive, true);
  EXPECT_TRUE(reset_bit(word)) << "no Fault Reset sent";
  EXPECT_FALSE(drive.faulted()) << "the drive saw no rising edge";
  EXPECT_FALSE(reset_bit(hal_cycle(word, drive, false))) << "the bit stayed high after its cycle";
}

TEST(FaultResetPulse, ASecondResetAfterANewFaultIsANewEdge)
{
  DiagramDrive drive;
  drive.fault();
  uint16_t word = hal_cycle(0, drive, true);
  for (int k = 0; k < 5; ++k)
  {
    word = hal_cycle(word, drive, false);
  }
  ASSERT_FALSE(drive.faulted());
  drive.fault();
  word = hal_cycle(word, drive, true);
  EXPECT_TRUE(reset_bit(word));
  EXPECT_FALSE(drive.faulted()) << "the second reset never reached the drive";
}

TEST(FaultResetPulse, AnEnableOrADescentNeverResets)
{
  for (const DeviceState to : { OPERATION_ENABLED, SWITCH_ON_DISABLED })
  {
    DiagramDrive drive;
    drive.fault();
    uint16_t word = 0;
    for (int k = 0; k < 10; ++k)
    {
      word = hal_cycle(word, drive, false, to);
      EXPECT_FALSE(reset_bit(word)) << "a walk toward " << name(to) << " sent Fault Reset";
    }
    EXPECT_TRUE(drive.faulted()) << name(to);
  }
}

TEST(FaultResetPulse, VendorBitsSurviveThePulseAndItsEnd)
{
  constexpr uint16_t VENDOR = 0x9000;
  DiagramDrive drive;
  drive.fault();
  uint16_t word = hal_cycle(VENDOR, drive, true);
  EXPECT_EQ(word & VENDOR, VENDOR);
  word = hal_cycle(word, drive, false);
  EXPECT_EQ(word & VENDOR, VENDOR);
  EXPECT_FALSE(reset_bit(word));
}

// A reset is walked every cycle until the drive leaves Fault. The last three tests set that against a single attempt
// on a drive that needs three edges.

TEST(FaultResetPulse, OneAttemptDoesNotClearADriveNeedingThreeEdges)
{
  DiagramDrive drive(SWITCH_ON_DISABLED, 3);
  drive.fault();
  uint16_t word = hal_cycle(0, drive, true);
  for (int k = 0; k < 10; ++k)
  {
    word = hal_cycle(word, drive, false);
  }
  EXPECT_TRUE(drive.faulted()) << "one attempt cleared a drive that needs three edges";
}

TEST(FaultResetPulse, AWalkedResetGoesOnUntilTheDriveLeavesFault)
{
  DiagramDrive drive(SWITCH_ON_DISABLED, 3);
  drive.fault();
  uint16_t word = 0;
  int cycles = 0;
  for (; cycles < 20 && drive.faulted(); ++cycles)
  {
    word = hal_cycle(word, drive, true);
  }
  EXPECT_FALSE(drive.faulted()) << "the walk gave up before the drive left Fault";
  EXPECT_EQ(drive.state(), SWITCH_ON_DISABLED);
  EXPECT_GT(cycles, 1) << "this drive should need more than one cycle";
}

TEST(FaultResetPulse, AWalkedResetStillPulses)
{
  DiagramDrive drive(SWITCH_ON_DISABLED, 3);
  drive.fault();
  uint16_t word = 0;
  int high = 0;
  for (int k = 0; k < 6; ++k)
  {
    word = hal_cycle(word, drive, true);
    high += reset_bit(word) ? 1 : 0;
  }
  EXPECT_GT(high, 0) << "the walk never sent Fault Reset";
  EXPECT_LT(high, 6) << "the bit stayed high, which is a single edge";
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
