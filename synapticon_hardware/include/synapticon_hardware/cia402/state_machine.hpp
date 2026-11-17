// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The CiA 402 (IEC 61800-7-201) drive state machine: statusword decode and a
/// target-directed controlword walk.
///
/// Vendor-neutral and dependency-free, in its own `cia402` namespace. The walk takes a target
/// state, so it can hold a drive in Quick Stop Active as well as climb to Operation Enabled, and
/// it preserves the halt and manufacturer-specific controlword bits.

#include <cstddef>
#include <cstdint>

namespace cia402
{

/// @brief The CiA 402 device states, as decoded from the statusword (0x6041).
///
/// `START` has no statusword encoding (it is the internal power-on state) and `UNKNOWN` covers
/// any bit pattern matching no defined state.
enum class DeviceState : uint8_t
{
  START,
  NOT_READY_TO_SWITCH_ON,
  SWITCH_ON_DISABLED,
  READY_TO_SWITCH_ON,
  SWITCHED_ON,
  OPERATION_ENABLED,
  QUICK_STOP_ACTIVE,
  FAULT_REACTION_ACTIVE,
  FAULT,
  UNKNOWN,
};

/// Number of DeviceState values. The transition table is square in it.
constexpr std::size_t NUM_DEVICE_STATES = static_cast<std::size_t>(DeviceState::UNKNOWN) + 1u;

/// The controlword (0x6040) bits the state machine owns: Switch on (0), Enable voltage (1),
/// Quick stop (2), Enable operation (3) and Fault reset (7). Every transition preserves every
/// other bit: halt (8), mode-specific (4..6, 9) and manufacturer-specific (15..11).
constexpr uint16_t CONTROLWORD_COMMAND_BITS = 0x008Fu;

/// Fault reset (bit 7). The only edge-triggered bit of the controlword: the drive takes the
/// rising edge as the command, so holding it high commands nothing further.
constexpr uint16_t FAULT_RESET_BIT = 0x0080u;

/// @brief Which route a walk takes when it has to leave a powered state.
///
/// Only the descent to Switch-on-Disabled has two routes. Every other target has just one and
/// ignores this.
///
/// The safety-relevant difference: Disable Voltage removes torque in the same cycle it is
/// commanded, so a joint under load without a brake drops or runs on. RAMPED therefore asks the drive to
/// run its own quick stop ramp first and cuts the voltage only once the motion has stopped (see
/// awaits_ramp()). DIRECT is for a caller that has decided de-energising now is the lesser risk.
enum class WalkPath : uint8_t
{
  RAMPED,  ///< Ramp down through Quick Stop, cut the voltage once stopped. The safe default.
  DIRECT,  ///< Cut the voltage as soon as the state machine allows it, with no ramp.
};

/// @brief Whether a walk may leave Fault.
///
/// Fault is left only by a Fault Reset (transition 15), which acknowledges the fault and clears the
/// drive's error code. A walk therefore never issues it on its own: with HOLD a drive in Fault stays
/// there whatever the target. Only a caller acting on an explicit request to reset passes RESET.
enum class FaultExit : uint8_t
{
  HOLD,   ///< Keep a faulted drive in Fault. The default.
  RESET,  ///< Leave Fault by a Fault Reset, then walk on toward the target.
};

namespace detail
{
/// A statusword pattern: the state whose bits under `mask` read `value`.
struct StatusPattern
{
  uint16_t mask;
  uint16_t value;
  DeviceState state;
};

/// The statusword patterns of CiA 402, bits 0 to 3, 5 and 6. Bit 5 (quick stop) only tells the powered states apart.
constexpr StatusPattern STATUS_PATTERNS[] = {
  { 0x4F, 0x00, DeviceState::NOT_READY_TO_SWITCH_ON }, { 0x4F, 0x40, DeviceState::SWITCH_ON_DISABLED },
  { 0x6F, 0x21, DeviceState::READY_TO_SWITCH_ON },     { 0x6F, 0x23, DeviceState::SWITCHED_ON },
  { 0x6F, 0x27, DeviceState::OPERATION_ENABLED },      { 0x6F, 0x07, DeviceState::QUICK_STOP_ACTIVE },
  { 0x4F, 0x0F, DeviceState::FAULT_REACTION_ACTIVE },  { 0x4F, 0x08, DeviceState::FAULT },
};

/// Readable names, in DeviceState order.
constexpr const char* STATE_NAMES[NUM_DEVICE_STATES] = {
  "Start",
  "Not Ready to Switch On",
  "Switch On Disabled",
  "Ready to Switch On",
  "Switched On",
  "Operation Enabled",
  "Quick Stop Active",
  "Fault Reaction Active",
  "Fault",
  "Unknown",
};
}  // namespace detail

/// @brief The device state a statusword (0x6041) reports.
inline DeviceState device_state(uint16_t statusword)
{
  for (const auto& pattern : detail::STATUS_PATTERNS)
  {
    if ((statusword & pattern.mask) == pattern.value)
    {
      return pattern.state;
    }
  }
  return DeviceState::UNKNOWN;
}

/// @brief The readable name of a state, for diagnostics.
inline const char* device_state_name(DeviceState state)
{
  const auto slot = static_cast<std::size_t>(state);
  return slot < NUM_DEVICE_STATES ? detail::STATE_NAMES[slot] : "Unknown";
}

/// @brief Whether the drive function may be active, so the motor can be producing torque.
///
/// True for the three states in which the power stage is switched on, and for UNKNOWN: an
/// undecodable statusword is no evidence that the motor has stopped, so it is reported as
/// possibly energised rather than assumed safe.
inline bool may_be_energised(DeviceState state)
{
  return state == DeviceState::OPERATION_ENABLED || state == DeviceState::QUICK_STOP_ACTIVE ||
         state == DeviceState::FAULT_REACTION_ACTIVE || state == DeviceState::UNKNOWN;
}

namespace detail
{

/// The CiA 402 controlword commands, each valued by the pattern it sets over CONTROLWORD_COMMAND_BITS. KEEP sets
/// nothing: the controlword passes through as it is.
enum class Command : uint16_t
{
  KEEP = 0xFFFF,
  SHUTDOWN = 0x0006,          ///< transitions 2, 6, 8
  SWITCH_ON = 0x0007,         ///< transition 3, and 5 (Disable operation has the same pattern)
  ENABLE_OPERATION = 0x000F,  ///< transitions 4, 16
  DISABLE_VOLTAGE = 0x0000,   ///< transitions 7, 9, 10, 12
  QUICK_STOP = 0x0002,        ///< transitions 7, 10, 11
  FAULT_RESET = 0x0080,       ///< transition 15, on the rising edge of bit 7
};

// Four-letter names, so the tables below line up as grids.
constexpr Command KEEP = Command::KEEP;
constexpr Command SHUT = Command::SHUTDOWN;
constexpr Command SWON = Command::SWITCH_ON;
constexpr Command OPER = Command::ENABLE_OPERATION;
constexpr Command CUTV = Command::DISABLE_VOLTAGE;
constexpr Command QSTP = Command::QUICK_STOP;
constexpr Command RSET = Command::FAULT_RESET;

/// @brief The walk: for a target (row) and the current state (column), the command that takes the drive one step
/// closer.
///
/// Laid out as a grid, so each cell can be checked against the CiA 402 state diagram. Worth knowing:
///  - On the diagonal the drive is there, and the cell keeps asserting the command that holds it. A controlword of 0
///    would drop a drive that got there another way, for example one still energised after a restart.
///  - The way down to Switch On Disabled leads through Quick Stop from every powered state, so the drive ramps down
///    first: Disable Voltage straight from power leaves a loaded joint without brake free to drop. Quick Stop also
///    serves the unpowered states (transitions 7 and 10).
///  - Quick Stop Active can only be entered from Operation Enabled (transition 11), so its row keeps everywhere else
///    instead of climbing there.
///  - Start, Not Ready to Switch On, Fault Reaction Active and Fault are entered by the drive itself and never
///    commanded, so their rows keep throughout.
///  - The Fault column keeps too. Only a Fault Reset leaves Fault, and only on request (FaultExit::RESET).
///  - Toward the powered rungs, the Unknown column keeps. The drive may have power, and a Shutdown from Operation
///    Enabled would take it without a ramp. The walk waits for a state it can decode, or for its deadline, after
///    which the caller takes the drive down on the ramped way.
//   Columns, in DeviceState order: ST Start, NR Not Ready to Switch On, SD Switch On Disabled, RS Ready to Switch On,
//   SO Switched On, OE Operation Enabled, QS Quick Stop Active, FR Fault Reaction Active, FA Fault, UN Unknown.
constexpr Command WALK[NUM_DEVICE_STATES][NUM_DEVICE_STATES] = {
  //              ST    NR    SD    RS    SO    OE    QS    FR    FA    UN
  /* ST */ { KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP },
  /* NR */ { KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP },
  /* SD */ { QSTP, QSTP, CUTV, QSTP, QSTP, QSTP, CUTV, QSTP, KEEP, QSTP },
  /* RS */ { SHUT, SHUT, SHUT, SHUT, SHUT, SHUT, CUTV, SHUT, KEEP, KEEP },
  /* SO */ { SHUT, SHUT, SHUT, SWON, SWON, SWON, CUTV, SHUT, KEEP, KEEP },
  /* OE */ { SHUT, SHUT, SHUT, SWON, OPER, OPER, OPER, SHUT, KEEP, KEEP },
  /* QS */ { KEEP, KEEP, KEEP, KEEP, KEEP, QSTP, QSTP, KEEP, KEEP, KEEP },
  /* FR */ { KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP },
  /* FA */ { KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP },
  /* UN */ { KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP, KEEP },
};

/// @brief The way down to Switch On Disabled for WalkPath::DIRECT, without a ramp.
///
/// Disable Voltage is accepted in every powered state (transitions 7, 9, 10 and 12), so the target is one step away.
/// Fault keeps, as in WALK. Not the default, since the torque is gone at once and a loaded joint without brake can
/// drop.
constexpr Command DIRECT_DOWN[NUM_DEVICE_STATES] = {
  //  ST    NR    SD    RS    SO    OE    QS    FR    FA    UN
  CUTV, CUTV, CUTV, CUTV, CUTV, CUTV, CUTV, CUTV, KEEP, CUTV,
};

constexpr std::size_t at(DeviceState state)
{
  return static_cast<std::size_t>(state);
}

/// The command of one step toward `target`. DIRECT only changes the way down to Switch On Disabled, the one target
/// CiA 402 offers two routes to. With RESET a faulted drive leaves Fault toward any target reachable from Switch On
/// Disabled, where a Fault Reset lands.
constexpr Command step_command(DeviceState current, DeviceState target, WalkPath path, FaultExit fault_exit)
{
  if (current == DeviceState::FAULT)
  {
    const bool leads_on = WALK[at(target)][at(DeviceState::SWITCH_ON_DISABLED)] != KEEP;
    return fault_exit == FaultExit::RESET && leads_on ? RSET : KEEP;
  }
  if (path == WalkPath::DIRECT && target == DeviceState::SWITCH_ON_DISABLED)
  {
    return DIRECT_DOWN[at(current)];
  }
  return WALK[at(target)][at(current)];
}

}  // namespace detail

/// @brief Next controlword moving `current` one step toward `target`.
///
/// Preserves every bit the transition does not own (see CONTROLWORD_COMMAND_BITS), so a halt or a
/// vendor action riding the controlword survives the walk. On arrival the returned controlword
/// keeps asserting the target's own command. `path` picks the route down to Switch-on-Disabled and
/// is ignored for every other target. `fault_exit` decides whether a drive in Fault is reset.
inline uint16_t controlword_toward(DeviceState current, DeviceState target, uint16_t control_word,
                                   WalkPath path = WalkPath::RAMPED, FaultExit fault_exit = FaultExit::HOLD)
{
  const detail::Command command = detail::step_command(current, target, path, fault_exit);
  if (command == detail::KEEP)
  {
    return control_word;
  }
  return static_cast<uint16_t>((control_word & ~CONTROLWORD_COMMAND_BITS) | static_cast<uint16_t>(command));
}

/// @brief The controlword to send this cycle, given the one sent last cycle.
///
/// controlword_toward() plus the pulse rule for Fault reset: bit 7 is never sent high two cycles
/// in a row, so every Fault reset the walk (or a caller) issues is a fresh rising edge for the
/// drive, and a drive left in Fault is re-commanded every second cycle instead of once.
///
/// `target` of UNKNOWN means nothing is being walked toward: the word is held, minus a Fault
/// reset that has already been sent.
inline uint16_t next_controlword(uint16_t last_word, DeviceState current, DeviceState target,
                                 WalkPath path = WalkPath::RAMPED, FaultExit fault_exit = FaultExit::HOLD)
{
  const bool reset_sent = (last_word & FAULT_RESET_BIT) != 0u;
  const uint16_t next =
      controlword_toward(current, target, static_cast<uint16_t>(last_word & ~FAULT_RESET_BIT), path, fault_exit);
  return reset_sent ? static_cast<uint16_t>(next & ~FAULT_RESET_BIT) : next;
}

/// @brief Whether a ramped walk must wait in Quick Stop Active for the motion to stop.
///
/// CiA 402 enters Quick Stop Active when the quick stop ramp starts, not when it ends, and the
/// next step down (Disable Voltage) removes torque at once. Taken on arrival, it would cut the
/// ramp short. While this returns true the caller should walk toward QUICK_STOP_ACTIVE instead of
/// `target`, until it judges the motion stopped. That judgement needs the drive's units or a
/// timeout, so it is left to the caller. The direct route and walks back up are unaffected.
inline bool awaits_ramp(DeviceState current, DeviceState target, WalkPath path)
{
  return path == WalkPath::RAMPED && current == DeviceState::QUICK_STOP_ACTIVE &&
         detail::step_command(current, target, path, FaultExit::HOLD) == detail::CUTV;
}

/// @brief Whether the walk toward `target` is done.
///
/// Exact arrival, plus two cases where the walk cannot arrive and does not need to:
///  - A Quick Stop asked of a drive whose power stage is already off. Such a drive no longer
///    drives the joint, though a joint without brake may still move under load, and Quick Stop Active is
///    reachable only from Operation Enabled (transition 11). Without this the walk would hold until
///    its deadline and report a timeout.
///  - A descent to Switch-on-Disabled from Fault without a reset: the power stage is off already,
///    and leaving Fault is for an explicit reset (FaultExit).
///
/// In these two unpowered completion cases the joint may still be moving.
inline bool reached(DeviceState current, DeviceState target, FaultExit fault_exit = FaultExit::HOLD)
{
  if (current == target)
  {
    return true;
  }
  if (current == DeviceState::FAULT && target == DeviceState::SWITCH_ON_DISABLED)
  {
    return fault_exit == FaultExit::HOLD;
  }
  return target == DeviceState::QUICK_STOP_ACTIVE && !may_be_energised(current);
}

/// @brief A manufacturer-specific action: a raw controlword bit operation with its own
/// completion test. Generic, so any vendor's controlword bits 15..11 and any statusword
/// completion flag fit through it without this header knowing the vendor.
struct VendorAction
{
  uint16_t set_bits{ 0 };    ///< Controlword bits to raise.
  uint16_t clear_bits{ 0 };  ///< Controlword bits to lower (applied before set_bits).
  uint16_t done_mask{ 0 };   ///< Statusword bits that report completion.
  uint16_t done_value{ 0 };  ///< The value those bits take once the action is complete.
};

/// @brief Apply a vendor action to a controlword, leaving every other bit alone.
inline uint16_t apply_vendor_action(const VendorAction& action, uint16_t control_word)
{
  return static_cast<uint16_t>((control_word & static_cast<uint16_t>(~action.clear_bits)) | action.set_bits);
}

/// @brief Whether a statusword reports the vendor action as complete.
inline bool vendor_action_complete(const VendorAction& action, uint16_t statusword)
{
  return (statusword & action.done_mask) == action.done_value;
}

}  // namespace cia402
