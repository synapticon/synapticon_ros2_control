// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The drive fault policy interface: the part of the drive fault reaction that decides which
/// drives to stop or release. Conditions in, drive actions out, never a setpoint.
///
/// The policy runs inside the drive controller's update, so after configure() it must not
/// allocate. The runner latches every joint a policy acts on and only asks the policy whether a
/// clear may go ahead.

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "synapticon_hardware/drive_data.hpp"

namespace synapticon_controllers
{

/// @brief What the reaction sees of one joint each cycle.
enum class JointCondition : uint8_t
{
  HEALTHY = 0,
  /// The drive reports Fault or Fault Reaction Active: it is, or is about to be, de-energised.
  FAULTED = 1,
  /// The slave is not on the bus in OP: its data is stale and it takes no commands.
  UNREACHABLE = 2,
  /// The drive reports a following error without faulting, so it keeps driving toward a target it
  /// cannot follow.
  FOLLOWING_ERROR = 3,
};

/// @brief What the reaction did in one cycle. Index-based, the controller maps to names.
struct ReactionEvent
{
  /// The joints that entered a condition this cycle, with the condition each entered.
  std::vector<std::size_t> triggered;
  std::vector<JointCondition> conditions;
  /// The action requested, and the joints it was requested for. NONE with no joints on a clear.
  synapticon_hardware::DriveAction action{ synapticon_hardware::DriveAction::NONE };
  std::vector<std::size_t> acted_on;
  /// The injected event this reaction answered, a string literal of the policy's. Empty when the
  /// reaction answered a joint entering a condition.
  const char* event_name{ "" };

  void clear()
  {
    triggered.clear();
    conditions.clear();
    action = synapticon_hardware::DriveAction::NONE;
    acted_on.clear();
    event_name = "";
  }
};

/// @brief A policy's answer to a request from outside: accepted, or refused with a reason.
struct PolicyDecision
{
  bool accepted{ true };
  const char* reason{ "" };  ///< A string literal: the policy allocates nothing in the loop.
};

/// @brief What a drive fault policy implements.
///
/// configure() runs outside the real-time loop. The other calls run inside it, once per cycle
/// or on a request, with buffers the caller sized at activation.
class FaultPolicyI
{
public:
  using DriveAction = synapticon_hardware::DriveAction;

  virtual ~FaultPolicyI() = default;

  /// Take the joint names, for a policy that knows them. Returns false with `error` set when the
  /// policy cannot work with them.
  virtual bool configure(const std::vector<std::string>& joint_names, std::string& error) = 0;

  /// One cycle. `entered` lists the joints that entered a condition this cycle, `conditions` is
  /// every joint's condition. Fill `actions` (sized to the joints, NONE everywhere) with what to
  /// request, and `event` with what was decided. Returns true when anything was requested.
  virtual bool step(const std::vector<JointCondition>& conditions, const std::vector<std::size_t>& entered,
                    std::vector<DriveAction>& actions, ReactionEvent& event) = 0;

  /// A named event from outside, for `joints` (empty: every joint). The policy may refuse it.
  /// When accepted, `actions` and `event` are filled as in step().
  virtual PolicyDecision on_event(std::string_view name, const std::vector<std::size_t>& joints,
                                  const std::vector<JointCondition>& conditions, std::vector<DriveAction>& actions,
                                  ReactionEvent& event) = 0;

  /// Whether the latch on `joints` (empty: every joint) may be cleared now.
  virtual PolicyDecision may_clear(const std::vector<std::size_t>& joints,
                                   const std::vector<JointCondition>& conditions) const = 0;
};

}  // namespace synapticon_controllers
