// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The built-in drive fault policy: if any joint enters a condition, quick stop every joint.

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "synapticon_controllers/fault_policy.hpp"

namespace synapticon_controllers
{

/// @brief The simplest policy: any joint entering a condition quick stops every joint. It knows
/// nothing about the robot, so it cannot tell which joints may carry on, and it never refuses a
/// clear. The one event it accepts is "emergency_stop", which quick stops the joints it names
/// (every joint when it names none).
///
/// The action is quick stop and not disable on purpose: a quick stop leaves the drive energised
/// and holding, a disable de-energises it, which on a standing machine is often worse than the
/// fault itself. Quick stop needs the drive's quick stop option code 0x605A set to 6.
class QuickStopPolicy : public FaultPolicyI
{
public:
  bool configure(const std::vector<std::string>& /*joint_names*/, std::string& /*error*/) override
  {
    return true;
  }

  bool step(const std::vector<JointCondition>& /*conditions*/, const std::vector<std::size_t>& entered,
            std::vector<DriveAction>& actions, ReactionEvent& event) override
  {
    if (entered.empty())
    {
      return false;
    }
    event.action = DriveAction::QUICK_STOP;
    for (std::size_t j = 0; j < actions.size(); ++j)
    {
      actions[j] = event.action;
      event.acted_on.push_back(j);
    }
    return true;
  }

  PolicyDecision on_event(std::string_view name, const std::vector<std::size_t>& joints,
                          const std::vector<JointCondition>& /*conditions*/, std::vector<DriveAction>& actions,
                          ReactionEvent& event) override
  {
    if (name != EMERGENCY_STOP)
    {
      return { false, "the built-in policy knows only the event 'emergency_stop'" };
    }
    event.event_name = EMERGENCY_STOP;
    event.action = DriveAction::QUICK_STOP;
    if (joints.empty())
    {
      for (std::size_t j = 0; j < actions.size(); ++j)
      {
        actions[j] = event.action;
        event.acted_on.push_back(j);
      }
    }
    else
    {
      for (const std::size_t j : joints)
      {
        if (j < actions.size())
        {
          actions[j] = event.action;
          event.acted_on.push_back(j);
        }
      }
    }
    return { true, "" };
  }

  static constexpr const char* EMERGENCY_STOP = "emergency_stop";

  PolicyDecision may_clear(const std::vector<std::size_t>& /*joints*/,
                           const std::vector<JointCondition>& /*conditions*/) const override
  {
    return { true, "" };
  }
};

}  // namespace synapticon_controllers
