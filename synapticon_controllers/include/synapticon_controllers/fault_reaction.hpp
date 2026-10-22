// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The drive fault reaction's runner: detects joints entering a condition, lets the policy
/// decide, and owns the latch.
///
/// Runs inside the drive controller's update, so it holds no ROS types and allocates nothing after
/// construction. Every joint the policy acts on stays latched until a clear the policy accepts, so
/// a drive fault that clears on its own cannot hand the drives back.

#include <cstddef>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

#include "synapticon_controllers/fault_policy.hpp"

namespace synapticon_controllers
{

/// @brief What a clear request did.
enum class ClearResult : uint8_t
{
  CLEARED,          ///< At least one latched joint was released.
  NOTHING_LATCHED,  ///< Accepted, but no targeted joint was latched.
  REFUSED,          ///< The policy refused it, with the reason in last_decision().
};

/// Runs the drive fault policy once per cycle and keeps the latch.
class FaultReaction
{
public:
  using DriveAction = synapticon_hardware::DriveAction;

  /// Sizes every buffer for `num_joints`, so the per-cycle calls allocate nothing.
  FaultReaction(std::unique_ptr<FaultPolicyI> policy, std::size_t num_joints)
    : policy_(std::move(policy)), previous_(num_joints, JointCondition::HEALTHY), latched_(num_joints, false)
  {
    entered_.reserve(num_joints);
    event_.triggered.reserve(num_joints);
    event_.conditions.reserve(num_joints);
    event_.acted_on.reserve(num_joints);
  }

  /// Apply this cycle's conditions. The joints entering a condition are handed to the policy and
  /// every joint it acts on is latched. Returns true when the policy requested anything, with
  /// `actions` holding the action per joint (NONE where none) and event() filled.
  ///
  /// Entry is edge-triggered: a joint that stays in its condition does not count again, so the
  /// healthy joints can be cleared and re-enabled by name while a dead one stays down. The first
  /// call only takes the baseline, a joint already in a condition then is not reacted to.
  bool step(const std::vector<JointCondition>& conditions, std::vector<DriveAction>& actions)
  {
    event_.clear();
    entered_.clear();
    const bool baseline = !has_baseline_;
    has_baseline_ = true;
    for (std::size_t j = 0; j < previous_.size() && j < conditions.size(); ++j)
    {
      if (entered_condition(j, conditions[j]) && !baseline)
      {
        entered_.push_back(j);
        event_.triggered.push_back(j);
        event_.conditions.push_back(conditions[j]);
      }
    }
    if (entered_.empty() || !policy_->step(conditions, entered_, actions, event_))
    {
      return false;
    }
    latch(event_.acted_on);
    return true;
  }

  /// Hand a named event from outside to the policy, for `joints` (every joint when empty). When
  /// the policy accepts it, `actions` and event() are filled as by step() and every joint acted
  /// on is latched. The decision is returned and kept in last_decision().
  PolicyDecision inject(std::string_view name, const std::vector<std::size_t>& joints,
                        const std::vector<JointCondition>& conditions, std::vector<DriveAction>& actions)
  {
    event_.clear();
    last_decision_ = policy_->on_event(name, joints, conditions, actions, event_);
    if (last_decision_.accepted)
    {
      latch(event_.acted_on);
    }
    return last_decision_;
  }

  /// Ask the policy to end the latch on `joints` (every joint when empty), given this cycle's
  /// conditions. A refusal leaves everything as it is, with the reason in last_decision().
  ClearResult clear(const std::vector<std::size_t>& joints, const std::vector<JointCondition>& conditions)
  {
    last_decision_ = policy_->may_clear(joints, conditions);
    if (!last_decision_.accepted)
    {
      return ClearResult::REFUSED;
    }
    bool was_latched = false;
    if (joints.empty())
    {
      for (std::size_t j = 0; j < latched_.size(); ++j)
      {
        was_latched = was_latched || latched_[j];
        latched_[j] = false;
      }
    }
    else
    {
      for (const std::size_t j : joints)
      {
        if (j < latched_.size())
        {
          was_latched = was_latched || latched_[j];
          latched_[j] = false;
        }
      }
    }
    return was_latched ? ClearResult::CLEARED : ClearResult::NOTHING_LATCHED;
  }

  bool latched(std::size_t joint) const
  {
    return joint < latched_.size() && latched_[joint];
  }

  bool any_latched() const
  {
    for (const bool l : latched_)
    {
      if (l)
      {
        return true;
      }
    }
    return false;
  }

  /// The last event step() filled. Valid after step() returned true.
  const ReactionEvent& event() const
  {
    return event_;
  }

  /// The policy's answer to the last clear or inject request.
  const PolicyDecision& last_decision() const
  {
    return last_decision_;
  }

  FaultPolicyI& policy()
  {
    return *policy_;
  }

private:
  /// Whether joint `j` entered a condition this cycle, remembering its condition for the next.
  ///
  /// While the joint is latched the remembered condition never returns to healthy, so a drive that
  /// flickers between a condition and healthy (a drive settling into its quick stop can report a
  /// following error on and off for a few cycles) gives one event, not several. After a clear, a
  /// return to healthy is remembered again and the next entry counts.
  bool entered_condition(std::size_t j, JointCondition condition)
  {
    const bool entered = condition != JointCondition::HEALTHY && previous_[j] == JointCondition::HEALTHY;
    if (!latched_[j] || condition != JointCondition::HEALTHY)
    {
      previous_[j] = condition;
    }
    return entered;
  }

  void latch(const std::vector<std::size_t>& joints)
  {
    for (const std::size_t j : joints)
    {
      if (j < latched_.size())
      {
        latched_[j] = true;
      }
    }
  }

  std::unique_ptr<FaultPolicyI> policy_;
  std::vector<JointCondition> previous_;
  std::vector<bool> latched_;
  std::vector<std::size_t> entered_;
  bool has_baseline_{ false };
  ReactionEvent event_;
  PolicyDecision last_decision_;
};

}  // namespace synapticon_controllers
