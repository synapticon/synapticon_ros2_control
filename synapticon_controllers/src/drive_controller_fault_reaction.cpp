// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief DriveController's drive fault reaction: the step per cycle, the service queries and the events.

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"

#include "drive_controller_detail.hpp"
#include "synapticon_controllers/drive_controller.hpp"

namespace synapticon_controllers
{
using namespace drive_controller_detail;
using synapticon_hardware::action_to_string;

namespace
{
constexpr const char* DISABLED_MESSAGE =
    "Refused: the fault reaction is off. Set fault_reaction.enabled to turn it on.";
}  // namespace

void DriveController::step_reaction(const rclcpp::Time& time)
{
  if (const auto* query = reaction_inbox_.readFromRT(); query && *query && (*query)->id != answered_query_id_.load())
  {
    answer_reaction_query(time, **query);
  }

  if (reaction_->step(joint_conditions_, reaction_wants_))
  {
    const ReactionEvent& event = reaction_->event();
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Drive fault reaction: '%s' entered condition %u, requesting '%s' on %zu joint(s) and "
                 "latching them. Clear with ~/clear_fault_reaction once the robot is ready.",
                 joint_names_[event.triggered.front()].c_str(), static_cast<unsigned>(event.conditions.front()),
                 action_to_string(event.action), event.acted_on.size());
    publish_reaction_event(time, &event);
  }

  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    latched_flags_[j].store(reaction_->latched(j));
  }
}

void DriveController::answer_reaction_query(const rclcpp::Time& time, const ReactionQuery& query)
{
  queried_joints_.clear();
  for (std::size_t j = 0; j < joint_names_.size() && !query.joints.empty(); ++j)
  {
    if (names_joint(query.joints, joint_names_[j]))
    {
      queried_joints_.push_back(j);
    }
  }
  const auto logger = get_node()->get_logger();
  bool accepted = true;
  const char* reason = "";
  if (query.kind == ReactionQuery::Kind::CLEAR)
  {
    const ClearResult result = reaction_->clear(queried_joints_, joint_conditions_);
    if (result == ClearResult::CLEARED)
    {
      RCLCPP_WARN(logger, "Drive fault reaction cleared.");
      publish_reaction_event(time, nullptr);
    }
    else if (result == ClearResult::NOTHING_LATCHED)
    {
      reason = "nothing was latched";
    }
    else
    {
      accepted = false;
      reason = reaction_->last_decision().reason;
      RCLCPP_WARN(logger, "Drive fault reaction not cleared: %s.", reason);
    }
  }
  else
  {
    const PolicyDecision decision = reaction_->inject(query.event, queried_joints_, joint_conditions_, reaction_wants_);
    accepted = decision.accepted;
    reason = decision.reason;
    if (!accepted)
    {
      RCLCPP_WARN(logger, "Drive fault reaction: event '%s' refused: %s.", query.event.c_str(), reason);
    }
    else
    {
      const ReactionEvent& event = reaction_->event();
      RCLCPP_ERROR(logger,
                   "Drive fault reaction: event '%s' accepted, requesting '%s' on %zu joint(s) and latching "
                   "them. Clear with ~/clear_fault_reaction once the robot is ready.",
                   event.event_name, action_to_string(event.action), event.acted_on.size());
      publish_reaction_event(time, &event);
    }
  }
  answer_accepted_.store(accepted);
  answer_reason_.store(reason);
  answered_query_id_.store(query.id);
}

bool DriveController::ask_update_loop(ReactionQuery query, std::string& reason)
{
  const std::lock_guard<std::mutex> lock(reaction_call_mutex_);
  const uint64_t id = ++last_query_id_;
  query.id = id;
  reaction_inbox_.writeFromNonRT(std::make_shared<ReactionQuery>(std::move(query)));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ANSWER_TIMEOUT_MS);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (answered_query_id_.load() == id)
    {
      reason = answer_reason_.load();
      return answer_accepted_.load();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  reaction_inbox_.writeFromNonRT(nullptr);  // update() must not act on it any more
  reason = "no decision within " + std::to_string(ANSWER_TIMEOUT_MS) + " ms: is the controller active?";
  return false;
}

void DriveController::publish_reaction_event(const rclcpp::Time& time, const ReactionEvent* fired)
{
  if (!reaction_event_rt_ || !reaction_event_rt_->trylock())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Fault reaction event not published: publisher busy.");
    return;
  }
  // Cleared rather than replaced, so the vectors keep their capacity.
  ReactionEventMsg& msg = reaction_event_rt_->msg_;
  msg.header.stamp = time;
  msg.event.clear();
  msg.action.clear();
  msg.triggered_by.clear();
  msg.condition.clear();
  msg.condition_name.clear();
  msg.error_code.clear();
  msg.error_report.clear();
  msg.acted_on.clear();
  msg.latched_joints.clear();
  if (fired != nullptr)
  {
    msg.event = fired->event_name;
    msg.action = action_to_string(fired->action);
    for (std::size_t k = 0; k < fired->triggered.size(); ++k)
    {
      const std::size_t j = fired->triggered[k];
      msg.triggered_by.push_back(joint_names_[j]);
      msg.condition.push_back(static_cast<uint8_t>(fired->conditions[k]));
      msg.condition_name.push_back(name_in(CONDITION_NAMES, fired->conditions[k], "unknown"));
      msg.error_code.push_back(state_interfaces_[slots_[j].error_code].get_optional<uint16_t>().value_or(0));
      msg.error_report.push_back(error_report_of(j));
    }
    for (const std::size_t j : fired->acted_on)
    {
      msg.acted_on.push_back(joint_names_[j]);
    }
  }
  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    if (reaction_->latched(j))
    {
      msg.latched_joints.push_back(joint_names_[j]);
    }
  }
  msg.latched = reaction_->any_latched();
  reaction_event_rt_->unlockAndPublish();
}

std::optional<std::string> DriveController::first_latched_joint(const std::vector<std::string>& joints) const
{
  if (!reaction_enabled_ || !latched_flags_)
  {
    return std::nullopt;
  }
  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    if (latched_flags_[j].load() && names_joint(joints, joint_names_[j]))
    {
      return joint_names_[j];
    }
  }
  return std::nullopt;
}

void DriveController::clear_fault_reaction_callback(const std::shared_ptr<srv::ClearFaultReaction::Request> request,
                                                    std::shared_ptr<srv::ClearFaultReaction::Response> response)
{
  response->success = false;
  if (!reaction_enabled_)
  {
    response->message = DISABLED_MESSAGE;
    return;
  }
  if (const auto unknown = first_unknown_joint(request->dof_names))
  {
    response->message = unknown_joint_message(*unknown);
    return;
  }
  ReactionQuery query;
  query.kind = ReactionQuery::Kind::CLEAR;
  query.joints = request->dof_names;
  std::string reason;
  response->success = ask_update_loop(std::move(query), reason);
  if (!response->success)
  {
    response->message = "Refused: " + reason;
  }
  else
  {
    response->message = reason.empty() ? "Fault reaction cleared" : "Cleared: " + reason;
  }
}

void DriveController::inject_fault_event_callback(const std::shared_ptr<srv::InjectFaultEvent::Request> request,
                                                  std::shared_ptr<srv::InjectFaultEvent::Response> response)
{
  response->accepted = false;
  if (!reaction_enabled_)
  {
    response->message = DISABLED_MESSAGE;
    return;
  }
  if (request->event.empty())
  {
    response->message = "Refused: the request names no event.";
    return;
  }
  if (const auto unknown = first_unknown_joint(request->dof_names))
  {
    response->message = unknown_joint_message(*unknown);
    return;
  }
  ReactionQuery query;
  query.kind = ReactionQuery::Kind::EVENT;
  query.event = request->event;
  query.joints = request->dof_names;
  std::string reason;
  response->accepted = ask_update_loop(std::move(query), reason);
  response->message =
      response->accepted ? "The fault policy accepted event '" + request->event + "'" : "Refused: " + reason;
}

}  // namespace synapticon_controllers
