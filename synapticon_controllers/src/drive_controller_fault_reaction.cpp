// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief DriveController's drive fault reaction: the step per cycle, the service queries and the events.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
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

/// The name lists of the event message, which borrow their strings from the spare ones.
template <typename Msg>
std::array<std::vector<std::string>*, 4> name_lists(Msg& msg)
{
  return { &msg.triggered_by, &msg.condition_name, &msg.acted_on, &msg.latched_joints };
}

/// Give the strings of `list` back to `spare` and empty it. Moving a string hands over its buffer, so nothing
/// allocates or frees, as long as `spare` has the capacity.
void give_back(std::vector<std::string>& list, std::vector<std::string>& spare)
{
  for (auto& text : list)
  {
    spare.push_back(std::move(text));
  }
  list.clear();
}

/// Append `text` to `list` in a spare string. Without a spare string left, this allocates.
void append(std::vector<std::string>& list, std::vector<std::string>& spare, const char* text)
{
  if (spare.empty())
  {
    list.emplace_back(text);
    return;
  }
  std::string borrowed = std::move(spare.back());
  spare.pop_back();
  borrowed.assign(text);
  list.push_back(std::move(borrowed));
}
}  // namespace

void DriveController::prepare_reaction_event_message()
{
  if (!reaction_event_rt_)
  {
    return;
  }
  const std::size_t count = joint_names_.size();
  std::size_t longest = 0;
  for (const auto& name : joint_names_)
  {
    longest = std::max(longest, name.size());
  }
  for (const auto& condition : CONDITION_NAMES)
  {
    longest = std::max(longest, std::strlen(condition.name));
  }
  constexpr std::size_t NAME_LISTS = 4;
  reaction_event_rt_->lock();
  ReactionEventMsg& msg = reaction_event_rt_->msg_;
  spare_strings_.reserve(NAME_LISTS * count);
  for (auto* list : name_lists(msg))
  {
    give_back(*list, spare_strings_);
    list->reserve(count);
  }
  spare_strings_.resize(NAME_LISTS * count);
  for (auto& text : spare_strings_)
  {
    text.reserve(longest);
  }
  msg.condition.reserve(count);
  msg.error_code.reserve(count);
  msg.error_report.reserve(count);
  msg.event.reserve(64);
  msg.action.reserve(32);
  reaction_event_rt_->unlock();
}

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
  const auto answered = [this, id, &reason] {
    if (answered_query_id_.load() != id)
    {
      return false;
    }
    reason = answer_reason_.load();
    return true;
  };
  const auto timeout = answer_timeout(get_update_rate());
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (answered())
    {
      return answer_accepted_.load();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  reaction_inbox_.writeFromNonRT(nullptr);  // update() must not act on it any more
  // A cycle that took the query just before may still answer it. It ends within one control period.
  std::this_thread::sleep_for(control_period(get_update_rate()) + std::chrono::milliseconds(1));
  if (answered())
  {
    return answer_accepted_.load();
  }
  reason = "no decision within " + std::to_string(timeout.count()) + " ms: is the controller active?";
  return false;
}

void DriveController::publish_reaction_event(const rclcpp::Time& time, const ReactionEvent* fired)
{
  if (!reaction_event_rt_ || !reaction_event_rt_->trylock())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Fault reaction event not published: publisher busy.");
    return;
  }
  // Cleared rather than replaced, so the vectors and strings keep their capacity. The name lists return their strings
  // to the spare ones first.
  ReactionEventMsg& msg = reaction_event_rt_->msg_;
  msg.header.stamp = time;
  msg.event.clear();
  msg.action.clear();
  for (auto* list : name_lists(msg))
  {
    give_back(*list, spare_strings_);
  }
  msg.condition.clear();
  msg.error_code.clear();
  msg.error_report.clear();
  if (fired != nullptr)
  {
    msg.event = fired->event_name;
    msg.action = action_to_string(fired->action);
    for (std::size_t k = 0; k < fired->triggered.size(); ++k)
    {
      const std::size_t j = fired->triggered[k];
      append(msg.triggered_by, spare_strings_, joint_names_[j].c_str());
      msg.condition.push_back(static_cast<uint8_t>(fired->conditions[k]));
      append(msg.condition_name, spare_strings_, name_in(CONDITION_NAMES, fired->conditions[k], "unknown"));
      msg.error_code.push_back(state_interfaces_[slots_[j].error_code].get_optional<uint16_t>().value_or(0));
      msg.error_report.push_back(error_report_of(j));
    }
    for (const std::size_t j : fired->acted_on)
    {
      append(msg.acted_on, spare_strings_, joint_names_[j].c_str());
    }
  }
  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    if (reaction_->latched(j))
    {
      append(msg.latched_joints, spare_strings_, joint_names_[j].c_str());
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
