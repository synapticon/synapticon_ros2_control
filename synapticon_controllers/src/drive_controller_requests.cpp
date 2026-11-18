// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The drive action services: check a request, then pass it to the control loop.

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drive_controller_detail.hpp"
#include "synapticon_controllers/drive_controller.hpp"

namespace synapticon_controllers
{
using namespace drive_controller_detail;

namespace
{
/// Which drives a request is for, as the answer names them.
std::string which_drives(const std::vector<std::string>& joints)
{
  return joints.empty() ? "all drives" : std::to_string(joints.size()) + " drive(s)";
}

constexpr const char* INACTIVE_MESSAGE = "Refused: the drive controller is not active.";

std::string latched_message(const std::string& joint)
{
  return "Refused: the fault reaction holds joint '" + joint + "'. Clear it with ~/clear_fault_reaction first.";
}
}  // namespace

std::optional<std::string> DriveController::first_unknown_joint(const std::vector<std::string>& joints) const
{
  const auto unknown = std::find_if(joints.begin(), joints.end(), [this](const std::string& joint) {
    return std::find(joint_names_.begin(), joint_names_.end(), joint) == joint_names_.end();
  });
  return unknown == joints.end() ? std::nullopt : std::optional<std::string>(*unknown);
}

void DriveController::post_request(DriveAction action, std::vector<std::string> joints)
{
  const auto slot = static_cast<std::size_t>(std::find(REQUESTABLE_ACTIONS.begin(), REQUESTABLE_ACTIONS.end(), action) -
                                             REQUESTABLE_ACTIONS.begin());
  std::lock_guard<std::mutex> guard(post_mutex_);
  auto request = std::make_shared<ActionRequest>();
  request->action = action;
  request->joints = std::move(joints);
  request->id = ++posted_request_ids_[slot];
  // A request that update() is not done with yet takes the new joints along, so that neither gets lost.
  const std::shared_ptr<ActionRequest> waiting = *request_slots_[slot].readFromNonRT();
  if (waiting && waiting->id != finished_request_ids_[slot].load())
  {
    request->joints = merged_joints(waiting->joints, request->joints);
  }
  request_slots_[slot].writeFromNonRT(request);
}

void DriveController::set_drive_action_callback(const std::shared_ptr<srv::SetDriveAction::Request> request,
                                                std::shared_ptr<srv::SetDriveAction::Response> response)
{
  response->success = false;
  if (!active_.load())
  {
    response->message = INACTIVE_MESSAGE;
    return;
  }
  const auto action = synapticon_hardware::action_from_string(request->action);
  if (!action || *action == DriveAction::NONE)
  {
    response->message = "Refused: there is no drive action '" + request->action + "'.";
    return;
  }
  if (!is_forwarded_action(*action))
  {
    response->message = "Refused: drive action '" + request->action + "' cannot be requested.";
    return;
  }
  if (const auto unknown = first_unknown_joint(request->dof_names))
  {
    response->message = unknown_joint_message(*unknown);
    return;
  }
  const auto latched = *action == DriveAction::ENABLE ? first_latched_joint(request->dof_names) : std::nullopt;
  if (latched)
  {
    response->message = latched_message(*latched);
    return;
  }
  post_request(*action, request->dof_names);
  response->success = true;
  response->message = "Posted '" + request->action + "' for " + which_drives(request->dof_names);
}

}  // namespace synapticon_controllers
