// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The drive controller's lifecycle and its control loop step.

#include <algorithm>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "drive_controller_detail.hpp"
#include "synapticon_controllers/drive_controller.hpp"

namespace synapticon_controllers
{
using namespace drive_controller_detail;
using controller_interface::CallbackReturn;

namespace
{
/// The state interfaces of a joint besides the diagnostics fields, with their slot.
constexpr std::array<std::pair<const char*, std::size_t JointSlots::*>, 7> STATE_SLOTS = { {
    { "mode_of_operation", &JointSlots::mode_of_operation },
    { "device_state", &JointSlots::device_state },
    { "status_word", &JointSlots::status_word },
    { "error_code", &JointSlots::error_code },
    { "error_report", &JointSlots::error_report },
    { "slave_al_state", &JointSlots::al_state },
    { "slave_online", &JointSlots::online },
} };

/// The index of `joint`/`name` in `interfaces`, nothing when it is not there.
template <typename Interfaces>
std::optional<std::size_t> find_slot(const Interfaces& interfaces, const std::string& joint, const std::string& name)
{
  for (std::size_t k = 0; k < interfaces.size(); ++k)
  {
    if (interfaces[k].get_prefix_name() == joint && interfaces[k].get_interface_name() == name)
    {
      return k;
    }
  }
  return std::nullopt;
}
}  // namespace

CallbackReturn DriveController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", {});
    // Off by default, so the drive fault reaction is a deliberate choice.
    auto_declare<bool>("fault_reaction.enabled", false);
    auto_declare<double>(CHECK_PERIOD_PARAMETER, DEFAULT_CHECK_PERIOD_S);
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "DriveController failed to declare its parameters: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DriveController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  reaction_enabled_ = get_node()->get_parameter("fault_reaction.enabled").as_bool();
  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints: the 'joints' parameter lists none.");
    return CallbackReturn::FAILURE;
  }
  const auto check_period = configured_check_period(get_node());
  if (!check_period)
  {
    return CallbackReturn::FAILURE;
  }
  unwritten_actions_.assign(joint_names_.size(), DriveAction::NONE);
  if (!create_reaction())
  {
    return CallbackReturn::ERROR;
  }

  try
  {
    auto node = get_node();
    diagnostics_.create(node, *check_period);
    drive_status_pub_ = node->create_publisher<DriveStatusMsg>("~/drive_status", rclcpp::SystemDefaultsQoS());
    drive_status_timer_ = node->create_wall_timer(DRIVE_STATUS_PERIOD, [this]() { publish_drive_status_heartbeat(); });
    reaction_event_pub_ =
        node->create_publisher<ReactionEventMsg>("~/fault_reaction", rclcpp::QoS(1).reliable().transient_local());
    reaction_event_rt_ = std::make_unique<realtime_tools::RealtimePublisher<ReactionEventMsg>>(reaction_event_pub_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Creating the publishers failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  create_services();
  RCLCPP_INFO(get_node()->get_logger(), "Configured for %zu joint(s).", joint_names_.size());
  return CallbackReturn::SUCCESS;
}

void DriveController::create_services()
{
  using namespace std::placeholders;
  auto node = get_node();
  services_ = {
    node->create_service<srv::SetDriveAction>("~/set_drive_action",
                                              std::bind(&DriveController::set_drive_action_callback, this, _1, _2)),
    node->create_service<srv::ClearFaultReaction>(
        "~/clear_fault_reaction", std::bind(&DriveController::clear_fault_reaction_callback, this, _1, _2)),
    node->create_service<srv::InjectFaultEvent>("~/inject_fault_event",
                                                std::bind(&DriveController::inject_fault_event_callback, this, _1, _2)),
  };
}

controller_interface::InterfaceConfiguration DriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_)
  {
    config.names.push_back(joint + "/drive_action");
  }
  return config;
}

controller_interface::InterfaceConfiguration DriveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size() * (STATE_SLOTS.size() + DIAGNOSTICS_FIELDS.size()));
  for (const auto& joint : joint_names_)
  {
    for (const auto& [name, slot] : STATE_SLOTS)
    {
      config.names.push_back(joint + "/" + name);
    }
    for (const char* field : DIAGNOSTICS_FIELDS)
    {
      config.names.push_back(joint + "/" + field);
    }
  }
  return config;
}

bool DriveController::resolve_slots()
{
  const auto missing = [this](const char* kind, const std::string& joint, const char* name) {
    RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' has no %s interface '%s'. Declare it in the URDF.",
                 joint.c_str(), kind, name);
    return false;
  };
  slots_.assign(joint_names_.size(), JointSlots{});
  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    const std::string& joint = joint_names_[j];
    for (const auto& [name, slot] : STATE_SLOTS)
    {
      const auto found = find_slot(state_interfaces_, joint, name);
      if (!found)
      {
        return missing("state", joint, name);
      }
      slots_[j].*slot = *found;
    }
    for (std::size_t f = 0; f < DIAGNOSTICS_FIELDS.size(); ++f)
    {
      const auto found = find_slot(state_interfaces_, joint, DIAGNOSTICS_FIELDS[f]);
      if (!found)
      {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "State interface '%s/%s' was not handed to this controller. The URDF must declare it, and "
                     "the drive's slave_config must map it (esi_to_slave_config.py writes such configs).",
                     joint.c_str(), DIAGNOSTICS_FIELDS[f]);
        return false;
      }
      slots_[j].diagnostics[f] = *found;
    }
    const auto action = find_slot(command_interfaces_, joint, "drive_action");
    if (!action)
    {
      return missing("command", joint, "drive_action");
    }
    slots_[j].drive_action = *action;
  }
  return true;
}

CallbackReturn DriveController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!resolve_slots())
  {
    return CallbackReturn::ERROR;
  }

  {
    std::lock_guard<std::mutex> guard(drive_status_mutex_);
    drive_status_msg_.drives.assign(joint_names_.size(), synapticon_control_msgs::msg::DriveStatus{});
    for (std::size_t j = 0; j < joint_names_.size(); ++j)
    {
      drive_status_msg_.drives[j].name = joint_names_[j];
    }
  }

  // The reaction keeps its latch from the last activation. Only a configure starts a new one.
  reset_reaction();
  prepare_reaction_event_message();
  RCLCPP_INFO(get_node()->get_logger(), "Drive fault reaction %s.",
              reaction_enabled_ ? "enabled: a joint entering Fault, leaving the bus or reporting a "
                                  "following error quick-stops every joint and latches" :
                                  "disabled (fault_reaction.enabled)");

  // Activation energises nothing: a request posted while inactive, or not yet written at the deactivation, is dropped.
  for (std::size_t s = 0; s < NUM_REQUEST_SLOTS; ++s)
  {
    finished_request_ids_[s] = posted_request_ids_[s].load();
    done_by_[s].assign(joint_names_.size(), 0);
  }
  unwritten_actions_.assign(joint_names_.size(), DriveAction::NONE);

  // An empty last report makes the first update publish.
  current_states_.assign(joint_names_.size(), ReportedState::UNDEFINED);
  last_reported_states_.clear();
  reports_.assign(joint_names_.size(), DriveReport{});
  diagnostics_.start(
      joint_names_,
      [this](std::size_t joint, const std::optional<DriveReport>& report, auto& status) {
        describe_drive(report, joint, status);
      },
      [this](std::size_t joint, const rclcpp::Time& time, const DriveReport& report) {
        publish_drive_status_change(joint, time, report);
      });
  refused_writes_total_ = 0;
  overruled_total_ = 0;
  active_.store(true);
  return CallbackReturn::SUCCESS;
}

bool DriveController::create_reaction()
{
  auto policy = std::make_unique<QuickStopPolicy>();
  std::string policy_error;
  if (!policy->configure(joint_names_, policy_error))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Drive fault policy: %s", policy_error.c_str());
    return false;
  }
  reaction_ = std::make_unique<FaultReaction>(std::move(policy), joint_names_.size());
  latched_flags_ = std::make_unique<std::atomic<bool>[]>(joint_names_.size());
  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    latched_flags_[j].store(false);
  }
  return true;
}

void DriveController::reset_reaction()
{
  const std::size_t count = joint_names_.size();
  joint_conditions_.assign(count, JointCondition::HEALTHY);
  reaction_wants_.assign(count, DriveAction::NONE);
  for (std::size_t j = 0; j < count; ++j)
  {
    latched_flags_[j].store(reaction_->latched(j));
  }
  queried_joints_.clear();
  queried_joints_.reserve(count);
  reaction_inbox_.writeFromNonRT(nullptr);
  latched_enables_total_ = 0;
}

CallbackReturn DriveController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  active_.store(false);
  diagnostics_.stop();
  if (refused_writes_total_ != 0)
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "%lu drive_action write(s) were refused during this activation. Expect these only "
                "when the hardware component is run asynchronously.",
                static_cast<unsigned long>(refused_writes_total_));
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type DriveController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  read_reported_states();
  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    joint_conditions_[j] = condition_of(j, current_states_[j]);
  }
  if (reaction_enabled_)
  {
    step_reaction(time);
  }
  // Every change of a joint's state goes out on /diagnostics and ~/drive_status, and the latest readings once a
  // period.
  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    fill_report(j, reports_[j]);
    if (j >= last_reported_states_.size() || current_states_[j] != last_reported_states_[j])
    {
      diagnostics_.push_change(j, time, reports_[j]);
    }
  }
  diagnostics_.update_latest([this](auto& latest) {
    for (std::size_t j = 0; j < latest.size(); ++j)
    {
      latest[j] = reports_[j];
    }
  });
  last_reported_states_ = current_states_;
  dispatch_actions();
  return controller_interface::return_type::OK;
}

void DriveController::dispatch_actions()
{
  // One copy of every source's request per cycle: reading a buffer takes its lock and may swap its storage.
  RequestSnapshot requests;
  for (std::size_t s = 0; s < NUM_REQUEST_SLOTS; ++s)
  {
    const auto* request = request_slots_[s].readFromRT();
    if (request && *request && (*request)->id != finished_request_ids_[s].load())
    {
      requests[s] = *request;
    }
  }

  // A request is dropped once every joint it names is done with it. Starting from true, a request that names no
  // configured joint cannot linger.
  std::array<bool, NUM_REQUEST_SLOTS> done;
  done.fill(true);
  CycleRefusals refusals;

  // Each joint has one action slot. Several requests for it resolve by precedence, the reaction's action included,
  // and a latched reaction first removes any enable.
  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    SourceActions requested = requested_of(requests, joint_names_[j]);
    // A request the joint is done with, or one merged into it, never writes there again.
    for (std::size_t s = 0; s < NUM_REQUEST_SLOTS; ++s)
    {
      if (requested[s] != DriveAction::NONE && done_by_[s][j] >= requests[s]->first_id)
      {
        requested[s] = DriveAction::NONE;
      }
    }
    if (reaction_enabled_ && reaction_->latched(j))
    {
      for (DriveAction& action : requested)
      {
        if (action == DriveAction::ENABLE)
        {
          action = DriveAction::NONE;
          ++refusals.latched_enables;
        }
      }
    }
    const DriveAction reaction = reaction_wants_[j];

    // An enable that waits for its retry is dropped once the joint is latched, or the reaction or a request stops it,
    // so that it is never written after the decision to stop.
    if (unwritten_actions_[j] == DriveAction::ENABLE)
    {
      if ((reaction_enabled_ && reaction_->latched(j)) || precedence(reaction) > precedence(DriveAction::ENABLE))
      {
        unwritten_actions_[j] = DriveAction::NONE;
        ++refusals.latched_enables;
      }
      else if (precedence(strongest(requested)) > precedence(DriveAction::ENABLE))
      {
        unwritten_actions_[j] = DriveAction::NONE;  // its request still takes part below, and counts as overruled
      }
    }

    // An unwritten action holds the slot until it is written, so a newer request cannot replace it.
    const bool held = unwritten_actions_[j] != DriveAction::NONE;
    if (!held)
    {
      const DriveAction chosen = strongest(requested);
      unwritten_actions_[j] = precedence(reaction) > precedence(chosen) ? reaction : chosen;
      const std::size_t candidates = count_active(requested) + (reaction != DriveAction::NONE ? 1 : 0);
      refusals.overruled += candidates > 0 ? candidates - 1 : 0;
    }

    const DriveAction written = unwritten_actions_[j];
    const bool refused = write_unwritten_action(j);
    refusals.refused_writes += refused ? 1 : 0;
    for (std::size_t s = 0; s < NUM_REQUEST_SLOTS; ++s)
    {
      if (requested[s] == DriveAction::NONE)
      {
        continue;
      }
      const bool done_here = done_with_joint(requested[s], written, held, refused);
      done[s] = done[s] && done_here;
      if (done_here)
      {
        done_by_[s][j] = requests[s]->id;
      }
    }
    if (reaction != DriveAction::NONE && done_with_joint(reaction, written, held, refused))
    {
      reaction_wants_[j] = DriveAction::NONE;
    }
  }
  report_refusals(refusals);
  for (std::size_t s = 0; s < NUM_REQUEST_SLOTS; ++s)
  {
    if (requests[s] && done[s])
    {
      finished_request_ids_[s].store(requests[s]->id);
    }
  }
}

bool DriveController::write_unwritten_action(std::size_t joint)
{
  if (!command_interfaces_[slots_[joint].drive_action].set_value(static_cast<uint8_t>(unwritten_actions_[joint])))
  {
    return true;
  }
  unwritten_actions_[joint] = DriveAction::NONE;
  return false;
}

void DriveController::report_refusals(const CycleRefusals& refusals)
{
  const auto logger = get_node()->get_logger();
  if (refusals.refused_writes > 0)
  {
    refused_writes_total_ += refusals.refused_writes;
    RCLCPP_ERROR_THROTTLE(logger, throttle_clock_, LOG_THROTTLE_MS,
                          "%zu drive_action write(s) refused this cycle (%lu since activation). The request is "
                          "held and retried on the next cycle, not lost.",
                          refusals.refused_writes, static_cast<unsigned long>(refused_writes_total_));
  }
  if (refusals.overruled > 0)
  {
    overruled_total_ += refusals.overruled;
    RCLCPP_WARN_THROTTLE(logger, throttle_clock_, LOG_THROTTLE_MS,
                         "%zu drive_action request(s) overruled this cycle by a higher-precedence one, disable over "
                         "enable over reset_fault (%lu since activation). They are not executed.",
                         refusals.overruled, static_cast<unsigned long>(overruled_total_));
  }
  if (refusals.latched_enables > 0)
  {
    latched_enables_total_ += refusals.latched_enables;
    RCLCPP_WARN_THROTTLE(logger, throttle_clock_, LOG_THROTTLE_MS,
                         "%zu enable request(s) refused this cycle by the latched drive fault reaction (%lu since "
                         "activation). Clear it with ~/clear_fault_reaction first.",
                         refusals.latched_enables, static_cast<unsigned long>(latched_enables_total_));
  }
}

}  // namespace synapticon_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(synapticon_controllers::DriveController, controller_interface::ControllerInterface)
