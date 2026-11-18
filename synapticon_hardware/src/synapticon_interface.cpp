// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The ros2_control hardware for Synapticon drives: it hands the URDF joints to a HAL.

#include "synapticon_hardware/synapticon_interface.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/cia402/state_machine.hpp"
#include "synapticon_hardware/command_shaping.hpp"

namespace synapticon_hardware
{

namespace
{
using CallbackReturn = hardware_interface::CallbackReturn;
using hardware_interface::return_type;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("SynapticonSystemInterface");
}

/// The names in `names`, comma separated, or "none".
std::string list_of(const std::vector<std::string>& names)
{
  std::string text;
  for (const auto& name : names)
  {
    text += (text.empty() ? "" : ", ") + name;
  }
  return text.empty() ? "none" : text;
}
}  // namespace

CallbackReturn SynapticonSystemInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  // Only 'hal' is read here. Each HAL plugin parses its own parameters, as they differ per transport.
  hal_plugin_ = param_or(info_.hardware_parameters, "hal", "ethercat_soem");
  drives_ = build_drive_joint_map(info_);
  hal_config_ = build_hal_config(info_, drives_);

  const std::size_t joints = info_.joints.size();
  joint_states_.assign(joints, DriveState{});
  joint_commands_.assign(joints, DriveCommand{});
  clean_commands_.assign(joints, DriveCommand{});
  drive_states_.assign(drives_.drive_count(), DriveState{});
  drive_commands_.assign(drives_.drive_count(), DriveCommand{});
  drive_powered_.assign(drives_.drive_count(), 0u);
  joint_powered_.assign(joints, 0u);
  release_commands_.assign(joints, not_given_command());
  if (!transmissions_.load(info_, drives_))
  {
    return CallbackReturn::ERROR;
  }
  FallbackConfig fallbacks;
  try
  {
    fallbacks = read_command_fallbacks(info_);
  }
  catch (const std::invalid_argument& error)
  {
    RCLCPP_ERROR(logger(), "Command defaults: %s", error.what());
    return CallbackReturn::ERROR;
  }
  if (!check_joints_without_drive(fallbacks.own_torque_limit))
  {
    return CallbackReturn::ERROR;
  }
  command_fill_.configure(std::move(fallbacks));
  if (command_fill_.any_configured())
  {
    RCLCPP_INFO(logger(), "A command that leaves out stiffness, damping or max_joint_torque gets %s.",
                command_fill_.keeps_last_given() ? "the last given value" : "the configured default");
  }
  // A joint without a drive reports a healthy link. The others get the HAL's report from configure on.
  joint_link_health_.assign(joints, EthercatSlaveHealth{ al_state::OP, true });
  bus_link_health_ = healthy_link(drives_.drive_count()).bus;

  RCLCPP_INFO(logger(), "%zu joint(s) through %zu transmission(s) on %zu drive(s), HAL '%s'.", joints,
              transmissions_.size(), drives_.drive_count(), hal_plugin_.c_str());
  return CallbackReturn::SUCCESS;
}

bool SynapticonSystemInterface::check_joints_without_drive(const std::vector<bool>& own_torque_limit) const
{
  bool ok = true;
  for (std::size_t j = 0; j < info_.joints.size(); ++j)
  {
    if (drives_.has_drive(j))
    {
      continue;
    }
    const std::string& name = info_.joints[j].name;
    if (const auto names = drive_only_interfaces(info_.joints[j]); !names.empty())
    {
      RCLCPP_ERROR(logger(),
                   "Joint '%s' has no drive, but declares %s. Only a joint with a drive has these. Remove them, or "
                   "declare them on the joint of its drive.",
                   name.c_str(), list_of(names).c_str());
      ok = false;
    }
    if (j < own_torque_limit.size() && own_torque_limit[j])
    {
      RCLCPP_ERROR(logger(),
                   "Joint '%s' has no drive, but sets its own max_joint_torque. The torque limit is a setting of the "
                   "drive: set it on the joint of the drive, in the drive's units.",
                   name.c_str());
      ok = false;
    }
  }
  return ok;
}

std::shared_ptr<SynapticonHalI> SynapticonSystemInterface::make_hal()
{
  if (!hal_loader_)
  {
    hal_loader_ = std::make_unique<pluginlib::ClassLoader<SynapticonHalI>>("synapticon_hardware",
                                                                           "synapticon_hardware::SynapticonHalI");
  }
  try
  {
    return hal_loader_->createSharedInstance(hal_plugin_);
  }
  catch (const pluginlib::PluginlibException& error)
  {
    RCLCPP_ERROR(logger(), "No HAL plugin '%s'. Installed: %s. %s", hal_plugin_.c_str(),
                 list_of(hal_loader_->getDeclaredClasses()).c_str(), error.what());
    return nullptr;
  }
}

CallbackReturn SynapticonSystemInterface::drop_hal(const char* reason)
{
  RCLCPP_ERROR(logger(), "Configure failed: %s", reason);
  hal_.reset();
  return CallbackReturn::ERROR;
}

CallbackReturn SynapticonSystemInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // The HAL is made here, not in on_init(), so that deactivating and activating again keeps the bus.
  hal_ = make_hal();
  if (!hal_)
  {
    return CallbackReturn::ERROR;
  }
  if (!hal_->setup(hal_config_))
  {
    return drop_hal("the HAL refused its configuration.");
  }
  if (!hal_->connect())
  {
    return drop_hal("the HAL could not connect to the drives.");
  }
  if (!binding_.bind(info_, *this, joint_states_, joint_commands_, joint_link_health_, bus_link_health_))
  {
    return drop_hal("the declared interfaces cannot be bound.");
  }
  command_fill_.reset();
  // The health and the default states are visible before the first cycle.
  refresh_link_health();
  access_report_.state_writes(binding_.push_states());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SynapticonSystemInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!hal_)
  {
    RCLCPP_ERROR(logger(), "Cannot activate: configure has not succeeded.");
    return CallbackReturn::ERROR;
  }
  access_report_.reset();
  command_fill_.reset();
  if (!hal_->start())
  {
    RCLCPP_ERROR(logger(), "Activate failed: the HAL did not start the data exchange.");
    return CallbackReturn::ERROR;
  }
  // The data exchange runs now, so one read gives the measured position.
  if (!hal_->read(drive_states_))
  {
    RCLCPP_ERROR(logger(), "Activate failed: the first read after the start failed.");
    return CallbackReturn::ERROR;
  }
  states_from_drives();
  refresh_link_health();
  access_report_.state_writes(binding_.push_states());

  // No joint has a command yet, so each one holds where it is, and a drive that gets power later holds there too
  // instead of following an old command.
  seed_not_given(joint_commands_);
  hold_.reset(joint_states_);
  access_report_.command_seeds(binding_.push_command_seeds());

  // Activation never powers a drive. That takes an explicit request to the drive controller.
  RCLCPP_INFO(logger(), "Exchanging data with %zu drive(s). None has power.", drives_.drive_count());
  exchanging_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn SynapticonSystemInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // First stop read() and write(), then wind the HAL down.
  exchanging_ = false;
  access_report_.summarise();
  if (!hal_)
  {
    return CallbackReturn::SUCCESS;
  }
  const bool powered_off = hal_->disable();
  hal_->stop();
  if (!powered_off)
  {
    RCLCPP_ERROR(logger(), "Deactivate failed: not every drive confirmed it is off. Without process data, the drive's "
                           "own watchdog takes it down.");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SynapticonSystemInterface::on_error(const rclcpp_lifecycle::State& previous_state)
{
  // After an error ros2_control goes back to unconfigured without calling on_cleanup(). Letting go of the HAL here
  // frees the bus for the next configure.
  return on_cleanup(previous_state);
}

CallbackReturn SynapticonSystemInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  exchanging_ = false;
  if (hal_)
  {
    // Both succeed without doing anything when no drive has power or the data exchange does not run.
    hal_->disable();
    hal_->stop();
  }
  return on_cleanup(previous_state);
}

CallbackReturn SynapticonSystemInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  exchanging_ = false;
  // The bindings point into the buffers, so they go first.
  binding_.clear();
  if (hal_)
  {
    hal_->disconnect();
    hal_.reset();
  }
  return CallbackReturn::SUCCESS;
}

void SynapticonSystemInterface::states_from_drives()
{
  for (std::size_t drive = 0; drive < drive_states_.size(); ++drive)
  {
    const std::size_t j = drives_.drive_to_joint[drive];
    DriveState& joint = joint_states_[j];
    joint = drive_states_[drive];
    const cia402::DeviceState state = cia402::device_state(joint.status_word);
    joint.device_state = static_cast<std::uint8_t>(state);
    drive_powered_[drive] = cia402::may_be_energised(state) ? 1u : 0u;
    joint_powered_[j] = drive_powered_[drive];
  }
  transmissions_.propagate_states(drive_states_, joint_states_);
  transmissions_.spread_to_joints_without_drive(drive_powered_, joint_powered_);
}

void SynapticonSystemInterface::commands_to_drives()
{
  // A missing position becomes "hold", a missing velocity or effort becomes 0.
  // The rule for missing and invalid values may change with issue #88.
  hold_.update(joint_commands_, joint_states_, joint_powered_);
  sanitise_commands(joint_commands_, hold_.positions(), clean_commands_);
  command_fill_.fill_joints(clean_commands_);
  for (std::size_t drive = 0; drive < drive_commands_.size(); ++drive)
  {
    drive_commands_[drive] = clean_commands_[drives_.drive_to_joint[drive]];
  }
  // A drive behind a transmission takes the transmission's output instead. What it does not map gets the values of
  // the drive's own joint.
  transmissions_.apply_commands(clean_commands_, drive_commands_);
  command_fill_.fill_drives(drive_commands_, drives_.drive_to_joint);
}

void SynapticonSystemInterface::refresh_link_health()
{
  hal_->link_health(hal_link_health_);
  const std::size_t reported = std::min(drives_.drive_count(), hal_link_health_.drives.size());
  for (std::size_t drive = 0; drive < reported; ++drive)
  {
    joint_link_health_[drives_.drive_to_joint[drive]] = hal_link_health_.drives[drive];
  }
  bus_link_health_ = hal_link_health_.bus;
}

return_type SynapticonSystemInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (!exchanging_)
  {
    return return_type::OK;
  }
  if (!hal_ || !hal_->read(drive_states_))
  {
    return return_type::ERROR;
  }
  states_from_drives();
  refresh_link_health();
  access_report_.state_writes(binding_.push_states());
  return return_type::OK;
}

return_type SynapticonSystemInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (!exchanging_)
  {
    return return_type::OK;
  }
  if (!hal_)
  {
    return return_type::ERROR;
  }
  const auto refused = binding_.pull_commands();
  access_report_.command_reads(refused.setpoints, refused.actions);
  commands_to_drives();
  return hal_->write(drive_commands_) ? return_type::OK : return_type::ERROR;
}

return_type SynapticonSystemInterface::perform_command_mode_switch(const std::vector<std::string>& /*start_interfaces*/,
                                                                   const std::vector<std::string>& stop_interfaces)
{
  if (stop_interfaces.empty())
  {
    return return_type::OK;
  }
  for (std::size_t j = 0; j < release_commands_.size(); ++j)
  {
    const auto state = static_cast<cia402::DeviceState>(joint_states_[j].device_state);
    const bool powered = exchanging_ && drives_.has_drive(j) && cia402::may_be_energised(state);
    release_commands_[j].drive_action = powered ? DriveAction::DISABLE : DriveAction::NONE;
    const std::string action = info_.joints[j].name + "/drive_action";
    if (powered && std::find(stop_interfaces.begin(), stop_interfaces.end(), action) != stop_interfaces.end())
    {
      RCLCPP_WARN(logger(), "No controller holds '%s' any more while its drive may have power. Disabling the drive.",
                  action.c_str());
    }
  }
  if (const std::size_t refused = binding_.reset_released(stop_interfaces, release_commands_); refused != 0)
  {
    RCLCPP_ERROR(logger(), "%zu released command interface(s) could not be reset.", refused);
  }
  return return_type::OK;
}

}  // namespace synapticon_hardware

PLUGINLIB_EXPORT_CLASS(synapticon_hardware::SynapticonSystemInterface, hardware_interface::SystemInterface)
