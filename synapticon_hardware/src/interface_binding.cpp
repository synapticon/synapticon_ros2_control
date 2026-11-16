// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Lookup tables from interface names to drive values, and the copying between handles and values.

#include "synapticon_hardware/interface_binding.hpp"

#include <type_traits>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "framed_report.hpp"

namespace synapticon_hardware
{

namespace
{
constexpr const char* LOGGER_NAME = "SynapticonSystemInterface";

// The tables below map interface names to members. A handle has one data_type, so each C++ type gets its own table.

// Commands of a joint. All drive requests share the one drive_action interface, so a joint gets one at a time.
constexpr NamedField<double, DriveCommand> JOINT_COMMAND_REAL[] = {
  { hardware_interface::HW_IF_POSITION, &DriveCommand::position },
  { hardware_interface::HW_IF_VELOCITY, &DriveCommand::velocity },
  { "effort", &DriveCommand::effort },
  { "max_joint_torque", &DriveCommand::max_joint_torque },
  { "stiffness", &DriveCommand::stiffness },
  { "damping", &DriveCommand::damping },
};
constexpr NamedField<DriveAction, DriveCommand> JOINT_COMMAND_ACTION[] = {
  { "drive_action", &DriveCommand::drive_action },
};

// States of a joint. device_state saves controllers from decoding the statusword themselves.
constexpr NamedField<double, DriveState> JOINT_STATE_REAL[] = {
  { hardware_interface::HW_IF_POSITION, &DriveState::position },
  { hardware_interface::HW_IF_VELOCITY, &DriveState::velocity },
  { hardware_interface::HW_IF_EFFORT, &DriveState::effort },
  { "joint_torque_demand", &DriveState::joint_torque_demand },
  { "motor_current", &DriveState::motor_current },
  { "dc_link_voltage", &DriveState::dc_link_voltage },
  { "temperature_core", &DriveState::temperature_core },
  { "temperature_inverter", &DriveState::temperature_inverter },
  { "error_report", &DriveState::error_report },
};
constexpr NamedField<uint16_t, DriveState> JOINT_STATE_WORD[] = {
  { "status_word", &DriveState::status_word },
  { "error_code", &DriveState::error_code },
};
constexpr NamedField<uint8_t, DriveState> JOINT_STATE_UINT8[] = {
  { "device_state", &DriveState::device_state },
};
constexpr NamedField<int8_t, DriveState> JOINT_STATE_INT8[] = {
  { "mode_of_operation", &DriveState::mode_of_operation },
};

// EtherCAT status. A joint carries the status of its drive, a <gpio> the status of the bus.
constexpr NamedField<bool, EthercatSlaveHealth> DRIVE_LINK_FLAG[] = {
  { "slave_online", &EthercatSlaveHealth::online },
};
constexpr NamedField<uint8_t, EthercatSlaveHealth> DRIVE_LINK_UINT8[] = {
  { "slave_al_state", &EthercatSlaveHealth::al_state },
};
constexpr NamedField<bool, EthercatBusHealth> BUS_FLAG[] = {
  { "link_up", &EthercatBusHealth::link_up },
};
constexpr NamedField<uint8_t, EthercatBusHealth> BUS_UINT8[] = {
  { "domain_wc_state", &EthercatBusHealth::domain_wc_state },
  { "al_states", &EthercatBusHealth::al_states },
};
constexpr NamedField<uint32_t, EthercatBusHealth> BUS_UINT32[] = {
  { "domain_working_counter", &EthercatBusHealth::domain_working_counter },
  { "slaves_responding", &EthercatBusHealth::slaves_responding },
};

/// The `data_type` that a handle for a value of type T has to declare.
template <typename T>
constexpr hardware_interface::HandleDataType::Value expected_handle_type()
{
  using hardware_interface::HandleDataType;
  if constexpr (std::is_same_v<T, double>)
  {
    return HandleDataType::DOUBLE;
  }
  else if constexpr (std::is_same_v<T, bool>)
  {
    return HandleDataType::BOOL;
  }
  // Handles know no enums, so a DriveAction goes as its uint8_t value.
  else if constexpr (std::is_same_v<T, uint8_t> || std::is_same_v<T, DriveAction>)
  {
    return HandleDataType::UINT8;
  }
  else if constexpr (std::is_same_v<T, int8_t>)
  {
    return HandleDataType::INT8;
  }
  else if constexpr (std::is_same_v<T, uint16_t>)
  {
    return HandleDataType::UINT16;
  }
  else if constexpr (std::is_same_v<T, uint32_t>)
  {
    return HandleDataType::UINT32;
  }
  else
  {
    return HandleDataType::UNKNOWN;
  }
}

/// True when `handle` declares the data_type that fits T. Otherwise logs which data_type to declare, so a wrong
/// declaration fails at configure and not in the control loop.
template <typename T>
bool check_handle_type(const hardware_interface::Handle& handle, const std::string& handle_name)
{
  const hardware_interface::HandleDataType expected{ expected_handle_type<T>() };
  if (handle.get_data_type() == expected)
  {
    return true;
  }
  RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
               "Interface '%s' has data_type=\"%s\", but its drive value is a \"%s\". Declare it in the URDF with "
               "data_type=\"%s\".",
               handle_name.c_str(), handle.get_data_type().to_string().c_str(), expected.to_string().c_str(),
               expected.to_string().c_str());
  return false;
}

/// Link `handle` to its value in `owner` when `table` knows `interface_name`, and set `matched` then. A name the
/// table lacks leaves `matched` alone, for the next table to try. False only for a data_type that does not fit.
template <typename FieldT, typename OwnerT, typename Extra, std::size_t N>
bool bind_state_field(const NamedField<FieldT, OwnerT, Extra> (&table)[N], const std::string& interface_name,
                      const std::string& handle_name, const OwnerT& owner,
                      const hardware_interface::StateInterface::SharedPtr& handle,
                      std::vector<StateSink<FieldT>>& sinks, bool& matched)
{
  FieldT OwnerT::*const field = find_field(table, interface_name);
  if (field == nullptr)
  {
    return true;
  }
  matched = true;
  if (!check_handle_type<FieldT>(*handle, handle_name))
  {
    return false;
  }
  sinks.push_back(StateSink<FieldT>{ handle, &(owner.*field) });
  return true;
}

/// bind_state_field() for command handles.
template <typename FieldT, typename OwnerT, typename Extra, std::size_t N>
bool bind_command_field(const NamedField<FieldT, OwnerT, Extra> (&table)[N], const std::string& interface_name,
                        const std::string& handle_name, OwnerT& owner,
                        const hardware_interface::CommandInterface::SharedPtr& handle,
                        std::vector<CommandSource<FieldT>>& sources, bool& matched)
{
  FieldT OwnerT::*const field = find_field(table, interface_name);
  if (field == nullptr)
  {
    return true;
  }
  matched = true;
  if (!check_handle_type<FieldT>(*handle, handle_name))
  {
    return false;
  }
  sources.push_back(CommandSource<FieldT>{ handle, &(owner.*field) });
  return true;
}

/// Publish each value of `sinks` on its handle. Returns the number of handles that refused.
template <typename T>
std::size_t push_sinks(const std::vector<StateSink<T>>& sinks)
{
  std::size_t refused = 0;
  for (const auto& sink : sinks)
  {
    if (!sink.handle->set_value(*sink.source))
    {
      ++refused;
    }
  }
  return refused;
}
}  // namespace

bool InterfaceBinding::bind(const hardware_interface::HardwareInfo& info,
                            hardware_interface::HardwareComponentInterface& component, std::vector<DriveState>& states,
                            std::vector<DriveCommand>& commands, std::vector<EthercatSlaveHealth>& slave_health,
                            EthercatBusHealth& bus_health)
{
  clear();
  const std::size_t joints = info.joints.size();
  if (states.size() != joints || commands.size() != joints || slave_health.size() != joints)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                 "Cannot bind %zu joint(s) to %zu state(s), %zu command(s) and %zu drive health entries.", joints,
                 states.size(), commands.size(), slave_health.size());
    return false;
  }
  bool ok = true;
  for (std::size_t j = 0; j < joints; ++j)
  {
    ok = bind_joint_states(info.joints[j], component, states[j], slave_health[j]) && ok;
    ok = bind_joint_commands(info.joints[j], component, commands[j]) && ok;
  }
  for (const auto& gpio : info.gpios)
  {
    ok = bind_gpio_states(gpio, component, bus_health) && ok;
  }
  if (!unknown_.empty())
  {
    report_unknown();
  }
  return ok;
}

void InterfaceBinding::report_unknown() const
{
  const std::string report =
      FramedReport("INTERFACES WITHOUT DRIVE DATA")
          .say("The description declares these interfaces, but no drive value has their name. They exist, so\n"
               "other components may claim them, yet no drive data ever flows through them. Check the names.\n")
          .list("Without drive data", unknown_)
          .close();
  RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "%s", report.c_str());
}

bool InterfaceBinding::bind_joint_states(const hardware_interface::ComponentInfo& joint,
                                         hardware_interface::HardwareComponentInterface& component, DriveState& state,
                                         EthercatSlaveHealth& health)
{
  bool ok = true;
  for (const auto& si : joint.state_interfaces)
  {
    const std::string handle_name = joint.name + "/" + si.name;
    if (!component.has_state(handle_name))
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "State interface '%s' is in the description, but has no handle.",
                   handle_name.c_str());
      ok = false;
      continue;
    }
    const auto handle = component.get_state_interface_handle(handle_name);
    bool matched = false;
    ok = bind_state_field(JOINT_STATE_REAL, si.name, handle_name, state, handle, state_sinks_double_, matched) && ok;
    if (!matched)
    {
      ok = bind_state_field(JOINT_STATE_WORD, si.name, handle_name, state, handle, state_sinks_uint16_, matched) && ok;
    }
    if (!matched)
    {
      ok = bind_state_field(JOINT_STATE_INT8, si.name, handle_name, state, handle, state_sinks_int8_, matched) && ok;
    }
    if (!matched)
    {
      ok = bind_state_field(JOINT_STATE_UINT8, si.name, handle_name, state, handle, state_sinks_uint8_, matched) && ok;
    }
    if (!matched)
    {
      ok = bind_state_field(DRIVE_LINK_UINT8, si.name, handle_name, health, handle, state_sinks_uint8_, matched) && ok;
    }
    if (!matched)
    {
      ok = bind_state_field(DRIVE_LINK_FLAG, si.name, handle_name, health, handle, state_sinks_bool_, matched) && ok;
    }
    if (!matched)
    {
      unknown_.push_back(handle_name + " (state)");
    }
  }
  return ok;
}

bool InterfaceBinding::bind_joint_commands(const hardware_interface::ComponentInfo& joint,
                                           hardware_interface::HardwareComponentInterface& component,
                                           DriveCommand& command)
{
  bool ok = true;
  for (const auto& ci : joint.command_interfaces)
  {
    const std::string handle_name = joint.name + "/" + ci.name;
    if (!component.has_command(handle_name))
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Command interface '%s' is in the description, but has no handle.",
                   handle_name.c_str());
      ok = false;
      continue;
    }
    const auto handle = component.get_command_interface_handle(handle_name);
    bool matched = false;
    ok = bind_command_field(JOINT_COMMAND_REAL, ci.name, handle_name, command, handle, command_sources_double_,
                            matched) &&
         ok;
    if (!matched)
    {
      ok = bind_command_field(JOINT_COMMAND_ACTION, ci.name, handle_name, command, handle, command_sources_action_,
                              matched) &&
           ok;
    }
    if (!matched)
    {
      unknown_.push_back(handle_name + " (command)");
    }
  }
  return ok;
}

bool InterfaceBinding::bind_gpio_states(const hardware_interface::ComponentInfo& gpio,
                                        hardware_interface::HardwareComponentInterface& component,
                                        EthercatBusHealth& health)
{
  // Every <gpio> shows the same bus status. Only the interface name matters.
  bool ok = true;
  for (const auto& si : gpio.state_interfaces)
  {
    const std::string handle_name = gpio.name + "/" + si.name;
    if (!component.has_state(handle_name))
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "State interface '%s' is in the description, but has no handle.",
                   handle_name.c_str());
      ok = false;
      continue;
    }
    const auto handle = component.get_state_interface_handle(handle_name);
    bool matched = false;
    ok = bind_state_field(BUS_FLAG, si.name, handle_name, health, handle, state_sinks_bool_, matched) && ok;
    if (!matched)
    {
      ok = bind_state_field(BUS_UINT8, si.name, handle_name, health, handle, state_sinks_uint8_, matched) && ok;
    }
    if (!matched)
    {
      ok = bind_state_field(BUS_UINT32, si.name, handle_name, health, handle, state_sinks_uint32_, matched) && ok;
    }
    if (!matched)
    {
      unknown_.push_back(handle_name + " (state)");
    }
  }
  return ok;
}

void InterfaceBinding::clear()
{
  unknown_.clear();
  state_sinks_double_.clear();
  state_sinks_uint16_.clear();
  state_sinks_int8_.clear();
  state_sinks_uint8_.clear();
  state_sinks_uint32_.clear();
  state_sinks_bool_.clear();
  command_sources_double_.clear();
  command_sources_action_.clear();
}

std::size_t InterfaceBinding::push_states() const
{
  std::size_t refused = 0;
  refused += push_sinks(state_sinks_double_);
  refused += push_sinks(state_sinks_uint16_);
  refused += push_sinks(state_sinks_int8_);
  refused += push_sinks(state_sinks_uint8_);
  refused += push_sinks(state_sinks_uint32_);
  refused += push_sinks(state_sinks_bool_);
  return refused;
}

InterfaceBinding::RefusedReads InterfaceBinding::pull_commands() const
{
  RefusedReads refused;
  for (const auto& source : command_sources_double_)
  {
    if (const auto value = source.handle->get_optional<double>())
    {
      *source.destination = *value;
    }
    else
    {
      ++refused.setpoints;
    }
  }
  // Read fresh in every cycle, so a HAL sees a new request in the cycle the controller sets it. A refused read keeps
  // the last value, since a change of value is what starts a request.
  for (const auto& source : command_sources_action_)
  {
    if (const auto value = source.handle->get_optional<uint8_t>())
    {
      *source.destination = static_cast<DriveAction>(*value);
    }
    else
    {
      ++refused.actions;
    }
  }
  return refused;
}

std::size_t InterfaceBinding::push_command_seeds() const
{
  std::size_t refused = 0;
  for (const auto& source : command_sources_double_)
  {
    if (!source.handle->set_value(*source.destination))
    {
      ++refused;
    }
  }
  for (const auto& source : command_sources_action_)
  {
    if (!source.handle->set_value(static_cast<uint8_t>(*source.destination)))
    {
      ++refused;
    }
  }
  return refused;
}

}  // namespace synapticon_hardware
