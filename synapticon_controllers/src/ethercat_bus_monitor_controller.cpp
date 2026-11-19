// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Reports the health of the EtherCAT bus on /diagnostics.

#include <algorithm>
#include <cstdio>
#include <string>
#include <tuple>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/logging.hpp"
#include "synapticon_controllers/ethercat_bus_monitor_controller.hpp"

namespace synapticon_controllers
{
using controller_interface::CallbackReturn;

namespace
{
using Level = diagnostic_msgs::msg::DiagnosticStatus;

/// The bus interfaces of the <gpio>, in the order of slots_. The names are those of the hardware interface.
enum Field : std::size_t
{
  LINK_UP,
  SLAVES_RESPONDING,
  AL_STATES,
  WORKING_COUNTER,
  WC_STATE,
};
constexpr std::array<const char*, 5> FIELD_NAMES = { "link_up", "slaves_responding", "al_states",
                                                     "domain_working_counter", "domain_wc_state" };

/// The working counter states of the process data domain, by value.
constexpr std::array<const char*, 3> WC_STATE_NAMES = { "ZERO", "INCOMPLETE", "COMPLETE" };
constexpr uint8_t WC_COMPLETE = 2;

/// The EtherCAT states (AL states) of the slaves, by bit.
constexpr std::array<const char*, 4> AL_STATE_NAMES = { "INIT", "PREOP", "SAFEOP", "OP" };
constexpr uint8_t AL_STATES_ALL_OP = 0x08;

std::string wc_state_name(uint8_t wc_state)
{
  return wc_state < WC_STATE_NAMES.size() ? WC_STATE_NAMES[wc_state] : "invalid " + std::to_string(wc_state);
}

/// The states that occur on the bus, for example "PREOP, OP". Bits that are no state are listed in hex.
std::string al_state_names(uint8_t al_states)
{
  std::string names;
  const auto add = [&names](const std::string& name) { names += (names.empty() ? "" : ", ") + name; };
  for (std::size_t bit = 0; bit < AL_STATE_NAMES.size(); ++bit)
  {
    if (al_states & (1u << bit))
    {
      add(AL_STATE_NAMES[bit]);
    }
  }
  if (const unsigned other = al_states & 0xF0u; other != 0)
  {
    char hex[8];
    std::snprintf(hex, sizeof(hex), "0x%02X", other);
    add(std::string("other bits ") + hex);
  }
  return names.empty() ? "none" : names;
}

}  // namespace

CallbackReturn EthercatBusMonitorController::on_init()
{
  try
  {
    auto_declare<std::string>("bus_name", "ethercat_bus");
    auto_declare<double>(CHECK_PERIOD_PARAMETER, DEFAULT_CHECK_PERIOD_S);
  }
  catch (const std::exception& error)
  {
    fprintf(stderr, "EthercatBusMonitorController: cannot declare its parameters: %s\n", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatBusMonitorController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  bus_name_ = get_node()->get_parameter("bus_name").as_string();
  if (bus_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter bus_name is empty. Set it to the <gpio> with the bus health.");
    return CallbackReturn::FAILURE;
  }
  const auto check_period = configured_check_period(get_node());
  if (!check_period)
  {
    return CallbackReturn::FAILURE;
  }
  try
  {
    diagnostics_.create(get_node(), *check_period);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot publish on /diagnostics: %s", error.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Reporting the bus health of <gpio> '%s' on /diagnostics.", bus_name_.c_str());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration EthercatBusMonitorController::command_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::NONE, {} };
}

controller_interface::InterfaceConfiguration EthercatBusMonitorController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config{ controller_interface::interface_configuration_type::INDIVIDUAL,
                                                       {} };
  for (const char* field : FIELD_NAMES)
  {
    config.names.push_back(bus_name_ + "/" + field);
  }
  return config;
}

CallbackReturn EthercatBusMonitorController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (std::size_t field = 0; field < FIELD_NAMES.size(); ++field)
  {
    const auto found = std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
      return interface.get_prefix_name() == bus_name_ && interface.get_interface_name() == FIELD_NAMES[field];
    });
    if (found == state_interfaces_.end())
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "No state interface %s/%s. The bus health comes from <gpio name=\"%s\"> in the URDF, which must "
                   "declare it.",
                   bus_name_.c_str(), FIELD_NAMES[field], bus_name_.c_str());
      return CallbackReturn::ERROR;
    }
    slots_[field] = static_cast<std::size_t>(found - state_interfaces_.begin());
  }
  // Forget the last report, so the first update reports.
  last_reported_.reset();
  diagnostics_.start({ bus_name_ }, [this](std::size_t, const std::optional<BusReading>& bus, auto& status) {
    describe(bus, status);
    status.hardware_id = bus_name_;
  });
  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatBusMonitorController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  diagnostics_.stop();
  return CallbackReturn::SUCCESS;
}

BusReading EthercatBusMonitorController::read_bus() const
{
  const auto read = [this](Field field, auto fallback) {
    return state_interfaces_[slots_[field]].get_optional<decltype(fallback)>().value_or(fallback);
  };
  return BusReading{ read(LINK_UP, false), read(SLAVES_RESPONDING, uint32_t{ 0 }), read(AL_STATES, uint8_t{ 0 }),
                     read(WORKING_COUNTER, uint32_t{ 0 }), read(WC_STATE, uint8_t{ 0 }) };
}

controller_interface::return_type EthercatBusMonitorController::update(const rclcpp::Time& time,
                                                                       const rclcpp::Duration& /*period*/)
{
  // Report when the link, the slaves, their states or the domain change, and at least once a second.
  const BusReading bus = read_bus();
  const std::tuple<bool, uint32_t, uint8_t, uint8_t> now{ bus.link_up, bus.slaves_responding, bus.al_states,
                                                          bus.wc_state };
  diagnostics_.update_latest([&bus](auto& latest) { latest[0] = bus; });
  if (now != last_reported_)
  {
    diagnostics_.push_change(0, time, bus);
  }
  last_reported_ = now;
  return controller_interface::return_type::OK;
}

void EthercatBusMonitorController::describe(const std::optional<BusReading>& bus,
                                            RealtimeDiagnostics<BusReading>::Status& status)
{
  // An error without link, a warning while not every slave answers or is in OP, OK otherwise.
  if (!bus)
  {
    status.summary(Level::STALE, "No reading of the bus yet");
    return;
  }
  if (!bus->link_up)
  {
    status.summary(Level::ERROR, "No EtherCAT link");
  }
  else
  {
    const bool all_op = bus->al_states == AL_STATES_ALL_OP;
    status.summary(bus->wc_state == WC_COMPLETE && all_op ? Level::OK : Level::WARN,
                   "EtherCAT link up, process data " + wc_state_name(bus->wc_state) +
                       (all_op ? "" : ", not every slave in OP"));
  }
  status.add(FIELD_NAMES[SLAVES_RESPONDING], std::to_string(bus->slaves_responding));
  status.add(FIELD_NAMES[AL_STATES], al_state_names(bus->al_states));
  status.add(FIELD_NAMES[WORKING_COUNTER], std::to_string(bus->working_counter));
  status.add(FIELD_NAMES[WC_STATE], wc_state_name(bus->wc_state));
}

}  // namespace synapticon_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(synapticon_controllers::EthercatBusMonitorController, controller_interface::ControllerInterface)
