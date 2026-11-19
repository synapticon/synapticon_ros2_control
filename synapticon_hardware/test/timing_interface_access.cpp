// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Cost of the ros2_control interface layer at scale, with no bus involved.
///
/// `SynapticonSystemInterface` exports typed interfaces, which the framework backs with storage inside
/// each handle. Every state published and every command consumed therefore goes through a handle
/// access instead of a plain store or load. This tool measures what that costs.
///
/// Two measurements:
///
///  - Component: `read()` and `write()` on a `SynapticonSystemInterface` running the mock HAL, the
///    real code path with no transport in it. This is the absolute cost against the control period.
///  - Interface layer: the same population of handles accessed directly, against plain stores
///    and loads over the same data. This isolates the cost of the typed export.
///
/// A standalone executable, not a test: timing assertions are unstable on a non-realtime desktop,
/// and the useful output is the distribution.
///
/// Usage:
///   timing_interface_access [--joints N] [--cycles N]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "synapticon_hardware/synapticon_interface.hpp"
#include "description_builders.hpp"

namespace
{
using synapticon_hardware::SynapticonSystemInterface;
using synapticon_hardware::test::bus_gpio;
using synapticon_hardware::test::interface_info;
using synapticon_hardware::test::joint_info;
using synapticon_hardware::test::system_info;

// The interface set a drive declares in impedance mode.
const std::vector<std::pair<const char*, const char*>> COMMAND_INTERFACES = {
  { "position", "double" },  { "velocity", "double" }, { "effort", "double" },
  { "stiffness", "double" }, { "damping", "double" },  { "drive_action", "uint8" },
};
const std::vector<std::pair<const char*, const char*>> STATE_INTERFACES = {
  { "position", "double" },
  { "velocity", "double" },
  { "effort", "double" },
  { "dc_link_voltage", "double" },
  { "temperature_inverter", "double" },
  { "temperature_core", "double" },
  { "motor_current", "double" },
  { "joint_torque_demand", "double" },
  { "error_report", "double" },
  { "status_word", "uint16" },
  { "error_code", "uint16" },
  { "mode_of_operation", "int8" },
  { "slave_al_state", "uint8" },
  { "slave_online", "bool" },
};

hardware_interface::HardwareInfo make_hardware_info(std::size_t joint_count)
{
  auto info = system_info("timing_interface_access");
  for (std::size_t j = 0; j < joint_count; ++j)
  {
    hardware_interface::ComponentInfo joint = joint_info("joint_" + std::to_string(j + 1));
    for (const auto& [name, type] : COMMAND_INTERFACES)
    {
      joint.command_interfaces.push_back(interface_info(name, type));
    }
    for (const auto& [name, type] : STATE_INTERFACES)
    {
      joint.state_interfaces.push_back(interface_info(name, type));
    }
    info.joints.push_back(joint);
  }
  info.gpios.push_back(bus_gpio());
  return info;
}

/// Percentile of an already sorted sample.
double percentile(const std::vector<double>& sorted, double fraction)
{
  if (sorted.empty())
  {
    return 0.0;
  }
  const auto index = static_cast<std::size_t>(fraction * static_cast<double>(sorted.size() - 1));
  return sorted[index];
}

/// Print mean, p50, p99 and max of `samples` [us] on one line.
void report(const std::string& label, std::vector<double> samples)
{
  std::sort(samples.begin(), samples.end());
  double sum = 0.0;
  for (const double s : samples)
  {
    sum += s;
  }
  std::cout << std::left << std::setw(34) << label << std::right << std::fixed << std::setprecision(2) << " mean "
            << std::setw(9) << sum / static_cast<double>(samples.size()) << " | p50 " << std::setw(9)
            << percentile(samples, 0.50) << " | p99 " << std::setw(9) << percentile(samples, 0.99) << " | max "
            << std::setw(9) << samples.back() << "  [us]\n";
}

/// Time `body` over `cycles` iterations, returning the per-iteration cost in microseconds.
template <typename Body>
std::vector<double> measure(std::size_t cycles, Body&& body)
{
  std::vector<double> samples;
  samples.reserve(cycles);
  for (std::size_t i = 0; i < cycles; ++i)
  {
    const auto start = std::chrono::steady_clock::now();
    body();
    const auto stop = std::chrono::steady_clock::now();
    samples.push_back(std::chrono::duration<double, std::micro>(stop - start).count());
  }
  return samples;
}

/// The isolated interface layer: one handle per interface a joint declares, plus a parallel
/// array of plain values, so the same population can be driven either way.
struct InterfaceLayerBenchmark
{
  // Held by shared pointer, as the framework holds exported interfaces: a Handle keeps a pointer
  // into its own stored value, which a vector reallocation would leave dangling.
  std::vector<hardware_interface::StateInterface::SharedPtr> state_handles;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_handles;
  std::vector<double> plain_values;

  explicit InterfaceLayerBenchmark(std::size_t joint_count)
  {
    const auto info = make_hardware_info(joint_count);
    for (const auto& joint : info.joints)
    {
      for (const auto& si : joint.state_interfaces)
      {
        state_handles.push_back(std::make_shared<hardware_interface::StateInterface>(
            hardware_interface::InterfaceDescription(joint.name, si)));
      }
      for (const auto& ci : joint.command_interfaces)
      {
        command_handles.push_back(std::make_shared<hardware_interface::CommandInterface>(
            hardware_interface::InterfaceDescription(joint.name, ci)));
      }
    }
    for (const auto& gpio : info.gpios)
    {
      for (const auto& si : gpio.state_interfaces)
      {
        state_handles.push_back(std::make_shared<hardware_interface::StateInterface>(
            hardware_interface::InterfaceDescription(gpio.name, si)));
      }
    }
    plain_values.assign(state_handles.size() + command_handles.size(), 0.0);
  }

  /// What the typed export costs: a handle access per interface.
  void through_handles()
  {
    for (auto& handle : state_handles)
    {
      switch (handle->get_data_type())
      {
        case hardware_interface::HandleDataType::DOUBLE:
          std::ignore = handle->set_value(1.5);
          break;
        case hardware_interface::HandleDataType::UINT16:
          std::ignore = handle->set_value(static_cast<uint16_t>(0x0027));
          break;
        case hardware_interface::HandleDataType::INT8:
          std::ignore = handle->set_value(static_cast<int8_t>(-6));
          break;
        case hardware_interface::HandleDataType::UINT8:
          std::ignore = handle->set_value(static_cast<uint8_t>(8));
          break;
        case hardware_interface::HandleDataType::UINT32:
          std::ignore = handle->set_value(static_cast<uint32_t>(40));
          break;
        case hardware_interface::HandleDataType::BOOL:
          std::ignore = handle->set_value(true);
          break;
        default:
          break;
      }
    }
    for (auto& handle : command_handles)
    {
      if (handle->get_data_type() == hardware_interface::HandleDataType::BOOL)
      {
        std::ignore = handle->get_optional<bool>();
      }
      else
      {
        std::ignore = handle->get_optional<double>();
      }
    }
  }

  /// The baseline: a plain store or load per interface.
  void through_pointers()
  {
    double sink = 0.0;
    const std::size_t states = state_handles.size();
    for (std::size_t i = 0; i < states; ++i)
    {
      plain_values[i] = 1.5;
    }
    for (std::size_t i = 0; i < command_handles.size(); ++i)
    {
      sink += plain_values[states + i];
    }
    // Keeps the loads from being optimised away.
    plain_values[0] = sink * 0.0 + 1.5;
  }
};

/// The value following `flag` on the command line, or `fallback`.
std::size_t arg_value(int argc, char** argv, const char* flag, std::size_t fallback)
{
  for (int i = 1; i + 1 < argc; ++i)
  {
    if (std::strcmp(argv[i], flag) == 0)
    {
      return static_cast<std::size_t>(std::strtoul(argv[i + 1], nullptr, 10));
    }
  }
  return fallback;
}
}  // namespace

int main(int argc, char** argv)
{
  const std::size_t joints = arg_value(argc, argv, "--joints", 40);
  const std::size_t cycles = arg_value(argc, argv, "--cycles", 20000);
  rclcpp::init(argc, argv);

  const auto info = make_hardware_info(joints);
  const std::size_t state_count = joints * STATE_INTERFACES.size() + bus_gpio().state_interfaces.size();
  const std::size_t command_count = joints * COMMAND_INTERFACES.size();
  std::cout << "joints=" << joints << "  cycles=" << cycles << "  state interfaces=" << state_count
            << "  command interfaces=" << command_count << "\n\n";

  // Component: the real read() and write() path on the mock HAL.
  SynapticonSystemInterface system;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  if (system.on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
  {
    std::cerr << "on_init failed\n";
    return 1;
  }
  system.on_export_state_interfaces();
  system.on_export_command_interfaces();
  const rclcpp_lifecycle::State unconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  const rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  if (system.on_configure(unconfigured) != hardware_interface::CallbackReturn::SUCCESS ||
      system.on_activate(inactive) != hardware_interface::CallbackReturn::SUCCESS)
  {
    std::cerr << "configure/activate failed\n";
    return 1;
  }

  const rclcpp::Time now(0);
  const rclcpp::Duration period(0, 0);
  measure(cycles / 10, [&] {
    system.read(now, period);
    system.write(now, period);
  });  // warm up
  const auto read_samples = measure(cycles, [&] { system.read(now, period); });
  const auto write_samples = measure(cycles, [&] { system.write(now, period); });

  std::cout << "Component (mock HAL, no transport)\n";
  report("  read()", read_samples);
  report("  write()", write_samples);

  // Interface layer: typed handle access against plain stores and loads.
  InterfaceLayerBenchmark bench(joints);
  measure(cycles / 10, [&] { bench.through_handles(); });  // warm up
  const auto handle_samples = measure(cycles, [&] { bench.through_handles(); });
  const auto pointer_samples = measure(cycles, [&] { bench.through_pointers(); });

  std::cout << "\nInterface layer, one full cycle over every interface\n";
  report("  typed handle access", handle_samples);
  report("  plain pointer access", pointer_samples);

  system.on_deactivate(inactive);
  system.on_cleanup(inactive);
  rclcpp::shutdown();
  return 0;
}
