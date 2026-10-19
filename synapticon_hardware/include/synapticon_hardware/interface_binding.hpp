// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Connects each ros2_control interface of a description to the drive value it carries.

#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/hardware_info.hpp>

#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/ethercat_health.hpp"

namespace synapticon_hardware
{

/// @brief A name and the struct member it stands for.
///
/// Tables of these turn an interface name into a member pointer. Every table holds members of one type and one
/// struct, so a lookup needs no type switch. `Extra` carries whatever else a table needs per entry, for example
/// a unit.
template <typename FieldT, typename OwnerT, typename Extra = std::monostate>
struct NamedField
{
  const char* name;
  FieldT OwnerT::*field;
  Extra extra{};
};

/// The member that `name` stands for in `table`, or a null member pointer when the table lacks the name.
template <typename FieldT, typename OwnerT, typename Extra, std::size_t N>
FieldT OwnerT::*find_field(const NamedField<FieldT, OwnerT, Extra> (&table)[N], const std::string& name)
{
  for (const auto& entry : table)
  {
    if (name == entry.name)
    {
      return entry.field;
    }
  }
  return nullptr;
}

/// A state handle and the value it publishes. The value sits in a buffer that is never resized, so the pointer
/// stays valid while the binding exists.
template <typename T>
struct StateSink
{
  hardware_interface::StateInterface::SharedPtr handle;
  const T* source{ nullptr };
};

/// A command handle and the field it fills, the command side of StateSink.
template <typename T>
struct CommandSource
{
  hardware_interface::CommandInterface::SharedPtr handle;
  T* destination{ nullptr };
};

/// @brief Links the handles of a component to the drive data, once at configure, and copies between them every
/// cycle.
///
/// The interface name alone decides which drive value a handle carries. So the hardware exports exactly what the
/// description declares, whatever the control mode. A name with no matching drive value still gets its handle, but
/// nothing ever flows through it. bind() lists all such names in a single warning.
class InterfaceBinding
{
public:
  /// Command handles that could not be read in this cycle.
  struct RefusedReads
  {
    std::size_t setpoints{ 0 };
    std::size_t actions{ 0 };
  };

  /// Link every joint and gpio interface of `info` to its handle in `component` and to its value in `states`,
  /// `commands`, `slave_health` or `bus_health`. Those buffers must not move afterwards.
  /// False, with the reason logged, when a handle is missing or declares a data_type that does not fit.
  bool bind(const hardware_interface::HardwareInfo& info, hardware_interface::HardwareComponentInterface& component,
            std::vector<DriveState>& states, std::vector<DriveCommand>& commands,
            std::vector<EthercatSlaveHealth>& slave_health, EthercatBusHealth& bus_health);

  /// Forget all links. Call it before the linked buffers change.
  void clear();

  /// Publish every linked state value on its handle. Returns the number of handles that refused the value.
  std::size_t push_states() const;

  /// Copy every command handle into its field. When a drive action cannot be read, the field becomes NONE: the
  /// request is dropped, not kept for the next cycle.
  RefusedReads pull_commands() const;

  /// Put the current command fields on their handles, so that a controller which reads a command back starts from
  /// the hardware's value. Returns the number of handles that refused the value.
  std::size_t push_command_seeds() const;

  /// Names of the last bind() without a drive value, each as `<component>/<interface> (state)` or
  /// `<component>/<interface> (command)`.
  const std::vector<std::string>& unknown_interfaces() const
  {
    return unknown_;
  }

private:
  bool bind_joint_states(const hardware_interface::ComponentInfo& joint,
                         hardware_interface::HardwareComponentInterface& component, DriveState& state,
                         EthercatSlaveHealth& health);
  bool bind_joint_commands(const hardware_interface::ComponentInfo& joint,
                           hardware_interface::HardwareComponentInterface& component, DriveCommand& command);
  bool bind_gpio_states(const hardware_interface::ComponentInfo& gpio,
                        hardware_interface::HardwareComponentInterface& component, EthercatBusHealth& health);

  /// Warn once, in a framed block, about every name without a drive value.
  void report_unknown() const;

  std::vector<StateSink<double>> state_sinks_double_;
  std::vector<StateSink<uint16_t>> state_sinks_uint16_;
  std::vector<StateSink<int8_t>> state_sinks_int8_;
  std::vector<StateSink<uint8_t>> state_sinks_uint8_;
  std::vector<StateSink<uint32_t>> state_sinks_uint32_;
  std::vector<StateSink<bool>> state_sinks_bool_;
  std::vector<CommandSource<double>> command_sources_double_;
  std::vector<CommandSource<DriveAction>> command_sources_action_;
  std::vector<std::string> unknown_;
};

}  // namespace synapticon_hardware
