// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Builders for the ros2_control descriptions the hardware tests feed to SynapticonSystemInterface and its
/// helpers.

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/hardware_info.hpp>

namespace synapticon_hardware::test
{

/// One <state_interface> or <command_interface> declaration.
inline hardware_interface::InterfaceInfo interface_info(const std::string& name,
                                                        const std::string& data_type = "double")
{
  hardware_interface::InterfaceInfo info;
  info.name = name;
  info.data_type = data_type;
  info.size = 1;
  info.enable_limits = false;
  return info;
}

/// A joint with its command and state interfaces and its <param> entries.
inline hardware_interface::ComponentInfo joint_info(const std::string& name,
                                                    std::vector<hardware_interface::InterfaceInfo> commands = {},
                                                    std::vector<hardware_interface::InterfaceInfo> states = {},
                                                    const std::map<std::string, std::string>& parameters = {})
{
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  joint.command_interfaces = std::move(commands);
  joint.state_interfaces = std::move(states);
  joint.parameters.insert(parameters.begin(), parameters.end());
  return joint;
}

/// The position, velocity and effort state interfaces.
inline std::vector<hardware_interface::InterfaceInfo> motion_states()
{
  return { interface_info("position"), interface_info("velocity"), interface_info("effort") };
}

/// A transmission_interface/SimpleTransmission from `actuator` to `joint`.
inline hardware_interface::TransmissionInfo simple_transmission(const std::string& name, const std::string& joint,
                                                                const std::string& actuator, double reduction = 1.0,
                                                                double offset = 0.0)
{
  hardware_interface::TransmissionInfo tx;
  tx.name = name;
  tx.type = "transmission_interface/SimpleTransmission";
  hardware_interface::JointInfo joint_side;
  joint_side.name = joint;
  joint_side.role = "joint1";
  joint_side.mechanical_reduction = reduction;
  joint_side.offset = offset;
  tx.joints.push_back(joint_side);
  hardware_interface::ActuatorInfo actuator_side;
  actuator_side.name = actuator;
  actuator_side.role = "actuator1";
  tx.actuators.push_back(actuator_side);
  return tx;
}

/// The <gpio> with the bus health state interfaces.
inline hardware_interface::ComponentInfo bus_gpio()
{
  hardware_interface::ComponentInfo gpio;
  gpio.name = "ethercat_bus";
  gpio.type = "gpio";
  gpio.state_interfaces = {
    interface_info("link_up", "bool"),          interface_info("slaves_responding", "uint32"),
    interface_info("al_states", "uint8"),       interface_info("domain_working_counter", "uint32"),
    interface_info("domain_wc_state", "uint8"),
  };
  return gpio;
}

/// A SynapticonSystemInterface description named `name` on the mock HAL, without joints.
inline hardware_interface::HardwareInfo system_info(const std::string& name)
{
  hardware_interface::HardwareInfo info;
  info.name = name;
  info.type = "system";
  info.hardware_plugin_name = "synapticon_ros2_control/SynapticonSystemInterface";
  info.hardware_parameters["hal"] = "mock";
  return info;
}

}  // namespace synapticon_hardware::test
