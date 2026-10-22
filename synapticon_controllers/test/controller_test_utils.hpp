// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Bringing a controller under test up and down over handles the test owns.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>

#include "controller_interface/controller_interface_params.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace synapticon_controllers::test
{

/// One interface declaration.
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

/// The handles behind the interfaces a controller under test claims. The test keeps them to set
/// states and read commands, the controller gets loans of them.
struct InterfaceHandles
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  std::vector<hardware_interface::StateInterface::SharedPtr> states;

  void add_command(const std::string& joint, const std::string& name, const std::string& data_type = "double")
  {
    commands.push_back(std::make_shared<hardware_interface::CommandInterface>(
        hardware_interface::InterfaceDescription(joint, interface_info(name, data_type))));
  }
  void add_state(const std::string& joint, const std::string& name, const std::string& data_type = "double")
  {
    states.push_back(std::make_shared<hardware_interface::StateInterface>(
        hardware_interface::InterfaceDescription(joint, interface_info(name, data_type))));
  }
};

/// Initialise `controller` as `name` with `parameters`.
template <typename Controller>
controller_interface::return_type init(Controller& controller, const std::string& name,
                                       const std::vector<rclcpp::Parameter>& parameters)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = name;
  params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  return controller.init(params);
}

/// Hand a configured `controller` loans of `handles` and activate it.
template <typename Controller>
controller_interface::CallbackReturn activate(Controller& controller, const InterfaceHandles& handles)
{
  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  for (const auto& handle : handles.commands)
  {
    loaned_commands.emplace_back(handle, nullptr);
  }
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  for (const auto& handle : handles.states)
  {
    loaned_states.emplace_back(handle, nullptr);
  }
  controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
  return controller.on_activate(rclcpp_lifecycle::State());
}

/// Initialise `controller` as `name` with `parameters`, configure it, hand it loans of `handles`
/// and activate it.
template <typename Controller>
void bring_up(Controller& controller, const std::string& name, const std::vector<rclcpp::Parameter>& parameters,
              const InterfaceHandles& handles)
{
  ASSERT_EQ(init(controller, name, parameters), controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(controller, handles), controller_interface::CallbackReturn::SUCCESS);
}

/// Deactivate `controller` and return its loans.
template <typename Controller>
void shut_down(Controller& controller)
{
  controller.on_deactivate(rclcpp_lifecycle::State());
  controller.release_interfaces();
}

/// One update of `controller`.
template <typename Controller>
void update(Controller& controller)
{
  ASSERT_EQ(controller.update(rclcpp::Time(0), rclcpp::Duration(0, 0)), controller_interface::return_type::OK);
}

}  // namespace synapticon_controllers::test
