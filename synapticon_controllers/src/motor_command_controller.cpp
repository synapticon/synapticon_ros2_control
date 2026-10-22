// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Passes impedance commands from a topic to the joints.

#include "synapticon_controllers/motor_command_controller.hpp"

#include <algorithm>
#include <limits>

#include <pluginlib/class_list_macros.hpp>

namespace synapticon_controllers
{
using controller_interface::CallbackReturn;

namespace
{
/// The command interfaces of a joint, in the order of Setpoint.
constexpr std::array<const char*, 5> INTERFACES = { "position", "velocity", "effort", "stiffness", "damping" };

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

CallbackReturn MotorCommandController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", {});
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "MotorCommandController: cannot declare its parameters: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorCommandController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter joints is empty. List the joints to command.");
    return CallbackReturn::ERROR;
  }
  subscription_ = get_node()->create_subscription<MotorCommand>(
      "~/commands", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<MotorCommand> msg) { on_command(msg); });
  RCLCPP_INFO(get_node()->get_logger(), "Passing MotorCommand to %zu joint(s).", joints_.size());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MotorCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_)
  {
    for (const char* name : INTERFACES)
    {
      config.names.push_back(joint + "/" + name);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration MotorCommandController::state_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::NONE, {} };
}

CallbackReturn MotorCommandController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  slots_.assign(joints_.size(), {});
  for (std::size_t j = 0; j < joints_.size(); ++j)
  {
    for (std::size_t k = 0; k < INTERFACES.size(); ++k)
    {
      const auto found = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&](const auto& handle) {
        return handle.get_prefix_name() == joints_[j] && handle.get_interface_name() == INTERFACES[k];
      });
      if (found == command_interfaces_.end())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Missing command interface %s/%s. Is the joint in impedance mode?",
                     joints_[j].c_str(), INTERFACES[k]);
        return CallbackReturn::ERROR;
      }
      slots_[j][k] = static_cast<std::size_t>(found - command_interfaces_.begin());
    }
  }
  // A command from before the activation does not count: the joints hold until a new one arrives.
  targets_.writeFromNonRT(nullptr);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorCommandController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

void MotorCommandController::on_command(const std::shared_ptr<MotorCommand>& msg)
{
  // Names are matched here, so that update() only copies numbers.
  auto targets = std::make_shared<Targets>();
  const std::size_t names = msg->name.size();
  const auto one_each = [names](const auto& values) { return values.size() == names; };
  const auto one_each_or_none = [names](const auto& values) { return values.size() == names || values.empty(); };
  targets->usable = one_each(msg->position) && one_each(msg->velocity) && one_each(msg->torque_feedforward) &&
                    one_each_or_none(msg->stiffness) && one_each_or_none(msg->damping);
  if (!targets->usable)
  {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Ignoring a MotorCommand with %zu name(s): position, velocity and torque_feedforward need "
                          "one value each, stiffness and damping one each or none. The joints hold.",
                          names);
  }
  else
  {
    for (const auto& joint : joints_)
    {
      const auto named = std::find(msg->name.begin(), msg->name.end(), joint);
      if (named == msg->name.end())
      {
        targets->per_joint.push_back(HOLD);
        continue;
      }
      const auto i = static_cast<std::size_t>(named - msg->name.begin());
      const auto gain = [i](const std::vector<double>& values) { return values.empty() ? NaN : values[i]; };
      targets->per_joint.push_back(
          { msg->position[i], msg->velocity[i], msg->torque_feedforward[i], gain(msg->stiffness), gain(msg->damping) });
    }
  }
  targets_.writeFromNonRT(targets);
}

void MotorCommandController::write(std::size_t joint, const Setpoint& setpoint)
{
  const double values[] = { setpoint.position, setpoint.velocity, setpoint.feedforward, setpoint.stiffness,
                            setpoint.damping };
  for (std::size_t field = 0; field < slots_[joint].size(); ++field)
  {
    (void)command_interfaces_[slots_[joint][field]].set_value(values[field]);
  }
}

controller_interface::return_type MotorCommandController::update(const rclcpp::Time& /*time*/,
                                                                 const rclcpp::Duration& /*period*/)
{
  const auto* latest = targets_.readFromRT();
  const Targets* targets = latest && *latest ? latest->get() : nullptr;
  const bool hold = !targets || !targets->usable;
  for (std::size_t joint = 0; joint < joints_.size(); ++joint)
  {
    write(joint, hold ? HOLD : targets->per_joint[joint]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace synapticon_controllers

PLUGINLIB_EXPORT_CLASS(synapticon_controllers::MotorCommandController, controller_interface::ControllerInterface)
