// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Loading the URDF transmissions, and running them in both directions every cycle.

#include "synapticon_hardware/transmission_set.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <optional>
#include <tuple>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <transmission_interface/handle.hpp>

namespace synapticon_hardware
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("SynapticonSystemInterface");
}

/// Where joint `name` is in the URDF, if it is there.
std::optional<std::size_t> find_joint(const hardware_interface::HardwareInfo& hardware, const std::string& name)
{
  for (std::size_t index = 0; index < hardware.joints.size(); ++index)
  {
    if (hardware.joints[index].name == name)
    {
      return index;
    }
  }
  return std::nullopt;
}

/// The handles of `name` for position, velocity, effort and both gains, pointing into `values`.
template <typename Handle, typename Values>
void add_handles(std::vector<Handle>& handles, const std::string& name, Values& values)
{
  const std::pair<const char*, double*> fields[] = {
    { hardware_interface::HW_IF_POSITION, &values.position },
    { hardware_interface::HW_IF_VELOCITY, &values.velocity },
    { hardware_interface::HW_IF_EFFORT, &values.effort },
    { STIFFNESS_INTERFACE, &values.stiffness },
    { DAMPING_INTERFACE, &values.damping },
  };
  for (const auto& [interface, value] : fields)
  {
    handles.emplace_back(name, interface, value);
  }
}
}  // namespace

bool TransmissionSet::load(const hardware_interface::HardwareInfo& info, const DriveJointMap& map)
{
  clear();
  if (info.transmissions.empty())
  {
    return true;
  }
  loader_ = std::make_unique<pluginlib::ClassLoader<transmission_interface::TransmissionLoader>>(
      "transmission_interface", "transmission_interface::TransmissionLoader");
  // The entries get their final size first: the handles point into them.
  entries_.resize(info.transmissions.size());
  for (std::size_t index = 0; index < entries_.size(); ++index)
  {
    if (!load_entry(info.transmissions[index], info, map, entries_[index]))
    {
      entries_.clear();
      return false;
    }
  }
  return true;
}

bool TransmissionSet::load_entry(const hardware_interface::TransmissionInfo& urdf,
                                 const hardware_interface::HardwareInfo& hardware, const DriveJointMap& map,
                                 Entry& entry)
{
  // ros2_control hands the full robot description to transmission loaders under this parameter. Linkage
  // transmissions take their geometry from it.
  hardware_interface::TransmissionInfo info = urdf;
  info.parameters["__robot_description"] = hardware.original_xml;
  entry.name = info.name;
  try
  {
    entry.transmission = loader_->createSharedInstance(info.type)->load(info);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(logger(), "Cannot load transmission '%s' of type '%s': %s", info.name.c_str(), info.type.c_str(),
                 error.what());
    return false;
  }
  if (!entry.transmission)
  {
    RCLCPP_ERROR(logger(), "Transmission '%s' of type '%s': the plugin produced no transmission.", info.name.c_str(),
                 info.type.c_str());
    return false;
  }

  entry.at_joints.assign(info.joints.size(), HandleValues{});
  entry.at_motors.assign(info.actuators.size(), HandleValues{});
  std::vector<transmission_interface::JointHandle> joint_handles;
  std::vector<transmission_interface::ActuatorHandle> motor_handles;
  if (!connect_joints(info, hardware, map, entry, joint_handles) ||
      !connect_motors(info, hardware, map, entry, motor_handles))
  {
    return false;
  }
  try
  {
    entry.transmission->configure(joint_handles, motor_handles);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(logger(), "Transmission '%s' does not accept its joints and actuators: %s", info.name.c_str(),
                 error.what());
    return false;
  }
  return true;
}

bool TransmissionSet::connect_joints(const hardware_interface::TransmissionInfo& info,
                                     const hardware_interface::HardwareInfo& hardware, const DriveJointMap& map,
                                     Entry& entry, std::vector<transmission_interface::JointHandle>& handles) const
{
  // Handles in URDF order: the plugin maps them to its roles by position.
  for (std::size_t slot = 0; slot < info.joints.size(); ++slot)
  {
    const auto joint = find_joint(hardware, info.joints[slot].name);
    if (!joint)
    {
      RCLCPP_ERROR(logger(), "Transmission '%s' names joint '%s', which the URDF's <ros2_control> does not have.",
                   info.name.c_str(), info.joints[slot].name.c_str());
      return false;
    }
    const bool no_drive = !map.has_drive(*joint);
    entry.joint_index.push_back(*joint);
    entry.without_drive.push_back(no_drive);
    if (no_drive && !hardware.joints[*joint].command_interfaces.empty())
    {
      entry.outputs.push_back(slot);
    }
    add_handles(handles, info.joints[slot].name, entry.at_joints[slot]);
  }
  // All or none: the plugin maps all joints to the motors at once, and a joint without a command would send its
  // measured state instead.
  const auto joints_without_drive = std::count(entry.without_drive.begin(), entry.without_drive.end(), true);
  if (!entry.outputs.empty() && entry.outputs.size() != static_cast<std::size_t>(joints_without_drive))
  {
    RCLCPP_ERROR(logger(),
                 "Transmission '%s': only %zu of its %ld joints without a drive take commands. Declare command "
                 "interfaces on all of them, or on none.",
                 info.name.c_str(), entry.outputs.size(), static_cast<long>(joints_without_drive));
    return false;
  }
  return true;
}

bool TransmissionSet::connect_motors(const hardware_interface::TransmissionInfo& info,
                                     const hardware_interface::HardwareInfo& hardware, const DriveJointMap& map,
                                     Entry& entry, std::vector<transmission_interface::ActuatorHandle>& handles) const
{
  for (std::size_t slot = 0; slot < info.actuators.size(); ++slot)
  {
    const std::string& motor = info.actuators[slot].name;
    const auto joint = find_joint(hardware, motor);
    if (!joint || !map.has_drive(*joint))
    {
      RCLCPP_ERROR(logger(), "Transmission '%s': its actuator '%s' must be a joint with a drive.", info.name.c_str(),
                   motor.c_str());
      return false;
    }
    entry.drive_index.push_back(static_cast<std::size_t>(map.joint_to_drive[*joint]));
    add_handles(handles, motor, entry.at_motors[slot]);
  }
  return true;
}

void TransmissionSet::motors_to_joints(Entry& entry, const std::vector<DriveState>& drive_states,
                                       std::vector<DriveState>& states)
{
  for (std::size_t slot = 0; slot < entry.at_motors.size(); ++slot)
  {
    const DriveState& motor = drive_states[entry.drive_index[slot]];
    entry.at_motors[slot] = { motor.position, motor.velocity, motor.effort };
  }
  entry.transmission->actuator_to_joint();
  for (std::size_t slot = 0; slot < entry.at_joints.size(); ++slot)
  {
    // A joint with a drive of its own keeps what its drive measured.
    if (!entry.without_drive[slot])
    {
      continue;
    }
    DriveState& joint = states[entry.joint_index[slot]];
    const HandleValues& computed = entry.at_joints[slot];
    joint.position = computed.position;
    joint.velocity = computed.velocity;
    joint.effort = computed.effort;
  }
}

void TransmissionSet::joints_to_motors(Entry& entry, const std::vector<DriveCommand>& commands,
                                       std::vector<DriveCommand>& drive_commands)
{
  constexpr double NOT_GIVEN = std::numeric_limits<double>::quiet_NaN();
  bool stiffness_given = false;
  bool damping_given = false;
  for (const std::size_t slot : entry.outputs)
  {
    const DriveCommand& joint = commands[entry.joint_index[slot]];
    entry.at_joints[slot] = { joint.position, joint.velocity, joint.effort, joint.stiffness, joint.damping };
    stiffness_given = stiffness_given || !std::isnan(joint.stiffness);
    damping_given = damping_given || !std::isnan(joint.damping);
  }
  // NaN in every motor gain first, so after joint_to_actuator() a NaN marks a gain the transmission did not set.
  for (HandleValues& motor : entry.at_motors)
  {
    motor.stiffness = NOT_GIVEN;
    motor.damping = NOT_GIVEN;
  }
  entry.transmission->joint_to_actuator();

  // The torque limit stays a setting of each drive: every motor keeps the limit of its own joint.
  bool gains_dropped = false;
  for (std::size_t slot = 0; slot < entry.at_motors.size(); ++slot)
  {
    const HandleValues& mapped = entry.at_motors[slot];
    DriveCommand& motor = drive_commands[entry.drive_index[slot]];
    motor.position = mapped.position;
    motor.velocity = mapped.velocity;
    motor.effort = mapped.effort;
    // A NaN leaves the motor's own gain in place, and if the joint had a gain, that loss is reported below. An infinite
    // gain goes on, so that the HAL refuses it.
    // The rule for missing and invalid values may change with issue #88.
    const std::tuple<double*, double, bool> gains[] = { { &motor.stiffness, mapped.stiffness, stiffness_given },
                                                        { &motor.damping, mapped.damping, damping_given } };
    for (const auto& [target, value, given] : gains)
    {
      if (!std::isnan(value))
      {
        *target = value;
      }
      else
      {
        gains_dropped = gains_dropped || given;
      }
    }
  }
  if (gains_dropped && !entry.gain_warning_given)
  {
    entry.gain_warning_given = true;
    RCLCPP_WARN(logger(),
                "Transmission '%s' maps no gains: its motors take the stiffness and damping of their own joints. Its "
                "type may not support gains, or a joint gain is NaN. Not repeated.",
                entry.name.c_str());
  }
}

void TransmissionSet::propagate_states(const std::vector<DriveState>& drive_states, std::vector<DriveState>& states)
{
  for (Entry& entry : entries_)
  {
    motors_to_joints(entry, drive_states, states);
  }
}

void TransmissionSet::apply_commands(const std::vector<DriveCommand>& commands,
                                     std::vector<DriveCommand>& drive_commands)
{
  for (Entry& entry : entries_)
  {
    // A transmission whose joints only measure leaves each motor with the command of its own joint.
    if (!entry.outputs.empty())
    {
      joints_to_motors(entry, commands, drive_commands);
    }
  }
}

void TransmissionSet::spread_to_joints_without_drive(const std::vector<uint8_t>& drive_flags,
                                                     std::vector<uint8_t>& joint_flags) const
{
  for (const Entry& entry : entries_)
  {
    const bool any = std::any_of(entry.drive_index.begin(), entry.drive_index.end(),
                                 [&drive_flags](std::size_t drive) { return drive_flags[drive] != 0u; });
    for (std::size_t slot = 0; slot < entry.joint_index.size(); ++slot)
    {
      if (entry.without_drive[slot])
      {
        joint_flags[entry.joint_index[slot]] = any ? 1u : 0u;
      }
    }
  }
}

void TransmissionSet::clear()
{
  entries_.clear();
  loader_.reset();
}

}  // namespace synapticon_hardware
