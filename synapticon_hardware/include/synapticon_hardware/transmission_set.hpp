// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The URDF transmissions between the joints and the drives.

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <pluginlib/class_loader.hpp>
#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_loader.hpp>

#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/drive_joint_map.hpp"

namespace synapticon_hardware
{

/// The names of the impedance gain interfaces. Every transmission gets handles of these names too.
inline constexpr const char* STIFFNESS_INTERFACE = "stiffness";
inline constexpr const char* DAMPING_INTERFACE = "damping";

/// @brief The transmissions of a URDF, loaded through pluginlib, between the joints and the drives.
///
/// A transmission is like a gearbox: its actuators are the motors, its joints are what the motors move. Joints
/// without a drive get their state from the motors, and their commands go to the motors.
///
/// Position, velocity and effort always pass through. The impedance gains pass through a transmission that
/// supports them. With any other, each motor keeps its own gains.
///
/// Everything is sized when the transmissions load, so the calls in the control loop allocate nothing.
class TransmissionSet
{
public:
  /// Load the transmissions of `info`. Returns false after logging when one cannot be loaded or does not fit the
  /// joints, for example an actuator without a drive.
  bool load(const hardware_interface::HardwareInfo& info, const DriveJointMap& map);

  /// Fill in the state of every joint without a drive, from the states of its motors.
  void propagate_states(const std::vector<DriveState>& drive_states, std::vector<DriveState>& states);

  /// Turn the commands of the joints without a drive into commands for their motors.
  void apply_commands(const std::vector<DriveCommand>& commands, std::vector<DriveCommand>& drive_commands);

  /// Give each joint without a drive the flag of its motors, set when any of them has it. `drive_flags` has one entry
  /// per drive, `joint_flags` one per joint. Allocates nothing.
  void spread_to_joints_without_drive(const std::vector<uint8_t>& drive_flags, std::vector<uint8_t>& joint_flags) const;

  std::size_t size() const
  {
    return entries_.size();
  }

  void clear();

private:
  /// The numbers a transmission reads and writes through its handles, for one joint or one motor. The gains
  /// start as NaN, "not given".
  struct HandleValues
  {
    double position{ 0.0 };
    double velocity{ 0.0 };
    double effort{ 0.0 };
    double stiffness{ std::numeric_limits<double>::quiet_NaN() };
    double damping{ std::numeric_limits<double>::quiet_NaN() };
  };

  /// One loaded transmission and where its joints and motors are.
  ///
  /// at_joints, joint_index and without_drive run in parallel, over the transmission's joints as the URDF lists
  /// them: the values, the index of that joint in the hardware info, and whether the transmission alone moves it.
  /// at_motors and drive_index do the same over its actuators. outputs picks the driveless joints that take commands.
  struct Entry
  {
    std::string name;
    std::shared_ptr<transmission_interface::Transmission> transmission;
    std::vector<HandleValues> at_joints;
    std::vector<HandleValues> at_motors;
    std::vector<std::size_t> joint_index;
    std::vector<std::size_t> drive_index;
    std::vector<bool> without_drive;
    std::vector<std::size_t> outputs;
    bool gain_warning_given{ false };
  };

  bool load_entry(const hardware_interface::TransmissionInfo& info, const hardware_interface::HardwareInfo& hardware,
                  const DriveJointMap& map, Entry& entry);
  bool connect_joints(const hardware_interface::TransmissionInfo& info,
                      const hardware_interface::HardwareInfo& hardware, const DriveJointMap& map, Entry& entry,
                      std::vector<transmission_interface::JointHandle>& handles) const;
  bool connect_motors(const hardware_interface::TransmissionInfo& info,
                      const hardware_interface::HardwareInfo& hardware, const DriveJointMap& map, Entry& entry,
                      std::vector<transmission_interface::ActuatorHandle>& handles) const;

  /// One transmission, motors to joints.
  static void motors_to_joints(Entry& entry, const std::vector<DriveState>& drive_states,
                               std::vector<DriveState>& states);
  /// One transmission, joints to motors.
  static void joints_to_motors(Entry& entry, const std::vector<DriveCommand>& commands,
                               std::vector<DriveCommand>& drive_commands);

  /// Owns the plugin libraries. Every transmission is code from one of them, so this member is declared first and
  /// destroyed last.
  std::unique_ptr<pluginlib::ClassLoader<transmission_interface::TransmissionLoader>> loader_;
  std::vector<Entry> entries_;
};

}  // namespace synapticon_hardware
