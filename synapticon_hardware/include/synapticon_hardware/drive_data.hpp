// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The data exchanged with each drive once per control cycle.

#include <cmath>
#include <cstdint>

#include "synapticon_hardware/drive_action.hpp"

namespace synapticon_hardware
{

/// @brief Setpoints for one joint, for one cycle.
///
/// NaN in a setpoint field means "keep the value sent last" for that field. Before the first finite value that is
/// what the HAL starts with, for example the hold position that enable() sends. An infinite value is an error: the
/// field keeps its last value, the other values still go out, and write() returns false.
struct DriveCommand
{
  double position{ 0.0 };          ///< [rad] position setpoint.
  double velocity{ 0.0 };          ///< [rad/s] velocity setpoint.
  double effort{ 0.0 };            ///< [Nm] extra torque on top of the drive's own control, object 0x2053.
  double stiffness{ 0.0 };         ///< [Nm/rad] spring constant of the impedance control.
  double damping{ 0.0 };           ///< [Nm s/rad] damping constant of the impedance control.
  double max_joint_torque{ 0.0 };  ///< [Nm] torque limit, never negative.
  /// Request for this joint's CiA 402 state machine, see DriveAction. Only this joint's controlword changes.
  /// A new request takes the place of one the joint has not finished yet.
  DriveAction drive_action{ DriveAction::NONE };
};

/// @brief True when a setpoint of `command` is infinite. NaN does not count.
inline bool has_infinite_setpoint(const DriveCommand& command)
{
  for (const double value : { command.position, command.velocity, command.effort, command.stiffness, command.damping,
                              command.max_joint_torque })
  {
    if (std::isinf(value))
    {
      return true;
    }
  }
  return false;
}

/// @brief Feedback of one drive, for one cycle.
///
/// Physical values are SI units in a `double`. Drive registers keep the integer type they have on the drive.
/// The last six fields are optional: each one stays zero unless the slave_config maps its object.
struct DriveState
{
  double position{ 0.0 };  ///< [rad] measured position.
  double velocity{ 0.0 };  ///< [rad/s] measured velocity.
  double effort{ 0.0 };    ///< [Nm] measured torque, on the same shaft as position and velocity.

  uint16_t status_word{ 0 };  ///< CiA 402 statusword (0x6041).
  /// The CiA 402 state, a `cia402::DeviceState` value. The HAL does not set it: the hardware interface derives it
  /// from `status_word` after every read.
  uint8_t device_state{ 0 };
  uint16_t error_code{ 0 };  ///< Error code (0x603F), 0 when the drive has no error.
  /// Mode the drive runs in right now (0x6061). During a mode change it still shows the old mode for a while.
  int8_t mode_of_operation{ 0 };

  double dc_link_voltage{ 0.0 };       ///< [V] DC link voltage of the power stage (0x6079).
  double temperature_inverter{ 0.0 };  ///< [degrees C] temperature of the power stage (0x2031:1).
  double temperature_core{ 0.0 };      ///< [degrees C] temperature of the control board (0x2030:1).
  double motor_current{ 0.0 };         ///< [A] size of the motor current, from Id and Iq (0x2041:1).
  /// Name of the active fault or warning (0x203F:1). This is text, not a number: the drive's STRING(8) is stored
  /// in the memory of the double. Decode it with a byte copy.
  double error_report{ 0.0 };
  /// [Nm] torque that the drive's controller asks for, after its limits (0x2052). Follows the joint direction, so
  /// it flips sign for an inverted joint.
  double joint_torque_demand{ 0.0 };
};

}  // namespace synapticon_hardware
