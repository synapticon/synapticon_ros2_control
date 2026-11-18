// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief A simulated CiA 402 drive with a simple joint, for testing HAL and test logic without hardware.

#include <cstdint>
#include <limits>

#include "synapticon_hardware/cia402/state_machine.hpp"
#include "synapticon_hardware/drive_data.hpp"

namespace synapticon_hardware
{

/// How a simulated drive treats a following error.
enum class FollowingErrorMode : uint8_t
{
  OFF,     ///< Not monitored.
  REPORT,  ///< Reported in statusword bit 13 while it lasts, the drive keeps running.
  FAULT,   ///< The drive runs its fault reaction (a quick stop ramp) and ends in Fault (0x8611).
};

/// The configuration of one simulated drive.
struct SimulatedDriveSettings
{
  int mode_of_operation{ 8 };  ///< -6 impedance, 8 CSP or 9 CSV.
  /// 0x605A: 6 stops on the ramp and stays in Quick Stop Active, 2 stops and then de-energises,
  /// anything else de-energises at once.
  int quick_stop_option_code{ 6 };
  double quick_stop_deceleration{ 20.0 };  ///< [rad/s^2], also the fault reaction ramp.
  FollowingErrorMode following_error{ FollowingErrorMode::OFF };
  double following_error_window{ 0.006 };  ///< [rad]
  /// [s] the error must last this long. Below one cycle, so a single cycle's error counts.
  double following_error_time{ 0.0005 };
};

/// @brief A CiA 402 drive and the joint it moves, stepped in time.
///
/// The state machine follows the controlword like a SOMANET drive: shutdown, switch on, enable
/// operation, disable voltage, quick stop (with the option code), and a fault reset on the rising
/// edge of bit 7. The joint has inertia and Coulomb friction. In Operation Enabled it follows the
/// mode: CSP runs a position loop bounded in speed and acceleration, CSV follows the target velocity
/// with a short lag, and impedance applies K (q_target - q) + D (v_target - v) + effort as torque.
/// Quick Stop Active and the fault reaction brake on the quick stop ramp in CSP and CSV, while the
/// impedance mode keeps applying its law to the target, as the real drive does. De-energised, the
/// joint coasts to rest on friction.
class SimulatedDrive
{
public:
  explicit SimulatedDrive(const SimulatedDriveSettings& settings);

  /// Latch the command for the next steps: the controlword and the setpoints. NaN in a gain keeps
  /// the last one.
  void command(uint16_t controlword, const DriveCommand& setpoints);

  /// Advance the drive by `dt` seconds.
  void step(double dt);

  /// The drive's state as a HAL reports it.
  DriveState state() const;

  uint16_t statusword() const;
  cia402::DeviceState device_state() const
  {
    return state_;
  }

private:
  void apply_controlword();
  void update_following_error(double dt);
  /// The joint's torque in Operation Enabled, or while the impedance law runs in Quick Stop Active.
  double impedance_torque() const;
  /// Integrate the joint for `dt` under `torque`, with Coulomb friction.
  void integrate_torque(double torque, double dt);
  /// Brake the speed toward zero at `deceleration` for `dt`.
  void ramp_down(double deceleration, double dt);
  void enter_fault_reaction(uint16_t error_code);

  SimulatedDriveSettings settings_;
  cia402::DeviceState state_{ cia402::DeviceState::SWITCH_ON_DISABLED };

  uint16_t controlword_{ 0 };
  bool fault_reset_bit_{ false };
  DriveCommand setpoints_{};
  double stiffness_{ 0.0 };
  double damping_{ 0.0 };

  double position_{ 0.0 };
  double velocity_{ 0.0 };
  double torque_{ 0.0 };
  uint16_t error_code_{ 0 };
  double following_error_since_{ 0.0 };  ///< how long the following error has lasted [s]
  bool following_error_reported_{ false };
  double state_time_{ 0.0 };    ///< time in the present state [s]
  double stopped_time_{ 0.0 };  ///< how long the joint has been at rest [s]
};

}  // namespace synapticon_hardware
