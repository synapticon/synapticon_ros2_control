// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief EthercatPdoHal, the common part of the EtherCAT HALs, which works on the process data of each joint.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "synapticon_hardware/cia402/cia402_hal.hpp"
#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/ethercat/drive_checks.hpp"
#include "synapticon_hardware/ethercat/joint_config.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"

namespace synapticon_hardware::ethercat
{

/// @brief Common base of the HALs that run SOMANET drives over EtherCAT.
///
/// The split of work:
/// - This class owns two arrays of doubles per joint, `outputs_` to the drive and `inputs_` from it, in the slot
///   order of the PDO channel tables. On them it does the CiA 402 hooks, the SI conversion of states and commands,
///   the bring-up checks and the wait for valid data after the bus starts.
/// - The derived HAL owns the master. It sets up and starts the bus, moves the arrays to and from the drives in
///   exchange_read() and exchange_write(), and implements upload() for SDO reads.
class EthercatPdoHal : public Cia402Hal, protected SdoReader
{
public:
  /// Whether the joint can quick stop under control, that is its 0x605A from bring-up is 6 or 2. If not, the
  /// quick_stop drive action becomes a disable.
  bool quick_stop_available(std::size_t joint) const override;
  /// The joint's quick stop option code (0x605A) from bring-up. -1 if it was not read.
  int quick_stop_option_code(std::size_t joint) const;
  /// The joint's quick stop ramp from bring-up in rad/s^2, motor side. NaN if it was not read.
  double quick_stop_deceleration(std::size_t joint) const;

protected:
  /// `logger_name` is the logger the HAL writes to.
  explicit EthercatPdoHal(std::string logger_name);

  /// First step of connect(), before the bus gets the arrays: allocate them for `joints` and prepare the CiA 402
  /// logic. Targets start as NaN, the controlword, gains and torque ceiling as 0 until check_drives() fills them.
  /// `quick_stop_hold` makes the HAL hold an impedance joint where its quick stop started, because the drive does not
  /// brake in impedance mode.
  void set_up_drives(const std::vector<JointConfig>& joints, bool quick_stop_hold, double control_frequency);

  /// The bring-up checks of `joints`, read through upload(). Their results go into the arrays, the factors and the
  /// quick stop getters. Call it while the bus is configured but not yet running. False means bring-up must stop.
  bool check_drives(const std::vector<JointConfig>& joints);

  /// Right after the bus reached OP: exchange process data, with controlword 0, until the drive of every joint that
  /// may be powered sends a statusword. False after `timeout_s` or on a failed exchange. Joints that are only read
  /// are not waited for. Reads no SDO: the bus is running, and an SDO read would block the master.
  bool wait_for_streaming(const std::vector<JointConfig>& joints, double control_frequency, double timeout_s);

  /// Convert the arrays from the drives into one DriveState per joint, in SI units. `states` is resized to fit.
  void unpack_states(std::vector<DriveState>& states) const;

  // Cia402Hal hooks. Each reads or writes one slot of a joint's arrays.
  uint16_t statusword(std::size_t joint) const override;
  uint16_t error_code(std::size_t joint) const override;
  uint16_t last_controlword(std::size_t joint) const override;
  double speed(std::size_t joint) const override;
  double transport_position(std::size_t joint) const override;
  void write_controlword(std::size_t joint, uint16_t controlword) override;
  void write_setpoints(std::size_t joint, const DriveCommand& command) override;
  void write_hold(std::size_t joint, double position) override;

  /// Read an unsigned object of at most 32 bits through upload(). Empty, with a warning, when the read fails.
  std::optional<uint32_t> sdo_u32(uint16_t position, CoeAddress object) const;

  // One entry per joint: the array to the drive, the array from the drive, the factor of each Unit.
  std::vector<std::vector<double>> outputs_;
  std::vector<std::vector<double>> inputs_;
  std::vector<SiFactors> si_factors_;

private:
  /// Store the bring-up results of `joint`: factors, quick stop values, and the first streamed gains and ceiling.
  void apply_readback(std::size_t joint, const DriveReadback& readback);
  /// True when no joint that may be powered still reads statusword 0.
  bool all_streaming(const std::vector<JointConfig>& joints) const;

  // Quick stop values per joint, from bring-up.
  std::vector<int> option_codes_;
  std::vector<double> quick_stop_decelerations_;
};

}  // namespace synapticon_hardware::ethercat
