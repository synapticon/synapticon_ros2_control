// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief What the EtherCAT HALs learn from each drive over SDO at bring-up, and the checks that can stop it.

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <vector>

#include <rclcpp/logger.hpp>

#include "synapticon_hardware/ethercat/coe_objects.hpp"
#include "synapticon_hardware/ethercat/drive_units.hpp"
#include "synapticon_hardware/ethercat/joint_config.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"

namespace synapticon_hardware::ethercat
{

/// @brief SDO read access to the drives. Each EtherCAT HAL provides it through its own master.
class SdoReader
{
public:
  virtual ~SdoReader() = default;

  /// Read entry `index`:`sub_index` from the drive at ring `position`. The data goes into `bytes`, which is not
  /// grown, and `size` tells how many bytes came. Returns 0 on success, else an error code of the master, with the
  /// abort code of the drive in `abort_code`.
  virtual int upload(uint16_t position, uint16_t index, uint8_t sub_index, std::vector<uint8_t>& bytes,
                     std::size_t& size, uint32_t& abort_code) const = 0;
};

/// @brief SDO write access to the drives. Each EtherCAT HAL provides it through its own master.
class SdoWriter
{
public:
  virtual ~SdoWriter() = default;

  /// Write `bytes` to entry `index`:`sub_index` of the drive at ring `position`. Returns 0 on success, else an error
  /// code of the master, with the abort code of the drive in `abort_code`.
  virtual int download(uint16_t position, uint16_t index, uint8_t sub_index, const std::vector<uint8_t>& bytes,
                       uint32_t& abort_code) const = 0;
};

/// @brief How often a failed SDO read or write is tried: `attempts` tries in all, `wait_s` seconds apart.
struct SdoRetry
{
  int attempts{ 1 };
  double wait_s{ 0.0 };
};

/// @brief SDO access that tries a read or write again while the master gets no valid answer from the drive.
///
/// An answer of the drive, an abort too, is final. Once a transfer has used up its tries, the later ones to the same
/// drive get one try each, so a drive that never answers delays the bring-up only once. Other drives keep their tries.
class RetryingSdo : public SdoReader, public SdoWriter
{
public:
  /// Reads through `reader` and writes through `writer`, which must outlive this object. Each retry is logged to
  /// `logger`.
  RetryingSdo(const SdoReader& reader, const SdoWriter& writer, const SdoRetry& retry, const rclcpp::Logger& logger);

  int upload(uint16_t position, uint16_t index, uint8_t sub_index, std::vector<uint8_t>& bytes, std::size_t& size,
             uint32_t& abort_code) const override;
  int download(uint16_t position, uint16_t index, uint8_t sub_index, const std::vector<uint8_t>& bytes,
               uint32_t& abort_code) const override;

private:
  /// Run `transfer` until the drive answers or the tries are used up. `what` and `position` name it in the log.
  template <typename Transfer>
  int with_retries(const char* what, uint16_t position, uint16_t index, uint8_t sub_index, uint32_t& abort_code,
                   const Transfer& transfer) const;

  const SdoReader& reader_;
  const SdoWriter& writer_;
  SdoRetry retry_;
  rclcpp::Logger logger_;
  /// Ring positions of the drives for which a transfer has used up its tries.
  mutable std::set<uint16_t> given_up_;
};

/// @brief One drive's results of check_drives(), which the HAL keeps for the cyclic exchange.
struct DriveReadback
{
  /// Factor per Unit. A factor whose read failed keeps its start value. An inverted joint has negative factors for
  /// position, velocity and torque.
  SiFactors si_factors{ UNREAD_SI_FACTORS };
  /// True when the factors of position, velocity and torque all come from the drive or its slave_config.
  bool units_known{ false };
  /// First raw values for the streamed stiffness, damping and torque limit, used until a command brings its own.
  /// Each is the joint's configured value, else what the drive holds. Nothing when neither is known.
  std::optional<double> stiffness;
  std::optional<double> damping;
  std::optional<double> max_torque;
  /// The quick stop option code (0x605A). -1 if it could not be read.
  int quick_stop_option_code{ -1 };
  /// The quick stop ramp in rad/s^2, on the position shaft. NaN if it could not be read.
  double quick_stop_deceleration{ std::numeric_limits<double>::quiet_NaN() };
};

/// Read `object`, an unsigned value of at most four bytes, from the drive at ring `position`. On a failed read, a
/// warning goes to `logger` and the result is empty.
std::optional<uint32_t> read_u32(const SdoReader& sdo, const rclcpp::Logger& logger, uint16_t position,
                                 CoeAddress object);

/// @brief Ask every drive of `joints` for what the HAL needs, write the `sdo_si:` entries of its slave_config, and
/// decide whether bring-up may go on.
///
/// `units` holds one SlaveConfigUnits per joint. The drive's units are read and must be understood completely; the
/// `units:` section of a slave_config only stands in for what the drive cannot report. Then each `sdo_si:` entry is
/// converted with them, written through `writer` and read back through `sdo`, unless it has `verify: false`. A value
/// that reads back different is written once more. `readback` gets one DriveReadback per joint. Everything learned
/// is logged to `logger`. A problem that the user must fix gets a framed block in the log.
///
/// Bring-up stops (false) in these cases:
/// - an impedance joint that may be powered has no stiffness, damping or max_joint_torque configured,
/// - a joint that may be powered has units the stack cannot read or understand, or that contradict its slave_config,
/// - an `sdo_si:` entry cannot be converted, the drive refuses it, it cannot be read back, or the drive keeps another
///   value after the second write,
/// - an impedance joint's drive has no torque sensor type, or it cannot be read,
/// - an impedance joint's drive runs firmware older than MINIMUM_FIRMWARE, or its version cannot be read.
///
/// A quick stop option code other than 2 or 6 is reported, but does not stop bring-up. Neither does a geared drive
/// whose position encoder is the commutation encoder: its states are at the motor, its torque commands on the
/// output side, and a warning says so.
bool check_drives(const SdoReader& sdo, const SdoWriter& writer, const std::vector<JointConfig>& joints,
                  const std::vector<SlaveConfigUnits>& units, const rclcpp::Logger& logger,
                  std::vector<DriveReadback>& readback);

}  // namespace synapticon_hardware::ethercat
