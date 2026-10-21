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
#include <vector>

#include <rclcpp/logger.hpp>

#include "synapticon_hardware/ethercat/coe_objects.hpp"
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

struct SdoRetry
{
  int attempts{ 1 };
  double wait_s{ 0.0 };
};

///
/// each, so a drive that never answers delays the bring-up only once.
{
public:

  int upload(uint16_t position, uint16_t index, uint8_t sub_index, std::vector<uint8_t>& bytes, std::size_t& size,
             uint32_t& abort_code) const override;

private:
  const SdoReader& reader_;
  SdoRetry retry_;
  rclcpp::Logger logger_;
  mutable bool given_up_{ false };
};

/// @brief One drive's results of check_drives(), which the HAL keeps for the cyclic exchange.
struct DriveReadback
{
  /// Factor per Unit. A factor whose read failed keeps its start value. An inverted joint has negative factors for
  /// position, velocity and torque.
  SiFactors si_factors{ UNREAD_SI_FACTORS };
  /// First raw values for the streamed stiffness, damping and torque limit, used until a command brings its own.
  /// Each is the joint's configured value, else what the drive holds. Nothing when neither is known.
  std::optional<double> stiffness;
  std::optional<double> damping;
  std::optional<double> max_torque;
  /// The quick stop option code (0x605A). -1 if it could not be read.
  int quick_stop_option_code{ -1 };
  /// The quick stop ramp in rad/s^2, motor side. NaN if it could not be read.
  double quick_stop_deceleration{ std::numeric_limits<double>::quiet_NaN() };
};

/// Read `object`, an unsigned value of at most four bytes, from the drive at ring `position`. On a failed read, a
/// warning goes to `logger` and the result is empty.
std::optional<uint32_t> read_u32(const SdoReader& sdo, const rclcpp::Logger& logger, uint16_t position,
                                 CoeAddress object);

/// @brief Ask every drive of `joints` for what the HAL needs, and decide whether bring-up may go on.
///
/// `readback` gets one DriveReadback per joint. Everything learned is logged to `logger`. A problem that the user
/// must fix gets a framed block in the log.
///
/// Bring-up stops (false) in three cases:
/// - a joint that may be powered has a unit factor that could not be read,
/// - an impedance joint's drive has no torque sensor type, or it cannot be read,
/// - an impedance joint's drive runs firmware older than MINIMUM_FIRMWARE, or its version cannot be read.
///
/// A quick stop option code other than 2 or 6 is reported, but does not stop bring-up.
bool check_drives(const SdoReader& sdo, const std::vector<JointConfig>& joints, const rclcpp::Logger& logger,
                  std::vector<DriveReadback>& readback);

}  // namespace synapticon_hardware::ethercat
