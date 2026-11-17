// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief Object dictionary entries of a SOMANET drive that the EtherCAT HALs read over SDO, with helpers for their
/// values.
///
/// The indices from 0x1000 and 0x6000 come from the CiA 301 and CiA 402 standards. Everything at 0x2000 is defined by
/// the SOMANET firmware and may sit elsewhere on other products or firmware versions.

#include <cstdint>

namespace synapticon_hardware::ethercat
{

/// @brief Index and subindex of one object entry.
struct CoeAddress
{
  uint16_t index;
  uint8_t subindex;
};

/// The entries bring-up reads over SDO, sorted by what it does with them.
namespace coe
{
/// @name Unit conversion
/// Read once per bring-up to turn raw values into rad, rad/s and Nm.
/// @{
/// UDINT, the motor's rated torque in mNm. Torque values are given in thousandths of it.
inline constexpr CoeAddress RATED_TORQUE{ 0x6076, 0 };
/// UDINT in CiA notation: power of ten, distance unit, time unit. See velocity_step().
inline constexpr CoeAddress VELOCITY_UNIT{ 0x60A9, 0 };
/// UDINT, the feed of the feed constant. Any value but FEED_CONSTANT_DISABLED turns positions and velocities into
/// user-defined linear units.
inline constexpr CoeAddress FEED_CONSTANT_FEED{ 0x6092, 1 };
/// Number 1 or 2: the encoder whose count is the position. It replaces 0x608F, which SOMANET does not have.
inline constexpr CoeAddress POSITION_ENCODER{ 0x2012, 9 };
/// Number 1 or 2: the encoder for commutation and torque control, which sits on the motor shaft.
inline constexpr CoeAddress COMMUTATION_ENCODER{ 0x2010, 12 };
/// Counts per turn of encoder 1.
inline constexpr CoeAddress ENCODER_1_RESOLUTION{ 0x2110, 3 };
/// Counts per turn of encoder 2.
inline constexpr CoeAddress ENCODER_2_RESOLUTION{ 0x2112, 3 };
/// UDINT, motor turns of the gear ratio between the motor and the position shaft. The drive scales velocities to the
/// position shaft with it, unless the position encoder is the commutation encoder. Torque values stay at the motor.
inline constexpr CoeAddress GEAR_MOTOR_REVOLUTIONS{ 0x6091, 1 };
/// UDINT, position shaft turns of the gear ratio.
inline constexpr CoeAddress GEAR_SHAFT_REVOLUTIONS{ 0x6091, 2 };
/// @}

/// @name Start values of the streamed impedance outputs
/// The HAL sends these every cycle. It starts from what the drive holds, so a joint without own values keeps them.
/// @{
/// DINT, stiffness in mNm/rad.
inline constexpr CoeAddress IMPEDANCE_STIFFNESS{ 0x2060, 1 };
/// DINT, damping.
inline constexpr CoeAddress IMPEDANCE_DAMPING{ 0x2060, 2 };
/// UDINT, the torque ceiling in mNm. The drive clamps all its torque to it: at 0 the motor stays without force.
inline constexpr CoeAddress MAX_JOINT_TORQUE{ 0x2054, 0 };
/// @}

/// @name Preconditions that can refuse a bring-up
/// @{
/// The kind of torque sensor. The impedance controller needs one, and the drive faults with 0x6320 on enable without.
inline constexpr CoeAddress TORQUE_SENSOR_TYPE{ 0x2150, 1 };
/// Sensor type value for "no sensor".
inline constexpr uint32_t NO_TORQUE_SENSOR = 0;
/// Visible string with the firmware version, for example "v5.6.7".
inline constexpr CoeAddress SOFTWARE_VERSION{ 0x100A, 0 };
/// @}

/// @name How the drive stops
/// Reported in the log. The right values depend on the machine, so the package leaves them to the commissioning.
/// @{
/// INT16, what the drive does on a quick stop. See QuickStopReaction.
inline constexpr CoeAddress QUICK_STOP_OPTION{ 0x605A, 0 };
/// UDINT, the quick stop ramp in velocity units (0x60A9) per second, on the position shaft.
inline constexpr CoeAddress QUICK_STOP_DECELERATION{ 0x6085, 0 };
/// UDINT, the allowed following error in increments.
inline constexpr CoeAddress FOLLOWING_ERROR_WINDOW{ 0x6065, 0 };
/// With this window, the drive does not check the following error at all.
inline constexpr uint32_t FOLLOWING_ERROR_CHECK_OFF = 0xFFFFFFFFu;
/// USINT, what a following error does: 0 a report only, 1 a quick stop and then a fault, 2 an immediate fault.
inline constexpr CoeAddress FOLLOWING_ERROR_REACTION{ 0x2017, 1 };
/// @}
}  // namespace coe

/// @brief The two quick stop option codes (0x605A) that give a controlled stop. The other codes take the power off
/// at once or brake by shorting the motor.
enum class QuickStopReaction : int
{
  RAMP_THEN_DISABLE = 2,  ///< Brake on the ramp, then switch off.
  RAMP_AND_HOLD = 6,      ///< Brake on the ramp, then stay powered in Quick Stop Active and hold.
};

/// True for the option codes listed in QuickStopReaction.
constexpr bool is_supported_quick_stop(int option_code)
{
  return option_code == static_cast<int>(QuickStopReaction::RAMP_AND_HOLD) ||
         option_code == static_cast<int>(QuickStopReaction::RAMP_THEN_DISABLE);
}

/// Values of the mode of operation (0x6060) that a joint can run in.
namespace mode
{
inline constexpr int IMPEDANCE = -6;  ///< Impedance control of the SOMANET firmware.
inline constexpr int CSP = 8;         ///< Cyclic synchronous position, CiA 402.
inline constexpr int CSV = 9;         ///< Cyclic synchronous velocity, CiA 402.
}  // namespace mode

}  // namespace synapticon_hardware::ethercat
