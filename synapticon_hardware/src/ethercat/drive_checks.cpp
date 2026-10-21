// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The SDO reads of an EtherCAT bring-up: unit factors, first streamed values, and the checks on the drives.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/ethercat/coe_objects.hpp"
#include "synapticon_hardware/ethercat/drive_checks.hpp"
#include "synapticon_hardware/ethercat/firmware_version.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"
#include "framed_report.hpp"

namespace synapticon_hardware::ethercat
{

namespace
{
/// The bytes of `object`, at most `capacity` of them. On failure a warning is logged and the result is empty.
std::optional<std::vector<uint8_t>> upload(const SdoReader& sdo, const rclcpp::Logger& logger, uint16_t position,
                                           CoeAddress object, std::size_t capacity)
{
  std::vector<uint8_t> bytes(capacity, 0);
  std::size_t received = 0;
  uint32_t abort_code = 0;
  if (const int error = sdo.upload(position, object.index, object.subindex, bytes, received, abort_code); error != 0)
  {
    RCLCPP_WARN(logger, "Could not read 0x%04X:%02X over SDO (ring position %u): master error %d, abort code 0x%08X.",
                object.index, object.subindex, position, error, abort_code);
    return std::nullopt;
  }
  bytes.resize(std::min(received, capacity));
  return bytes;
}

/// A text object as a string. Spaces and zero bytes at its end are dropped.
std::optional<std::string> read_text(const SdoReader& sdo, const rclcpp::Logger& logger, uint16_t position,
                                     CoeAddress object)
{
  const auto bytes = upload(sdo, logger, position, object, 64);
  if (!bytes)
  {
    return std::nullopt;
  }
  std::string text(bytes->begin(), bytes->end());
  while (!text.empty() && (text.back() == ' ' || text.back() == '\0'))
  {
    text.pop_back();
  }
  return text;
}

/// The names joined by commas, for a log line.
std::string comma_list(const std::vector<std::string>& items)
{
  std::string text;
  for (std::size_t i = 0; i < items.size(); ++i)
  {
    text += (i > 0 ? ", " : "") + items[i];
  }
  return text;
}

/// The angle in rad of one position count. 0x2012:9 says which encoder counts, and a full turn is divided by that
/// encoder's counts per turn.
std::optional<double> position_factor(const SdoReader& sdo, const rclcpp::Logger& logger, const JointConfig& joint)
{
  const auto slot = read_u32(sdo, logger, joint.position, coe::POSITION_ENCODER);
  if (!slot)
  {
    return std::nullopt;
  }
  if (*slot != 1 && *slot != 2)
  {
    RCLCPP_WARN(logger,
                "Joint '%s': 0x2012:9 names encoder %u, which is neither 1 nor 2. The position stays in raw counts.",
                joint.name.c_str(), *slot);
    return std::nullopt;
  }
  const auto counts_per_turn =
      read_u32(sdo, logger, joint.position, *slot == 2 ? coe::ENCODER_2_RESOLUTION : coe::ENCODER_1_RESOLUTION);
  if (!counts_per_turn || *counts_per_turn == 0)
  {
    return std::nullopt;
  }
  const double one_turn = 2.0 * std::numbers::pi;
  return one_turn / *counts_per_turn;
}

/// The torque in Nm of one raw step, a thousandth of the rated torque. The drive gives the rated torque in mNm.
std::optional<double> torque_factor(const SdoReader& sdo, const rclcpp::Logger& logger, const JointConfig& joint)
{
  const auto rated_torque_mnm = read_u32(sdo, logger, joint.position, coe::RATED_TORQUE);
  if (rated_torque_mnm && *rated_torque_mnm > 0)
  {
    return *rated_torque_mnm / 1.0e6;
  }
  return std::nullopt;
}

/// The first raw torque ceiling to send. max_joint_torque wins when it is configured, else the drive's 0x2054 is used.
std::optional<double> torque_limit_to_stream(const SdoReader& sdo, const rclcpp::Logger& logger,
                                             const JointConfig& joint, const SiFactors& factors)
{
  if (std::isfinite(joint.max_joint_torque))
  {
    const double raw = joint.max_joint_torque / si_factor(factors, Unit::TORQUE_LIMIT);
    RCLCPP_INFO(logger,
                "Joint '%s': torque ceiling %.3f Nm (%.0f mNm) from max_joint_torque, in place of the drive's 0x2054.",
                joint.name.c_str(), joint.max_joint_torque, raw);
    return raw;
  }
  const auto stored = read_u32(sdo, logger, joint.position, coe::MAX_JOINT_TORQUE);
  if (stored && *stored == 0)
  {
    RCLCPP_WARN(logger,
                "Joint '%s' will have no torque: 0x2054 is 0 on the drive, and no max_joint_torque is configured.",
                joint.name.c_str());
  }
  return stored ? std::optional<double>(*stored) : std::nullopt;
}

/// The first raw value to send for one setting: the configured value in raw steps when there is one, else what the
/// drive holds.
std::optional<double> first_raw_value(double configured, double factor, std::optional<uint32_t> held)
{
  if (std::isfinite(configured))
  {
    return configured / factor;
  }
  return held ? std::optional<double>(*held) : std::nullopt;
}

/// `raw` in SI units with three decimals, for a log line, or "unreadable".
std::string si_text(std::optional<double> raw, double factor)
{
  if (!raw)
  {
    return "unreadable";
  }
  char text[32];
  std::snprintf(text, sizeof(text), "%.3f", *raw * factor);
  return text;
}

/// Put the first raw torque ceiling, stiffness and damping into `readback`, and log them next to what the drive
/// held. A configured value replaces the drive's.
void seed_command_values(const SdoReader& sdo, const rclcpp::Logger& logger, const JointConfig& joint,
                         DriveReadback& readback)
{
  const SiFactors& f = readback.si_factors;
  const double torque_step = si_factor(f, Unit::TORQUE_LIMIT);
  const double stiffness_step = si_factor(f, Unit::STIFFNESS);
  const double damping_step = si_factor(f, Unit::DAMPING);
  const auto held_ceiling = read_u32(sdo, logger, joint.position, coe::MAX_JOINT_TORQUE);
  const auto held_stiffness = read_u32(sdo, logger, joint.position, coe::IMPEDANCE_STIFFNESS);
  const auto held_damping = read_u32(sdo, logger, joint.position, coe::IMPEDANCE_DAMPING);
  readback.max_torque = first_raw_value(joint.max_joint_torque, torque_step, held_ceiling);
  readback.stiffness = first_raw_value(joint.stiffness, stiffness_step, held_stiffness);
  readback.damping = first_raw_value(joint.damping, damping_step, held_damping);
  const auto held = [](std::optional<uint32_t> value) { return value ? std::optional<double>(*value) : std::nullopt; };
  RCLCPP_INFO(logger,
              "Joint '%s' starts with stiffness %s Nm/rad, damping %s Nm s/rad and torque ceiling %s Nm. The drive "
              "held %s, %s and %s.",
              joint.name.c_str(), si_text(readback.stiffness, stiffness_step).c_str(),
              si_text(readback.damping, damping_step).c_str(), si_text(readback.max_torque, torque_step).c_str(),
              si_text(held(held_stiffness), stiffness_step).c_str(), si_text(held(held_damping), damping_step).c_str(),
              si_text(held(held_ceiling), torque_step).c_str());
  if (!std::isfinite(joint.max_joint_torque) && held_ceiling && *held_ceiling == 0)
  {
    RCLCPP_WARN(logger,
                "Joint '%s' cannot produce torque. Its ceiling is 0, as neither the drive (0x2054) nor the robot "
                "definition (max_joint_torque) sets one.",
                joint.name.c_str());
  }
}

/// Log the factors of `joint` in one line, with the gear ratio for information.
void log_scaling(const SdoReader& sdo, const rclcpp::Logger& logger, const JointConfig& joint, const SiFactors& f)
{
  const auto motor_turns = read_u32(sdo, logger, joint.position, coe::GEAR_MOTOR_REVOLUTIONS);
  const auto output_turns = read_u32(sdo, logger, joint.position, coe::GEAR_SHAFT_REVOLUTIONS);
  std::string gear = "gear ratio not readable";
  if (motor_turns && output_turns && *output_turns != 0)
  {
    gear = "gear ratio " + std::to_string(*motor_turns) + ":" + std::to_string(*output_turns) + " (information only)";
  }
  RCLCPP_INFO(logger, "Joint '%s': one raw step on the motor side is %.6e Nm, %.6e rad/s and %.6e rad, %s%s.",
              joint.name.c_str(), si_factor(f, Unit::MOTOR_TORQUE), si_factor(f, Unit::ANGULAR_VELOCITY),
              si_factor(f, Unit::ANGLE), gear.c_str(), joint.invert ? ", mounted in reverse" : "");
}

/// Fill the unit factors and the first streamed values of `joint` into `readback`. True when the drive gave the
/// factors of torque, velocity and position. A factor it did not give keeps its start value.
bool read_scaling(const SdoReader& sdo, const rclcpp::Logger& logger, const JointConfig& joint, DriveReadback& readback)
{
  const auto torque = torque_factor(sdo, logger, joint);
  const auto velocity_unit = read_u32(sdo, logger, joint.position, coe::VELOCITY_UNIT);
  const auto angle = position_factor(sdo, logger, joint);
  const std::optional<double> velocity =
      velocity_unit ? std::optional<double>(velocity_step_in_rad_per_s(*velocity_unit)) : std::nullopt;

  SiFactors& f = readback.si_factors;
  const std::pair<Unit, std::optional<double>> found[] = {
    { Unit::MOTOR_TORQUE, torque },
    { Unit::ANGULAR_VELOCITY, velocity },
    { Unit::ANGLE, angle },
  };
  for (const auto& [unit, value] : found)
  {
    if (value)
    {
      si_factor(f, unit) = *value;
    }
  }
  // Reverse mounting negates the factors of the four signed quantities below.
  if (joint.invert)
  {
    for (const Unit directed : { Unit::OUTPUT_TORQUE, Unit::MOTOR_TORQUE, Unit::ANGULAR_VELOCITY, Unit::ANGLE })
    {
      si_factor(f, directed) = -si_factor(f, directed);
    }
  }
  seed_command_values(sdo, logger, joint, readback);
  log_scaling(sdo, logger, joint, f);
  return torque && velocity && angle;
}

/// Decide what a missing unit factor means. A joint that may be powered stops the bring-up (false). A joint that is
/// only read gets a warning.
bool scaling_usable(const rclcpp::Logger& logger, const std::vector<JointConfig>& joints,
                    const std::vector<bool>& scaled)
{
  std::vector<std::string> refused;
  std::vector<std::string> warned;
  for (std::size_t j = 0; j < joints.size(); ++j)
  {
    if (!scaled[j])
    {
      (joints[j].energize ? refused : warned).push_back(joints[j].name);
    }
  }
  if (refused.empty() && warned.empty())
  {
    return true;
  }
  const std::string report = FramedReport("UNITS OF A DRIVE UNKNOWN")
                                 .say("A failed SDO read (see the warnings above) left a unit factor unknown.\n"
                                      "Commands in rad, rad/s or Nm would reach the drive as raw numbers.\n")
                                 .list("Refused, these joints may be powered", refused)
                                 .list("Allowed with raw states, these joints are never powered", warned)
                                 .close();
  if (!refused.empty())
  {
    RCLCPP_ERROR(logger, "%s", report.c_str());
    return false;
  }
  RCLCPP_WARN(logger, "%s", report.c_str());
  return true;
}

/// What a value of 0 does, for the warning about it.
const char* effect_of_zero(const std::string& key)
{
  if (key == "stiffness")
  {
    return "the joint is not pulled back to its target";
  }
  if (key == "damping")
  {
    return "the joint is not damped";
  }
  return "the joint has no torque";
}

/// True when every impedance joint that may be powered has its stiffness, damping and max_joint_torque. These fill
/// every command that leaves them out, so the drive never runs on what it last stored. A value of 0 is allowed, with
/// a warning.
bool command_values_complete(const rclcpp::Logger& logger, const std::vector<JointConfig>& joints)
{
  std::vector<std::string> incomplete;
  for (const JointConfig& joint : joints)
  {
    if (joint.mode_of_operation != mode::IMPEDANCE || !joint.energize)
    {
      continue;
    }
    const std::pair<std::string, double> values[] = { { "stiffness", joint.stiffness },
                                                      { "damping", joint.damping },
                                                      { "max_joint_torque", joint.max_joint_torque } };
    std::string missing;
    for (const auto& [key, value] : values)
    {
      if (std::isnan(value))
      {
        missing += (missing.empty() ? "" : ", ") + key;
      }
      else if (value == 0.0)
      {
        RCLCPP_WARN(logger, "Joint '%s' has %s 0 for commands that leave it out, so %s.", joint.name.c_str(),
                    key.c_str(), effect_of_zero(key));
      }
    }
    if (!missing.empty())
    {
      incomplete.push_back(joint.name + " (" + missing + ")");
    }
  }
  if (incomplete.empty())
  {
    return true;
  }
  const std::string report =
      FramedReport("IMPEDANCE JOINTS WITHOUT COMMAND DEFAULTS")
          .say("A command that leaves out stiffness, damping or max_joint_torque gets a configured\n"
               "value. Without one, the drive would run on whatever it last stored.\n")
          .list("Missing", incomplete)
          .say("\nSet them in the command_defaults section of the robot definition, or per joint.\n")
          .close();
  RCLCPP_ERROR(logger, "%s", report.c_str());
  return false;
}

/// True when the drive of every impedance joint has a torque sensor type. Impedance mode controls on the measured
/// torque, and without a sensor type the drive faults when it is enabled. An unreadable type counts as missing.
bool impedance_sensors_ready(const SdoReader& sdo, const rclcpp::Logger& logger, const std::vector<JointConfig>& joints)
{
  std::vector<std::string> type_none;
  std::vector<std::string> not_read;
  for (const auto& joint : joints)
  {
    if (joint.mode_of_operation == mode::IMPEDANCE)
    {
      const auto type = read_u32(sdo, logger, joint.position, coe::TORQUE_SENSOR_TYPE);
      if (!type)
      {
        not_read.push_back(joint.name);
      }
      else if (*type == coe::NO_TORQUE_SENSOR)
      {
        type_none.push_back(joint.name);
      }
    }
  }
  if (type_none.empty() && not_read.empty())
  {
    return true;
  }
  const std::string report = FramedReport("IMPEDANCE JOINTS WITHOUT A TORQUE SENSOR")
                                 .say("These joints would fault on enable. Their drives need a torque sensor type\n"
                                      "in 0x2150:1 for impedance mode.\n")
                                 .list("0x2150:1 is 0 (None)", type_none)
                                 .list("0x2150:1 not readable (firmware? bus? ring position?)", not_read)
                                 .say("\nFix, once per drive, with SDO downloads of your EtherCAT master:\n"
                                      "  0x2150:1 = 1          analog input\n"
                                      "  0x1010:1 = 0x65766173 \"save\", stores it on the drive\n"
                                      "A drive with no physical sensor keeps all analog input coefficients at 0.\n")
                                 .close();
  RCLCPP_ERROR(logger, "%s", report.c_str());
  return false;
}

/// Keep each drive's quick stop option code in `readback`, and report the drives that would not stop under control.
void check_quick_stop_reactions(const SdoReader& sdo, const rclcpp::Logger& logger,
                                const std::vector<JointConfig>& joints, std::vector<DriveReadback>& readback)
{
  std::vector<std::string> unsupported;
  std::vector<std::string> unreadable;
  std::vector<std::string> de_energising;
  for (std::size_t j = 0; j < joints.size(); ++j)
  {
    const auto raw = read_u32(sdo, logger, joints[j].position, coe::QUICK_STOP_OPTION);
    if (!raw)
    {
      unreadable.push_back(joints[j].name);
      continue;
    }
    const int code = static_cast<int16_t>(*raw & 0xFFFFu);
    readback[j].quick_stop_option_code = code;
    if (code == static_cast<int>(QuickStopReaction::RAMP_THEN_DISABLE))
    {
      de_energising.push_back(joints[j].name);
    }
    else if (!is_supported_quick_stop(code))
    {
      unsupported.push_back(joints[j].name + " (=" + std::to_string(code) + ")");
    }
  }
  if (!de_energising.empty())
  {
    RCLCPP_INFO(logger, "Joints whose quick stop ends without power (0x605A = 2): %s.",
                comma_list(de_energising).c_str());
  }
  if (unsupported.empty() && unreadable.empty())
  {
    return;
  }
  const std::string report =
      FramedReport("NO CONTROLLED QUICK STOP: 0x605A IS NOT 6 OR 2")
          .say("With 6 the drive brakes on its ramp and then holds the joint with power on.\n"
               "With 2 it brakes and then switches off. Other codes switch off at once or short\n"
               "the motor, so a loaded joint without a brake may fall. On the joints below,\n"
               "the quick_stop drive_action therefore does a disable.\n")
          .list("Other option code", unsupported)
          .list("Option code unreadable (check the firmware, the bus and the ring position)", unreadable)
          .say("\nTo set it at every bring-up, add this line to the 'sdo:' list of the slave_config:\n"
               "  - { index: 0x605a, sub_index: 0, type: int16, value: 6 }\n")
          .close();
  RCLCPP_ERROR(logger, "%s", report.c_str());
}

/// Log one line per drive on how it stops: its quick stop ramp, kept in `readback`, and its following error check.
void log_stopping_setup(const SdoReader& sdo, const rclcpp::Logger& logger, const std::vector<JointConfig>& joints,
                        std::vector<DriveReadback>& readback)
{
  for (std::size_t j = 0; j < joints.size(); ++j)
  {
    const uint16_t at = joints[j].position;
    char ramp[96] = "quick stop ramp not readable";
    const auto deceleration = read_u32(sdo, logger, at, coe::QUICK_STOP_DECELERATION);
    const auto unit = read_u32(sdo, logger, at, coe::VELOCITY_UNIT);
    if (deceleration && unit)
    {
      readback[j].quick_stop_deceleration = *deceleration * velocity_step_in_rad_per_s(*unit);
      std::snprintf(ramp, sizeof(ramp), "quick stop ramp %.4g rad/s^2 at the motor (0x6085 = %u)",
                    readback[j].quick_stop_deceleration, *deceleration);
    }

    char following[96] = "following error check not readable";
    const auto option = read_u32(sdo, logger, at, coe::FOLLOWING_ERROR_REACTION);
    const auto window = read_u32(sdo, logger, at, coe::FOLLOWING_ERROR_WINDOW);
    if (option && window && *window == coe::FOLLOWING_ERROR_CHECK_OFF)
    {
      std::snprintf(following, sizeof(following), "no following error check (0x6065 off)");
    }
    else if (option && window && *option == 0)
    {
      std::snprintf(following, sizeof(following), "a following error is only reported (0x2017:1 = 0)");
    }
    else if (option && window)
    {
      std::snprintf(following, sizeof(following),
                    "a following error faults the drive (0x2017:1 = %u, window %u counts)", *option, *window);
    }
    RCLCPP_INFO(logger, "Joint '%s' stops with: %s, %s.", joints[j].name.c_str(), ramp, following);
  }
}

/// Compare each drive's firmware with MINIMUM_FIRMWARE. Too old is an error for an impedance joint (false) and a
/// warning for the others.
bool firmware_recent_enough(const SdoReader& sdo, const rclcpp::Logger& logger, const std::vector<JointConfig>& joints)
{
  std::vector<std::string> refused;  // impedance mode
  std::vector<std::string> warned;   // position and velocity modes
  for (const auto& joint : joints)
  {
    const auto text = read_text(sdo, logger, joint.position, coe::SOFTWARE_VERSION);
    const auto version = text ? parse_firmware_version(*text) : std::nullopt;
    if (version && !(*version < MINIMUM_FIRMWARE))
    {
      continue;
    }
    const std::string detail = !text ? " (not readable)" : !version ? " (\"" + *text + "\")" : " (" + *text + ")";
    (joint.mode_of_operation == mode::IMPEDANCE ? refused : warned).push_back(joint.name + detail);
  }
  if (refused.empty() && warned.empty())
  {
    return true;
  }
  const std::string minimum = std::to_string(MINIMUM_FIRMWARE.major) + "." + std::to_string(MINIMUM_FIRMWARE.minor) +
                              "." + std::to_string(MINIMUM_FIRMWARE.patch);
  const std::string report = FramedReport("FIRMWARE UPDATE NEEDED: SOMANET " + minimum + " OR NEWER")
                                 .say("Impedance gains, joint torque offset and quick stop handling rely on\n"
                                      "firmware " +
                                      minimum + " or newer. A version that cannot be read counts as older.\n")
                                 .list("Refused, impedance joints", refused)
                                 .list("Allowed at your own risk, position and velocity joints", warned)
                                 .close();
  if (!refused.empty())
  {
    RCLCPP_ERROR(logger, "%s", report.c_str());
    return false;
  }
  RCLCPP_WARN(logger, "%s", report.c_str());
  return true;
}

}  // namespace

{
  const int attempts = given_up_ ? 1 : std::max(retry_.attempts, 1);
  int error = 0;
  for (int attempt = 1; attempt <= attempts; ++attempt)
  {
    abort_code = 0;
    if (error == 0 || abort_code != 0)
    {
      return error;
    }
    if (attempt < attempts)
    {
      RCLCPP_WARN(logger_,
                  "Try %d of %d in %.1f s.",
      std::this_thread::sleep_for(std::chrono::duration<double>(retry_.wait_s));
    }
  }
  given_up_ = true;
  return error;
}

std::optional<uint32_t> read_u32(const SdoReader& sdo, const rclcpp::Logger& logger, uint16_t position,
                                 CoeAddress object)
{
  const auto bytes = upload(sdo, logger, position, object, sizeof(uint32_t));
  if (!bytes)
  {
    return std::nullopt;
  }
  // Least significant byte first. Bytes the drive did not send count as 0.
  uint32_t value = 0;
  for (std::size_t shift = 0; shift < bytes->size(); ++shift)
  {
    value |= static_cast<uint32_t>((*bytes)[shift]) << (8 * shift);
  }
  return value;
}

bool check_drives(const SdoReader& sdo, const std::vector<JointConfig>& joints, const rclcpp::Logger& logger,
                  std::vector<DriveReadback>& readback)
{
  readback.assign(joints.size(), DriveReadback{});
  if (!command_values_complete(logger, joints))
  {
    return false;
  }
  std::vector<bool> scaled(joints.size());
  for (std::size_t j = 0; j < joints.size(); ++j)
  {
    scaled[j] = read_scaling(sdo, logger, joints[j], readback[j]);
  }
  if (!scaling_usable(logger, joints, scaled) || !impedance_sensors_ready(sdo, logger, joints))
  {
    return false;
  }
  // A drive holds what its last commissioning stored, so the stop settings are read, never assumed.
  check_quick_stop_reactions(sdo, logger, joints, readback);
  log_stopping_setup(sdo, logger, joints, readback);
  return firmware_recent_enough(sdo, logger, joints);
}

}  // namespace synapticon_hardware::ethercat
