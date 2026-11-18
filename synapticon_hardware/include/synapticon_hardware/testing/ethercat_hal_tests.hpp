// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The hardware-in-the-loop run of an EtherCAT HAL: the drive action suite on SOMANET drives, set up through
/// slave_config SDO entries, plus the bring-up tests every EtherCAT HAL shares.
///
/// An executable includes this header after it declared, in synapticon_hardware::testing, the HAL it tests:
///
///   using EthercatHalUnderTest = <a HAL derived from ethercat::EthercatPdoHal>;
///
/// It defines set_up_hal(), declared here, which sets the HAL up on the bench's bus, and its main() returns
/// run_ethercat_hil_tests(argc, argv). Each test's drive settings become SDO entries of a
/// slave_config derived from a base config. The suite writes the quick stop objects and the following error objects
/// (the vendor defaults, except in the fault tests), and restores the quick stop deceleration it found at the end.
/// Some bring-up tests change a drive object (MaxJointTorque, the torque sensor type, the max motor speed, the
/// following error window, the impedance gains) and write back what they found. A test that fails or is killed in
/// between leaves the changed value. Drives keep SDO values until a power cycle.
///
/// Environment (all optional):
///   SYNAPTICON_HIL_ROBOT         robot definition YAML of the bench: its first joint is the bench drive, its second
///                                the second drive, with their ring position, slave_config and command defaults.
///                                Its bus section sets the bus, except the control frequency of the suite. Its
///                                hil_tests section gives the impedance tests their gains. A joint with own gains
///                                needs its max_joint_torque there too
///   SYNAPTICON_HIL_SLAVE_CONFIG  base slave_config of a drive that names none (default: the installed example config)
///   SYNAPTICON_HIL_ALIAS         slave alias, overrides the robot definition (default 0)
///   SYNAPTICON_HIL_POSITION      ring position, overrides the robot definition (default 0)
///   SYNAPTICON_HIL_POSITION_2    second drive's ring position, for --two-drives (default: the robot definition's
///                                second joint, or SYNAPTICON_HIL_POSITION + 1)
///   SYNAPTICON_HIL_STRICT        1: a failed impedance bring-up fails its test instead of
///                                skipping it (default: skip, for a bench without a torque sensor)
///
/// Command line options (any other argument goes to gtest):
///   --two-drives                 run the DrivePair tests, which need a second drive on the
///                                same bus (skipped otherwise)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/ethercat/bus_config.hpp"
#include "synapticon_hardware/ethercat/coe_objects.hpp"
#include "synapticon_hardware/ethercat/ethercat_pdo_hal.hpp"
#include "synapticon_hardware/ethercat/joint_config.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"
#include "synapticon_hardware/ethercat/robot_config.hpp"
#include "synapticon_hardware/hal_config.hpp"
#include "synapticon_hardware/testing/cia402_hal_tests.hpp"

namespace synapticon_hardware::testing
{
namespace ec = synapticon_hardware::ethercat;
using ec::BusConfig;

static_assert(BENCH_RATE_HZ == BusConfig{}.control_frequency, "the bench runs at the default bus rate");
using ec::JointConfig;

/// The HAL under test with the SDO reads, the streamed outputs and the cycles a bring-up test checks.
class ProbedHal : public EthercatHalUnderTest
{
public:
  using EthercatHalUnderTest::sdo_u32;

  double streamed(std::size_t slot) const
  {
    return this->outputs_[0][slot];
  }

  /// The angle of one position increment (0x6064) of joint `joint` [rad].
  double position_step(std::size_t joint) const
  {
    return std::abs(ec::si_factor(this->si_factors_[joint], ec::Unit::ANGLE));
  }

  /// The velocity of one raw velocity unit (0x60FF) of joint `joint` [rad/s].
  double velocity_step(std::size_t joint) const
  {
    return std::abs(ec::si_factor(this->si_factors_[joint], ec::Unit::ANGULAR_VELOCITY));
  }

  /// The joint's position after a few cycles that write only what start() left in the outputs.
  double position()
  {
    for (int cycle = 0; cycle < 10; ++cycle)
    {
      EXPECT_TRUE(this->exchange_read() && this->exchange_write());
      std::this_thread::sleep_for(std::chrono::duration<double>(DT));
    }
    std::vector<DriveState> states;
    EXPECT_TRUE(this->read(states));
    return states.at(0).position;
  }
};

/// One entry of a slave_config: in `sdo:` with its raw value, or with `si` in `sdo_si:` with its value in SI units.
struct Sdo
{
  std::string index;  ///< e.g. "0x6085", lower case as in the configs
  int sub_index;
  std::string type;
  std::string value;
  bool si{ false };   ///< an `sdo_si:` entry, converted by bring-up into the drive's unit
  bool raw{ false };  ///< `raw: true`, for a raw value of an object whose unit depends on the drive
};

/// `sdo` as an entry of its list in a slave_config.
inline YAML::Node sdo_entry(const Sdo& sdo)
{
  YAML::Node entry(YAML::NodeType::Map);
  entry.SetStyle(YAML::EmitterStyle::Flow);
  entry["index"] = sdo.index;
  entry["sub_index"] = sdo.sub_index;
  entry["type"] = sdo.type;
  entry[sdo.si ? "si_value" : "value"] = sdo.value;
  if (sdo.raw)
  {
    entry["raw"] = true;
  }
  return entry;
}

/// True when the slave_config entry `entry` writes the object of `sdo`.
inline bool same_object(const YAML::Node& entry, const Sdo& sdo)
{
  return entry["index"] && entry["sub_index"] &&
         entry["index"].as<unsigned long>() == std::stoul(sdo.index, nullptr, 16) &&
         entry["sub_index"].as<int>() == sdo.sub_index;
}

inline std::string base_config_path()
{
  return env_or("SYNAPTICON_HIL_SLAVE_CONFIG", ament_index_cpp::get_package_share_directory("synapticon_hardware") +
                                                   "/config/example_somanet_circulo_safe_motion.yaml");
}

/// The slave_config `base` with `overrides` replacing any entry of the same object, written to a temporary file.
/// The overrides go first into the `sdo:` list, or into the `sdo_si:` list for an SI entry. Returns its path.
inline std::string write_config(const std::vector<Sdo>& overrides, const std::string& name, const std::string& base)
{
  YAML::Node config = YAML::LoadFile(base);
  for (const bool si : { false, true })
  {
    const char* key = si ? "sdo_si" : "sdo";
    YAML::Node entries(YAML::NodeType::Sequence);
    for (const auto& sdo : overrides)
    {
      if (sdo.si == si)
      {
        entries.push_back(sdo_entry(sdo));
      }
    }
    for (const auto& entry : config[key])
    {
      const auto replaces_entry = [&entry](const Sdo& sdo) { return same_object(entry, sdo); };
      if (std::none_of(overrides.begin(), overrides.end(), replaces_entry))
      {
        entries.push_back(entry);
      }
    }
    config[key] = entries;
  }
  YAML::Emitter out;
  out << config;
  const std::string path = ::testing::TempDir() + "hil_" + name + ".yaml";
  std::ofstream(path) << out.c_str() << "\n";
  return path;
}

/// Following error window of the REPORT and FAULT modes [rad]: about 1000 increments of a geared SOMANET Circulo.
inline constexpr double FOLLOWING_ERROR_WINDOW_RAD = 0.006;
/// Set up `hal` on the bench's bus `bus` with `joints`. Each HAL's test executable defines it.
bool set_up_hal(EthercatHalUnderTest& hal, const BusConfig& bus, const std::vector<JointConfig>& joints);

/// The brake objects that decide whether a disable_voltage waits for the brake.
inline constexpr ec::CoeAddress BRAKE_RELEASE_STRATEGY{ 0x2004, 4 };    ///< 0: the drive does not control a brake
inline constexpr ec::CoeAddress CONTROLLER_DISABLE_DELAY{ 0x2004, 5 };  ///< ms the drive keeps control after a disable

/// The following error objects for `mode`. DEFAULT is the vendor default (report only, window off, 100 ms). REPORT
/// and FAULT make a small commanded jump trip within a cycle (window FOLLOWING_ERROR_WINDOW_RAD, 1 ms).
inline std::vector<Sdo> following_error_objects(FollowingError mode)
{
  if (mode == FollowingError::DEFAULT)
  {
    // "Window off" is the all-ones raw value, which no angle expresses. The SDO parser reads values as int, so it is
    // written as -1.
    return { { "0x2017", 1, "uint8", "0" },
             { "0x6065", 0, "uint32", "-1", false, true },
             { "0x6066", 0, "uint16", "100" } };
  }
  const std::string faults = mode == FollowingError::FAULT ? "1" : "0";
  return { { "0x2017", 1, "uint8", faults },
           { "0x6065", 0, "uint32", std::to_string(FOLLOWING_ERROR_WINDOW_RAD), true },
           { "0x6066", 0, "uint16", "1" } };
}

inline uint16_t env_u16(const char* name, const std::string& fallback)
{
  return static_cast<uint16_t>(std::stoul(env_or(name, fallback)));
}

/// @brief The robot definition that SYNAPTICON_HIL_ROBOT names, merged the way the HALs merge it.
///
/// Its bus entries become hardware parameters, and each joint gets its parameters. A joint that names no slave_config
/// gets base_config_path(). Empty without SYNAPTICON_HIL_ROBOT.
/// @throws std::runtime_error when the file is no valid robot definition.
inline HalConfig robot_config()
{
  HalConfig config;
  const std::string path = env_or("SYNAPTICON_HIL_ROBOT", "");
  if (path.empty())
  {
    return config;
  }
  config.hardware_parameters[ec::ROBOT_CONFIG_PARAM] = path;
  config.hardware_parameters[ec::DEFAULT_SLAVE_CONFIG_PARAM] = base_config_path();
  try
  {
    for (const auto& entry : YAML::LoadFile(path)["joints"])
    {
      config.joints.push_back({ entry["name"].as<std::string>(), {} });
    }
  }
  catch (const YAML::Exception& e)
  {
    throw std::runtime_error("robot definition '" + path + "': " + e.what());
  }
  std::string error;
  if (!ec::apply_robot_config(config, error))
  {
    throw std::runtime_error(error);
  }
  return config;
}

/// The joints of the robot definition, read the way the HALs read them. Empty without SYNAPTICON_HIL_ROBOT.
/// @throws std::runtime_error or std::invalid_argument when the file is no valid robot definition.
inline std::vector<JointConfig> robot_joints()
{
  std::vector<JointConfig> joints;
  for (const auto& spec : robot_config().joints)
  {
    joints.push_back(ec::joint_from(spec.name, spec.parameters));
  }
  return joints;
}

/// The robot definition's bus entry `key`, or `fallback` when it has none.
inline std::string robot_bus_parameter(const std::string& key, const std::string& fallback)
{
  const auto parameters = robot_config().hardware_parameters;
  const auto found = parameters.find(key);
  return found != parameters.end() ? found->second : fallback;
}

/// The bench's bus: the robot definition's bus settings, at the bus rate the suite's timing is written for.
/// @throws std::invalid_argument for a bus setting out of range.
inline BusConfig bench_bus_config()
{
  BusConfig bus = ec::bus_from(robot_config().hardware_parameters);
  bus.control_frequency = BENCH_RATE_HZ;
  return bus;
}

/// @brief Drive `index` of the bench (0 or 1) in `mode`, on its base slave_config.
///
/// It is the joint of the same index in the robot definition, or a joint on base_config_path() when there is none.
/// SYNAPTICON_HIL_ALIAS, SYNAPTICON_HIL_POSITION and SYNAPTICON_HIL_POSITION_2 override what the file gives.
/// An impedance joint keeps the file's stiffness, damping and torque ceiling, and takes those of the example robot
/// definitions for any the file leaves out. In the other modes the drive keeps its own.
/// @throws std::runtime_error for an impedance joint with own test gains but without a torque ceiling: the example's
/// 50 Nm suits only the default test gains.
inline JointConfig bench_drive(std::size_t index, int mode)
{
  const auto joints = robot_joints();
  JointConfig joint;
  if (index < joints.size())
  {
    joint = joints[index];
  }
  else
  {
    joint.name = "joint_" + std::to_string(index + 1);
    joint.slave_config_file = base_config_path();
    joint.position = index == 0 ? 0 : static_cast<uint16_t>(bench_drive(0, mode).position + 1);
  }
  const char* position_variable = index == 0 ? "SYNAPTICON_HIL_POSITION" : "SYNAPTICON_HIL_POSITION_2";
  joint.position = env_u16(position_variable, std::to_string(joint.position));
  joint.alias = env_u16("SYNAPTICON_HIL_ALIAS", std::to_string(joint.alias));
  joint.mode_of_operation = mode;
  const bool impedance = mode == ec::mode::IMPEDANCE;
  if (impedance && std::isnan(joint.max_joint_torque) && own_test_gains(test_gains(index)))
  {
    throw std::runtime_error("joint '" + joint.name +
                             "': the robot definition gives it own hil_tests gains, so it must also give its torque "
                             "ceiling (max_joint_torque). The default of 50 Nm suits only the default gains.");
  }
  const auto fill = [impedance](double& value, double example) {
    value = !impedance ? QUIET_NAN : std::isnan(value) ? example : value;
  };
  fill(joint.stiffness, 0.4);
  fill(joint.damping, 0.1);
  fill(joint.max_joint_torque, 50.0);
  return joint;
}

/// The name of the running test, for the temporary slave_config files. The '/' of a parameterized test's name
/// becomes '_', so the name stays one file name.
inline std::string test_name()
{
  const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
  std::string name = info != nullptr ? info->name() : "suite";
  std::replace(name.begin(), name.end(), '/', '_');
  return name;
}

/// @brief SOMANET drives on the bus of the HAL under test, set up through slave_config SDO entries.
class EthercatBench : public HalBench
{
public:
  explicit EthercatBench(bool two_drives) : two_drives_(two_drives)
  {
  }

  std::shared_ptr<Cia402Hal> open_drives(const std::vector<DriveSettings>& drives) override
  {
    finished_ = false;  // this bring-up may change what finish() restores
    std::vector<std::vector<Sdo>> sdo;
    for (std::size_t j = 0; j < drives.size(); ++j)
    {
      sdo.push_back(overrides(drives[j]));
    }
    return connect_with_sdo(drives, sdo);
  }

  void released() override
  {
    // Right after a release of the bus, the drive's mailbox is not ready for the next bring-up.
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }

  /// Note of each drive what the stack read: its quick stop ramp, to restore at the end, and its velocity step. Then
  /// read whether a disable_voltage waits for its brake. Drives of different models differ in all of them.
  bool prepare() override
  {
    if (!learned_.empty())
    {
      return true;
    }
    std::vector<LearnedDrive> learned(two_drives_ ? 2 : 1);
    const std::vector<DriveSettings> defaults(learned.size());
    auto hal = connect_with_sdo(defaults, overrides(defaults));
    if (!hal)
    {
      return false;
    }
    for (std::size_t j = 0; j < learned.size(); ++j)
    {
      learned[j].original_deceleration = hal->quick_stop_deceleration(j);
      learned[j].velocity_step = hal->velocity_step(j);
    }
    release(hal);
    for (std::size_t j = 0; j < learned.size(); ++j)
    {
      learned[j].disable_delay = brake_disable_delay(j);
    }
    learned_ = std::move(learned);
    return true;
  }

  /// The brake's controller disable delay of drive `index` [s], 0 when the drive controls no brake. Read on a bus
  /// that is set up but not started: a started bus answers SDO reads only while it exchanges cycles.
  double brake_disable_delay(std::size_t index)
  {
    JointConfig joint = bench_drive(index, ec::mode::CSP);
    joint.slave_config_file =
        write_config(overrides(DriveSettings{}), "brake_" + std::to_string(index), joint.slave_config_file);
    auto hal = std::make_shared<ProbedHal>();
    double delay = 0.0;
    if (set_up_hal(*hal, bench_bus_config(), { joint }) && hal->connect())
    {
      const auto strategy = hal->sdo_u32(joint.position, BRAKE_RELEASE_STRATEGY);
      const auto delay_ms = hal->sdo_u32(joint.position, CONTROLLER_DISABLE_DELAY);
      delay = strategy && *strategy != 0 && delay_ms ? *delay_ms / 1000.0 : 0.0;
    }
    release(hal);
    return delay;
  }

  /// Restore each drive's quick stop deceleration found and the vendor following error defaults.
  void finish() override
  {
    if (learned_.empty() || finished_)
    {
      return;
    }
    const std::vector<DriveSettings> defaults(learned_.size());
    auto sdo = overrides(defaults);
    for (std::size_t j = 0; j < sdo.size(); ++j)
    {
      if (std::isfinite(learned_[j].original_deceleration))
      {
        sdo[j].push_back(quick_stop_deceleration(learned_[j].original_deceleration));
      }
    }
    if (auto hal = connect_with_sdo(defaults, sdo))
    {
      release(hal);
    }
    finished_ = true;
  }

  bool two_drives() const override
  {
    return two_drives_;
  }

  double disable_delay(std::size_t drive) const override
  {
    return drive < learned_.size() ? learned_[drive].disable_delay : 0.0;
  }

  double velocity_step(std::size_t drive) const override
  {
    return drive < learned_.size() ? learned_[drive].velocity_step : 0.0;
  }

private:
  /// What prepare() noted of one drive.
  struct LearnedDrive
  {
    /// The quick stop ramp the drive held [rad/s^2], restored at the end. NaN when it could not be read.
    double original_deceleration{ std::numeric_limits<double>::quiet_NaN() };
    double velocity_step{ 0.0 };  ///< velocity of one raw velocity unit [rad/s]
    double disable_delay{ 0.0 };  ///< the brake's controller disable delay [s], 0 without brake control
  };

  static Sdo quick_stop_option_code(int code)
  {
    return { "0x605a", 0, "int16", std::to_string(code) };
  }
  static Sdo quick_stop_deceleration(double rad_per_s2)
  {
    return { "0x6085", 0, "uint32", std::to_string(rad_per_s2), true };
  }

  /// The SDO entries for `drive`, the quick stop option code first. Bring-up converts the SI entries into each
  /// drive's units.
  static std::vector<Sdo> overrides(const DriveSettings& drive)
  {
    std::vector<Sdo> sdo{ quick_stop_option_code(drive.quick_stop_option_code) };
    if (std::isfinite(drive.quick_stop_deceleration))
    {
      sdo.push_back(quick_stop_deceleration(drive.quick_stop_deceleration));
    }
    const auto following_error = following_error_objects(drive.following_error);
    sdo.insert(sdo.end(), following_error.begin(), following_error.end());
    return sdo;
  }

  /// The SDO entries of each drive, entry j for `drives[j]`.
  static std::vector<std::vector<Sdo>> overrides(const std::vector<DriveSettings>& drives)
  {
    std::vector<std::vector<Sdo>> sdo;
    for (const DriveSettings& drive : drives)
    {
      sdo.push_back(overrides(drive));
    }
    return sdo;
  }

  /// Bring the drives up with their SDO entries, retrying a failed bring-up. A bring-up that reports another quick
  /// stop option code than asked for counts as failed.
  std::shared_ptr<ProbedHal> connect_with_sdo(const std::vector<DriveSettings>& drives,
                                              const std::vector<std::vector<Sdo>>& sdo)
  {
    for (int attempt = 0; attempt < 3; ++attempt)
    {
      auto hal = connect_once(drives, sdo);
      if (!hal)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        continue;
      }
      bool taken = true;
      for (std::size_t j = 0; j < drives.size(); ++j)
      {
        taken = taken && hal->quick_stop_option_code(j) == std::stoi(sdo[j].front().value);
      }
      if (taken)
      {
        return hal;
      }
      release(hal);
    }
    return nullptr;
  }

  std::shared_ptr<ProbedHal> connect_once(const std::vector<DriveSettings>& drives,
                                          const std::vector<std::vector<Sdo>>& sdo)
  {
    BusConfig bus = bench_bus_config();
    bus.quick_stop_hold = drives.front().quick_stop_hold;
    std::vector<JointConfig> joints;
    for (std::size_t j = 0; j < drives.size(); ++j)
    {
      JointConfig joint = bench_drive(j, drives[j].mode);
      joint.energize = drives[j].energize;
      joint.slave_config_file = write_config(sdo[j], test_name() + "_" + joint.name, joint.slave_config_file);
      joints.push_back(joint);
    }
    auto hal = std::make_shared<ProbedHal>();
    if (!set_up_hal(*hal, bus, joints) || !hal->connect() || !hal->start())
    {
      return nullptr;
    }
    return hal;
  }

  void release(std::shared_ptr<ProbedHal>& hal)
  {
    hal->stop();
    hal->disconnect();
    hal.reset();
    released();
  }

  bool two_drives_;
  bool finished_{ false };
  std::vector<LearnedDrive> learned_;  ///< one per drive, filled by prepare()
};

/// Bring-up reads back the quick stop deceleration (0x6085) it wrote.
TEST_F(SingleDrive, BringUpReadsBackTheQuickStopDeceleration)
{
  ASSERT_TRUE(start({ .quick_stop_deceleration = 20.0 }));
  const auto hal = std::dynamic_pointer_cast<ec::EthercatPdoHal>(drive_);
  ASSERT_TRUE(hal);
  EXPECT_NEAR(hal->quick_stop_deceleration(0), 20.0, 0.5);
}

/// @brief Bring-ups of the drive at the bench position with one change to its joint or its slave_config, each one
/// checked for what bring-up reports. Nothing here energises the drive.
///
/// A test that changes a drive object writes back what it found before it ends.
class EthercatBringUp : public ::testing::Test
{
protected:
  void SetUp() override
  {
    SKIP_AFTER_STOP_REQUEST();
  }

  /// The joint of the bench drive in `mode`, on its base slave_config.
  static JointConfig bench_joint(int mode = ec::mode::CSP)
  {
    return bench_drive(0, mode);
  }

  static BusConfig bench_bus()
  {
    return bench_bus_config();
  }

  /// Bring up `joint` with its base slave_config and `sdo`, started when `start` is set. Nothing when the bring-up
  /// fails. 0x605A = 6 goes first, as in the suite.
  static std::shared_ptr<ProbedHal> connected_hal(JointConfig joint, const std::vector<Sdo>& sdo = {},
                                                  bool start = false)
  {
    std::vector<Sdo> entries{ { "0x605a", 0, "int16", "6" } };
    entries.insert(entries.end(), sdo.begin(), sdo.end());
    joint.slave_config_file = write_config(entries, test_name(), joint.slave_config_file);
    for (int attempt = 0; attempt < 3; ++attempt)
    {
      auto hal = std::make_shared<ProbedHal>();
      if (set_up_hal(*hal, bench_bus(), { joint }) && hal->connect() && hal->quick_stop_option_code(0) == 6 &&
          (!start || hal->start()))
      {
        return hal;
      }
      release(hal);
    }
    return nullptr;
  }

  /// Stop and release the bus, then give the drive's mailbox time before the next bring-up.
  static void release(std::shared_ptr<ProbedHal>& hal)
  {
    if (hal)
    {
      hal->stop();
      hal->disconnect();
      hal.reset();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }

  /// One object of the bench drive, read in a bring-up without changes. Nothing when it cannot be read.
  static std::optional<uint32_t> read_object(ec::CoeAddress object, int mode = ec::mode::CSP)
  {
    auto hal = connected_hal(bench_joint(mode));
    const auto value = hal ? hal->sdo_u32(bench_joint().position, object) : std::nullopt;
    release(hal);
    return value;
  }
};

/// An inverted joint reads the position of the same joint at rest with the opposite sign.
TEST_F(EthercatBringUp, AnInvertedJointReadsTheMirroredPosition)
{
  auto hal = connected_hal(bench_joint(), {}, true);
  ASSERT_TRUE(hal);
  const double position = hal->position();
  release(hal);

  JointConfig inverted = bench_joint();
  inverted.invert = true;
  hal = connected_hal(inverted, {}, true);
  ASSERT_TRUE(hal);
  EXPECT_NEAR(hal->position(), -position, 1e-3);
  release(hal);
}

/// The exchange starts again after a stop(), as on a deactivation and activation of the hardware component, several
/// times on the same connection. The pause lets the drive leave OP without process data, as it does while inactive.
TEST_F(EthercatBringUp, StartsAgainAfterAStop)
{
  auto hal = connected_hal(bench_joint(), {}, true);
  ASSERT_TRUE(hal);
  const double position = hal->position();
  for (int round = 1; round <= 4; ++round)
  {
    const bool stopped = hal->stop();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const bool restarted = stopped && hal->start();
    EXPECT_TRUE(restarted) << "round " << round;
    if (!restarted)
    {
      break;
    }
    EXPECT_NEAR(hal->position(), position, 1e-3) << "round " << round;
  }
  release(hal);
}

/// Without a configured limit the drive's MaxJointTorque is streamed, with one the configured value.
TEST_F(EthercatBringUp, AConfiguredTorqueLimitIsStreamedInsteadOfTheDrives)
{
  auto hal = connected_hal(bench_joint());
  ASSERT_TRUE(hal);
  const auto drive_limit = hal->sdo_u32(bench_joint().position, ec::coe::MAX_JOINT_TORQUE);
  ASSERT_TRUE(drive_limit);
  EXPECT_EQ(hal->streamed(ec::slot::TORQUE_LIMIT), static_cast<double>(*drive_limit));
  release(hal);

  JointConfig limited = bench_joint();
  limited.max_joint_torque = 2.0;
  hal = connected_hal(limited);
  ASSERT_TRUE(hal);
  EXPECT_DOUBLE_EQ(hal->streamed(ec::slot::TORQUE_LIMIT), 2000.0);  // mNm
  release(hal);
}

/// A drive with MaxJointTorque at 0 still comes up, with 0 streamed and a warning in the log.
TEST_F(EthercatBringUp, ADriveTorqueLimitOfZeroIsStreamedAsIs)
{
  const auto found = read_object(ec::coe::MAX_JOINT_TORQUE);
  ASSERT_TRUE(found);

  auto hal = connected_hal(bench_joint(), { { "0x2054", 0, "uint32", "0" } });
  ASSERT_TRUE(hal);
  EXPECT_EQ(hal->sdo_u32(bench_joint().position, ec::coe::MAX_JOINT_TORQUE), 0u);
  EXPECT_EQ(hal->streamed(ec::slot::TORQUE_LIMIT), 0.0);
  release(hal);

  hal = connected_hal(bench_joint(), { { "0x2054", 0, "uint32", std::to_string(*found) } });
  ASSERT_TRUE(hal);
  EXPECT_EQ(hal->sdo_u32(bench_joint().position, ec::coe::MAX_JOINT_TORQUE), found);
  release(hal);
}

/// Without configured gains, the slave_config leaves them alone: the drive's own stiffness and damping are streamed
/// until a command brings its own.
/// A value given in SI units reaches the drive in the drive's own unit: what the drive then holds, converted with the
/// unit the stack read, is the value within one raw step. Run for a velocity and a position object, on every drive
/// model of the bench, this checks the conversion against the real drive.
class SiValueReachesTheDrive : public EthercatBringUp,
                               public ::testing::WithParamInterface<std::pair<const char*, double>>
{
};

TEST_P(SiValueReachesTheDrive, InItsUnit)
{
  const auto [index, value] = GetParam();
  const ec::CoeAddress object{ static_cast<uint16_t>(std::stoul(index, nullptr, 16)), 0 };
  const auto found = read_object(object);
  ASSERT_TRUE(found);

  auto hal = connected_hal(bench_joint(), { { index, 0, "uint32", std::to_string(value), true } });
  ASSERT_TRUE(hal);
  const auto held = hal->sdo_u32(bench_joint().position, object);
  const double step = object.index == 0x6080 ? hal->velocity_step(0) : hal->position_step(0);
  ASSERT_TRUE(held);
  EXPECT_NEAR(*held * step, value, step) << "raw " << *held << ", one raw step " << step;
  release(hal);

  // The SDO parsers read values as int, so the found bits go back as a signed number.
  const std::string restored = std::to_string(static_cast<int32_t>(*found));
  hal = connected_hal(bench_joint(), { { index, 0, "uint32", restored, false, true } });
  ASSERT_TRUE(hal);
  EXPECT_EQ(hal->sdo_u32(bench_joint().position, object), found);
  release(hal);
}

INSTANTIATE_TEST_SUITE_P(Objects, SiValueReachesTheDrive,
                         ::testing::Values(std::pair{ "0x6080", 10.0 },   // max motor speed, rad/s
                                           std::pair{ "0x6065", 0.05 }),  // following error window, rad
                         [](const auto& info) {
                           return info.param.first == std::string("0x6080") ? "MaxMotorSpeed" : "FollowingErrorWindow";
                         });

TEST_F(EthercatBringUp, TheDrivesOwnImpedanceGainsAreStreamed)
{
  auto hal = connected_hal(bench_joint());
  ASSERT_TRUE(hal);
  const auto stiffness = hal->sdo_u32(bench_joint().position, ec::coe::IMPEDANCE_STIFFNESS);
  const auto damping = hal->sdo_u32(bench_joint().position, ec::coe::IMPEDANCE_DAMPING);
  release(hal);
  ASSERT_TRUE(stiffness && damping);

  const std::vector<Sdo> marked{ { "0x2060", 1, "int32", std::to_string(*stiffness + 123) },
                                 { "0x2060", 2, "int32", std::to_string(*damping + 45) } };
  hal = connected_hal(bench_joint(), marked);
  ASSERT_TRUE(hal);
  release(hal);

  // The drive keeps the marked values until a power cycle, so a bring-up that wrote gains would overwrite them.
  hal = connected_hal(bench_joint());
  ASSERT_TRUE(hal);
  EXPECT_EQ(hal->streamed(ec::slot::STIFFNESS), static_cast<double>(*stiffness + 123));
  EXPECT_EQ(hal->streamed(ec::slot::DAMPING), static_cast<double>(*damping + 45));
  release(hal);

  hal = connected_hal(bench_joint(), { { "0x2060", 1, "int32", std::to_string(*stiffness) },
                                       { "0x2060", 2, "int32", std::to_string(*damping) } });
  ASSERT_TRUE(hal);
  EXPECT_EQ(hal->sdo_u32(bench_joint().position, ec::coe::IMPEDANCE_STIFFNESS), stiffness);
  EXPECT_EQ(hal->sdo_u32(bench_joint().position, ec::coe::IMPEDANCE_DAMPING), damping);
  release(hal);
}

/// Configured gains replace the drive's from the start, in raw steps of mNm/rad and mNm s/rad.
TEST_F(EthercatBringUp, ConfiguredGainsAreStreamedInsteadOfTheDrives)
{
  JointConfig configured = bench_joint();
  configured.stiffness = 1.234;
  configured.damping = 0.056;
  auto hal = connected_hal(configured);
  ASSERT_TRUE(hal);
  EXPECT_NEAR(hal->streamed(ec::slot::STIFFNESS), 1234.0, 1e-6);
  EXPECT_NEAR(hal->streamed(ec::slot::DAMPING), 56.0, 1e-6);
  release(hal);
}

/// An impedance joint without a stiffness, damping or torque ceiling is refused before the bus goes active, as the
/// drive would otherwise run on whatever it stored last.
TEST_F(EthercatBringUp, AnImpedanceJointWithoutCommandDefaultsIsRefused)
{
  for (const auto clear : { &JointConfig::stiffness, &JointConfig::damping, &JointConfig::max_joint_torque })
  {
    JointConfig impedance = bench_joint(ec::mode::IMPEDANCE);
    impedance.*clear = std::numeric_limits<double>::quiet_NaN();
    impedance.slave_config_file =
        write_config({ { "0x605a", 0, "int16", "6" } }, test_name(), impedance.slave_config_file);
    auto hal = std::make_shared<ProbedHal>();
    ASSERT_TRUE(set_up_hal(*hal, bench_bus(), { impedance }));
    EXPECT_FALSE(hal->connect());
    release(hal);
  }
}

/// With the torque sensor type set to None, an impedance bring-up is refused before the bus goes active.
TEST_F(EthercatBringUp, ImpedanceWithoutATorqueSensorIsRefused)
{
  const auto found = read_object(ec::coe::TORQUE_SENSOR_TYPE);
  ASSERT_TRUE(found);
  if (*found == ec::coe::NO_TORQUE_SENSOR)
  {
    GTEST_SKIP() << "the bench drive has no torque sensor configured";
  }

  JointConfig impedance = bench_joint(ec::mode::IMPEDANCE);
  impedance.slave_config_file = write_config({ { "0x605a", 0, "int16", "6" }, { "0x2150", 1, "uint8", "0" } },
                                             test_name(), impedance.slave_config_file);
  // Bring-up reads every startup SDO back, so the sensor type is None when the impedance check runs.
  {
    auto hal = std::make_shared<ProbedHal>();
    ASSERT_TRUE(set_up_hal(*hal, bench_bus(), { impedance }));
    EXPECT_FALSE(hal->connect());
    release(hal);
  }

  auto hal = connected_hal(bench_joint(), { { "0x2150", 1, "uint8", std::to_string(*found) } });
  ASSERT_TRUE(hal);
  EXPECT_EQ(hal->sdo_u32(bench_joint().position, ec::coe::TORQUE_SENSOR_TYPE), found);
  release(hal);
}

/// A joint at a ring position without a drive fails the bring-up.
TEST_F(EthercatBringUp, AJointWithoutADriveFailsTheBringUp)
{
  JointConfig missing = bench_joint();
  missing.position = 99;
  auto hal = std::make_shared<ProbedHal>();
  ASSERT_TRUE(set_up_hal(*hal, bench_bus(), { missing }));
  EXPECT_FALSE(hal->connect());
  release(hal);
}

/// A slave_config that cannot be read fails the bring-up.
TEST_F(EthercatBringUp, AMissingSlaveConfigFailsTheBringUp)
{
  JointConfig joint = bench_joint();
  joint.slave_config_file = ::testing::TempDir() + "no_such_slave_config.yaml";
  auto hal = std::make_shared<ProbedHal>();
  ASSERT_TRUE(set_up_hal(*hal, bench_bus(), { joint }));
  EXPECT_FALSE(hal->connect());
  release(hal);
}

/// The main() of a HIL test executable: the suite and the bring-up tests on the HAL under test.
inline int run_ethercat_hil_tests(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);  // leaves the arguments it does not know
  bool two_drives = false;
  int kept = 1;
  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "--two-drives")
    {
      two_drives = true;
    }
    else
    {
      argv[kept++] = argv[i];
    }
  }
  argc = kept;
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  install_stop_handlers();
  bench_instance() = std::make_shared<EthercatBench>(two_drives);
  const int result = RUN_ALL_TESTS();
  bench().finish();
  rclcpp::shutdown();
  return result;
}

}  // namespace synapticon_hardware::testing
