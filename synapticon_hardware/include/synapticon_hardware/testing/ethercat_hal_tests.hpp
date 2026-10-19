// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The hardware-in-the-loop run of an EtherCAT HAL: the drive action suite on SOMANET drives, set up through
/// slave_config SDO entries, plus the bring-up tests every EtherCAT HAL shares.
///
/// An executable includes this header after it declared, in synapticon_hardware::testing, the HAL it tests and how
/// to set it up on the bench's bus:
///
///   using EthercatHalUnderTest = <a HAL derived from ethercat::EthercatPdoHal>;
///   bool set_up_hal(EthercatHalUnderTest& hal, const ethercat::BusConfig& bus,
///                   const std::vector<ethercat::JointConfig>& joints);
///
/// and its main() returns run_ethercat_hil_tests(argc, argv). Each test's drive settings become SDO entries of a
/// slave_config derived from a base config. The suite writes the quick stop objects and the following error objects
/// (the vendor defaults, except in the fault tests), and restores the quick stop deceleration it found at the end.
/// The bring-up tests set MaxJointTorque and the torque sensor type for one bring-up each and write back what they
/// found. Drives keep SDO values until a power cycle.
///
/// Environment (all optional):
///   SYNAPTICON_HIL_SLAVE_CONFIG  base slave_config (default: the installed example config)
///   SYNAPTICON_HIL_ALIAS         slave alias (default 0)
///   SYNAPTICON_HIL_POSITION      slave ring position (default 0)
///   SYNAPTICON_HIL_POSITION_2    second drive's ring position, for --two-drives (default: +1)
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
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/ethercat/bus_config.hpp"
#include "synapticon_hardware/ethercat/coe_objects.hpp"
#include "synapticon_hardware/ethercat/ethercat_pdo_hal.hpp"
#include "synapticon_hardware/ethercat/joint_config.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"
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

/// One `sdo:` entry of a slave_config.
struct Sdo
{
  std::string index;  ///< e.g. "0x6085", lower case as in the configs
  int sub_index;
  std::string type;
  std::string value;
};

inline std::string base_config_path()
{
  return env_or("SYNAPTICON_HIL_SLAVE_CONFIG", ament_index_cpp::get_package_share_directory("synapticon_hardware") +
                                                   "/config/example_somanet_circulo_safe_motion.yaml");
}

/// The base slave_config with `overrides` replacing any entry of the same index, written to a temporary file.
/// Returns its path.
inline std::string write_config(const std::vector<Sdo>& overrides, const std::string& name)
{
  std::ifstream in(base_config_path());
  std::stringstream out;
  std::string line;
  while (std::getline(in, line))
  {
    bool replaced = false;
    for (const auto& sdo : overrides)
    {
      if (line.find("index: " + sdo.index + ",") != std::string::npos &&
          line.find("sub_index: " + std::to_string(sdo.sub_index) + ",") != std::string::npos &&
          line.find("_interface") == std::string::npos)  // a PDO channel, not an SDO entry
      {
        replaced = true;
      }
    }
    if (replaced)
    {
      continue;
    }
    out << line << "\n";
    if (line.rfind("sdo:", 0) == 0)
    {
      for (const auto& sdo : overrides)
      {
        out << "  - { index: " << sdo.index << ", sub_index: " << sdo.sub_index << ", type: " << sdo.type
            << ", value: " << sdo.value << " }\n";
      }
    }
  }
  const std::string path = ::testing::TempDir() + "hil_" + name + ".yaml";
  std::ofstream(path) << out.str();
  return path;
}

/// The following error objects for `mode`. DEFAULT is the vendor default (report only, window off,
/// 100 ms). REPORT and FAULT make a small commanded jump trip within a cycle (window 1000 increments, 1 ms).
inline std::vector<Sdo> following_error_objects(FollowingError mode)
{
  if (mode == FollowingError::DEFAULT)
  {
    // The SDO parser reads values as int, so the all-ones "window off" is written as -1.
    return { { "0x2017", 1, "uint8", "0" }, { "0x6065", 0, "uint32", "-1" }, { "0x6066", 0, "uint16", "100" } };
  }
  const std::string faults = mode == FollowingError::FAULT ? "1" : "0";
  return { { "0x2017", 1, "uint8", faults }, { "0x6065", 0, "uint32", "1000" }, { "0x6066", 0, "uint16", "1" } };
}

inline uint16_t env_u16(const char* name, const std::string& fallback)
{
  return static_cast<uint16_t>(std::stoul(env_or(name, fallback)));
}

/// The name of the running test, for the temporary slave_config files.
inline std::string test_name()
{
  const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
  return info != nullptr ? info->name() : "suite";
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
    std::vector<std::vector<Sdo>> sdo;
    for (const auto& drive : drives)
    {
      sdo.push_back(overrides(drive));
    }
    return connect_with_sdo(drives, sdo);
  }

  void released() override
  {
    // Right after a release of the bus, the drive's mailbox is not ready for the next bring-up.
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }

  /// Learn the drive's 0x6085 unit and its present value from two read-backs.
  bool prepare() override
  {
    if (std::isfinite(rad_per_raw_))
    {
      return true;
    }
    auto hal = connect_with_sdo({ DriveSettings{} }, { overrides(DriveSettings{}) });
    if (!hal)
    {
      return false;
    }
    const double found = hal->quick_stop_deceleration(0);
    release(hal);
    std::vector<Sdo> sdo = overrides(DriveSettings{});
    sdo.push_back(quick_stop_deceleration(100000));
    hal = connect_with_sdo({ DriveSettings{} }, { sdo });
    if (!hal)
    {
      return false;
    }
    rad_per_raw_ = hal->quick_stop_deceleration(0) / 100000.0;
    release(hal);
    original_raw_ = static_cast<uint32_t>(std::lround(found / rad_per_raw_));
    return std::isfinite(rad_per_raw_) && rad_per_raw_ > 0.0;
  }

  /// Restore the quick stop deceleration found and the vendor following error defaults.
  void finish() override
  {
    if (!std::isfinite(rad_per_raw_) || finished_)
    {
      return;
    }
    std::vector<Sdo> sdo = overrides(DriveSettings{});
    sdo.push_back(quick_stop_deceleration(original_raw_));
    if (auto hal = connect_with_sdo({ DriveSettings{} }, { sdo }))
    {
      release(hal);
    }
    finished_ = true;
  }

  bool two_drives() const override
  {
    return two_drives_;
  }

private:
  static Sdo quick_stop_option_code(int code)
  {
    return { "0x605a", 0, "int16", std::to_string(code) };
  }
  static Sdo quick_stop_deceleration(uint32_t raw)
  {
    return { "0x6085", 0, "uint32", std::to_string(raw) };
  }

  /// The SDO entries for `drive`, the quick stop option code first: the first config SDO of a
  /// bring-up is occasionally lost, and this one's read-back shows whether it was.
  std::vector<Sdo> overrides(const DriveSettings& drive) const
  {
    std::vector<Sdo> sdo{ quick_stop_option_code(drive.quick_stop_option_code) };
    if (std::isfinite(drive.quick_stop_deceleration))
    {
      sdo.push_back(
          quick_stop_deceleration(static_cast<uint32_t>(std::lround(drive.quick_stop_deceleration / rad_per_raw_))));
    }
    const auto following_error = following_error_objects(drive.following_error);
    sdo.insert(sdo.end(), following_error.begin(), following_error.end());
    return sdo;
  }

  /// Bring the drives up with their SDO entries, retrying a failed bring-up or a lost first SDO.
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
    BusConfig bus;
    bus.quick_stop_hold = drives.front().quick_stop_hold;
    const uint16_t first = env_u16("SYNAPTICON_HIL_POSITION", "0");
    const uint16_t positions[] = { first, env_u16("SYNAPTICON_HIL_POSITION_2", std::to_string(first + 1)) };
    std::vector<JointConfig> joints(drives.size());
    for (std::size_t j = 0; j < drives.size(); ++j)
    {
      joints[j].name = "joint_" + std::to_string(j + 1);
      joints[j].alias = env_u16("SYNAPTICON_HIL_ALIAS", "0");
      joints[j].position = positions[j];
      joints[j].slave_config_file = write_config(sdo[j], test_name() + "_" + joints[j].name);
      joints[j].mode_of_operation = drives[j].mode;
      joints[j].energize = drives[j].energize;
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
  double rad_per_raw_{ std::numeric_limits<double>::quiet_NaN() };  ///< quick stop deceleration per 0x6085 unit
  uint32_t original_raw_{ 0 };                                      ///< 0x6085 as found, restored at the end
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
  /// The joint of the bench drive in `mode`, before a slave_config is chosen.
  static JointConfig bench_joint(int mode = ec::mode::CSP)
  {
    JointConfig joint;
    joint.name = "joint_1";
    joint.alias = env_u16("SYNAPTICON_HIL_ALIAS", "0");
    joint.position = env_u16("SYNAPTICON_HIL_POSITION", "0");
    joint.mode_of_operation = mode;
    return joint;
  }

  static BusConfig bench_bus()
  {
    return BusConfig{};
  }

  /// Bring up `joint` with the base slave_config and `sdo`, started when `start` is set. Nothing when the bring-up
  /// fails. 0x605A = 6 goes first, as in the suite, so a lost first SDO does not hide one a test asked for.
  static std::shared_ptr<ProbedHal> connected_hal(JointConfig joint, const std::vector<Sdo>& sdo = {},
                                                  bool start = false)
  {
    std::vector<Sdo> entries{ { "0x605a", 0, "int16", "6" } };
    entries.insert(entries.end(), sdo.begin(), sdo.end());
    joint.slave_config_file = write_config(entries, test_name());
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

/// until a command brings its own.
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
  impedance.slave_config_file =
      write_config({ { "0x605a", 0, "int16", "6" }, { "0x2150", 1, "uint8", "0" } }, test_name());
  bool refused = false;
  for (int attempt = 0; attempt < 3 && !refused; ++attempt)
  {
    auto hal = std::make_shared<ProbedHal>();
    ASSERT_TRUE(set_up_hal(*hal, bench_bus(), { impedance }));
    refused = !hal->connect();  // a lost SDO leaves the sensor configured and the bring-up passes
    release(hal);
  }
  EXPECT_TRUE(refused);

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
  missing.slave_config_file = base_config_path();
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
  rclcpp::init(argc, argv);
  bench_instance() = std::make_shared<EthercatBench>(two_drives);
  const int result = RUN_ALL_TESTS();
  bench().finish();
  rclcpp::shutdown();
  return result;
}

}  // namespace synapticon_hardware::testing
