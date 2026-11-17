// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The bring-up checks of the EtherCAT HALs, against a scripted object dictionary instead of a drive.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/logger.hpp>

#include "synapticon_hardware/ethercat/coe_objects.hpp"
#include "synapticon_hardware/ethercat/drive_checks.hpp"
#include "../log_capture.hpp"

namespace
{
using namespace synapticon_hardware::ethercat;

constexpr int CSP = mode::CSP;
constexpr int IMPEDANCE = mode::IMPEDANCE;
constexpr double RAD_PER_RPM = 2.0 * M_PI / 60.0;
constexpr uint32_t ABORT_NO_OBJECT = 0x06020000;

/// One joint at ring position `position` in `mode`. An impedance joint gets the command defaults it needs.
JointConfig joint(uint16_t position = 0, int mode = CSP)
{
  JointConfig config;
  config.name = "joint_" + std::to_string(position + 1);
  config.position = position;
  config.slave_config_file = "unused.yaml";
  config.mode_of_operation = mode;
  if (mode == IMPEDANCE)
  {
    config.stiffness = 0.4;
    config.damping = 0.1;
    config.max_joint_torque = 50.0;
  }
  return config;
}

/// @brief Drives that answer SDO reads from a scripted dictionary and keep what is written to it.
class ScriptedDrives : public SdoReader, public SdoWriter
{
public:
  /// The objects of a healthy drive at `position`: CSP and impedance ready, firmware 5.6.11.
  void healthy_drive(uint16_t position = 0)
  {
    put(coe::POSITION_ENCODER, 1, position);
    put(coe::COMMUTATION_ENCODER, 1, position);
    put(coe::ENCODER_1_RESOLUTION, 4096, position);
    put(coe::VELOCITY_UNIT, 0x00B44700, position);  // rev/min
    put(coe::FEED_CONSTANT_FEED, FEED_CONSTANT_DISABLED, position);
    put(coe::RATED_TORQUE, 1000, position);  // 1 Nm
    put(coe::IMPEDANCE_STIFFNESS, 400, position);
    put(coe::IMPEDANCE_DAMPING, 5, position);
    put(coe::MAX_JOINT_TORQUE, 50000, position);  // 50 Nm
    put(coe::GEAR_MOTOR_REVOLUTIONS, 100, position);
    put(coe::GEAR_SHAFT_REVOLUTIONS, 1, position);
    put(coe::TORQUE_SENSOR_TYPE, 1, position);
    put(coe::QUICK_STOP_OPTION, 6, position);
    put(coe::QUICK_STOP_DECELERATION, 3000, position);
    put(coe::FOLLOWING_ERROR_REACTION, 0, position);
    put(coe::FOLLOWING_ERROR_WINDOW, coe::FOLLOWING_ERROR_CHECK_OFF, position);
    put_text(coe::SOFTWARE_VERSION, "v5.6.11", position);
  }

  void put(CoeAddress object, uint32_t value, uint16_t position = 0)
  {
    objects_[{ position, object.index, object.subindex }] = { static_cast<uint8_t>(value),
                                                              static_cast<uint8_t>(value >> 8),
                                                              static_cast<uint8_t>(value >> 16),
                                                              static_cast<uint8_t>(value >> 24) };
  }

  /// A visible string, padded with a space and NULs as a drive may send it.
  void put_text(CoeAddress object, const std::string& text, uint16_t position = 0)
  {
    std::vector<uint8_t> bytes(text.begin(), text.end());
    bytes.insert(bytes.end(), { ' ', 0, 0 });
    objects_[{ position, object.index, object.subindex }] = bytes;
  }

  /// Make the object unreadable, as on a drive that does not implement it.
  void drop(CoeAddress object, uint16_t position = 0)
  {
    objects_.erase({ position, object.index, object.subindex });
  }

  int upload(uint16_t position, uint16_t index, uint8_t sub_index, std::vector<uint8_t>& bytes, std::size_t& size,
             uint32_t& abort_code) const override
  {
    const auto found = objects_.find({ position, index, sub_index });
    if (found == objects_.end())
    {
      abort_code = ABORT_NO_OBJECT;
      return -1;
    }
    size = std::min(found->second.size(), bytes.size());
    std::copy_n(found->second.begin(), size, bytes.begin());
    return 0;
  }

  int download(uint16_t position, uint16_t index, uint8_t sub_index, const std::vector<uint8_t>& bytes,
               uint32_t& /*abort_code*/) const override
  {
    const std::tuple<uint16_t, uint16_t, uint8_t> key{ position, index, sub_index };
    ++downloads[key];
    written[key] = bytes;
    if (ignored_writes_[key] > 0)
    {
      --ignored_writes_[key];  // acknowledged, but not applied
      return 0;
    }
    objects_[key] = bytes;
    return 0;
  }

  /// Acknowledge the next `count` writes of `object` without applying them.
  void ignore_writes(CoeAddress object, int count, uint16_t position = 0)
  {
    ignored_writes_[{ position, object.index, object.subindex }] = count;
  }

  /// How often `object` was written.
  int download_count(CoeAddress object, uint16_t position = 0) const
  {
    const auto found = downloads.find({ position, object.index, object.subindex });
    return found == downloads.end() ? 0 : found->second;
  }

  /// What was written to the drive at `position`, as an integer of the written size. Empty when nothing was.
  std::optional<int64_t> written_value(CoeAddress object, uint16_t position = 0) const
  {
    const auto found = written.find({ position, object.index, object.subindex });
    if (found == written.end())
    {
      return std::nullopt;
    }
    uint64_t bits = 0;
    for (std::size_t i = 0; i < found->second.size(); ++i)
    {
      bits |= static_cast<uint64_t>(found->second[i]) << (8 * i);
    }
    const int shift = 64 - 8 * static_cast<int>(found->second.size());
    return static_cast<int64_t>(bits << shift) >> shift;  // sign-extend from the written size
  }

  mutable std::map<std::tuple<uint16_t, uint16_t, uint8_t>, std::vector<uint8_t>> written;
  mutable std::map<std::tuple<uint16_t, uint16_t, uint8_t>, int> downloads;

private:
  mutable std::map<std::tuple<uint16_t, uint16_t, uint8_t>, std::vector<uint8_t>> objects_;
  mutable std::map<std::tuple<uint16_t, uint16_t, uint8_t>, int> ignored_writes_;
};

/// Healthy drives for the joints a test checks.
class DriveChecks : public ::testing::Test
{
protected:
  /// Run the checks over `joints`, with a healthy drive for each unless `healthy` is false.
  bool check(const std::vector<JointConfig>& joints, bool healthy = true)
  {
    if (healthy)
    {
      for (const auto& j : joints)
      {
        drives_.healthy_drive(j.position);
      }
    }
    units_.resize(joints.size());
    return check_drives(drives_, drives_, joints, units_, rclcpp::get_logger("test_drive_checks"), readback_);
  }

  /// The SI factor of joint 0 for `scale`.
  double si(Unit scale) const
  {
    return si_factor(readback_.at(0).si_factors, scale);
  }

  ScriptedDrives drives_;
  std::vector<DriveReadback> readback_;
  std::vector<SlaveConfigUnits> units_;  ///< per joint, empty sections unless a test sets them
};

// --- SI factors, gains and torque limit ------------------------------------------

/// The SI factors come from the encoder resolution, the velocity unit and the rated torque.
TEST_F(DriveChecks, ReadsTheSiFactorsFromTheDrive)
{
  ASSERT_TRUE(check({ joint() }));
  EXPECT_NEAR(si(Unit::ANGLE), 2.0 * M_PI / 4096.0, 1e-15);
  EXPECT_NEAR(si(Unit::ANGULAR_VELOCITY), RAD_PER_RPM, 1e-15);
  EXPECT_NEAR(si(Unit::MOTOR_TORQUE), 1e-3, 1e-15);  // a thousandth of 1 Nm
}

/// With the position counted on the motor shaft, the drive ignores its gear ratio, and so does the effort state.
TEST_F(DriveChecks, GivesTheTorqueAtTheMotorWhenThePositionIsCountedThere)
{
  ASSERT_TRUE(check({ joint() }));
  EXPECT_DOUBLE_EQ(si(Unit::SHAFT_TORQUE), si(Unit::MOTOR_TORQUE));
}

/// A geared drive whose position is counted on the motor shaft comes up, with a warning that its states are at the
/// motor while its torque commands are on the output side.
TEST_F(DriveChecks, WarnsThatAGearedDriveCountingAtTheMotorHasItsTorqueCommandsOnTheOutput)
{
  const synapticon_hardware::test::LogCapture log;
  ASSERT_TRUE(check({ joint() }));  // encoder 1 counts the position and commutates, gear ratio 100:1
  EXPECT_EQ(log.count("Joint 'joint_1': its position encoder is the commutation encoder"), 1u);
  EXPECT_EQ(log.count("ignores its gear ratio 100:1 (0x6091)"), 1u);
}

/// Without that warning when the position is counted behind the gear, or the gear ratio is 1:1.
TEST_F(DriveChecks, DoesNotWarnAboutTheGearWhenStatesAndTorqueCommandsAreOnTheSameShaft)
{
  const synapticon_hardware::test::LogCapture log;
  drives_.healthy_drive();
  drives_.put(coe::POSITION_ENCODER, 2);
  drives_.put(coe::ENCODER_2_RESOLUTION, 1000);
  ASSERT_TRUE(check({ joint() }, false));
  drives_.healthy_drive();
  drives_.put(coe::GEAR_MOTOR_REVOLUTIONS, 1);
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_EQ(log.count("ignores its gear ratio"), 0u);
}

/// With the position counted behind the gear, the effort state is the motor torque times the gear ratio.
TEST_F(DriveChecks, GivesTheTorqueBehindTheGearWhenThePositionIsCountedThere)
{
  drives_.healthy_drive();
  drives_.put(coe::POSITION_ENCODER, 2);
  drives_.put(coe::ENCODER_2_RESOLUTION, 1000);
  drives_.put(coe::RATED_TORQUE, 774);
  drives_.put(coe::GEAR_MOTOR_REVOLUTIONS, 775);
  drives_.put(coe::GEAR_SHAFT_REVOLUTIONS, 100);
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_NEAR(si(Unit::MOTOR_TORQUE), 0.774e-3, 1e-15);
  EXPECT_NEAR(si(Unit::SHAFT_TORQUE), 0.774e-3 * 7.75, 1e-15);
}

TEST_F(DriveChecks, TakesAGearRatioOfOneToOneAsNoGear)
{
  drives_.healthy_drive();
  drives_.put(coe::POSITION_ENCODER, 2);
  drives_.put(coe::ENCODER_2_RESOLUTION, 1000);
  drives_.put(coe::GEAR_MOTOR_REVOLUTIONS, 3);
  drives_.put(coe::GEAR_SHAFT_REVOLUTIONS, 3);
  drives_.drop(coe::COMMUTATION_ENCODER);  // not needed without a gear
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_DOUBLE_EQ(si(Unit::SHAFT_TORQUE), si(Unit::MOTOR_TORQUE));
}

TEST_F(DriveChecks, TakesADriveWithoutAGearRatioAsUngeared)
{
  drives_.healthy_drive();
  drives_.put(coe::POSITION_ENCODER, 2);
  drives_.put(coe::ENCODER_2_RESOLUTION, 1000);
  drives_.drop(coe::GEAR_MOTOR_REVOLUTIONS);
  drives_.drop(coe::GEAR_SHAFT_REVOLUTIONS);
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_DOUBLE_EQ(si(Unit::SHAFT_TORQUE), si(Unit::MOTOR_TORQUE));
}

/// A gear ratio of 0 is no ratio at all: the torque on the position shaft stays unknown.
TEST_F(DriveChecks, RefusesAGearRatioWithAZero)
{
  for (const CoeAddress zeroed : { coe::GEAR_MOTOR_REVOLUTIONS, coe::GEAR_SHAFT_REVOLUTIONS })
  {
    drives_.healthy_drive();
    drives_.put(zeroed, 0);
    EXPECT_FALSE(check({ joint() }, false));
  }
}

/// Behind a gear, the encoders tell whether the drive applies the ratio. Unreadable, the torque stays unknown.
TEST_F(DriveChecks, RefusesAGearedDriveWhoseCommutationEncoderCannotBeRead)
{
  drives_.healthy_drive();
  drives_.drop(coe::COMMUTATION_ENCODER);
  EXPECT_FALSE(check({ joint() }, false));
}

/// Behind a gear, a commutation encoder other than 1 or 2 cannot tell whether the drive applies the ratio either.
TEST_F(DriveChecks, RefusesAGearedDriveWhoseCommutationEncoderIsNoEncoder)
{
  for (const uint32_t encoder : { 0u, 3u })
  {
    drives_.healthy_drive();
    drives_.put(coe::COMMUTATION_ENCODER, encoder);
    EXPECT_FALSE(check({ joint() }, false)) << encoder;
  }
}

TEST_F(DriveChecks, ReadsThePositionResolutionOfTheSecondEncoderWhenItFeedsThePosition)
{
  drives_.healthy_drive();
  drives_.put(coe::POSITION_ENCODER, 2);
  drives_.put(coe::ENCODER_2_RESOLUTION, 1000);
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_NEAR(si(Unit::ANGLE), 2.0 * M_PI / 1000.0, 1e-15);
}

/// A joint that may be energised needs every factor: a command in rad must not land on raw counts.
TEST_F(DriveChecks, RefusesAnUnknownEncoderSource)
{
  drives_.healthy_drive();
  drives_.put(coe::POSITION_ENCODER, 3);
  EXPECT_FALSE(check({ joint() }, false));
}

struct MissingFactorCase
{
  const char* name;
  CoeAddress object;
};

class MissingFactor : public DriveChecks, public ::testing::WithParamInterface<MissingFactorCase>
{
};

TEST_P(MissingFactor, RefusesAJointThatMayBeEnergised)
{
  drives_.healthy_drive();
  drives_.drop(GetParam().object);
  EXPECT_FALSE(check({ joint() }, false));
}

TEST_P(MissingFactor, LeavesAMonitoredJointRaw)
{
  drives_.healthy_drive();
  drives_.drop(GetParam().object);
  JointConfig monitored = joint();
  monitored.energize = false;
  EXPECT_TRUE(check({ monitored }, false));
}

INSTANTIATE_TEST_SUITE_P(Cases, MissingFactor,
                         ::testing::Values(MissingFactorCase{ "EncoderSource", coe::POSITION_ENCODER },
                                           MissingFactorCase{ "EncoderResolution", coe::ENCODER_1_RESOLUTION },
                                           MissingFactorCase{ "VelocityUnit", coe::VELOCITY_UNIT },
                                           MissingFactorCase{ "RatedTorque", coe::RATED_TORQUE },
                                           MissingFactorCase{ "GearShaftTurns", coe::GEAR_SHAFT_REVOLUTIONS }),
                         [](const auto& info) { return std::string(info.param.name); });

/// One joint without its scaling refuses the bring-up of all.
TEST_F(DriveChecks, OneUnscaledJointRefusesAll)
{
  drives_.healthy_drive(0);
  drives_.healthy_drive(1);
  drives_.drop(coe::RATED_TORQUE, 1);
  EXPECT_FALSE(check({ joint(0), joint(1) }, false));
}

/// An inverted joint flips position, velocity and the torques, but not the gains or the torque limit.
TEST_F(DriveChecks, NegatesTheDirectedFactorsOfAnInvertedJoint)
{
  JointConfig inverted = joint();
  inverted.invert = true;
  ASSERT_TRUE(check({ inverted }));
  EXPECT_LT(si(Unit::ANGLE), 0.0);
  EXPECT_LT(si(Unit::ANGULAR_VELOCITY), 0.0);
  EXPECT_LT(si(Unit::MOTOR_TORQUE), 0.0);
  EXPECT_LT(si(Unit::SHAFT_TORQUE), 0.0);
  EXPECT_LT(si(Unit::OUTPUT_TORQUE), 0.0);
  EXPECT_GT(si(Unit::STIFFNESS), 0.0);
  EXPECT_GT(si(Unit::TORQUE_LIMIT), 0.0);
}

/// The commissioned gains and torque limit become the values to stream until a command replaces them.
TEST_F(DriveChecks, SeedsTheGainsAndTheTorqueLimitFromTheDrive)
{
  ASSERT_TRUE(check({ joint() }));
  EXPECT_EQ(readback_[0].stiffness, 400.0);
  EXPECT_EQ(readback_[0].damping, 5.0);
  EXPECT_EQ(readback_[0].max_torque, 50000.0);
}

TEST_F(DriveChecks, SeedsAConfiguredTorqueLimitInsteadOfTheDrives)
{
  JointConfig limited = joint();
  limited.max_joint_torque = 2.5;
  ASSERT_TRUE(check({ limited }));
  EXPECT_DOUBLE_EQ(readback_[0].max_torque.value(), 2500.0);  // mNm
}

/// Configured gains replace the drive's, in raw steps of mNm/rad and mNm s/rad.
TEST_F(DriveChecks, SeedsConfiguredGainsInsteadOfTheDrives)
{
  JointConfig configured = joint();
  configured.stiffness = 1.5;
  configured.damping = 0.02;
  ASSERT_TRUE(check({ configured }));
  EXPECT_DOUBLE_EQ(readback_[0].stiffness.value(), 1500.0);
  EXPECT_DOUBLE_EQ(readback_[0].damping.value(), 20.0);
}

/// An impedance joint that may be powered needs all three command defaults. A monitored one does not, and 0 is
/// allowed.
TEST_F(DriveChecks, AnImpedanceJointNeedsItsCommandDefaults)
{
  for (const auto clear : { &JointConfig::stiffness, &JointConfig::damping, &JointConfig::max_joint_torque })
  {
    JointConfig incomplete = joint(0, IMPEDANCE);
    incomplete.*clear = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(check({ incomplete }));
    incomplete.energize = false;
    EXPECT_TRUE(check({ incomplete }));
  }
  JointConfig zero = joint(0, IMPEDANCE);
  zero.damping = 0.0;
  EXPECT_TRUE(check({ zero }));
}

/// A drive commissioned with no torque comes up, and bring-up warns about it.
TEST_F(DriveChecks, KeepsATorqueLimitOfZeroFromTheDrive)
{
  drives_.healthy_drive();
  drives_.put(coe::MAX_JOINT_TORQUE, 0);
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_EQ(readback_[0].max_torque, 0.0);
}

/// A drive that answers no SDO read is refused when it may be energised.
TEST_F(DriveChecks, ADriveWithoutAnswersIsRefused)
{
  EXPECT_FALSE(check({ joint() }, false));
}

/// A monitored drive that answers no SDO read still comes up in CSP, with default factors and nothing read back.
TEST_F(DriveChecks, AMonitoredDriveWithoutAnswersComesUpRawInCsp)
{
  JointConfig monitored = joint();
  monitored.energize = false;
  ASSERT_TRUE(check({ monitored }, false));
  EXPECT_EQ(readback_[0].si_factors, UNREAD_SI_FACTORS);
  EXPECT_FALSE(readback_[0].stiffness);
  EXPECT_FALSE(readback_[0].damping);
  EXPECT_FALSE(readback_[0].max_torque);
  EXPECT_EQ(readback_[0].quick_stop_option_code, -1);
  EXPECT_TRUE(std::isnan(readback_[0].quick_stop_deceleration));
}

// --- Torque sensor ------------------------------------------------------------------

struct SensorCase
{
  const char* name;
  int mode;
  std::optional<uint32_t> sensor_type;  ///< nothing: 0x2150:1 unreadable
  bool passes;
};

class TorqueSensor : public DriveChecks, public ::testing::WithParamInterface<SensorCase>
{
};

TEST_P(TorqueSensor, IsRequiredInImpedanceModeOnly)
{
  const SensorCase& c = GetParam();
  drives_.healthy_drive();
  if (c.sensor_type)
  {
    drives_.put(coe::TORQUE_SENSOR_TYPE, *c.sensor_type);
  }
  else
  {
    drives_.drop(coe::TORQUE_SENSOR_TYPE);
  }
  EXPECT_EQ(check({ joint(0, c.mode) }, false), c.passes);
}

INSTANTIATE_TEST_SUITE_P(Cases, TorqueSensor,
                         ::testing::Values(SensorCase{ "ImpedanceWithSensor", IMPEDANCE, 1, true },
                                           SensorCase{ "ImpedanceWithoutSensor", IMPEDANCE, coe::NO_TORQUE_SENSOR,
                                                       false },
                                           SensorCase{ "ImpedanceUnreadable", IMPEDANCE, std::nullopt, false },
                                           SensorCase{ "CspWithoutSensor", CSP, coe::NO_TORQUE_SENSOR, true }),
                         [](const auto& info) { return std::string(info.param.name); });

/// One impedance joint without a sensor fails the checks for all.
TEST_F(DriveChecks, AnyImpedanceJointWithoutSensorFailsTheChecks)
{
  drives_.healthy_drive(0);
  drives_.healthy_drive(1);
  drives_.put(coe::TORQUE_SENSOR_TYPE, 0, 1);
  EXPECT_FALSE(check({ joint(0, IMPEDANCE), joint(1, IMPEDANCE) }, false));
}

// --- Quick stop and stopping configuration ---------------------------------------------

struct QuickStopCase
{
  const char* name;
  std::optional<uint32_t> option_code;
  int read_back;
};

class QuickStopOptionCode : public DriveChecks, public ::testing::WithParamInterface<QuickStopCase>
{
};

/// The code is read back as the signed 16-bit value it is, -1 when unreadable, and the checks pass either way.
TEST_P(QuickStopOptionCode, IsReadBack)
{
  const QuickStopCase& c = GetParam();
  drives_.healthy_drive();
  if (c.option_code)
  {
    drives_.put(coe::QUICK_STOP_OPTION, *c.option_code);
  }
  else
  {
    drives_.drop(coe::QUICK_STOP_OPTION);
  }
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_EQ(readback_[0].quick_stop_option_code, c.read_back);
}

INSTANTIATE_TEST_SUITE_P(Cases, QuickStopOptionCode,
                         ::testing::Values(QuickStopCase{ "StopAndHold", 6, 6 },
                                           QuickStopCase{ "StopThenDisable", 2, 2 },
                                           QuickStopCase{ "CutThePower", 1, 1 }, QuickStopCase{ "ShortTheMotor", 9, 9 },
                                           QuickStopCase{ "Negative", 0xFFFE, -2 },
                                           QuickStopCase{ "Unreadable", std::nullopt, -1 }),
                         [](const auto& info) { return std::string(info.param.name); });

TEST_F(DriveChecks, ReadsTheQuickStopDecelerationInTheVelocityUnit)
{
  ASSERT_TRUE(check({ joint() }));
  EXPECT_NEAR(readback_[0].quick_stop_deceleration, 3000.0 * RAD_PER_RPM, 1e-9);
}

TEST_F(DriveChecks, LeavesAnUnreadableQuickStopDecelerationUnknown)
{
  drives_.healthy_drive();
  drives_.drop(coe::QUICK_STOP_DECELERATION);
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_TRUE(std::isnan(readback_[0].quick_stop_deceleration));
}

struct FollowingErrorCase
{
  const char* name;
  std::optional<uint32_t> option;
  uint32_t window;
};

class FollowingErrorReaction : public DriveChecks, public ::testing::WithParamInterface<FollowingErrorCase>
{
};

/// The following error reaction is only logged, whatever it is.
TEST_P(FollowingErrorReaction, IsLoggedWithoutFailingTheChecks)
{
  const FollowingErrorCase& c = GetParam();
  drives_.healthy_drive();
  drives_.put(coe::FOLLOWING_ERROR_WINDOW, c.window);
  if (c.option)
  {
    drives_.put(coe::FOLLOWING_ERROR_REACTION, *c.option);
  }
  else
  {
    drives_.drop(coe::FOLLOWING_ERROR_REACTION);
  }
  EXPECT_TRUE(check({ joint() }, false));
}

INSTANTIATE_TEST_SUITE_P(Cases, FollowingErrorReaction,
                         ::testing::Values(FollowingErrorCase{ "NotMonitored", 0, coe::FOLLOWING_ERROR_CHECK_OFF },
                                           FollowingErrorCase{ "ReportOnly", 0, 1000 },
                                           FollowingErrorCase{ "Fault", 1, 1000 },
                                           FollowingErrorCase{ "Unreadable", std::nullopt, 1000 }),
                         [](const auto& info) { return std::string(info.param.name); });

// --- Firmware --------------------------------------------------------------------------

struct FirmwareCase
{
  const char* name;
  std::optional<std::string> version;  ///< nothing: 0x100A unreadable
  int mode;
  bool passes;
};

class Firmware : public DriveChecks, public ::testing::WithParamInterface<FirmwareCase>
{
};

/// Firmware below 5.6.11, or a version that cannot be read, fails an impedance joint and only warns otherwise.
TEST_P(Firmware, BelowTheMinimumFailsImpedanceAndWarnsOtherwise)
{
  const FirmwareCase& c = GetParam();
  drives_.healthy_drive();
  if (c.version)
  {
    drives_.put_text(coe::SOFTWARE_VERSION, *c.version);
  }
  else
  {
    drives_.drop(coe::SOFTWARE_VERSION);
  }
  EXPECT_EQ(check({ joint(0, c.mode) }, false), c.passes);
}

INSTANTIATE_TEST_SUITE_P(Cases, Firmware,
                         ::testing::Values(FirmwareCase{ "MinimumImpedance", "v5.6.11", IMPEDANCE, true },
                                           FirmwareCase{ "NewerImpedance", "v5.7.0+g77", IMPEDANCE, true },
                                           FirmwareCase{ "OlderCsp", "v5.6.6", CSP, true },
                                           FirmwareCase{ "OlderImpedance", "v5.6.6", IMPEDANCE, false },
                                           FirmwareCase{ "GarbledCsp", "release", CSP, true },
                                           FirmwareCase{ "GarbledImpedance", "release", IMPEDANCE, false },
                                           FirmwareCase{ "UnreadableCsp", std::nullopt, CSP, true },
                                           FirmwareCase{ "UnreadableImpedance", std::nullopt, IMPEDANCE, false }),
                         [](const auto& info) { return std::string(info.param.name); });

/// @brief A reader whose first `failures` reads fail. With `abort_code` 0 they get no valid answer, as while another
/// exchange holds the drive's mailbox, else the drive aborts them.
class BusyMailbox : public SdoReader, public SdoWriter
{
public:
  explicit BusyMailbox(int failures, uint32_t abort_code = 0) : failures_(failures), abort_code_(abort_code)
  {
  }

  int upload(uint16_t, uint16_t, uint8_t, std::vector<uint8_t>& bytes, std::size_t& size,
             uint32_t& abort_code) const override
  {
    ++calls;
    if (calls <= failures_)
    {
      abort_code = abort_code_;
      return -5;
    }
    std::fill(bytes.begin(), bytes.end(), 0);
    size = bytes.size();
    return 0;
  }

  int download(uint16_t, uint16_t, uint8_t, const std::vector<uint8_t>&, uint32_t& abort_code) const override
  {
    ++calls;
    abort_code = calls <= failures_ ? abort_code_ : 0;
    return calls <= failures_ ? -5 : 0;
  }

  mutable int calls{ 0 };

private:
  int failures_;
  uint32_t abort_code_;
};

constexpr SdoRetry QUICK_RETRY{ 3, 0.001 };

/// Reads without a valid answer are tried again, up to the configured tries in all.
TEST(RetryingSdo, RetriesUntilTheDriveAnswers)
{
  BusyMailbox drive(2);
  const RetryingSdo reader(drive, drive, QUICK_RETRY, rclcpp::get_logger("test_drive_checks"));
  EXPECT_EQ(read_u32(reader, rclcpp::get_logger("test_drive_checks"), 0, coe::RATED_TORQUE), 0u);
  EXPECT_EQ(drive.calls, 3);
}

/// An abort is the drive's answer, so it is not tried again.
TEST(RetryingSdo, TakesAnAbortAsFinal)
{
  BusyMailbox drive(100, ABORT_NO_OBJECT);
  const RetryingSdo reader(drive, drive, QUICK_RETRY, rclcpp::get_logger("test_drive_checks"));
  EXPECT_FALSE(read_u32(reader, rclcpp::get_logger("test_drive_checks"), 0, coe::RATED_TORQUE));
  EXPECT_EQ(drive.calls, 1);
}

/// Once a read has used up its tries, the later reads get one try each.
TEST(RetryingSdo, TriesOnceAfterAReadGaveUp)
{
  BusyMailbox drive(100);
  const RetryingSdo reader(drive, drive, QUICK_RETRY, rclcpp::get_logger("test_drive_checks"));
  EXPECT_FALSE(read_u32(reader, rclcpp::get_logger("test_drive_checks"), 0, coe::RATED_TORQUE));
  EXPECT_EQ(drive.calls, 3);
  EXPECT_FALSE(read_u32(reader, rclcpp::get_logger("test_drive_checks"), 0, coe::VELOCITY_UNIT));
  EXPECT_EQ(drive.calls, 4);
}

/// A drive that never answers uses up only its own tries: the next drive still gets all of them.
TEST(RetryingSdo, GivingUpOnOneDriveKeepsTheTriesOfTheOthers)
{
  BusyMailbox drive(100);
  const RetryingSdo reader(drive, drive, QUICK_RETRY, rclcpp::get_logger("test_drive_checks"));
  EXPECT_FALSE(read_u32(reader, rclcpp::get_logger("test_drive_checks"), 0, coe::RATED_TORQUE));
  EXPECT_EQ(drive.calls, 3);
  EXPECT_FALSE(read_u32(reader, rclcpp::get_logger("test_drive_checks"), 1, coe::RATED_TORQUE));
  EXPECT_EQ(drive.calls, 6);
  EXPECT_FALSE(read_u32(reader, rclcpp::get_logger("test_drive_checks"), 0, coe::VELOCITY_UNIT));
  EXPECT_EQ(drive.calls, 7);
}

// Units read from the drive, the units: section of the slave_config, and the sdo_si entries.

constexpr double RAD_PER_RPS = 2.0 * M_PI;

/// A velocity unit per second is understood, not taken for per minute.
TEST_F(DriveChecks, DecodesAVelocityUnitPerSecond)
{
  drives_.healthy_drive();
  drives_.put(coe::VELOCITY_UNIT, 0x00B40300);  // rev/s
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_DOUBLE_EQ(si(Unit::ANGULAR_VELOCITY), RAD_PER_RPS);
}

/// A finer velocity unit scales by its power of ten.
TEST_F(DriveChecks, DecodesAMilliRpmVelocityUnit)
{
  drives_.healthy_drive();
  drives_.put(coe::VELOCITY_UNIT, 0xFDB44700);  // 10^-3 rev/min
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_DOUBLE_EQ(si(Unit::ANGULAR_VELOCITY), 1e-3 * RAD_PER_RPM);
}

class UnknownVelocityUnit : public DriveChecks, public ::testing::WithParamInterface<uint32_t>
{
};

/// A velocity unit the stack does not understand refuses a joint that may be powered.
TEST_P(UnknownVelocityUnit, RefusesAJointThatMayBeEnergised)
{
  drives_.healthy_drive();
  drives_.put(coe::VELOCITY_UNIT, GetParam());
  EXPECT_FALSE(check({ joint() }, false));
}

INSTANTIATE_TEST_SUITE_P(Cases, UnknownVelocityUnit,
                         ::testing::Values(0x00B44701u,    // last byte not 0
                                           0x00B54700u,    // distance not revolutions
                                           0x00B40400u,    // time neither minute nor second
                                           0xFCB44700u));  // power of ten below 10^-3

/// With the feed constant enabled, positions and velocities are in linear units the stack does not support.
TEST_F(DriveChecks, RefusesAnEnabledFeedConstant)
{
  drives_.healthy_drive();
  drives_.put(coe::FEED_CONSTANT_FEED, 10);
  EXPECT_FALSE(check({ joint() }, false));
}

/// A drive without the feed constant object has none.
TEST_F(DriveChecks, TakesAMissingFeedConstantAsDisabled)
{
  drives_.healthy_drive();
  drives_.drop(coe::FEED_CONSTANT_FEED);
  EXPECT_TRUE(check({ joint() }, false));
}

/// The units: section stands in for a unit the drive cannot report.
TEST_F(DriveChecks, TakesTheSlaveConfigUnitsForUnreadableOnes)
{
  drives_.healthy_drive();
  drives_.drop(coe::VELOCITY_UNIT);
  drives_.drop(coe::ENCODER_1_RESOLUTION);
  drives_.drop(coe::RATED_TORQUE);
  units_.resize(1);
  units_[0].overrides.velocity_unit = 0x00B40300;  // rev/s
  units_[0].overrides.counts_per_revolution = 1000;
  units_[0].overrides.rated_torque = 2.0;
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_DOUBLE_EQ(si(Unit::ANGULAR_VELOCITY), RAD_PER_RPS);
  EXPECT_DOUBLE_EQ(si(Unit::ANGLE), 2.0 * M_PI / 1000);
  EXPECT_DOUBLE_EQ(si(Unit::MOTOR_TORQUE), 2.0e-3);
}

/// The units: section also stands in for a velocity unit the drive reports but the stack does not understand.
TEST_F(DriveChecks, TakesTheSlaveConfigVelocityUnitForAnUnknownOne)
{
  drives_.healthy_drive();
  drives_.put(coe::VELOCITY_UNIT, 0x00B44701);
  units_.resize(1);
  units_[0].overrides.velocity_unit = 0x00B44700;
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_DOUBLE_EQ(si(Unit::ANGULAR_VELOCITY), RAD_PER_RPM);
}

/// A units: entry that contradicts what the drive reports refuses the joint: the drive's commissioning wins.
TEST_F(DriveChecks, RefusesSlaveConfigUnitsThatContradictTheDrive)
{
  const auto contradicting = [](UnitOverrides& overrides, int which) {
    if (which == 0)
    {
      overrides.velocity_unit = 0xFDB44700;  // the drive has rev/min
    }
    else if (which == 1)
    {
      overrides.counts_per_revolution = 8192;  // the drive has 4096
    }
    else
    {
      overrides.rated_torque = 2.0;  // the drive has 1 Nm
    }
  };
  for (int which = 0; which < 3; ++which)
  {
    drives_ = ScriptedDrives{};
    drives_.healthy_drive();
    units_.assign(1, SlaveConfigUnits{});
    contradicting(units_[0].overrides, which);
    EXPECT_FALSE(check({ joint() }, false)) << "case " << which;
  }
}

/// A units: entry that agrees with the drive changes nothing.
TEST_F(DriveChecks, AcceptsSlaveConfigUnitsThatAgreeWithTheDrive)
{
  drives_.healthy_drive();
  units_.resize(1);
  units_[0].overrides.velocity_unit = 0x00B44700;
  units_[0].overrides.counts_per_revolution = 4096;
  units_[0].overrides.rated_torque = 1.0;
  units_[0].overrides.feed_constant_disabled = true;
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_DOUBLE_EQ(si(Unit::ANGULAR_VELOCITY), RAD_PER_RPM);
}

/// One sdo_si entry for each quantity, converted with the drive's units.
TEST_F(DriveChecks, WritesTheSdoSiEntriesInTheDrivesUnits)
{
  drives_.healthy_drive();  // rev/min, 4096 counts per turn, rated torque 1 Nm
  units_.resize(1);
  units_[0].si_sdo = { { { 0x6080, 0 }, "uint32", 15.7, SiQuantity::VELOCITY },
                       { { 0x6085, 0 }, "uint32", 1000.0, SiQuantity::ACCELERATION },
                       { { 0x6065, 0 }, "uint32", 0.1, SiQuantity::POSITION },
                       { { 0x6072, 0 }, "uint16", 0.96, SiQuantity::TORQUE } };
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_EQ(drives_.written_value({ 0x6080, 0 }), 150);   // 15.7 rad/s = 149.9 rev/min
  EXPECT_EQ(drives_.written_value({ 0x6085, 0 }), 9549);  // 1000 rad/s^2 = 9549.3 rev/min/s
  EXPECT_EQ(drives_.written_value({ 0x6065, 0 }), 65);    // 0.1 rad = 65.2 counts
  EXPECT_EQ(drives_.written_value({ 0x6072, 0 }), 960);   // 0.96 Nm = 960 per mille of 1 Nm
  EXPECT_NEAR(readback_.at(0).quick_stop_deceleration, 1000.0, 0.1) << "the ramp is read after it was written";
}

/// The same SI value becomes a different raw value on a drive with another unit.
TEST_F(DriveChecks, WritesAnSdoSiEntryInAFinerUnit)
{
  drives_.healthy_drive();
  drives_.put(coe::VELOCITY_UNIT, 0xFDB44700);  // 10^-3 rev/min
  units_.resize(1);
  units_[0].si_sdo = { { { 0x6085, 0 }, "uint32", 1000.0, SiQuantity::ACCELERATION } };
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_EQ(drives_.written_value({ 0x6085, 0 }), 9549297);
}

/// Every sdo_si entry is read back after the write. One the drive acknowledged but did not apply is written once more.
TEST_F(DriveChecks, WritesAnSdoSiEntryAgainThatTheDriveDidNotApply)
{
  drives_.healthy_drive();
  drives_.ignore_writes({ 0x6065, 0 }, 1);
  units_.resize(1);
  units_[0].si_sdo = { { { 0x6065, 0 }, "uint32", 0.1, SiQuantity::POSITION },
                       { { 0x6085, 0 }, "uint32", 1000.0, SiQuantity::ACCELERATION } };
  ASSERT_TRUE(check({ joint() }, false));
  EXPECT_EQ(drives_.download_count({ 0x6065, 0 }), 2);
  EXPECT_EQ(drives_.download_count({ 0x6085, 0 }), 1);
  EXPECT_EQ(read_u32(drives_, rclcpp::get_logger("test_drive_checks"), 0, { 0x6065, 0 }), 65u);
}

/// A drive that keeps another value after the second write stops the bring-up.
TEST_F(DriveChecks, StopsTheBringUpWhenTheDriveKeepsAnotherSdoSiValue)
{
  drives_.healthy_drive();
  drives_.ignore_writes(coe::QUICK_STOP_DECELERATION, 2);
  units_.resize(1);
  units_[0].si_sdo = { { coe::QUICK_STOP_DECELERATION, "uint32", 1000.0, SiQuantity::ACCELERATION } };
  EXPECT_FALSE(check({ joint() }, false));
  EXPECT_EQ(drives_.download_count(coe::QUICK_STOP_DECELERATION), 2);
}

/// A joint with unknown units is never powered, so its sdo_si entries are not written.
TEST_F(DriveChecks, LeavesOutTheSdoSiEntriesOfAJointWithUnknownUnits)
{
  drives_.healthy_drive();
  drives_.drop(coe::VELOCITY_UNIT);
  JointConfig monitored = joint();
  monitored.energize = false;
  units_.resize(1);
  units_[0].si_sdo = { { { 0x6085, 0 }, "uint32", 1000.0, SiQuantity::ACCELERATION } };
  EXPECT_TRUE(check({ monitored }, false));
  EXPECT_FALSE(drives_.written_value({ 0x6085, 0 }));
}

struct BadSiCase
{
  const char* name;
  SiSdo entry;
};

class BadSiEntry : public DriveChecks, public ::testing::WithParamInterface<BadSiCase>
{
};

/// An sdo_si entry that the drive's unit cannot hold stops the bring-up, and nothing of it is written.
TEST_P(BadSiEntry, StopsTheBringUp)
{
  drives_.healthy_drive();
  units_.resize(1);
  units_[0].si_sdo = { GetParam().entry };
  EXPECT_FALSE(check({ joint() }, false));
  EXPECT_FALSE(drives_.written_value(GetParam().entry.object));
}

INSTANTIATE_TEST_SUITE_P(
    Cases, BadSiEntry,
    ::testing::Values(BadSiCase{ "RoundsToZero", { { 0x6085, 0 }, "uint32", 0.01, SiQuantity::ACCELERATION } },
                      BadSiCase{ "TooCoarse", { { 0x6065, 0 }, "uint32", 0.006, SiQuantity::POSITION } },
                      BadSiCase{ "TooLargeForTheType", { { 0x6072, 0 }, "uint16", 100.0, SiQuantity::TORQUE } },
                      BadSiCase{ "NegativeForUnsigned", { { 0x6080, 0 }, "uint32", -10.0, SiQuantity::VELOCITY } }),
    [](const auto& info) { return std::string(info.param.name); });

}  // namespace
