// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The bring-up checks of the EtherCAT HALs, against a scripted object dictionary instead of a drive.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/logger.hpp>

#include "synapticon_hardware/ethercat/coe_objects.hpp"
#include "synapticon_hardware/ethercat/drive_checks.hpp"

namespace
{
using namespace synapticon_hardware::ethercat;

constexpr int CSP = mode::CSP;
constexpr int IMPEDANCE = mode::IMPEDANCE;
constexpr double RAD_PER_RPM = 2.0 * M_PI / 60.0;
constexpr uint32_t ABORT_NO_OBJECT = 0x06020000;

/// One joint at ring position `position` in `mode`.
JointConfig joint(uint16_t position = 0, int mode = CSP)
{
  JointConfig config;
  config.name = "joint_" + std::to_string(position + 1);
  config.position = position;
  config.slave_config_file = "unused.yaml";
  config.mode_of_operation = mode;
  return config;
}

/// @brief Drives that answer SDO reads from a scripted dictionary.
class ScriptedDrives : public SdoReader
{
public:
  /// The objects of a healthy drive at `position`: CSP and impedance ready, firmware 5.6.11.
  void healthy_drive(uint16_t position = 0)
  {
    put(coe::POSITION_ENCODER, 1, position);
    put(coe::ENCODER_1_RESOLUTION, 4096, position);
    put(coe::VELOCITY_UNIT, 0x00B44700, position);  // rev/min
    put(coe::RATED_TORQUE, 1000, position);         // 1 Nm
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

private:
  std::map<std::tuple<uint16_t, uint16_t, uint8_t>, std::vector<uint8_t>> objects_;
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
    return check_drives(drives_, joints, rclcpp::get_logger("test_drive_checks"), readback_);
  }

  /// The SI factor of joint 0 for `scale`.
  double si(Unit scale) const
  {
    return si_factor(readback_.at(0).si_factors, scale);
  }

  ScriptedDrives drives_;
  std::vector<DriveReadback> readback_;
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
                                           MissingFactorCase{ "RatedTorque", coe::RATED_TORQUE }),
                         [](const auto& info) { return std::string(info.param.name); });

/// One joint without its scaling refuses the bring-up of all.
TEST_F(DriveChecks, OneUnscaledJointRefusesAll)
{
  drives_.healthy_drive(0);
  drives_.healthy_drive(1);
  drives_.drop(coe::RATED_TORQUE, 1);
  EXPECT_FALSE(check({ joint(0), joint(1) }, false));
}

/// An inverted joint flips position, velocity and both torques, but not the gains or the torque limit.
TEST_F(DriveChecks, NegatesTheDirectedFactorsOfAnInvertedJoint)
{
  JointConfig inverted = joint();
  inverted.invert = true;
  ASSERT_TRUE(check({ inverted }));
  EXPECT_LT(si(Unit::ANGLE), 0.0);
  EXPECT_LT(si(Unit::ANGULAR_VELOCITY), 0.0);
  EXPECT_LT(si(Unit::MOTOR_TORQUE), 0.0);
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

}  // namespace
