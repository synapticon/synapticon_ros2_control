// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief EthercatIcubeHal's bring-up and start against a scripted drive and bus: the order of the calls, the start,
/// and the read-back of the drive checks in its buffers, its scaling and its quick stop getters.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "ethercat_driver/ethercat_bus_manager.hpp"
#include "ethercat_icube_common.hpp"
#include "synapticon_hardware/ethercat/coe_objects.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"
#include "synapticon_ethercat_icube/ethercat_icube_hal.hpp"

namespace
{
using synapticon_ethercat_icube::EthercatConfig;
using synapticon_ethercat_icube::EthercatIcubeHal;
using synapticon_ethercat_icube::EthercatJointConfig;
using synapticon_hardware::DriveState;
namespace ec = synapticon_hardware::ethercat;
using synapticon_hardware::ethercat::CoeAddress;

constexpr int CSP = ec::mode::CSP;
constexpr int IMPEDANCE = ec::mode::IMPEDANCE;
constexpr double RAD_PER_RPM = 2.0 * M_PI / 60.0;
constexpr uint32_t ABORT_NO_OBJECT = 0x06020000;

/// The part of a slave_config that keeps the driver's hands off the drive's power.
constexpr const char* HAL_POWERS = "auto_state_transitions: false\n";

/// A slave_config with `content` after its identity, the measured velocity and the controlword, which a joint that
/// gets power needs, written to a temporary file. `power` comes first, the HAL refuses the file without it. Bring-up
/// reads only these, since the master that would read the rest is scripted. Returns its path.
std::string slave_config(const std::string& content = "", const std::string& power = HAL_POWERS)
{
  static int count = 0;
  const std::string path = ::testing::TempDir() + "icube_bringup_" + std::to_string(count++) + ".yaml";
  std::ofstream(path) << "vendor_id: 0x22d2\nproduct_id: 0x302\n"
                      << power
                      << "tpdo:\n  - index: 0x1a00\n    channels:\n"
                         "      - { index: 0x606c, sub_index: 0, type: int32, state_interface: velocity }\n"
                         "rpdo:\n  - index: 0x1600\n    channels:\n"
                         "      - { index: 0x6040, sub_index: 0, type: uint16, command_interface: control_word }\n"
                      << content;
  return path;
}

/// One joint at ring position `position` in `mode`.
EthercatJointConfig joint(uint16_t position = 0, int mode = CSP)
{
  EthercatJointConfig config;
  config.name = "joint_" + std::to_string(position + 1);
  config.position = position;
  config.slave_config_file = slave_config();
  config.mode_of_operation = mode;
  return config;
}

/// @brief EthercatIcubeHal whose drives answer SDO reads from a scripted dictionary, with no master behind it.
///
/// After connect(), stream() lets read() unpack what a test writes into the input buffers.
class ScriptedDriveHal : public EthercatIcubeHal
{
public:
  ~ScriptedDriveHal() override
  {
    disconnect();  // while the stand-ins for the master are still in place
  }

  /// The objects of a healthy drive at `position`: CSP and impedance ready, firmware 5.6.11.
  void healthy_drive(uint16_t position = 0)
  {
    put(ec::coe::POSITION_ENCODER, 1, position);
    put(ec::coe::COMMUTATION_ENCODER, 1, position);
    put(ec::coe::ENCODER_1_RESOLUTION, 4096, position);
    put(ec::coe::VELOCITY_UNIT, 0x00B44700, position);  // rev/min
    put(ec::coe::RATED_TORQUE, 1000, position);         // 1 Nm
    put(ec::coe::IMPEDANCE_STIFFNESS, 400, position);
    put(ec::coe::IMPEDANCE_DAMPING, 5, position);
    put(ec::coe::MAX_JOINT_TORQUE, 50000, position);  // 50 Nm
    put(ec::coe::GEAR_MOTOR_REVOLUTIONS, 100, position);
    put(ec::coe::GEAR_SHAFT_REVOLUTIONS, 1, position);
    put(ec::coe::TORQUE_SENSOR_TYPE, 1, position);
    put(ec::coe::QUICK_STOP_OPTION, 6, position);
    put(ec::coe::QUICK_STOP_DECELERATION, 3000, position);
    put(ec::coe::FOLLOWING_ERROR_REACTION, 0, position);
    put(ec::coe::FOLLOWING_ERROR_WINDOW, ec::coe::FOLLOWING_ERROR_CHECK_OFF, position);
    put_text(ec::coe::SOFTWARE_VERSION, "v5.6.11", position);
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

  /// What the drive at `position` holds in `object`, as an unsigned value.
  std::optional<uint32_t> held(CoeAddress object, uint16_t position = 0) const
  {
    const auto found = objects_.find({ position, object.index, object.subindex });
    if (found == objects_.end())
    {
      return std::nullopt;
    }
    uint32_t value = 0;
    for (std::size_t i = 0; i < found->second.size() && i < 4; ++i)
    {
      value |= static_cast<uint32_t>(found->second[i]) << (8 * i);
    }
    return value;
  }

  void fail_bus_configuration()
  {
    configures_ = false;
  }
  void fail_bus_activation(bool fail = true)
  {
    activates_ = !fail;
  }
  void fail_exchange()
  {
    exchanges_ = false;
  }
  void report_operational(bool operational)
  {
    operational_ = operational;
  }
  /// Process data comes back incomplete from now on, or complete again.
  void lose_process_data(bool lost)
  {
    complete_ = !lost;
  }
  int activations() const
  {
    return activations_;
  }
  int deactivations() const
  {
    return deactivations_;
  }

  void stream()
  {
    streaming_ = true;
  }

  std::vector<double>& input(std::size_t joint)
  {
    return inputs_[joint];
  }

  double output(std::size_t joint, std::size_t slot) const
  {
    return outputs_[joint][slot];
  }

protected:
  bool configure_bus() override
  {
    return configures_;
  }

  bool activate_bus() override
  {
    activations_ += activates_ ? 1 : 0;
    return activates_;
  }

  void deactivate_bus() override
  {
    ++deactivations_;
  }

  bool drives_operational() const override
  {
    return operational_;
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
    objects_[{ position, index, sub_index }] = bytes;
    return 0;
  }

  ethercat_driver::EthercatCycleResult bus_read() override
  {
    return exchanges_ ? ethercat_driver::EthercatCycleResult::kCompleted : ethercat_driver::EthercatCycleResult::kError;
  }

  ethercat_driver::EthercatCycleResult bus_write() override
  {
    return bus_read();
  }

  bool process_data_complete() const override
  {
    return complete_;
  }

private:
  mutable std::map<std::tuple<uint16_t, uint16_t, uint8_t>, std::vector<uint8_t>> objects_;
  bool configures_{ true };
  bool activates_{ true };
  bool exchanges_{ true };
  bool operational_{ true };
  bool complete_{ true };
  int activations_{ 0 };
  int deactivations_{ 0 };
};

/// A HAL with a healthy drive for each of `joints`, set up but not brought up.
class BringUp : public ::testing::Test
{
protected:
  void set_up(const std::vector<EthercatJointConfig>& joints)
  {
    for (const auto& j : joints)
    {
      hal_.healthy_drive(j.position);
    }
    ASSERT_TRUE(hal_.setup(EthercatConfig{}, joints));
  }

  /// The state read with `raw` position, velocity and torque in the input buffer of joint 0.
  DriveState read_with(double position, double velocity, double torque)
  {
    hal_.stream();
    hal_.input(0)[ec::slot::STATUS_WORD] = 0x40;  // Switch On Disabled, so the stop() at teardown finds it off
    hal_.input(0)[ec::slot::ACTUAL_POSITION] = position;
    hal_.input(0)[ec::slot::ACTUAL_VELOCITY] = velocity;
    hal_.input(0)[ec::slot::ACTUAL_TORQUE] = torque;  // the effort channel
    std::vector<DriveState> states;
    EXPECT_TRUE(hal_.read(states));
    return states.at(0);
  }

  ScriptedDriveHal hal_;
};

// --- Order of the calls ----------------------------------------------------

TEST_F(BringUp, IsRefusedBeforeSetup)
{
  EXPECT_FALSE(hal_.connect());
}

/// A second connect() succeeds and changes nothing. A new setup() needs disconnect() first.
TEST_F(BringUp, ConnectsOnce)
{
  set_up({ joint() });
  ASSERT_TRUE(hal_.connect());
  EXPECT_TRUE(hal_.is_connected());
  EXPECT_TRUE(hal_.connect());
  EXPECT_TRUE(hal_.is_connected());
  EXPECT_FALSE(hal_.setup(EthercatConfig{}, { joint() }));
}

/// stop(), disable() and disconnect() succeed on rungs the HAL is not on.
TEST_F(BringUp, StepsDownFromRungsItIsNotOn)
{
  EXPECT_TRUE(hal_.stop());
  EXPECT_TRUE(hal_.disable());
  EXPECT_TRUE(hal_.disconnect());
  set_up({ joint() });
  ASSERT_TRUE(hal_.connect());
  EXPECT_TRUE(hal_.stop());
  EXPECT_TRUE(hal_.disable());
  EXPECT_TRUE(hal_.is_connected());
  EXPECT_TRUE(hal_.disconnect());
  EXPECT_FALSE(hal_.is_connected());
  EXPECT_FALSE(hal_.is_streaming());
  EXPECT_FALSE(hal_.is_enabled());
}

/// The EtherCAT driver walks a drive to Operation Enabled by itself unless the slave_config forbids it.
TEST_F(BringUp, IsRefusedForASlaveConfigThatLetsTheDriverPowerTheDrive)
{
  for (const std::string power : { "", "auto_state_transitions: true\n" })
  {
    EthercatJointConfig own_power = joint();
    own_power.slave_config_file = slave_config("", power);
    set_up({ own_power });
    EXPECT_FALSE(hal_.connect()) << "'" << power << "'";
    EXPECT_FALSE(hal_.is_connected());
  }
}

TEST_F(BringUp, StopsWhenTheBusCannotBeConfigured)
{
  set_up({ joint() });
  hal_.fail_bus_configuration();
  EXPECT_FALSE(hal_.connect());
  EXPECT_FALSE(hal_.is_connected());
}

TEST_F(BringUp, ReadingStatesIsRefusedBeforeStart)
{
  set_up({ joint() });
  ASSERT_TRUE(hal_.connect());
  std::vector<DriveState> states;
  EXPECT_FALSE(hal_.read(states));
}

// --- Start ------------------------------------------------------------------------

TEST_F(BringUp, StartIsRefusedBeforeBringUp)
{
  set_up({ joint() });
  EXPECT_FALSE(hal_.start());
}

/// A bus that does not reach OP in time is released, so a new connect() starts over.
TEST_F(BringUp, StartFailsWhenTheBusCannotBeActivated)
{
  set_up({ joint() });
  ASSERT_TRUE(hal_.connect());
  hal_.fail_bus_activation();
  EXPECT_FALSE(hal_.start());
  EXPECT_FALSE(hal_.is_streaming());
  EXPECT_FALSE(hal_.is_connected());
  EXPECT_FALSE(hal_.start()) << "refused until connect() runs again";
  EXPECT_EQ(hal_.deactivations(), 0) << "a master that never became active is not deactivated";

  hal_.fail_bus_activation(false);
  ASSERT_TRUE(hal_.connect());
  hal_.input(0)[ec::slot::STATUS_WORD] = 0x40;
  hal_.input(0)[ec::slot::MODE_DISPLAY] = CSP;
  EXPECT_TRUE(hal_.start());
}

/// A failed exchange while waiting for the drives stops the bus again.
TEST_F(BringUp, StartFailsWhenTheExchangeFailsWhileWaiting)
{
  set_up({ joint() });
  ASSERT_TRUE(hal_.connect());
  hal_.fail_exchange();
  EXPECT_FALSE(hal_.start());
  EXPECT_FALSE(hal_.is_streaming());
  EXPECT_EQ(hal_.deactivations(), 0) << "the master stays active until disconnect()";
}

/// A drive whose statusword stays 0 does not stream, so start() gives up at the readiness timeout.
TEST(StartWithoutStreaming, TimesOutAndStopsTheExchange)
{
  ScriptedDriveHal hal;
  hal.healthy_drive();
  EthercatConfig bus;
  bus.readiness_timeout_s = 0.05;
  ASSERT_TRUE(hal.setup(bus, { joint() }));
  ASSERT_TRUE(hal.connect());
  EXPECT_FALSE(hal.start());
  EXPECT_FALSE(hal.is_streaming());
  hal.disconnect();
  EXPECT_EQ(hal.deactivations(), 1);
}

/// Once every energisable drive streams a statusword, start() succeeds. A monitored joint does not hold it up.
TEST_F(BringUp, StartSucceedsOnceTheEnergisableDrivesStream)
{
  EthercatJointConfig monitored = joint(1);
  monitored.energize = false;
  set_up({ joint(), monitored });
  ASSERT_TRUE(hal_.connect());
  hal_.input(0)[ec::slot::STATUS_WORD] = 0x40;
  hal_.input(0)[ec::slot::MODE_DISPLAY] = CSP;
  ASSERT_TRUE(hal_.start());
  EXPECT_TRUE(hal_.is_streaming());
  EXPECT_TRUE(hal_.start());  // streaming already, nothing changes
  EXPECT_EQ(hal_.activations(), 1);
  ASSERT_TRUE(hal_.stop());
  EXPECT_FALSE(hal_.is_streaming());
}

/// A start() after a stop() resumes the exchange on the master activated before: a second activation would leave the
/// frames on the old process data buffer. The master is deactivated once, at disconnect().
TEST_F(BringUp, StartsAgainWithoutActivatingTheMasterTwice)
{
  set_up({ joint() });
  ASSERT_TRUE(hal_.connect());
  hal_.input(0)[ec::slot::STATUS_WORD] = 0x40;
  hal_.input(0)[ec::slot::MODE_DISPLAY] = CSP;
  for (int round = 1; round <= 3; ++round)
  {
    ASSERT_TRUE(hal_.start()) << "round " << round;
    ASSERT_TRUE(hal_.stop()) << "round " << round;
  }
  EXPECT_EQ(hal_.activations(), 1);
  EXPECT_EQ(hal_.deactivations(), 0);
  EXPECT_TRUE(hal_.disconnect());
  EXPECT_EQ(hal_.deactivations(), 1);
}

/// A start() after a stop() waits for the drives to return to OP, at most readiness_timeout_s.
TEST(StartAgain, FailsWhenADriveDoesNotReturnToOp)
{
  ScriptedDriveHal hal;
  hal.healthy_drive();
  EthercatConfig bus;
  bus.readiness_timeout_s = 0.5;
  ASSERT_TRUE(hal.setup(bus, { joint() }));
  ASSERT_TRUE(hal.connect());
  hal.input(0)[ec::slot::STATUS_WORD] = 0x40;
  hal.input(0)[ec::slot::MODE_DISPLAY] = CSP;
  ASSERT_TRUE(hal.start());
  ASSERT_TRUE(hal.stop());
  hal.report_operational(false);
  EXPECT_FALSE(hal.start());
  EXPECT_FALSE(hal.is_streaming());
  hal.report_operational(true);
  EXPECT_TRUE(hal.start());
}

/// Incomplete process data is bridged for fewer than max_bad_cycles cycles in a row. Then read() fails, so the
/// hardware interface stops, until the process data is complete again.
TEST(LostProcessData, FailsReadAfterMaxBadCyclesAndRecovers)
{
  ScriptedDriveHal hal;
  hal.healthy_drive();
  EthercatConfig bus;
  bus.max_bad_cycles = 3;
  ASSERT_TRUE(hal.setup(bus, { joint() }));
  ASSERT_TRUE(hal.connect());
  hal.input(0)[ec::slot::STATUS_WORD] = 0x40;
  hal.input(0)[ec::slot::MODE_DISPLAY] = CSP;
  ASSERT_TRUE(hal.start());
  std::vector<DriveState> states;
  const synapticon_hardware::DriveCommand hold{};
  const auto cycle = [&]() {
    hal.write({ hold });
    return hal.read(states);
  };
  hal.lose_process_data(true);
  EXPECT_TRUE(cycle()) << "bad cycle 1 is bridged";
  EXPECT_TRUE(cycle()) << "bad cycle 2 is bridged";
  EXPECT_FALSE(cycle()) << "bad cycle 3 reaches the limit";
  EXPECT_FALSE(cycle()) << "read() keeps failing while the data stays incomplete";
  hal.lose_process_data(false);
  EXPECT_TRUE(cycle()) << "a complete cycle makes the data usable again";
  hal.lose_process_data(true);
  EXPECT_TRUE(cycle()) << "the count starts over";
}

/// Before streaming, incomplete process data is expected on the way to OP and does not count.
TEST(LostProcessData, IsNotCountedBeforeStreaming)
{
  ScriptedDriveHal hal;
  hal.healthy_drive();
  EthercatConfig bus;
  bus.max_bad_cycles = 1;
  ASSERT_TRUE(hal.setup(bus, { joint() }));
  ASSERT_TRUE(hal.connect());
  hal.input(0)[ec::slot::STATUS_WORD] = 0x40;
  hal.input(0)[ec::slot::MODE_DISPLAY] = CSP;
  hal.lose_process_data(true);
  EXPECT_TRUE(hal.start());
}

TEST_F(BringUp, TearDownWhileStreamingStopsTheBus)
{
  set_up({ joint() });
  ASSERT_TRUE(hal_.connect());
  hal_.input(0)[ec::slot::STATUS_WORD] = 0x40;
  hal_.input(0)[ec::slot::MODE_DISPLAY] = CSP;
  ASSERT_TRUE(hal_.start());
  EXPECT_TRUE(hal_.disconnect());
  EXPECT_FALSE(hal_.is_streaming());
  EXPECT_FALSE(hal_.is_connected());
  EXPECT_EQ(hal_.deactivations(), 1);
}

// --- Scaling, gains and torque limit -----------------------------------------

/// The SI factors come from the encoder resolution, the velocity unit and the rated torque.
TEST_F(BringUp, ScalesTheStatesWithTheFactorsReadFromTheDrive)
{
  set_up({ joint() });
  ASSERT_TRUE(hal_.connect());
  const DriveState state = read_with(1024.0, 60.0, 500.0);
  EXPECT_NEAR(state.position, M_PI / 2.0, 1e-12);  // a quarter of 4096 increments
  EXPECT_NEAR(state.velocity, 60.0 * RAD_PER_RPM, 1e-12);
  EXPECT_NEAR(state.effort, 0.5, 1e-12);  // 500 permille of 1 Nm
}

/// An inverted joint flips position, velocity and torque, but not the gains or the torque limit.
TEST_F(BringUp, MirrorsTheDirectedQuantitiesOfAnInvertedJoint)
{
  EthercatJointConfig inverted = joint();
  inverted.invert = true;
  set_up({ inverted });
  ASSERT_TRUE(hal_.connect());
  const DriveState state = read_with(1024.0, 60.0, 500.0);
  EXPECT_NEAR(state.position, -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(state.velocity, -60.0 * RAD_PER_RPM, 1e-12);
  EXPECT_NEAR(state.effort, -0.5, 1e-12);
  EXPECT_EQ(hal_.output(0, ec::slot::STIFFNESS), 400.0);
  EXPECT_EQ(hal_.output(0, ec::slot::TORQUE_LIMIT), 50000.0);
}

/// The commissioned gains and torque limit are streamed until a command replaces them.
TEST_F(BringUp, SeedsTheGainsAndTheTorqueLimitFromTheDrive)
{
  set_up({ joint() });
  ASSERT_TRUE(hal_.connect());
  EXPECT_EQ(hal_.output(0, ec::slot::STIFFNESS), 400.0);
  EXPECT_EQ(hal_.output(0, ec::slot::DAMPING), 5.0);
  EXPECT_EQ(hal_.output(0, ec::slot::TORQUE_LIMIT), 50000.0);
}

TEST_F(BringUp, StreamsAConfiguredTorqueLimitInsteadOfTheDrives)
{
  EthercatJointConfig limited = joint();
  limited.max_joint_torque = 2.5;
  set_up({ limited });
  ASSERT_TRUE(hal_.connect());
  EXPECT_DOUBLE_EQ(hal_.output(0, ec::slot::TORQUE_LIMIT), 2500.0);  // mNm
}

/// A drive that answers no SDO read is refused when it may be energised, since its scaling is unknown.
TEST(BringUpWithoutAnswers, IsRefusedForAJointThatMayBeEnergised)
{
  ScriptedDriveHal hal;
  ASSERT_TRUE(hal.setup(EthercatConfig{}, { joint() }));
  EXPECT_FALSE(hal.connect());
  EXPECT_FALSE(hal.is_connected());
}

/// A monitored drive that answers no SDO read still comes up in CSP, with raw values and nothing read back.
TEST(BringUpWithoutAnswers, ComesUpRawInCspWhenMonitoredOnly)
{
  EthercatJointConfig monitored = joint();
  monitored.energize = false;
  ScriptedDriveHal hal;
  ASSERT_TRUE(hal.setup(EthercatConfig{}, { monitored }));
  ASSERT_TRUE(hal.connect());
  EXPECT_EQ(hal.quick_stop_option_code(0), -1);
  EXPECT_FALSE(hal.quick_stop_available(0));
  EXPECT_TRUE(std::isnan(hal.quick_stop_deceleration(0)));
  EXPECT_EQ(hal.output(0, ec::slot::TORQUE_LIMIT), 0.0);
}

// --- Checks ---------------------------------------------------------------------------

/// A drive check that fails, here an impedance joint without a torque sensor, stops the bring-up.
TEST_F(BringUp, AFailedDriveCheckStopsTheBringUp)
{
  set_up({ joint(0, IMPEDANCE) });
  hal_.put(ec::coe::TORQUE_SENSOR_TYPE, ec::coe::NO_TORQUE_SENSOR);
  EXPECT_FALSE(hal_.connect());
  EXPECT_FALSE(hal_.is_connected());
}

// --- Quick stop and stopping configuration -------------------------------------

struct QuickStopCase
{
  const char* name;
  std::optional<uint32_t> option_code;
  int read_back;
  bool available;
};

class QuickStopOptionCode : public BringUp, public ::testing::WithParamInterface<QuickStopCase>
{
};

/// 6 and 2 are a quick stop, anything else or an unreadable code is reported, and bring-up goes on.
TEST_P(QuickStopOptionCode, IsReadBackAndReported)
{
  const QuickStopCase& c = GetParam();
  set_up({ joint() });
  if (c.option_code)
  {
    hal_.put(ec::coe::QUICK_STOP_OPTION, *c.option_code);
  }
  else
  {
    hal_.drop(ec::coe::QUICK_STOP_OPTION);
  }
  ASSERT_TRUE(hal_.connect());
  EXPECT_EQ(hal_.quick_stop_option_code(0), c.read_back);
  EXPECT_EQ(hal_.quick_stop_available(0), c.available);
}

INSTANTIATE_TEST_SUITE_P(Cases, QuickStopOptionCode,
                         ::testing::Values(QuickStopCase{ "StopAndHold", 6, 6, true },
                                           QuickStopCase{ "StopThenDisable", 2, 2, true },
                                           QuickStopCase{ "CutThePower", 1, 1, false },
                                           QuickStopCase{ "ShortTheMotor", 9, 9, false },
                                           QuickStopCase{ "Unreadable", std::nullopt, -1, false }),
                         [](const auto& info) { return std::string(info.param.name); });

TEST_F(BringUp, ReadsTheQuickStopDecelerationInTheVelocityUnit)
{
  set_up({ joint() });
  ASSERT_TRUE(hal_.connect());
  EXPECT_NEAR(hal_.quick_stop_deceleration(0), 3000.0 * RAD_PER_RPM, 1e-9);
}

/// An sdo_si entry reaches the drive in its unit, written after the units were read and before the ramp is read.
TEST_F(BringUp, WritesAnSdoSiEntryInTheDrivesUnit)
{
  EthercatJointConfig configured = joint();
  configured.slave_config_file =
      slave_config("sdo_si:\n  - { index: 0x6085, sub_index: 0, type: uint32, si_value: 1000.0 }\n");
  set_up({ configured });
  ASSERT_TRUE(hal_.connect());
  EXPECT_EQ(hal_.held(ec::coe::QUICK_STOP_DECELERATION), 9549u);  // 1000 rad/s^2 in rev/min per s
  EXPECT_NEAR(hal_.quick_stop_deceleration(0), 1000.0, 0.1);
}

/// A slave_config that breaks a unit rule stops the bring-up before the bus is configured.
TEST_F(BringUp, StopsOnARawValueOfAUnitDependentObject)
{
  EthercatJointConfig configured = joint();
  configured.slave_config_file = slave_config("sdo:\n  - { index: 0x6085, sub_index: 0, type: uint32, value: 5000 }\n");
  set_up({ configured });
  EXPECT_FALSE(hal_.connect());
  EXPECT_EQ(hal_.held(ec::coe::QUICK_STOP_DECELERATION), 3000u) << "nothing may be written";
}

}  // namespace
