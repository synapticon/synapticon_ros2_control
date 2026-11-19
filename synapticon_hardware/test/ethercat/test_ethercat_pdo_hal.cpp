// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The wait of EthercatPdoHal for valid data after OP, on drives whose process data a test sets.

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/ethercat/coe_objects.hpp"
#include "synapticon_hardware/ethercat/ethercat_pdo_hal.hpp"

namespace
{
namespace ec = synapticon_hardware::ethercat;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::HalConfig;

constexpr uint16_t SWITCH_ON_DISABLED = 0x40;
constexpr double CONTROL_FREQUENCY = 1000.0;
constexpr double TIMEOUT_S = 0.05;

/// A slave_config with the measured velocity and the controlword as its only channels, each left out when its flag is
/// false, written to a temporary file. Returns its path.
std::string slave_config(bool velocity, bool control_word)
{
  const std::string path =
      ::testing::TempDir() + "ethercat_pdo_hal_" + std::to_string(velocity) + std::to_string(control_word) + ".yaml";
  std::ofstream file(path);
  file << "vendor_id: 0x22d2\nproduct_id: 0x302\n";
  if (velocity)
  {
    file << "tpdo:\n  - index: 0x1a00\n    channels:\n"
            "      - { index: 0x606c, sub_index: 0, type: int32, state_interface: velocity }\n";
  }
  if (control_word)
  {
    file << "rpdo:\n  - index: 0x1600\n    channels:\n"
            "      - { index: 0x6040, sub_index: 0, type: uint16, command_interface: control_word, default: 0 }\n";
  }
  return path;
}

/// @brief An EthercatPdoHal whose drives report the statusword and mode a test sets, from a given read on.
class ScriptedBus : public ec::EthercatPdoHal
{
public:
  /// Drives for `joints`, on slave_configs that map the velocity and the controlword unless a flag is false.
  explicit ScriptedBus(std::vector<ec::JointConfig> joints, bool velocity = true, bool control_word = true)
    : EthercatPdoHal("test_ethercat_pdo_hal"), joints_(std::move(joints))
  {
    for (auto& joint : joints_)
    {
      joint.slave_config_file = slave_config(velocity, control_word);
    }
    set_up_ = set_up_drives(joints_, true, CONTROL_FREQUENCY);
  }

  /// The result of set_up_drives() in the constructor.
  bool set_up() const
  {
    return set_up_;
  }

  /// From read `after_reads` on, `joint` reports `statusword` and the mode of operation `mode`.
  void report(std::size_t joint, uint16_t statusword, int mode, int after_reads = 0)
  {
    scripts_.push_back({ joint, statusword, mode, after_reads });
  }

  bool wait()
  {
    streaming_ = true;
    return wait_for_streaming(joints_, CONTROL_FREQUENCY, TIMEOUT_S);
  }

  /// Put the setpoints of `command` into the outputs of `joint`.
  void put(std::size_t joint, const DriveCommand& command)
  {
    write_setpoints(joint, command);
  }

  /// The output of `joint` in `slot`.
  double output(std::size_t joint, std::size_t slot) const
  {
    return outputs_[joint][slot];
  }

  bool setup(const HalConfig&) override
  {
    return true;
  }
  bool connect() override
  {
    return true;
  }
  bool start() override
  {
    return true;
  }
  bool stop() override
  {
    return true;
  }
  bool disconnect() override
  {
    return true;
  }
  bool read(std::vector<DriveState>&) override
  {
    return true;
  }
  bool is_connected() const override
  {
    return true;
  }

protected:
  bool exchange_read() override
  {
    ++reads_;
    for (const auto& script : scripts_)
    {
      if (reads_ > script.after_reads)
      {
        inputs_[script.joint][ec::slot::STATUS_WORD] = script.statusword;
        inputs_[script.joint][ec::slot::MODE_DISPLAY] = script.mode;
      }
    }
    return true;
  }
  bool exchange_write() override
  {
    return true;
  }
  int upload(uint16_t, uint16_t, uint8_t, std::vector<uint8_t>&, std::size_t&, uint32_t&) const override
  {
    return -1;
  }
  int download(uint16_t, uint16_t, uint8_t, const std::vector<uint8_t>&, uint32_t&) const override
  {
    return -1;
  }

private:
  struct Script
  {
    std::size_t joint;
    uint16_t statusword;
    int mode;
    int after_reads;
  };
  std::vector<ec::JointConfig> joints_;
  std::vector<Script> scripts_;
  int reads_{ 0 };
  bool set_up_{ false };
};

ec::JointConfig joint(const std::string& name, int mode, bool energize = true)
{
  ec::JointConfig config;
  config.name = name;
  config.mode_of_operation = mode;
  config.energize = energize;
  return config;
}

TEST(WaitForStreaming, EndsWhenEveryDriveRunsItsJointsMode)
{
  ScriptedBus bus({ joint("a", ec::mode::IMPEDANCE), joint("b", ec::mode::CSP) });
  bus.report(0, SWITCH_ON_DISABLED, ec::mode::IMPEDANCE);
  bus.report(1, SWITCH_ON_DISABLED, ec::mode::CSP);
  EXPECT_TRUE(bus.wait());
}

TEST(WaitForStreaming, WaitsForADriveThatTakesItsModeLater)
{
  ScriptedBus bus({ joint("a", ec::mode::CSP) });
  bus.report(0, SWITCH_ON_DISABLED, 0);
  bus.report(0, SWITCH_ON_DISABLED, ec::mode::CSP, 5);
  EXPECT_TRUE(bus.wait());
}

TEST(WaitForStreaming, RefusesADriveInAnotherMode)
{
  ScriptedBus bus({ joint("a", ec::mode::IMPEDANCE) });
  bus.report(0, SWITCH_ON_DISABLED, ec::mode::CSP);
  EXPECT_FALSE(bus.wait());
}

TEST(WaitForStreaming, RefusesADriveThatReportsNoMode)
{
  ScriptedBus bus({ joint("a", ec::mode::CSP) });
  bus.report(0, SWITCH_ON_DISABLED, 0);
  EXPECT_FALSE(bus.wait());
}

TEST(WaitForStreaming, IgnoresTheModeOfAJointThatIsNeverPowered)
{
  ScriptedBus bus({ joint("a", ec::mode::IMPEDANCE, false), joint("b", ec::mode::CSP) });
  bus.report(0, SWITCH_ON_DISABLED, ec::mode::CSP);
  bus.report(1, SWITCH_ON_DISABLED, ec::mode::CSP);
  EXPECT_TRUE(bus.wait());
}

TEST(WaitForStreaming, StillRefusesADriveWithoutStatusword)
{
  ScriptedBus bus({ joint("a", ec::mode::CSP) });
  bus.report(0, 0, ec::mode::CSP);
  EXPECT_FALSE(bus.wait());
}

/// A joint that gets power needs the measured velocity in its process data, to tell when it stands still.
TEST(SetUpDrives, RefusesAPoweredJointWithoutVelocityState)
{
  EXPECT_TRUE(ScriptedBus({ joint("a", ec::mode::CSP) }).set_up());
  EXPECT_FALSE(ScriptedBus({ joint("a", ec::mode::CSP) }, false).set_up());
  EXPECT_TRUE(ScriptedBus({ joint("a", ec::mode::CSP, false) }, false).set_up()) << "a joint without power needs none";
}

/// A joint that gets power needs the controlword routed to its drive, since only the HAL may switch it on.
TEST(SetUpDrives, RefusesAPoweredJointWithoutControlword)
{
  EXPECT_FALSE(ScriptedBus({ joint("a", ec::mode::CSP) }, true, false).set_up());
  EXPECT_TRUE(ScriptedBus({ joint("a", ec::mode::CSP, false) }, true, false).set_up())
      << "a joint without power needs none";
}

/// A negative torque ceiling, stiffness or damping keeps the value sent last, so it cannot wrap in an unsigned field.
/// Negative position, velocity and torque offset go out.
TEST(WriteSetpoints, ANegativeCeilingOrGainKeepsTheValueSentLast)
{
  ScriptedBus bus({ joint("a", ec::mode::IMPEDANCE) });
  DriveCommand command;
  command.max_joint_torque = 2.0;
  command.stiffness = 3.0;
  command.damping = 4.0;
  bus.put(0, command);
  const double ceiling = bus.output(0, ec::slot::TORQUE_LIMIT);
  const double stiffness = bus.output(0, ec::slot::STIFFNESS);
  const double damping = bus.output(0, ec::slot::DAMPING);

  command = { -1.0, -1.0, -1.0, -1.0, -1.0, -1.0 };
  bus.put(0, command);
  EXPECT_EQ(bus.output(0, ec::slot::TORQUE_LIMIT), ceiling);
  EXPECT_EQ(bus.output(0, ec::slot::STIFFNESS), stiffness);
  EXPECT_EQ(bus.output(0, ec::slot::DAMPING), damping);
  EXPECT_LT(bus.output(0, ec::slot::TARGET_POSITION), 0.0);
  EXPECT_LT(bus.output(0, ec::slot::TARGET_VELOCITY), 0.0);
  EXPECT_LT(bus.output(0, ec::slot::TORQUE_OFFSET), 0.0);
}

}  // namespace
