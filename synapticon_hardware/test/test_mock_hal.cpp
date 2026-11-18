// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Tests of MockHal: which calls it takes in which order, and how commands loop back as states.

#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/mock_hal.hpp"

namespace
{
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::HalConfig;
using synapticon_hardware::MockHal;

const std::vector<std::string> ARM{ "shoulder_pan", "shoulder_lift", "elbow", "wrist" };
constexpr double NOT_SET = std::numeric_limits<double>::quiet_NaN();
constexpr double INFINITE = std::numeric_limits<double>::infinity();

/// The stage queries as one word, read left to right: Connected, Streaming, Enabled, Ready. A dash means false.
std::string stage_word(const MockHal& hal)
{
  return std::string{ hal.is_connected() ? 'C' : '-', hal.is_streaming() ? 'S' : '-', hal.is_enabled() ? 'E' : '-',
                      hal.is_ready() ? 'R' : '-' };
}

/// Run the call named `name` on `hal`. read and write use one zero command or state per joint.
bool run_call(MockHal& hal, const std::string& name)
{
  static const std::map<std::string, std::function<bool(MockHal&)>> calls{
    { "connect", [](MockHal& h) { return h.connect(); } },
    { "start", [](MockHal& h) { return h.start(); } },
    { "enable", [](MockHal& h) { return h.enable(); } },
    { "disable", [](MockHal& h) { return h.disable(); } },
    { "stop", [](MockHal& h) { return h.stop(); } },
    { "disconnect", [](MockHal& h) { return h.disconnect(); } },
    { "read",
      [](MockHal& h) {
        std::vector<DriveState> states;
        return h.read(states);
      } },
    { "write", [](MockHal& h) { return h.write(std::vector<DriveCommand>(h.joint_count())); } },
  };
  return calls.at(name)(hal);
}

// --- Call order ---------------------------------------------------------------------------------------------------

/// A sequence of calls on a fresh mock, written as text.
///
/// `name+` expects the call to succeed, `name-` expects it to be refused. `?WORD` compares the stage queries with
/// stage_word(). For example "connect+ enable- ?C---" connects, then fails to enable and stays connected only.
struct Script
{
  const char* label;
  bool set_up;
  const char* steps;
};

class CallOrderTest : public ::testing::TestWithParam<Script>
{
};

TEST_P(CallOrderTest, FollowsTheScript)
{
  MockHal mock;
  if (GetParam().set_up)
  {
    ASSERT_TRUE(mock.setup(ARM));
  }
  std::istringstream steps(GetParam().steps);
  std::string token;
  while (steps >> token)
  {
    if (token.front() == '?')
    {
      EXPECT_EQ(stage_word(mock), token.substr(1)) << "at " << token;
      continue;
    }
    const std::string call = token.substr(0, token.size() - 1);
    const bool should_succeed = token.back() == '+';
    const std::string word_before = stage_word(mock);
    EXPECT_EQ(run_call(mock, call), should_succeed) << call;
    if (!should_succeed)
    {
      EXPECT_EQ(stage_word(mock), word_before) << "a refused " << call << " must not move the stage";
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    Mock, CallOrderTest,
    ::testing::Values(
        Script{ "UpAndDownAgain", true,
                "?---- connect+ ?C--- start+ ?CS-- enable+ ?CSER disable+ ?CS-- stop+ ?C--- disconnect+ ?----" },
        Script{ "WithoutSetupNothingConnects", false, "connect- start- ?----" },
        Script{ "StartAndEnableWaitForTheLink", true, "start- enable- ?----" },
        Script{ "EnableWaitsForTheExchange", true, "connect+ enable- ?C---" },
        Script{ "DownCallsOffTheirRungChangeNothing", true,
                "disable+ stop+ disconnect+ ?---- connect+ disable+ stop+ ?C--- start+ disable+ ?CS--" },
        Script{ "ConnectAgainChangesNothing", true, "connect+ connect+ ?C--- start+ enable+ connect+ ?CSER" },
        Script{ "StartAgainChangesNothing", true, "connect+ start+ start+ ?CS-- enable+ start+ ?CSER" },
        Script{ "EnableAgainChangesNothing", true, "connect+ start+ enable+ enable+ ?CSER" },
        Script{ "DisableWithoutPowerIsHarmless", true, "connect+ start+ disable+ disable+ ?CS--" },
        Script{ "StopSwitchesThePowerOff", true, "connect+ start+ enable+ stop+ ?C--- start+ ?CS--" },
        Script{ "PowerTogglesOnARunningExchange", true,
                "connect+ start+ enable+ disable+ enable+ disable+ enable+ disable+ disable+ ?CS--" },
        Script{ "TheExchangeRestartsOnTheSameLink", true, "connect+ start+ stop+ start+ stop+ start+ stop+ ?C---" },
        Script{ "DisconnectWorksFromAnyStage", true,
                "disconnect+ ?---- connect+ start+ enable+ disconnect+ ?---- connect+ ?C---" },
        Script{ "DataFlowsOnlyWhileStreaming", true,
                "read- connect+ read- write- start+ write+ read+ enable+ write+ read+ disable+ stop+ read- write-" }),
    [](const auto& info) { return std::string(info.param.label); });

// --- Loopback -----------------------------------------------------------------------------------------------------

/// An arm mock that is connected, streaming and enabled.
class LoopbackTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(mock_.setup(ARM));
    ASSERT_TRUE(mock_.connect());
    ASSERT_TRUE(mock_.start());
    ASSERT_TRUE(mock_.enable());
  }

  /// One command per joint: position, velocity and effort at `value`, the gains and the ceiling at its size.
  static std::vector<DriveCommand> uniform(double value)
  {
    DriveCommand command;
    command.position = command.velocity = command.effort = value;
    command.stiffness = command.damping = command.max_joint_torque = std::abs(value);
    return std::vector<DriveCommand>(ARM.size(), command);
  }

  /// What read() returns now. The vector passed in holds a stale entry, so its size is checked too.
  std::vector<DriveState> read_back()
  {
    std::vector<DriveState> states(7);
    EXPECT_TRUE(mock_.read(states));
    EXPECT_EQ(states.size(), ARM.size());
    return states;
  }

  MockHal mock_;
};

TEST_F(LoopbackTest, BeforeTheFirstWriteEverythingIsZero)
{
  for (const DriveState& state : read_back())
  {
    EXPECT_EQ(state.position, 0.0);
    EXPECT_EQ(state.velocity, 0.0);
    EXPECT_EQ(state.effort, 0.0);
    EXPECT_EQ(state.status_word, 0u) << "no data arrived yet";
  }
}

TEST_F(LoopbackTest, EveryJointGetsBackWhatItWasSent)
{
  std::vector<DriveCommand> commands(ARM.size());
  for (std::size_t j = 0; j < ARM.size(); ++j)
  {
    const double n = static_cast<double>(j);
    commands[j].position = 1.1 - 0.7 * n;
    commands[j].velocity = 0.25 * n * n;
    commands[j].effort = -4.0 + 3.5 * n;
  }
  ASSERT_TRUE(mock_.write(commands));

  const auto states = read_back();
  for (std::size_t j = 0; j < ARM.size(); ++j)
  {
    SCOPED_TRACE(ARM[j]);
    EXPECT_DOUBLE_EQ(states[j].position, commands[j].position);
    EXPECT_DOUBLE_EQ(states[j].velocity, commands[j].velocity);
    EXPECT_DOUBLE_EQ(states[j].effort, commands[j].effort);
    EXPECT_NE(states[j].status_word, 0u) << "data arrived";
  }
}

TEST_F(LoopbackTest, GainsAndTorqueLimitDoNotShowUpInTheState)
{
  auto commands = uniform(0.0);
  commands[1].stiffness = 80.0;
  commands[1].damping = 1.5;
  commands[1].max_joint_torque = 42.0;
  ASSERT_TRUE(mock_.write(commands));
  const DriveState state = read_back()[1];
  EXPECT_EQ(state.dc_link_voltage, 0.0);
  EXPECT_EQ(state.joint_torque_demand, 0.0);
  EXPECT_EQ(state.error_code, 0u);
  EXPECT_DOUBLE_EQ(state.effort, 0.0);
}

TEST_F(LoopbackTest, AWrongNumberOfCommandsIsRefused)
{
  ASSERT_TRUE(mock_.write(uniform(0.4)));
  for (const std::size_t count : { std::size_t{ 0 }, ARM.size() - 1, ARM.size() + 3 })
  {
    EXPECT_FALSE(mock_.write(std::vector<DriveCommand>(count, uniform(9.0).front()))) << count << " commands";
  }
  EXPECT_DOUBLE_EQ(read_back()[3].position, 0.4) << "a refused write stores nothing";
}

TEST_F(LoopbackTest, TheStateOutlivesAStopAndAStart)
{
  ASSERT_TRUE(mock_.write(uniform(-1.8)));
  ASSERT_TRUE(mock_.disable());
  ASSERT_TRUE(mock_.stop());
  ASSERT_TRUE(mock_.start());
  EXPECT_DOUBLE_EQ(read_back()[2].effort, -1.8);
}

TEST_F(LoopbackTest, SetStatesOverwritesEveryJoint)
{
  ASSERT_TRUE(mock_.write(uniform(0.6)));
  std::vector<DriveState> forced(ARM.size());
  forced[3].velocity = 2.2;
  forced[3].status_word = 0x0637;
  forced[1].error_code = 0x8611;
  ASSERT_TRUE(mock_.set_states(forced));

  const auto states = read_back();
  EXPECT_DOUBLE_EQ(states[3].velocity, 2.2);
  EXPECT_EQ(states[3].status_word, 0x0637u);
  EXPECT_EQ(states[1].error_code, 0x8611u);
  EXPECT_DOUBLE_EQ(states[0].position, 0.0) << "joints left at their default are overwritten too";
}

TEST_F(LoopbackTest, SetStatesNeedsExactlyOnePerJoint)
{
  EXPECT_FALSE(mock_.set_states({}));
  EXPECT_FALSE(mock_.set_states(std::vector<DriveState>(ARM.size() - 1)));
  EXPECT_FALSE(mock_.set_states(std::vector<DriveState>(ARM.size() + 1)));
  EXPECT_TRUE(mock_.set_states(std::vector<DriveState>(ARM.size())));
}

/// One setpoint of DriveCommand, how to set it and, for the looped back ones, where it shows in DriveState.
struct Setpoint
{
  const char* label;
  std::function<void(DriveCommand&, double)> set;
  std::function<double(const DriveState&)> shows_as;  // empty when the setpoint does not loop back
};

const std::vector<Setpoint> LOOPED_BACK{
  { "Position", [](DriveCommand& c, double v) { c.position = v; }, [](const DriveState& s) { return s.position; } },
  { "Velocity", [](DriveCommand& c, double v) { c.velocity = v; }, [](const DriveState& s) { return s.velocity; } },
  { "Effort", [](DriveCommand& c, double v) { c.effort = v; }, [](const DriveState& s) { return s.effort; } },
};

const std::vector<Setpoint> NOT_LOOPED_BACK{
  { "Stiffness", [](DriveCommand& c, double v) { c.stiffness = v; }, {} },
  { "Damping", [](DriveCommand& c, double v) { c.damping = v; }, {} },
  { "MaxJointTorque", [](DriveCommand& c, double v) { c.max_joint_torque = v; }, {} },
};

/// Every setpoint of DriveCommand.
std::vector<Setpoint> all_setpoints()
{
  std::vector<Setpoint> all = LOOPED_BACK;
  all.insert(all.end(), NOT_LOOPED_BACK.begin(), NOT_LOOPED_BACK.end());
  return all;
}

class SetpointTest : public LoopbackTest, public ::testing::WithParamInterface<Setpoint>
{
protected:
  /// Commands of `base` on every joint, with this setpoint of the joint `joint` at `value`.
  static std::vector<DriveCommand> with(double base, std::size_t joint, double value)
  {
    auto commands = uniform(base);
    GetParam().set(commands[joint], value);
    return commands;
  }
};

TEST_P(SetpointTest, AnInfiniteValueFailsTheWriteAndKeepsItsLastValue)
{
  ASSERT_TRUE(mock_.write(uniform(0.9)));
  for (const double value : { INFINITE, -INFINITE })
  {
    EXPECT_FALSE(mock_.write(with(5.0, 2, value))) << value;
  }
  const auto states = read_back();
  for (const Setpoint& looped : LOOPED_BACK)
  {
    SCOPED_TRACE(looped.label);
    EXPECT_DOUBLE_EQ(looped.shows_as(states[1]), 5.0) << "the other joints get their commands";
    const bool infinite = std::string(looped.label) == GetParam().label;
    EXPECT_DOUBLE_EQ(looped.shows_as(states[2]), infinite ? 0.9 : 5.0) << "only the infinite field keeps its value";
  }
}

/// A negative value fails the write only in the fields that take no negative values.
TEST_P(SetpointTest, ANegativeValueFailsTheWriteOnlyWhereItIsInvalid)
{
  const std::string label = GetParam().label;
  const bool non_negative = label == "Stiffness" || label == "Damping" || label == "MaxJointTorque";
  EXPECT_EQ(mock_.write(with(0.5, 1, -1.0)), !non_negative);
}

INSTANTIATE_TEST_SUITE_P(Mock, SetpointTest, ::testing::ValuesIn(all_setpoints()),
                         [](const auto& info) { return std::string(info.param.label); });

class LoopedBackSetpointTest : public SetpointTest
{
};

TEST_P(LoopedBackSetpointTest, NaNKeepsTheValueOfTheLatestWrite)
{
  for (const double value : { -0.35, 7.0 })
  {
    ASSERT_TRUE(mock_.write(with(0.0, 0, value)));
  }
  ASSERT_TRUE(mock_.write(with(0.0, 0, NOT_SET)));
  const auto states = read_back();
  EXPECT_DOUBLE_EQ(GetParam().shows_as(states[0]), 7.0);
  EXPECT_DOUBLE_EQ(GetParam().shows_as(states[1]), 0.0) << "the other joints follow their own commands";
}

INSTANTIATE_TEST_SUITE_P(Mock, LoopedBackSetpointTest, ::testing::ValuesIn(LOOPED_BACK),
                         [](const auto& info) { return std::string(info.param.label); });

// --- Configuration ------------------------------------------------------------------------------------------------

TEST(MockConfigTest, JointsComeFromTheHalConfigInTheirOrder)
{
  HalConfig config;
  config.hardware_parameters["hal"] = "mock";
  config.joints = { { "base_yaw", {} }, { "lift", { { "position", "2" } } }, { "gripper", {} } };
  MockHal mock;
  ASSERT_TRUE(mock.setup(config));
  EXPECT_EQ(mock.joint_count(), 3u);
  ASSERT_TRUE(mock.connect());
  ASSERT_TRUE(mock.start());
  std::vector<DriveState> states;
  ASSERT_TRUE(mock.read(states));
  EXPECT_EQ(states.size(), 3u);
}

TEST(MockConfigTest, TheJointsCannotChangeWhileConnected)
{
  MockHal mock;
  ASSERT_TRUE(mock.setup(ARM));
  ASSERT_TRUE(mock.connect());
  EXPECT_FALSE(mock.setup(std::vector<std::string>{ "spare" }));
  EXPECT_EQ(mock.joint_count(), ARM.size());
  EXPECT_EQ(stage_word(mock), "C---");
  ASSERT_TRUE(mock.disconnect());
  EXPECT_TRUE(mock.setup(std::vector<std::string>{ "spare" }));
  EXPECT_EQ(mock.joint_count(), 1u);
}

TEST(MockConfigTest, NoJointsAtAllStillGoesThroughEveryStage)
{
  MockHal mock;
  ASSERT_TRUE(mock.setup(std::vector<std::string>{}));
  EXPECT_EQ(mock.joint_count(), 0u);
  for (const char* call : { "connect", "start", "enable", "write" })
  {
    EXPECT_TRUE(run_call(mock, call)) << call;
  }
  std::vector<DriveState> states(3);
  EXPECT_TRUE(mock.read(states));
  EXPECT_TRUE(states.empty());
  EXPECT_EQ(stage_word(mock), "CSER");
}

}  // namespace
