// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Tests for what MotorCommandController writes onto the command interfaces for a MotorCommand.
///
/// Each test checks the values one update() leaves on the interfaces: the message's values, or NaN
/// where the hardware is to hold what it has.

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>

#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "synapticon_control_msgs/msg/motor_command.hpp"
#include "synapticon_controllers/motor_command_controller.hpp"
#include "controller_test_utils.hpp"

namespace
{
using hardware_interface::CommandInterface;
namespace test = synapticon_controllers::test;
using MotorCommand = synapticon_control_msgs::msg::MotorCommand;

const std::vector<std::string> JOINTS{ "joint_1", "joint_2" };
constexpr const char* FIELDS[] = { "position", "velocity", "effort", "stiffness", "damping" };
constexpr std::size_t NUM_FIELDS = std::size(FIELDS);
enum Field : std::size_t
{
  POSITION,
  VELOCITY,
  TORQUE_FEEDFORWARD,
  STIFFNESS,
  DAMPING
};

/// Hands a message to the controller the way its subscription does.
class TestableMotorCommandController : public synapticon_controllers::MotorCommandController
{
public:
  void inject(const MotorCommand& command)
  {
    on_command(std::make_shared<MotorCommand>(command));
  }

  /// How many names of the last message the controller does not have.
  std::size_t unknown_names()
  {
    const auto* latest = targets_.readFromNonRT();
    return latest && *latest ? (*latest)->unknown_names : 0;
  }
};

/// A command for `names` with distinct values per joint, so that a swapped mapping shows.
MotorCommand command(const std::vector<std::string>& names, bool with_gains = true)
{
  MotorCommand msg;
  msg.name = names;
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const double base = 10.0 * static_cast<double>(i + 1);
    msg.position.push_back(base + 1.0);
    msg.velocity.push_back(base + 2.0);
    msg.torque_feedforward.push_back(base + 3.0);
    if (with_gains)
    {
      msg.stiffness.push_back(base + 4.0);
      msg.damping.push_back(base + 5.0);
    }
  }
  return msg;
}

/// Brings the controller up over JOINTS with every claimed interface backed by a handle.
class MotorCommandControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    controller_ = std::make_unique<TestableMotorCommandController>();
    for (const auto& joint : JOINTS)
    {
      for (const auto* field : FIELDS)
      {
        handles_.add_command(joint, field);
      }
    }
    test::bring_up(*controller_, "test_motor_command_controller", { rclcpp::Parameter("joints", JOINTS) }, handles_);
  }

  void TearDown() override
  {
    test::shut_down(*controller_);
    controller_.reset();
  }

  void update()
  {
    test::update(*controller_);
  }

  /// The value on joint `j`'s interface for `field`.
  double value(std::size_t j, Field field) const
  {
    return handles_.commands[j * NUM_FIELDS + field]->get_optional<double>().value();
  }

  void expect_held(std::size_t j) const
  {
    for (std::size_t f = 0; f < NUM_FIELDS; ++f)
    {
      EXPECT_TRUE(std::isnan(value(j, static_cast<Field>(f)))) << JOINTS[j] << "/" << FIELDS[f] << " is not held";
    }
  }

  /// Every field of joint `j` carries the values that command() gives message entry `m`.
  void expect_commanded(std::size_t j, std::size_t m) const
  {
    const double base = 10.0 * static_cast<double>(m + 1);
    EXPECT_DOUBLE_EQ(value(j, POSITION), base + 1.0);
    EXPECT_DOUBLE_EQ(value(j, VELOCITY), base + 2.0);
    EXPECT_DOUBLE_EQ(value(j, TORQUE_FEEDFORWARD), base + 3.0);
    EXPECT_DOUBLE_EQ(value(j, STIFFNESS), base + 4.0);
    EXPECT_DOUBLE_EQ(value(j, DAMPING), base + 5.0);
  }

  std::unique_ptr<TestableMotorCommandController> controller_;
  test::InterfaceHandles handles_;
};

TEST(MotorCommandControllerConfiguration, ConfigureFailsWithoutJoints)
{
  synapticon_controllers::MotorCommandController controller;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_motor_command_controller_no_joints";
  ASSERT_EQ(controller.init(params), controller_interface::return_type::OK);
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::ERROR);
}

TEST_F(MotorCommandControllerTest, ClaimsTheFiveFieldsOfEveryJointInOrder)
{
  const auto config = controller_->command_interface_configuration();
  EXPECT_EQ(config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  std::vector<std::string> expected;
  for (const auto& joint : JOINTS)
  {
    for (const auto* field : FIELDS)
    {
      expected.push_back(joint + "/" + field);
    }
  }
  EXPECT_EQ(config.names, expected);
  EXPECT_EQ(controller_->state_interface_configuration().type, controller_interface::interface_configuration_type::NONE);
}

TEST_F(MotorCommandControllerTest, HoldsEverythingUntilACommandArrives)
{
  update();
  expect_held(0);
  expect_held(1);
}

TEST_F(MotorCommandControllerTest, ACommandReachesEveryFieldOfEveryJoint)
{
  controller_->inject(command(JOINTS));
  update();
  expect_commanded(0, 0);
  expect_commanded(1, 1);
}

TEST_F(MotorCommandControllerTest, EmptyGainsLeaveTheDriveGainsUntouched)
{
  controller_->inject(command(JOINTS, false));
  update();
  for (std::size_t j = 0; j < JOINTS.size(); ++j)
  {
    EXPECT_FALSE(std::isnan(value(j, POSITION)));
    EXPECT_FALSE(std::isnan(value(j, VELOCITY)));
    EXPECT_FALSE(std::isnan(value(j, TORQUE_FEEDFORWARD)));
    EXPECT_TRUE(std::isnan(value(j, STIFFNESS)));
    EXPECT_TRUE(std::isnan(value(j, DAMPING)));
  }
}

TEST_F(MotorCommandControllerTest, InconsistentArraySizesHoldEverything)
{
  controller_->inject(command(JOINTS));
  update();
  expect_commanded(0, 0);

  MotorCommand bad = command(JOINTS);
  bad.velocity.pop_back();
  controller_->inject(bad);
  update();
  expect_held(0);
  expect_held(1);
}

/// An infinite value anywhere, or a negative gain, makes the whole message refused, like wrong sizes.
TEST_F(MotorCommandControllerTest, InvalidValuesHoldEverything)
{
  constexpr double INF = std::numeric_limits<double>::infinity();
  const std::vector<std::pair<const char*, void (*)(MotorCommand&)>> breakers = {
    { "infinite position", [](MotorCommand& m) { m.position[1] = INF; } },
    { "infinite velocity", [](MotorCommand& m) { m.velocity[0] = -INF; } },
    { "infinite torque_feedforward", [](MotorCommand& m) { m.torque_feedforward[0] = INF; } },
    { "infinite stiffness", [](MotorCommand& m) { m.stiffness[1] = INF; } },
    { "negative stiffness", [](MotorCommand& m) { m.stiffness[0] = -1.0; } },
    { "negative damping", [](MotorCommand& m) { m.damping[1] = -0.5; } },
  };
  for (const auto& [what, breaker] : breakers)
  {
    controller_->inject(command(JOINTS));
    update();
    expect_commanded(0, 0);

    MotorCommand bad = command(JOINTS);
    breaker(bad);
    controller_->inject(bad);
    update();
    SCOPED_TRACE(what);
    expect_held(0);
    expect_held(1);
  }
}

/// NaN stays allowed: it means "not given", and the gains then take the defaults.
TEST_F(MotorCommandControllerTest, NanValuesAreNotRefused)
{
  MotorCommand msg = command(JOINTS);
  msg.position[0] = std::numeric_limits<double>::quiet_NaN();
  msg.stiffness[1] = std::numeric_limits<double>::quiet_NaN();
  controller_->inject(msg);
  update();
  EXPECT_TRUE(std::isnan(value(0, POSITION)));
  EXPECT_DOUBLE_EQ(value(0, VELOCITY), 12.0);
  EXPECT_DOUBLE_EQ(value(0, STIFFNESS), 14.0);
  EXPECT_DOUBLE_EQ(value(1, POSITION), 21.0);
  EXPECT_TRUE(std::isnan(value(1, STIFFNESS)));
  EXPECT_DOUBLE_EQ(value(1, DAMPING), 25.0);
}

/// Names the controller does not have are counted for the warning, and the known joints still get their values.
TEST_F(MotorCommandControllerTest, UnknownNamesAreReportedAndIgnored)
{
  controller_->inject(command({ "joint_1", "joint_typo" }));
  update();
  EXPECT_EQ(controller_->unknown_names(), 1u);
  expect_commanded(0, 0);
  expect_held(1);

  controller_->inject(command(JOINTS));
  EXPECT_EQ(controller_->unknown_names(), 0u);
}

TEST_F(MotorCommandControllerTest, AJointMissingFromTheCommandIsHeld)
{
  controller_->inject(command({ "joint_2" }));
  update();
  expect_held(0);
  expect_commanded(1, 0);
}

TEST_F(MotorCommandControllerTest, JointsAreMatchedByNameNotByOrder)
{
  controller_->inject(command({ "joint_2", "joint_1" }));
  update();
  expect_commanded(0, 1);
  expect_commanded(1, 0);
}

TEST_F(MotorCommandControllerTest, AStaleCommandIsDroppedAtActivation)
{
  controller_->inject(command(JOINTS));
  update();
  expect_commanded(0, 0);

  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  update();
  expect_held(0);
  expect_held(1);
}

// Setup failures.

TEST(MotorCommandControllerSetup, AJointsParameterOfTheWrongTypeFailsTheInit)
{
  synapticon_controllers::MotorCommandController controller;
  EXPECT_EQ(test::init(controller, "test_motor_command_controller", { rclcpp::Parameter("joints", 5) }),
            controller_interface::return_type::ERROR);
}

TEST(MotorCommandControllerSetup, AMissingCommandInterfaceFailsTheActivation)
{
  test::InterfaceHandles handles;
  for (const auto& joint : JOINTS)
  {
    for (const auto* field : FIELDS)
    {
      if (!(joint == JOINTS.back() && std::string(field) == "damping"))
      {
        handles.add_command(joint, field);
      }
    }
  }
  synapticon_controllers::MotorCommandController controller;
  ASSERT_EQ(test::init(controller, "test_motor_command_controller", { rclcpp::Parameter("joints", JOINTS) }),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(test::activate(controller, handles), controller_interface::CallbackReturn::ERROR);
  controller.release_interfaces();
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
