// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Loading of URDF transmissions and the mapping between joint and drive space.

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/drive_joint_map.hpp"
#include "synapticon_hardware/transmission_set.hpp"
#include "description_builders.hpp"

namespace
{
using synapticon_hardware::build_drive_joint_map;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::TransmissionSet;
using synapticon_hardware::test::interface_info;
using synapticon_hardware::test::joint_info;
using synapticon_hardware::test::motion_states;
using synapticon_hardware::test::simple_transmission;

constexpr double REDUCTION = 4.0;
constexpr double OFFSET = 0.25;
constexpr double QUIET_NAN = std::numeric_limits<double>::quiet_NaN();

/// A commanded output joint without a drive, driven through a simple transmission by a motor.
hardware_interface::HardwareInfo robot(const std::string& output_name = "output")
{
  hardware_interface::HardwareInfo info;
  info.joints = { joint_info("output", { interface_info("position") }, motion_states()),
                  joint_info("motor", {}, { interface_info("position") }) };
  info.transmissions.push_back(simple_transmission("tx", output_name, "motor", REDUCTION, OFFSET));
  return info;
}

/// A differential transmission from the motors `first_motor` and `second_motor` to the joints `first` and `second`.
hardware_interface::TransmissionInfo differential(const std::string& first, const std::string& second,
                                                  const std::string& first_motor, const std::string& second_motor)
{
  auto tx = simple_transmission("differential", first, first_motor);
  tx.type = "transmission_interface/DifferentialTransmission";
  auto second_joint = tx.joints[0];
  second_joint.name = second;
  second_joint.role = "joint2";
  tx.joints.push_back(second_joint);
  auto motor = tx.actuators[0];
  motor.name = second_motor;
  motor.role = "actuator2";
  tx.actuators.push_back(motor);
  return tx;
}

/// The joints of the differential, commanded or not, and its two motors.
std::vector<hardware_interface::ComponentInfo> differential_joints(bool commanded)
{
  const std::vector<hardware_interface::InterfaceInfo> commands =
      commanded ? std::vector{ interface_info("position") } : std::vector<hardware_interface::InterfaceInfo>{};
  return { joint_info("left", commands, motion_states()), joint_info("right", commands, motion_states()),
           joint_info("motor_1", {}, { interface_info("position") }),
           joint_info("motor_2", {}, { interface_info("position") }) };
}

/// A drive-backed lift, the simple transmission of robot() and a differential, in one description. The drives are
/// lift, motor, motor_1 and motor_2, in this order.
hardware_interface::HardwareInfo two_transmissions(bool differential_commanded)
{
  auto info = robot();
  info.joints.insert(info.joints.begin(), joint_info("lift", { interface_info("position") }, motion_states()));
  const auto more = differential_joints(differential_commanded);
  info.joints.insert(info.joints.end(), more.begin(), more.end());
  info.transmissions.push_back(differential("left", "right", "motor_1", "motor_2"));
  return info;
}

/// The differential alone, with the drives motor_1 and motor_2.
hardware_interface::HardwareInfo differential_alone()
{
  hardware_interface::HardwareInfo info;
  info.joints = differential_joints(true);
  info.transmissions.push_back(differential("left", "right", "motor_1", "motor_2"));
  return info;
}

DriveState drive_state(double position, double velocity, double effort)
{
  DriveState state;
  state.position = position;
  state.velocity = velocity;
  state.effort = effort;
  return state;
}

DriveCommand joint_command(double position, double effort, double max_joint_torque)
{
  DriveCommand command;
  command.position = position;
  command.effort = effort;
  command.stiffness = 20.0;
  command.max_joint_torque = max_joint_torque;
  return command;
}

/// A standard transmission plugin loads, and clear() empties the set.
TEST(TransmissionSet, LoadsAStandardPlugin)
{
  const auto info = robot();
  TransmissionSet set;
  ASSERT_TRUE(set.load(info, build_drive_joint_map(info)));
  EXPECT_EQ(set.size(), 1u);
  set.clear();
  EXPECT_EQ(set.size(), 0u);
}

/// A transmission naming a joint the description does not have is refused.
TEST(TransmissionSet, RefusesAnUnknownJoint)
{
  const auto info = robot("nonexistent");
  TransmissionSet set;
  EXPECT_FALSE(set.load(info, build_drive_joint_map(info)));
}

/// A transmission type no installed plugin provides is refused.
TEST(TransmissionSet, RefusesAnUnknownType)
{
  auto info = robot();
  info.transmissions[0].type = "transmission_interface/NoSuchTransmission";
  TransmissionSet set;
  EXPECT_FALSE(set.load(info, build_drive_joint_map(info)));
  EXPECT_EQ(set.size(), 0u);
}

/// An actuator must be a joint with a drive.
TEST(TransmissionSet, RefusesAnActuatorWithoutADrive)
{
  auto info = robot();
  info.transmissions[0].actuators[0].name = "nonexistent";
  TransmissionSet set;
  EXPECT_FALSE(set.load(info, build_drive_joint_map(info)));
}

/// A plugin that refuses its handles fails the load: a simple transmission takes exactly one joint.
TEST(TransmissionSet, RefusesATransmissionItsPluginCannotConfigure)
{
  auto info = robot();
  info.joints.push_back(joint_info("second_output", { interface_info("position") }, motion_states()));
  auto second = info.transmissions[0].joints[0];
  second.name = "second_output";
  info.transmissions[0].joints.push_back(second);
  TransmissionSet set;
  EXPECT_FALSE(set.load(info, build_drive_joint_map(info)));
}

/// Actuator position, velocity and effort map to the output joint through the reduction and offset.
TEST(TransmissionSet, ActuatorStatesMapToTheJoint)
{
  const auto info = robot();
  TransmissionSet set;
  ASSERT_TRUE(set.load(info, build_drive_joint_map(info)));
  std::vector<DriveState> drive_states(1);
  drive_states[0].position = 1.0;
  drive_states[0].velocity = 2.0;
  drive_states[0].effort = 0.5;
  std::vector<DriveState> states(2);
  set.propagate_states(drive_states, states);
  EXPECT_NEAR(states[0].position, 1.0 / REDUCTION + OFFSET, 1e-12);
  EXPECT_NEAR(states[0].velocity, 2.0 / REDUCTION, 1e-12);
  EXPECT_NEAR(states[0].effort, 0.5 * REDUCTION, 1e-12);
  EXPECT_DOUBLE_EQ(states[1].position, 0.0) << "the drive-backed joint keeps its own state";
}

/// Joint setpoints map to the actuator, unprojected gains keep the drive's own, and the torque limit forwards 1:1.
TEST(TransmissionSet, JointCommandsMapToTheActuatorAndGainsFallBack)
{
  const auto info = robot();
  TransmissionSet set;
  ASSERT_TRUE(set.load(info, build_drive_joint_map(info)));
  std::vector<DriveCommand> commands(2);
  commands[0].position = 0.5;
  commands[0].velocity = 1.0;
  commands[0].effort = 2.0;
  commands[0].stiffness = 20.0;  // not projected by a simple transmission
  commands[0].damping = QUIET_NAN;
  commands[0].max_joint_torque = 3.0;
  std::vector<DriveCommand> drive_commands(1);
  drive_commands[0].stiffness = 7.0;
  set.apply_commands(commands, drive_commands);
  EXPECT_NEAR(drive_commands[0].position, (0.5 - OFFSET) * REDUCTION, 1e-12);
  EXPECT_NEAR(drive_commands[0].velocity, 1.0 * REDUCTION, 1e-12);
  EXPECT_NEAR(drive_commands[0].effort, 2.0 / REDUCTION, 1e-12);
  EXPECT_DOUBLE_EQ(drive_commands[0].stiffness, 7.0) << "an unprojected gain leaves the drive's own";
  EXPECT_DOUBLE_EQ(drive_commands[0].max_joint_torque, 3.0) << "the limit forwards 1:1";
}

/// An output joint with state interfaces only commands nothing: the motor drive keeps the command of its own joint
/// instead of the measured state mapped back.
TEST(TransmissionSet, AMeasuredOnlyOutputJointLeavesTheMotorCommandAlone)
{
  auto info = robot();
  info.joints[0].command_interfaces.clear();
  TransmissionSet set;
  ASSERT_TRUE(set.load(info, build_drive_joint_map(info)));
  std::vector<DriveState> drive_states(1);
  drive_states[0].position = 1.0;
  drive_states[0].effort = 0.5;
  std::vector<DriveState> states(2);
  set.propagate_states(drive_states, states);

  std::vector<DriveCommand> commands(2);
  commands[1].position = 0.3;
  commands[1].effort = 0.0;
  std::vector<DriveCommand> drive_commands{ commands[1] };
  set.apply_commands(commands, drive_commands);
  EXPECT_DOUBLE_EQ(drive_commands[0].position, 0.3);
  EXPECT_DOUBLE_EQ(drive_commands[0].effort, 0.0);
}

/// A transmission with commands on only some of its output joints is refused: the others would send their measured
/// state to the motors.
TEST(TransmissionSet, RefusesPartlyCommandedOutputJoints)
{
  hardware_interface::HardwareInfo info;
  info.joints = { joint_info("left", { interface_info("position") }, motion_states()),
                  joint_info("right", { interface_info("position") }, motion_states()),
                  joint_info("motor_1", {}, { interface_info("position") }),
                  joint_info("motor_2", {}, { interface_info("position") }) };
  info.transmissions.push_back(differential("left", "right", "motor_1", "motor_2"));
  TransmissionSet set;
  ASSERT_TRUE(set.load(info, build_drive_joint_map(info))) << "both output joints commanded";

  info.joints[1].command_interfaces.clear();
  EXPECT_FALSE(set.load(info, build_drive_joint_map(info)));
}

/// Each transmission of a set reads and writes only its own joints and drives, as if it were loaded alone.
TEST(TransmissionSet, TransmissionsInOneSetActAsIfAlone)
{
  const auto info = two_transmissions(true);
  TransmissionSet set;
  ASSERT_TRUE(set.load(info, build_drive_joint_map(info)));
  ASSERT_EQ(set.size(), 2u);
  const auto alone_info = differential_alone();
  TransmissionSet alone;
  ASSERT_TRUE(alone.load(alone_info, build_drive_joint_map(alone_info)));

  const std::vector<DriveState> drive_states{ drive_state(9.0, 9.0, 9.0), drive_state(1.0, 2.0, 0.5),
                                              drive_state(0.3, 0.1, 0.2), drive_state(-0.4, 0.5, -0.1) };
  std::vector<DriveState> states(7);
  set.propagate_states(drive_states, states);
  std::vector<DriveState> alone_states(4);
  alone.propagate_states({ drive_states[2], drive_states[3] }, alone_states);
  EXPECT_DOUBLE_EQ(states[0].position, 0.0) << "the lift is outside the transmissions";
  EXPECT_NEAR(states[1].position, 1.0 / REDUCTION + OFFSET, 1e-12);
  for (std::size_t joint = 0; joint < 2; ++joint)
  {
    EXPECT_DOUBLE_EQ(states[3 + joint].position, alone_states[joint].position);
    EXPECT_DOUBLE_EQ(states[3 + joint].velocity, alone_states[joint].velocity);
    EXPECT_DOUBLE_EQ(states[3 + joint].effort, alone_states[joint].effort);
  }

  std::vector<DriveCommand> commands(7);
  commands[1] = joint_command(0.5, 2.0, 1.0);
  commands[3] = joint_command(0.2, 0.4, 3.0);
  commands[4] = joint_command(-0.1, 0.6, 5.0);
  std::vector<DriveCommand> drive_commands(4);
  for (DriveCommand& command : drive_commands)
  {
    command.stiffness = 7.0;
    command.max_joint_torque = -1.0;
  }
  set.apply_commands(commands, drive_commands);
  std::vector<DriveCommand> alone_commands(2);
  alone.apply_commands({ commands[3], commands[4], DriveCommand{}, DriveCommand{} }, alone_commands);
  EXPECT_DOUBLE_EQ(drive_commands[0].max_joint_torque, -1.0) << "the lift's drive is outside the transmissions";
  EXPECT_NEAR(drive_commands[1].position, (0.5 - OFFSET) * REDUCTION, 1e-12);
  EXPECT_DOUBLE_EQ(drive_commands[1].max_joint_torque, 1.0);
  for (std::size_t drive = 2; drive < 4; ++drive)
  {
    EXPECT_DOUBLE_EQ(drive_commands[drive].position, alone_commands[drive - 2].position);
    EXPECT_DOUBLE_EQ(drive_commands[drive].effort, alone_commands[drive - 2].effort);
    EXPECT_DOUBLE_EQ(drive_commands[drive].stiffness, 7.0) << "an unprojected gain leaves the drive's own";
    EXPECT_DOUBLE_EQ(drive_commands[drive].max_joint_torque, 3.0) << "the limit of the first output joint";
  }
}

/// A transmission whose joints only measure leaves its motors alone, while the next one maps its commands.
TEST(TransmissionSet, AMeasuredOnlyTransmissionNextToACommandedOne)
{
  const auto info = two_transmissions(false);
  TransmissionSet set;
  ASSERT_TRUE(set.load(info, build_drive_joint_map(info)));
  std::vector<DriveCommand> commands(7);
  commands[1] = joint_command(0.5, 2.0, 1.0);
  commands[3] = joint_command(0.2, 0.4, 3.0);
  std::vector<DriveCommand> drive_commands(4);
  drive_commands[2].position = 0.7;
  drive_commands[3].position = -0.7;
  set.apply_commands(commands, drive_commands);
  EXPECT_NEAR(drive_commands[1].position, (0.5 - OFFSET) * REDUCTION, 1e-12);
  EXPECT_DOUBLE_EQ(drive_commands[2].position, 0.7);
  EXPECT_DOUBLE_EQ(drive_commands[3].position, -0.7);
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
