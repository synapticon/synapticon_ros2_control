// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The interface binding: NamedField and find_field on a throwaway struct, and InterfaceBinding on the
/// handles of a minimal system component.

#include <memory>
#include <future>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <gmock/gmock.h>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "synapticon_hardware/interface_binding.hpp"

namespace
{
using synapticon_hardware::DriveAction;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::EthercatBusHealth;
using synapticon_hardware::EthercatSlaveHealth;
using synapticon_hardware::find_field;
using synapticon_hardware::InterfaceBinding;
using synapticon_hardware::NamedField;

struct Owner
{
  double position{ 0.0 };
  double velocity{ 0.0 };
};

enum class Tag
{
  NONE,
  SCALED
};

constexpr NamedField<double, Owner> PLAIN[] = {
  { "position", &Owner::position },
  { "velocity", &Owner::velocity },
};

// Only the second entry states its tag, the first relies on the value-initialized Extra.
constexpr NamedField<double, Owner, Tag> TAGGED[] = {
  { "position", &Owner::position },
  { "velocity", &Owner::velocity, Tag::SCALED },
};

/// find_field() returns the member pointer of the named field.
TEST(InterfaceBindingTest, FindsABoundFieldAndWritesThroughIt)
{
  double Owner::*const field = find_field(PLAIN, "velocity");
  ASSERT_NE(field, nullptr);
  Owner owner;
  owner.*field = 2.5;
  EXPECT_EQ(owner.velocity, 2.5);
  EXPECT_EQ(owner.position, 0.0);
}

/// An unknown or empty name finds nothing.
TEST(InterfaceBindingTest, ReturnsNullForAnUnknownName)
{
  EXPECT_EQ(find_field(PLAIN, "effort"), nullptr);
  EXPECT_EQ(find_field(PLAIN, ""), nullptr);
}

/// An omitted Extra payload is value-initialized.
TEST(InterfaceBindingTest, OmittedExtraIsValueInitialized)
{
  static_assert(TAGGED[0].extra == Tag{});
  EXPECT_EQ(TAGGED[0].extra, Tag::NONE);
  EXPECT_EQ(TAGGED[1].extra, Tag::SCALED);
}

/// The lookup works the same with an Extra payload.
TEST(InterfaceBindingTest, LookupIsIndependentOfTheExtraPayload)
{
  EXPECT_EQ(find_field(TAGGED, "position"), &Owner::position);
  EXPECT_EQ(find_field(TAGGED, "unknown"), nullptr);
}

/// A system component that only holds handles.
class HandleHolder : public hardware_interface::SystemInterface
{
public:
  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    return hardware_interface::return_type::OK;
  }
};

/// An interface of `name` with `data_type`.
hardware_interface::InterfaceInfo interface(const std::string& name, const std::string& data_type = "double")
{
  hardware_interface::InterfaceInfo info;
  info.name = name;
  info.data_type = data_type;
  return info;
}

/// One joint with a position state and the commands position and drive_action, and buffers to bind it to.
class BoundJoint : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_1";
    joint.type = "joint";
    joint.state_interfaces = { interface("position") };
    joint.command_interfaces = { interface("position"), interface("drive_action", "uint8") };
    hardware_interface::HardwareComponentParams params;
    params.hardware_info.name = "binding_test";
    params.hardware_info.type = "system";
    params.hardware_info.joints = { joint };
    params.clock = std::make_shared<rclcpp::Clock>();
    ASSERT_EQ(component_.init(params), hardware_interface::CallbackReturn::SUCCESS);
    component_.on_export_state_interfaces();
    component_.on_export_command_interfaces();
    info_ = params.hardware_info;
  }

  bool bind()
  {
    return binding_.bind(info_, component_, states_, commands_, slave_health_, bus_health_);
  }

  const hardware_interface::CommandInterface::SharedPtr& handle(const std::string& name)
  {
    return component_.get_command_interface_handle("joint_1/" + name);
  }

  HandleHolder component_;
  hardware_interface::HardwareInfo info_;
  InterfaceBinding binding_;
  std::vector<DriveState> states_ = std::vector<DriveState>(1);
  std::vector<DriveCommand> commands_ = std::vector<DriveCommand>(1);
  std::vector<EthercatSlaveHealth> slave_health_ = std::vector<EthercatSlaveHealth>(1);
  EthercatBusHealth bus_health_;
};

TEST_F(BoundJoint, CopiesTheCommandHandlesIntoTheirFields)
{
  ASSERT_TRUE(bind());
  ASSERT_TRUE(handle("position")->set_value(1.5));
  ASSERT_TRUE(handle("drive_action")->set_value(static_cast<uint8_t>(DriveAction::ENABLE)));
  const auto refused = binding_.pull_commands();
  EXPECT_EQ(refused.setpoints, 0u);
  EXPECT_EQ(refused.actions, 0u);
  EXPECT_EQ(commands_[0].position, 1.5);
  EXPECT_EQ(commands_[0].drive_action, DriveAction::ENABLE);
}

/// A handle that cannot be read leaves its field alone, so a held drive_action stays the same request.
TEST_F(BoundJoint, ARefusedReadKeepsTheLastValue)
{
  ASSERT_TRUE(bind());
  ASSERT_TRUE(handle("position")->set_value(1.5));
  ASSERT_TRUE(handle("drive_action")->set_value(static_cast<uint8_t>(DriveAction::ENABLE)));
  binding_.pull_commands();

  // Another thread holds both handles, as a controller writing them would.
  std::promise<void> locked;
  std::promise<void> release;
  std::thread holder([&] {
    std::unique_lock<std::shared_mutex> position_lock(handle("position")->get_mutex());
    std::unique_lock<std::shared_mutex> action_lock(handle("drive_action")->get_mutex());
    locked.set_value();
    release.get_future().wait();
  });
  locked.get_future().wait();
  const auto refused = binding_.pull_commands();
  release.set_value();
  holder.join();

  EXPECT_EQ(refused.setpoints, 1u);
  EXPECT_EQ(refused.actions, 1u);
  EXPECT_EQ(commands_[0].position, 1.5);
  EXPECT_EQ(commands_[0].drive_action, DriveAction::ENABLE);
}

/// Buffers that do not hold one entry per joint are refused.
TEST_F(BoundJoint, BuffersOfTheWrongSizeAreRefused)
{
  std::vector<DriveState> no_states;
  std::vector<DriveCommand> two_commands(2);
  std::vector<EthercatSlaveHealth> no_health;
  EXPECT_FALSE(binding_.bind(info_, component_, no_states, commands_, slave_health_, bus_health_));
  EXPECT_FALSE(binding_.bind(info_, component_, states_, two_commands, slave_health_, bus_health_));
  EXPECT_FALSE(binding_.bind(info_, component_, states_, commands_, no_health, bus_health_));
  EXPECT_TRUE(bind());
}
}  // namespace
