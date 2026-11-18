// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The ros2_control hardware interface for Synapticon SOMANET drives.

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "synapticon_hardware/command_fallback.hpp"
#include "synapticon_hardware/command_shaping.hpp"
#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/drive_joint_map.hpp"
#include "synapticon_hardware/ethercat_health.hpp"
#include "synapticon_hardware/hal_config.hpp"
#include "synapticon_hardware/handle_access_report.hpp"
#include "synapticon_hardware/interface_binding.hpp"
#include "synapticon_hardware/synapticon_hal.hpp"
#include "synapticon_hardware/transmission_set.hpp"

namespace synapticon_hardware
{

/// @brief The ros2_control hardware for Synapticon drives, the same for every mode and any number of joints.
///
/// It exports whatever interfaces the URDF declares and connects them to the drives through a HAL plugin, which
/// the `<hardware>` parameter `hal` names. Whether a joint runs in CSP or impedance mode is configuration only:
/// the declared interfaces and the drive's mode of operation.
///
/// The HAL reads all other drive parameters itself, from the URDF and the robot definition it names.
/// URDF transmissions are loaded through pluginlib and sit between the joints and the drives.
///
/// Where a command leaves out the stiffness, the damping or the torque limit, this class fills in the joint's
/// configured value (see UnspecifiedCommandFill), on the joint side and again after the transmissions.
class SynapticonSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  /// Lets go of the HAL like on_cleanup(). A failed deactivation also lands here.
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  /// Sets the command interfaces a controller releases to "not given", so its last commands stop counting. A joint
  /// holds, see HoldReference.
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /// How often an interface access was refused since the last activation. Normally zero.
  const HandleAccessFailures& handle_access_failures() const
  {
    return access_report_.failures();
  }

  /// The declared interfaces this hardware does not know. They are exported, but nothing reads or writes them.
  const std::vector<std::string>& unknown_interfaces() const
  {
    return binding_.unknown_interfaces();
  }

protected:
  /// Load the HAL plugin. Returns nullptr after logging when it is not installed. Tests override it.
  virtual std::shared_ptr<SynapticonHalI> make_hal();

private:
  /// Give up on the HAL after a failed configure step, and log why.
  hardware_interface::CallbackReturn drop_hal(const char* reason);

  /// Copy the drive states to their joints, then let the transmissions fill in the joints without a drive.
  void states_from_drives();

  /// Build the drive commands from the joint commands, through the transmissions.
  void commands_to_drives();

  /// Copy the HAL's link health to the health interfaces. Joints without a drive keep "healthy".
  void refresh_link_health();

  /// False, after logging, when a joint without a drive declares an interface or a torque limit that only a drive
  /// has. `own_torque_limit` says per joint whether it sets max_joint_torque itself.
  bool check_joints_without_drive(const std::vector<bool>& own_torque_limit) const;

  std::string hal_plugin_{ "ethercat_soem" };
  HalConfig hal_config_{};

  // The loader must outlive the HAL it created, so it comes first.
  std::unique_ptr<pluginlib::ClassLoader<SynapticonHalI>> hal_loader_;
  std::shared_ptr<SynapticonHalI> hal_;

  // Whether read() and write() talk to the HAL. The controller manager can still call them while another thread
  // deactivates the hardware. They then return OK without doing anything, since an error would look like a fault.
  std::atomic<bool> exchanging_{ false };

  // One entry per URDF joint. The exported handles point into these vectors, so they keep their size.
  // clean_commands_ is what goes out: the controllers' commands with gaps filled in.
  std::vector<DriveState> joint_states_;
  std::vector<DriveCommand> joint_commands_;
  std::vector<DriveCommand> clean_commands_;
  std::vector<EthercatSlaveHealth> joint_link_health_;
  EthercatBusHealth bus_link_health_;
  LinkHealth hal_link_health_;

  // One entry per drive, in the order the HAL knows them.
  DriveJointMap drives_;
  std::vector<DriveState> drive_states_;
  std::vector<DriveCommand> drive_commands_;

  // Whether a drive may have power, per drive and per joint. A joint without a drive takes its motors' flag.
  std::vector<uint8_t> drive_powered_;
  std::vector<uint8_t> joint_powered_;
  HoldReference hold_;
  // What a released command interface is set to, one command per joint. Sized once, so a switch allocates no buffer.
  std::vector<DriveCommand> release_commands_;

  InterfaceBinding binding_;
  HandleAccessReport access_report_;
  TransmissionSet transmissions_;
  UnspecifiedCommandFill command_fill_;
};

}  // namespace synapticon_hardware
