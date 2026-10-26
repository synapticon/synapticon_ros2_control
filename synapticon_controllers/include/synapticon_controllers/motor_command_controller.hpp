// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief A controller that passes impedance commands from a topic to the joints.

#include <array>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <synapticon_control_msgs/msg/motor_command.hpp>

namespace synapticon_controllers
{

/// @brief Passes impedance commands from a topic to the joints.
///
/// It takes synapticon_control_msgs/MotorCommand on ~/commands and writes position, velocity, effort, stiffness and
/// damping of each joint. The drive runs the impedance law with these terms:
///
///   tau_demand = clamp(K * (q_target - q) + D * (dq_target - dq) + tau_offset, +/- MaxJointTorque)
///
/// K is the stiffness, D the damping, q_target and dq_target the position and velocity, and tau_offset is
/// torque_feedforward [Nm, joint side], which goes to the effort interface. So effort adds to the spring and the
/// damper. Only with stiffness and damping at 0 is it the joint torque itself.
///
/// A message needs one position, velocity and torque_feedforward per name. stiffness and damping have one value per
/// name, or none. None writes NaN, for which the hardware puts in the joint's configured default gains. A joint that
/// a message does not name holds where it is. A message with other sizes is ignored, and every joint holds.
class MotorCommandController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  using MotorCommand = synapticon_control_msgs::msg::MotorCommand;

  /// The five values one joint gets in a cycle. NaN means "hold" for a setpoint and "the default" for a gain.
  struct Setpoint
  {
    double position;
    double velocity;
    double feedforward;
    double stiffness;
    double damping;
  };

  /// Hold every setpoint and give no gains, so the defaults apply.
  static constexpr Setpoint HOLD{ std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN() };

  /// The setpoints of all joints from one message, worked out in the subscription callback.
  struct Targets
  {
    std::vector<Setpoint> per_joint;  ///< In the order of the joints parameter.
    bool usable = false;              ///< False when the message's array sizes do not fit its names.
  };

  /// Work out the setpoints of a received message, for the next update(). Called by the subscription.
  void on_command(const std::shared_ptr<MotorCommand>& msg);

  realtime_tools::RealtimeBuffer<std::shared_ptr<const Targets>> targets_;

private:
  void write(std::size_t joint, const Setpoint& setpoint);

  std::vector<std::string> joints_;
  /// For each joint, the place of its five command interfaces in command_interfaces_.
  std::vector<std::array<std::size_t, 5>> slots_;
  rclcpp::Subscription<MotorCommand>::SharedPtr subscription_;
};

}  // namespace synapticon_controllers
