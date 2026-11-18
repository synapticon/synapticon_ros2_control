// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief SimulatedDrive: the CiA 402 state machine and the joint dynamics of a simulated drive.

#include "simulated_drive.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace synapticon_hardware
{

namespace
{
using cia402::DeviceState;

constexpr double INERTIA = 0.01;  // [kg m^2]
constexpr double FRICTION = 0.1;  // Coulomb friction [Nm]
// CSP: a position loop with time constant CSP_TIME_CONSTANT, bounded in speed and acceleration.
constexpr double CSP_TIME_CONSTANT = 0.005;     // [s]
constexpr double CSP_MAX_SPEED = 10.0;          // [rad/s]
constexpr double CSP_MAX_ACCELERATION = 500.0;  // [rad/s^2]
constexpr double CSV_TIME_CONSTANT = 0.01;      // [s]
constexpr double MAX_TORQUE = 10.0;             // [Nm]
constexpr double REST_SPEED = 1e-3;             // [rad/s] below this the joint counts as at rest
constexpr double INTEGRATION_STEP = 2.5e-4;     // [s]
constexpr double OPTION_TWO_REST_TIME = 0.05;   // [s] at rest before option code 2 de-energises
constexpr double FAULT_REACTION_TIME = 0.02;    // [s] the shortest fault reaction
constexpr uint16_t FOLLOWING_ERROR_CODE = 0x8611;
constexpr uint16_t FOLLOWING_ERROR_BIT = 0x2000;

constexpr int IMPEDANCE_MODE = -6;
constexpr int CSV_MODE = 9;

/// The controlword command bits over 0x008F, see cia402::command_bits().
bool is_disable_voltage(uint16_t cw)
{
  return (cw & 0x0002u) == 0u;
}
bool is_quick_stop(uint16_t cw)
{
  return (cw & 0x0006u) == 0x0002u;
}
bool is_shutdown(uint16_t cw)
{
  return (cw & 0x0007u) == 0x0006u;
}
bool is_switch_on(uint16_t cw)
{
  return (cw & 0x000Fu) == 0x0007u;
}
bool is_enable_operation(uint16_t cw)
{
  return (cw & 0x000Fu) == 0x000Fu;
}

bool energised(DeviceState state)
{
  return state == DeviceState::OPERATION_ENABLED || state == DeviceState::QUICK_STOP_ACTIVE ||
         state == DeviceState::FAULT_REACTION_ACTIVE;
}
}  // namespace

SimulatedDrive::SimulatedDrive(const SimulatedDriveSettings& settings) : settings_(settings)
{
}

void SimulatedDrive::command(uint16_t controlword, const DriveCommand& setpoints)
{
  controlword_ = controlword;
  setpoints_.position = setpoints.position;
  setpoints_.velocity = setpoints.velocity;
  setpoints_.effort = setpoints.effort;
  if (std::isfinite(setpoints.stiffness))
  {
    stiffness_ = setpoints.stiffness;
  }
  if (std::isfinite(setpoints.damping))
  {
    damping_ = setpoints.damping;
  }
}

void SimulatedDrive::step(double dt)
{
  const DeviceState before = state_;
  apply_controlword();
  if (state_ != before)
  {
    state_time_ = 0.0;
    following_error_since_ = 0.0;
    following_error_reported_ = false;
  }
  state_time_ += dt;

  const bool impedance = settings_.mode_of_operation == IMPEDANCE_MODE;
  torque_ = 0.0;
  switch (state_)
  {
    case DeviceState::OPERATION_ENABLED:
      if (impedance)
      {
        torque_ = impedance_torque();
        integrate_torque(torque_, dt);
      }
      else if (settings_.mode_of_operation == CSV_MODE)
      {
        velocity_ += (setpoints_.velocity - velocity_) * std::min(1.0, dt / CSV_TIME_CONSTANT);
        position_ += velocity_ * dt;
      }
      else
      {
        const double wanted =
            std::clamp((setpoints_.position - position_) / CSP_TIME_CONSTANT, -CSP_MAX_SPEED, CSP_MAX_SPEED);
        const double change = CSP_MAX_ACCELERATION * dt;
        velocity_ += std::clamp(wanted - velocity_, -change, change);
        position_ += velocity_ * dt;
      }
      update_following_error(dt);
      break;
    case DeviceState::QUICK_STOP_ACTIVE:
      if (impedance)
      {
        torque_ = impedance_torque();
        integrate_torque(torque_, dt);
      }
      else
      {
        ramp_down(settings_.quick_stop_deceleration, dt);
      }
      break;
    case DeviceState::FAULT_REACTION_ACTIVE:
      ramp_down(settings_.quick_stop_deceleration, dt);
      if (state_time_ >= FAULT_REACTION_TIME && std::abs(velocity_) < REST_SPEED)
      {
        state_ = DeviceState::FAULT;
        state_time_ = 0.0;
      }
      break;
    default:
      integrate_torque(0.0, dt);  // de-energised: coast to rest on friction
      break;
  }

  stopped_time_ = std::abs(velocity_) < REST_SPEED ? stopped_time_ + dt : 0.0;
  if (state_ == DeviceState::QUICK_STOP_ACTIVE && settings_.quick_stop_option_code == 2 &&
      stopped_time_ >= OPTION_TWO_REST_TIME)
  {
    state_ = DeviceState::SWITCH_ON_DISABLED;
    state_time_ = 0.0;
  }
}

void SimulatedDrive::apply_controlword()
{
  const bool reset_edge = (controlword_ & cia402::FAULT_RESET_BIT) != 0u && !fault_reset_bit_;
  fault_reset_bit_ = (controlword_ & cia402::FAULT_RESET_BIT) != 0u;
  const uint16_t cw = controlword_;

  switch (state_)
  {
    case DeviceState::FAULT:
      if (reset_edge)
      {
        state_ = DeviceState::SWITCH_ON_DISABLED;
        error_code_ = 0;
      }
      return;
    case DeviceState::FAULT_REACTION_ACTIVE:
      return;  // runs to Fault whatever is commanded
    case DeviceState::SWITCH_ON_DISABLED:
      if (is_shutdown(cw))
      {
        state_ = DeviceState::READY_TO_SWITCH_ON;
      }
      return;
    case DeviceState::READY_TO_SWITCH_ON:
    case DeviceState::SWITCHED_ON:
      if (is_disable_voltage(cw) || is_quick_stop(cw))
      {
        state_ = DeviceState::SWITCH_ON_DISABLED;
      }
      else if (state_ == DeviceState::READY_TO_SWITCH_ON && is_switch_on(cw))
      {
        state_ = DeviceState::SWITCHED_ON;
      }
      else if (state_ == DeviceState::SWITCHED_ON && is_enable_operation(cw))
      {
        state_ = DeviceState::OPERATION_ENABLED;
      }
      else if (state_ == DeviceState::SWITCHED_ON && is_shutdown(cw))
      {
        state_ = DeviceState::READY_TO_SWITCH_ON;
      }
      return;
    case DeviceState::OPERATION_ENABLED:
      if (is_disable_voltage(cw))
      {
        state_ = DeviceState::SWITCH_ON_DISABLED;
      }
      else if (is_quick_stop(cw))
      {
        const int code = settings_.quick_stop_option_code;
        state_ = (code == 6 || code == 2) ? DeviceState::QUICK_STOP_ACTIVE : DeviceState::SWITCH_ON_DISABLED;
      }
      else if (is_switch_on(cw))
      {
        state_ = DeviceState::SWITCHED_ON;
      }
      else if (is_shutdown(cw))
      {
        state_ = DeviceState::READY_TO_SWITCH_ON;
      }
      return;
    case DeviceState::QUICK_STOP_ACTIVE:
      if (is_disable_voltage(cw))
      {
        state_ = DeviceState::SWITCH_ON_DISABLED;
      }
      else if (is_enable_operation(cw))
      {
        state_ = DeviceState::OPERATION_ENABLED;
      }
      return;
    default:
      return;
  }
}

void SimulatedDrive::update_following_error(double dt)
{
  if (settings_.following_error == FollowingErrorMode::OFF || settings_.mode_of_operation == IMPEDANCE_MODE)
  {
    return;
  }
  const double error = settings_.mode_of_operation == CSV_MODE ? 0.0 : std::abs(setpoints_.position - position_);
  following_error_since_ = error > settings_.following_error_window ? following_error_since_ + dt : 0.0;
  const bool active = following_error_since_ >= settings_.following_error_time;
  following_error_reported_ = active;
  if (active && settings_.following_error == FollowingErrorMode::FAULT)
  {
    enter_fault_reaction(FOLLOWING_ERROR_CODE);
  }
}

double SimulatedDrive::impedance_torque() const
{
  const double torque =
      stiffness_ * (setpoints_.position - position_) + damping_ * (setpoints_.velocity - velocity_) + setpoints_.effort;
  return std::clamp(torque, -MAX_TORQUE, MAX_TORQUE);
}

void SimulatedDrive::integrate_torque(double torque, double dt)
{
  // Semi-implicit Euler in sub-steps of at most INTEGRATION_STEP, with static friction holding a
  // joint at rest.
  const int sub_steps = std::max(1, static_cast<int>(std::ceil(dt / INTEGRATION_STEP)));
  const double h = dt / sub_steps;
  for (int i = 0; i < sub_steps; ++i)
  {
    if (std::abs(velocity_) < REST_SPEED && std::abs(torque) <= FRICTION)
    {
      velocity_ = 0.0;
      continue;
    }
    const double direction =
        std::abs(velocity_) >= REST_SPEED ? std::copysign(1.0, velocity_) : std::copysign(1.0, torque);
    const double v_next = velocity_ + (torque - FRICTION * direction) / INERTIA * h;
    // Friction stops the joint, it never reverses it.
    velocity_ =
        (std::abs(velocity_) >= REST_SPEED && v_next * velocity_ < 0.0 && std::abs(torque) <= FRICTION) ? 0.0 : v_next;
    position_ += velocity_ * h;
  }
}

void SimulatedDrive::ramp_down(double deceleration, double dt)
{
  const double change = deceleration * dt;
  velocity_ = std::abs(velocity_) <= change ? 0.0 : velocity_ - std::copysign(change, velocity_);
  position_ += velocity_ * dt;
}

void SimulatedDrive::enter_fault_reaction(uint16_t error_code)
{
  state_ = DeviceState::FAULT_REACTION_ACTIVE;
  state_time_ = 0.0;
  error_code_ = error_code;
  following_error_reported_ = false;
}

uint16_t SimulatedDrive::statusword() const
{
  uint16_t sw = 0;
  switch (state_)
  {
    case DeviceState::SWITCH_ON_DISABLED:
      sw = 0x0040;
      break;
    case DeviceState::READY_TO_SWITCH_ON:
      sw = 0x0021;
      break;
    case DeviceState::SWITCHED_ON:
      sw = 0x0023;
      break;
    case DeviceState::OPERATION_ENABLED:
      sw = 0x0027;
      break;
    case DeviceState::QUICK_STOP_ACTIVE:
      sw = 0x0007;
      break;
    case DeviceState::FAULT_REACTION_ACTIVE:
      sw = 0x000F;
      break;
    case DeviceState::FAULT:
      sw = 0x0008;
      break;
    default:
      break;
  }
  if (energised(state_))
  {
    sw |= 0x0010;  // voltage enabled
  }
  if (following_error_reported_ && state_ == DeviceState::OPERATION_ENABLED)
  {
    sw |= FOLLOWING_ERROR_BIT;
  }
  return sw;
}

DriveState SimulatedDrive::state() const
{
  DriveState state;
  state.position = position_;
  state.velocity = velocity_;
  state.effort = torque_;
  state.status_word = statusword();
  state.error_code = error_code_;
  state.mode_of_operation = static_cast<int8_t>(settings_.mode_of_operation);
  state.joint_torque_demand = torque_;
  return state;
}

}  // namespace synapticon_hardware
