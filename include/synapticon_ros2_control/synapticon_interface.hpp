// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025 Synapticon GmbH
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "ethercat.h"

namespace synapticon_ros2_control {

namespace {
struct SpringAdjustState {
    std::chrono::steady_clock::time_point time_prev_;
    std::optional<double> error_prev_;
};

// /**
//  * @brief Computes control output for spring adjust joint using a custom PID implementation
//  *
//  * @param target_position [in] The desired position in potentiometer ticks.
//  * @param state [in/out] A SpringAdjustState struct containing:
//  *                      - time_prev_: Previous timestamp for computing time derivatives
//  *                      - error_prev_: Previous error value for computing error derivatives
//  * @param allow_mode_change [in/out] Boolean flag that gets set to true when the target position is reached
//  *                                  and stable (error < 200 ticks and error_dt <= 1).
//  *                                  Allows us to leave this control mode.
//  * @return double The computed actuator torque in per-mill of rated torque. The output is clamped.
//  *
//  * @note This function implements a custom PD control loop for the spring adjust joint, which uses
//  *       a linear potentiometer for position feedback. Unlike other joints in the system that utilize
//  *       the built-in Synapticon PID control, this joint requires custom control logic to handle
//  *       the potentiometer-based position sensing.
//  */
// double spring_adjust_torque_pd(
//   double target_position,
//   SpringAdjustState& state,
//   bool& allow_mode_change);
} // namespace

#pragma pack(1)
// Somanet structs
typedef struct {
  uint16_t Statusword;
  int8_t OpModeDisplay;
  int32_t PositionValue;
  int32_t VelocityValue;
  int16_t TorqueValue;
  uint16_t AnalogInput1;
  uint16_t AnalogInput2;
  uint16_t AnalogInput3;
  uint16_t AnalogInput4;
  uint32_t TuningStatus;
  uint32_t DigitalInputs;
  uint32_t UserMISO;
  uint32_t Timestamp;
  int32_t PositionDemandInternalValue;
  int32_t VelocityDemandValue;
  int16_t TorqueDemand;
} InSomanet50t;

typedef struct {
  uint16_t Controlword;
  int8_t OpMode;
  int16_t TargetTorque;
  int32_t TargetPosition;
  int32_t TargetVelocity;
  int16_t TorqueOffset;
  int32_t TuningCommand;
  int32_t PhysicalOutputs;
  int32_t BitMask;
  int32_t UserMOSI;
  int32_t VelocityOffset;
} OutSomanet50t;
#pragma pack()

class SynapticonSystemInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SynapticonSystemInterface)

  ~SynapticonSystemInterface();

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger getLogger() const { return *logger_; }

private:
  /**
   * @brief Error checking. Typically runs in a separate thread.
   */
  OSAL_THREAD_FUNC ecatCheck(void *ptr);

  /**
   * @brief Somanet control loop runs in a dedicated thread
   * This steps through several states to get to Operational, if needed
   * @param in_normal_op_mode A flag to the main thread that the Somanet state
   * machine is ready
   */
  void somanetCyclicLoop(std::atomic<bool> &in_normal_op_mode);

  /**
   * @brief Use an ethercat SDO read to check if the emergency stop is engaged
   */
  bool eStopEngaged();

  std::optional<std::thread> somanet_control_thread_;

  size_t num_joints_;

  std::shared_ptr<rclcpp::Logger> logger_;

  std::vector<double> mechanical_reductions_;

  // Store the commands for the simulated robot
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  // hw_commands_quick_stop_ is never actually used, just a placeholder for compilation
  std::vector<double> hw_commands_quick_stop_;
  // hw_commands_spring_adjust_ is potentiometer ticks.
  std::vector<double> hw_commands_spring_adjust_;
  // hw_commands_compensate_for_load_ is never actually used, just a placeholder for compilation
  std::vector<double> hw_commands_compensate_for_removed_load_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_accelerations_;
  std::vector<double> hw_states_efforts_;
  // Threadsafe deques to share commands with somanet control loop thread
  std::deque<std::atomic<double>> threadsafe_commands_efforts_;
  std::deque<std::atomic<double>> threadsafe_commands_velocities_;
  std::deque<std::atomic<double>> threadsafe_commands_positions_;
  std::deque<std::atomic<double>> threadsafe_commands_spring_adjust_;

  // Enum defining current control level
  enum control_level_t : std::uint8_t {
    UNDEFINED = 0,
    EFFORT = 1, // aka torque
    VELOCITY = 2,
    POSITION = 3,
    QUICK_STOP = 4,
    // To manually specify spring adjust position, use this control mode
    SPRING_ADJUST = 5,
    // If a load was just removed, use this control mode
    COMPENSATE_FOR_REMOVED_LOAD = 6,
  };

  // Active control mode for each actuator
  std::vector<control_level_t> control_level_;

  // For SOEM
  OSAL_THREAD_HANDLE ecat_error_thread_;
  char io_map_[4096];

  std::vector<InSomanet50t *> in_somanet_1_;
  std::mutex in_somanet_mtx_;
  std::vector<OutSomanet50t *> out_somanet_1_;

  std::vector<uint32_t> encoder_resolutions_;

  // For coordination between threads
  volatile std::atomic<int> wkc_;
  std::atomic<int> expected_wkc_;
  std::atomic<bool> needlf_ = false;
  std::atomic<bool> in_normal_op_mode_ = false;
  // During spring adjust, don't allow control mode to change until the target position is reached
  std::atomic<bool> allow_mode_change_ = true;
  SpringAdjustState spring_adjust_state_;
};

} // namespace synapticon_ros2_control
