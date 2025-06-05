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
  rclcpp::Logger get_logger() const { return *logger_; }

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

  std::optional<std::thread> somanet_control_thread_;

  size_t num_joints_;

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;

  std::vector<double> mechanical_reductions_;

  // Store the commands for the simulated robot
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  // hw_commands_quick_stop_ is never actually used, just a placeholder for compilation
  std::vector<double> hw_commands_quick_stop_;
  // hw_commands_spring_adjust_ is currently potentiometer ticks.
  // ~64000 ticks corresponds to max spring force.
  // ~1232 ticks corresponds to the opposite end of travel.
  std::vector<double> hw_commands_spring_adjust_;
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
    SPRING_ADJUST = 5,
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
  struct SpringAdjust
    {
      // During spring adjust, don't allow control mode to change until the target position is reached
      std::atomic<bool> allow_mode_change_ = true;
      // Spring adjust derivative term variables
      std::optional<double> error_prev_;
      std::chrono::steady_clock::time_point time_prev_ = std::chrono::steady_clock::now();
    } spring_adjust_state_;
};

} // namespace synapticon_ros2_control
