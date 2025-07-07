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

#include "synapticon_ros2_control/synapticon_interface.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <inttypes.h>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace synapticon_ros2_control {
namespace {
constexpr int EC_TIMEOUTMON = 500;
constexpr char LOG_NAME[] = "synapticon_ros2_control";
constexpr double DEG_TO_RAD = 0.0174533;
constexpr size_t PROFILE_TORQUE_MODE = 4;
constexpr size_t CYCLIC_VELOCITY_MODE = 9;
constexpr size_t CYCLIC_POSITION_MODE = 8;
constexpr double RPM_TO_RAD_PER_S = 0.10472;
constexpr double RAD_PER_S_TO_RPM = 1 / RPM_TO_RAD_PER_S;
unsigned int NORMAL_OPERATION_BRAKES_OFF = 0b00001111;
// Bit 2 (0-indexed) goes to 0 to turn on Quick Stop
unsigned int NORMAL_OPERATION_BRAKES_ON = 0b00001011;
constexpr char EXPECTED_SLAVE_NAME[] = "SOMANET";
constexpr std::array<double, 7> TORQUE_FRICTION_OFFSET = {0, 0, 0, 0, 10, 0, 0}; // per mill
constexpr size_t SPRING_ADJUST_IDX = 2;
constexpr size_t INERTIAL_ACTUATOR_IDX = 3;
constexpr size_t WRIST_PITCH_IDX = 5;
constexpr size_t WRIST_ROLL_IDX = 6;
// TODO: update this if the wrist or EE is added
constexpr double SPRING_POSITION_WITHOUT_PAYLOAD = 31000;
// TODO: update this if the max payload changes
constexpr double SPRING_POSITION_MAX_PAYLOAD = 45000;
// Expected midpoint of the 16-bit analog inputs
constexpr int32_t ANALOG_INPUT_MIDPOINT = 32768;
constexpr int32_t WRIST_DIAL_MIN = 19000;
constexpr int32_t WRIST_DIAL_MAX = 46000;
constexpr double MAX_WRIST_PITCH_VELOCITY = 0.3;
constexpr double MAX_WRIST_ROLL_VELOCITY = 0.15;
// TODO: what's with the bullshit multiplier?
constexpr double MYSTERY_VELOCITY_MULTIPLIER = 10000;
constexpr double WRIST_PITCH_DEADBAND = 0.05;
constexpr double WRIST_ROLL_DEADBAND = 0.05;
// Motion threshold of the inertial actuator
constexpr double DYNAMIC_COMP_MOTION_THRESHOLD = 0.09;  // rad
constexpr double SPRING_ADJUST_MAX_TORQUE = 2500.0;  // per mill of rated torque

int32_t read_sdo_value(uint16_t slave_idx, uint16_t index, uint8_t subindex) {
    int32_t value_holder;
    int object_size = sizeof(value_holder);
    int timeout = EC_TIMEOUTRXM;
    ec_SDOread(slave_idx, index, subindex, FALSE, &object_size, &value_holder, timeout);
    return value_holder;
}

/**
 * @brief Convert an incremental position command in input shaft encoder ticks to output shaft radians
 */
double input_ticks_to_output_shaft_rad(int32_t ticks, double mechanical_reduction, uint32_t encoder_resolution) {
  return (static_cast<double>(ticks) / encoder_resolution) * 2.0 * M_PI / mechanical_reduction;
}

/**
 * @brief Convert an incremental output shaft radian command to input shaft encoder ticks
 */
int32_t output_shaft_rad_to_input_ticks(double output_shaft_rad, double mechanical_reduction, uint32_t encoder_resolution) {
  return (output_shaft_rad * encoder_resolution * mechanical_reduction / (2.0 * M_PI));
}

double spring_adjust_torque_pd(
  double target_position,
  int32_t current_spring_pot_position,
  SpringAdjustState& state,
  bool& allow_mode_change) {

  double K_P = 1.0;
  double K_D = 0.5;
  double error = target_position - static_cast<double>(current_spring_pot_position);
  std::chrono::steady_clock::time_point time_now = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_elapsed = time_now - state.time_prev_;
  double error_dt = 0;
  if (state.error_prev_) {
      error_dt = (error - *state.error_prev_) / time_elapsed.count();
  }
  state.error_prev_ = error;
  state.time_prev_ = time_now;
  double actuator_torque = K_P * error + K_D * error_dt;

  // A ceiling at X% of rated torque
  // With a floor of Y% torque (below that, the motor doesn't move)
  // We overdrive the motor, higher than rated torque, since it's a quick motion
  if (actuator_torque > 0) {
      // Per mill of rated torque
      actuator_torque = std::clamp(actuator_torque, 900.0, SPRING_ADJUST_MAX_TORQUE);
  } else {
      actuator_torque = std::clamp(actuator_torque, -SPRING_ADJUST_MAX_TORQUE, -900.0);
  }

  // Don't allow control mode to change until the target position is reached and is stable
  // This is also a deadband
  if (std::abs(error) < 500 && error_dt <= 1) {
      allow_mode_change = true;
      // We can safely set the target torque to zero b/c this actuator is not backdrivable
      actuator_torque = 0;
  } else {
      allow_mode_change = false;
  }

  return actuator_torque;
}

// This is related to making a member function static for osal_thread_create
OSAL_THREAD_FUNC ecatCheckWrapper(void *ptr) {
    SynapticonSystemInterface* interface = static_cast<SynapticonSystemInterface*>(ptr);
    return interface->ecatCheck(ptr);
}
} // namespace

hardware_interface::CallbackReturn SynapticonSystemInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("synapticon_interface"));

  num_joints_ = info_.joints.size();

  hw_states_positions_.resize(num_joints_,
                              std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(num_joints_,
                               std::numeric_limits<double>::quiet_NaN());
  hw_states_accelerations_.resize(num_joints_,
                                  std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(num_joints_,
                            std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(num_joints_,
                                std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(num_joints_, 0);
  hw_commands_efforts_.resize(num_joints_,
                              std::numeric_limits<double>::quiet_NaN());
  hw_commands_quick_stop_.resize(num_joints_,
                              std::numeric_limits<double>::quiet_NaN());
  hw_commands_spring_adjust_.resize(num_joints_,
                              std::numeric_limits<double>::quiet_NaN());
  hw_commands_compensate_for_removed_load_.resize(num_joints_,
                              std::numeric_limits<double>::quiet_NaN());
  hw_commands_compensate_for_added_load_.resize(num_joints_,
                              std::numeric_limits<double>::quiet_NaN());
  control_level_.resize(num_joints_, control_level_t::UNDEFINED);
  // Atomic deques are difficult to initialize
  mechanical_reductions_.resize(num_joints_);
  for (auto &reduction : mechanical_reductions_) {
    reduction.store(1.0);
  }
  threadsafe_commands_efforts_.resize(num_joints_);
  for (auto &effort : threadsafe_commands_efforts_) {
    effort.store(std::numeric_limits<double>::quiet_NaN());
  }
  threadsafe_commands_velocities_.resize(num_joints_);
  for (auto &velocity : threadsafe_commands_velocities_) {
    velocity.store(0.0);
  }
  threadsafe_commands_positions_.resize(num_joints_);
  for (auto &position : threadsafe_commands_positions_) {
    position.store(std::numeric_limits<double>::quiet_NaN());
  }
  threadsafe_commands_spring_adjust_.resize(num_joints_);
  for (auto &spring_adjust : threadsafe_commands_spring_adjust_) {
    spring_adjust.store(std::numeric_limits<double>::quiet_NaN());
  }

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (!(joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == "hand_guided" ||
          joint.command_interfaces[0].name == "quick_stop" ||
          joint.command_interfaces[0].name == "spring_adjust" ||
          joint.command_interfaces[0].name == "compensate_for_removed_load" ||
          joint.command_interfaces[0].name == "compensate_for_added_load")) {
      RCLCPP_FATAL(
          getLogger(),
          "Joint '%s' has %s command interface. Expected %s, %s, %s, %s, %s, %s,or %s.",
          joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY,
          "hand_guided",
          "quick_stop",
          "spring_adjust",
          "compensate_for_removed_load",
          "compensate_for_added_load");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_ACCELERATION ||
          joint.state_interfaces[0].name == "hand_guided")) {
      RCLCPP_FATAL(
          getLogger(),
          "Joint '%s' has %s state interface. Expected %s, %s, %s, or %s.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY,
          hardware_interface::HW_IF_ACCELERATION,
          "hand_guided");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (size_t i = 0; i < num_joints_; ++i)
  {
    std::string reduction_param_name = info_.joints.at(i).name + "_mechanical_reduction";
    if (info_.hardware_parameters.find(reduction_param_name) == info_.hardware_parameters.end())
    {
      mechanical_reductions_.at(i) = 1.0;
      continue;
    }
    mechanical_reductions_.at(i) = stod(info_.hardware_parameters[reduction_param_name]);
  }

  // A thread to handle ethercat errors
  osal_thread_create(&ecat_error_thread_, 128000,
                     (void*)&ecatCheckWrapper,
                     this);

  // Ethercat initialization
  // Define the interface name (e.g. eth0 or eno0) in the ros2_control.xacro
  std::string eth_device = info_.hardware_parameters["eth_device"];
  int ec_init_status = ec_init(eth_device.c_str());
  if (ec_init_status <= 0) {
    RCLCPP_FATAL_STREAM(getLogger(),
                        "Error during initialization of ethercat interface: "
                            << eth_device.c_str() << " with status: "
                            << ec_init_status);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (ec_config_init(false) <= 0) {
    RCLCPP_FATAL(getLogger(), "No ethercat slaves found!");
    ec_close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  ec_config_map(&io_map_);
  ec_configdc();
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  // Request operational state for all slaves
  expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  for (int slave_id = 0; slave_id < ec_slavecount; ++slave_id) {
    ec_slave[slave_id].state = EC_STATE_OPERATIONAL;
  }
  // send one valid process data to make outputs in slaves happy
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  // request OP state for all slaves
  ec_writestate(0);
  size_t chk = 200;

  // wait for all slaves to reach OP state
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
    RCLCPP_FATAL(getLogger(),
                 "An ethercat slave failed to reach OPERATIONAL state");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Connect struct pointers to I/O
  for (size_t joint_idx = 1; joint_idx < (num_joints_ + 1); ++joint_idx) {
    in_somanet_.push_back((InSomanet50t *)ec_slave[joint_idx].inputs);
    out_somanet_.push_back((OutSomanet50t *)ec_slave[joint_idx].outputs);
  }

  // Read encoder resolution for each joint
  encoder_resolutions_.resize(num_joints_);
  for (size_t joint_idx = 1; joint_idx < (num_joints_ + 1); ++joint_idx) {
    // Verify slave name
    if (strcmp(ec_slave[joint_idx].name, EXPECTED_SLAVE_NAME) != 0) {
      RCLCPP_FATAL(
          getLogger(),
          "Expected slave %s at position %zu, but got %s instead",
          EXPECTED_SLAVE_NAME, joint_idx, ec_slave[joint_idx].name);
      return hardware_interface::CallbackReturn::ERROR;
    }

    uint8_t encoder_source;
    int size = sizeof(encoder_source);
    ec_SDOread(joint_idx, 0x2012, 0x09, false, &size, &encoder_source, EC_TIMEOUTRXM);
    
    uint32_t encoder_resolution;
    size = sizeof(encoder_resolution);
    if (encoder_source == 1) {
      ec_SDOread(joint_idx, 0x2110, 0x03, false, &size, &encoder_resolution,
                EC_TIMEOUTRXM);
    } else if (encoder_source == 2) {
      ec_SDOread(joint_idx, 0x2112, 0x03, false, &size, &encoder_resolution,
                EC_TIMEOUTRXM);
    } else {
      RCLCPP_FATAL(
          getLogger(),
          "No encoder configured for position control on joint %zu. Terminating the program",
          joint_idx);
      return hardware_interface::CallbackReturn::ERROR;
    }
    encoder_resolutions_[joint_idx-1].store(encoder_resolution);
  }

  // Start the control loop, wait for it to reach normal operation mode
  somanet_control_thread_ =
      std::thread(&SynapticonSystemInterface::somanetCyclicLoop, this,
                  std::ref(in_normal_op_mode_));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SynapticonSystemInterface::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  if (!allow_mode_change_)
  {
    RCLCPP_ERROR(getLogger(), "A control mode change is disallowed at this moment.");
    return hardware_interface::return_type::ERROR;
  }

  // This should be cleared unless in COMPENSATE_FOR_ADDED_LOAD mode
  initial_inertial_actuator_position_ = std::nullopt;

  // Prepare for new command modes
  std::vector<control_level_t> new_modes = {};
  for (const std::string& key : start_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (key ==
          info_.joints[i].name + "/" + "hand_guided") {
        new_modes.push_back(control_level_t::HAND_GUIDED);
      } else if (key == info_.joints[i].name + "/" +
                            hardware_interface::HW_IF_VELOCITY) {
        new_modes.push_back(control_level_t::VELOCITY);
      } else if (key == info_.joints[i].name + "/" +
                            hardware_interface::HW_IF_POSITION) {
        new_modes.push_back(control_level_t::POSITION);
      } else if (key == info_.joints[i].name + "/quick_stop") {
        new_modes.push_back(control_level_t::QUICK_STOP);
      } else if (key == info_.joints[i].name + "/spring_adjust") {
        // Spring adjust puts all joints in QUICK_STOP mode except the spring adjust joint
        if (i == SPRING_ADJUST_IDX) {
          new_modes.push_back(control_level_t::SPRING_ADJUST);
          spring_adjust_state_.time_prev_ = std::chrono::steady_clock::now();
          spring_adjust_state_.error_prev_ = std::nullopt;
        } else {
          new_modes.push_back(control_level_t::QUICK_STOP);
        }
      } else if (key == info_.joints[i].name + "/compensate_for_removed_load") {
        // compensate_for_removed_load puts all joints in QUICK_STOP mode except the spring adjust joint
        if (i == SPRING_ADJUST_IDX) {
          new_modes.push_back(control_level_t::COMPENSATE_FOR_REMOVED_LOAD);
        } else {
          new_modes.push_back(control_level_t::QUICK_STOP);
        }
      } else if (key == info_.joints[i].name + "/compensate_for_added_load") {
        {
          std::lock_guard<std::mutex> lock(hw_state_mtx_);
          initial_inertial_actuator_position_ = hw_states_positions_[INERTIAL_ACTUATOR_IDX];
        }
        // compensate_for_added_load puts all joints in QUICK_STOP mode except those in the elevation link
        if ((i == SPRING_ADJUST_IDX) || (i == INERTIAL_ACTUATOR_IDX)) {
          new_modes.push_back(control_level_t::COMPENSATE_FOR_ADDED_LOAD);
        } else {
          new_modes.push_back(control_level_t::QUICK_STOP);
        }
      }
    }
  }

  // Stop motion on all relevant joints
  for (const std::string& key : stop_interfaces) {
    for (std::size_t i = 0; i < num_joints_; i++) {
      if (key.find(info_.joints[i].name) != std::string::npos) {
        hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
        hw_commands_velocities_[i] = 0;
        hw_commands_efforts_[i] = 0;
        hw_commands_spring_adjust_[i] = std::numeric_limits<double>::quiet_NaN();
        threadsafe_commands_efforts_[i] =
            std::numeric_limits<double>::quiet_NaN();
        threadsafe_commands_velocities_[i] = 0;
        threadsafe_commands_positions_[i] =
            std::numeric_limits<double>::quiet_NaN();
        threadsafe_commands_spring_adjust_[i] =
            std::numeric_limits<double>::quiet_NaN();
        control_level_[i] = control_level_t::UNDEFINED;
      }
    }
  }

  for (const std::string& key : start_interfaces) {
    for (std::size_t i = 0; i < num_joints_; i++) {
      if (key.find(info_.joints[i].name) != std::string::npos) {
        if (control_level_[i] != control_level_t::UNDEFINED) {
          // Something else is using the joint! Abort!
          RCLCPP_FATAL(getLogger(),
                       "Something else is using the joint. Abort!");
          return hardware_interface::return_type::ERROR;
        }
        control_level_[i] = new_modes[i];
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn SynapticonSystemInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  // Set some default values
  for (std::size_t i = 0; i < num_joints_; i++) {
    if (std::isnan(hw_states_velocities_[i])) {
      hw_states_velocities_[i] = 0;
    }
    if (std::isnan(hw_states_accelerations_[i])) {
      hw_states_accelerations_[i] = 0;
    }
    if (std::isnan(hw_states_efforts_[i])) {
      hw_states_efforts_[i] = 0;
    }

    hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_velocities_[i] = 0;
    hw_commands_efforts_[i] = 0;
    hw_commands_spring_adjust_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_compensate_for_removed_load_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_compensate_for_added_load_[i] = std::numeric_limits<double>::quiet_NaN();
    threadsafe_commands_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
    threadsafe_commands_velocities_[i] = 0;
    threadsafe_commands_positions_[i] =
        std::numeric_limits<double>::quiet_NaN();
    threadsafe_commands_spring_adjust_[i] =
        std::numeric_limits<double>::quiet_NaN();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SynapticonSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  for (std::size_t i = 0; i < num_joints_; i++) {
    control_level_[i] = control_level_t::UNDEFINED;

    hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_velocities_[i] = 0;
    hw_commands_efforts_[i] = 0;
    hw_commands_spring_adjust_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_compensate_for_removed_load_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_compensate_for_added_load_[i] = std::numeric_limits<double>::quiet_NaN();
    threadsafe_commands_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
    threadsafe_commands_velocities_[i] = 0;
    threadsafe_commands_positions_[i] =
        std::numeric_limits<double>::quiet_NaN();
    threadsafe_commands_spring_adjust_[i] =
        std::numeric_limits<double>::quiet_NaN();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SynapticonSystemInterface::read(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) {
  std::lock_guard<std::mutex> lock(hw_state_mtx_);
  for (std::size_t i = 0; i < num_joints_; i++) {
    // InSomanet50t doesn't include acceleration
    hw_states_accelerations_[i] = 0;
    hw_states_velocities_[i] = input_ticks_to_output_shaft_rad(in_somanet_[i]->VelocityValue, mechanical_reductions_.at(i).load(), encoder_resolutions_[i].load());
    hw_states_positions_[i] = input_ticks_to_output_shaft_rad(in_somanet_[i]->PositionValue, mechanical_reductions_.at(i).load(), encoder_resolutions_[i].load());
    hw_states_efforts_[i] = mechanical_reductions_.at(i).load() * in_somanet_[i]->TorqueValue;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SynapticonSystemInterface::write(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {
  // This function doesn't do much.
  // It's taken care of in separate thread, somanet_control_thread_

  // Share the commands with somanet control loop in a threadsafe way
  for (std::size_t i = 0; i < num_joints_; i++) {
    // Torque commands are "per thousand of rated torque"
    if (!std::isnan(hw_commands_efforts_[i]))
    {
      hw_commands_efforts_[i] =
          std::clamp(hw_commands_efforts_[i], -1000.0, 1000.0);
      threadsafe_commands_efforts_[i] = hw_commands_efforts_[i];
    }
    if (!std::isnan(hw_commands_velocities_[i]))
    {
      // TODO: should this command be ticks per second?
      threadsafe_commands_velocities_[i] = mechanical_reductions_.at(i).load() * hw_commands_velocities_[i] * RAD_PER_S_TO_RPM;
    }
    if (!std::isnan(hw_commands_positions_[i]))
    {
      threadsafe_commands_positions_[i] = output_shaft_rad_to_input_ticks(hw_commands_positions_[i], mechanical_reductions_.at(i).load(), encoder_resolutions_[i].load());
    }
    if (!std::isnan(hw_commands_spring_adjust_[i]))
    {
      hw_commands_spring_adjust_[i] = std::clamp(hw_commands_spring_adjust_[i], SPRING_POSITION_WITHOUT_PAYLOAD, SPRING_POSITION_MAX_PAYLOAD);
      threadsafe_commands_spring_adjust_[i] = hw_commands_spring_adjust_[i];
    }
    // No need to do anything for compensate_for_added/removed_load, these algorithms are automated
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
SynapticonSystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < num_joints_; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION,
        &hw_states_accelerations_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, "hand_guided",
        &hw_states_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SynapticonSystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < num_joints_; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "hand_guided",
        &hw_commands_efforts_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "quick_stop",
        &hw_commands_quick_stop_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "spring_adjust",
        &hw_commands_spring_adjust_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "compensate_for_removed_load",
        &hw_commands_compensate_for_removed_load_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "compensate_for_added_load",
        &hw_commands_compensate_for_added_load_[i]));
  }
  return command_interfaces;
}

SynapticonSystemInterface::~SynapticonSystemInterface() {
  // A flag to ecat_error_check_ thread
  in_normal_op_mode_ = false;

  if (somanet_control_thread_ && somanet_control_thread_->joinable()) {
    somanet_control_thread_->join();
  }

  // Close the ethercat connection
  ec_close();
}

OSAL_THREAD_FUNC SynapticonSystemInterface::ecatCheck(void * /*ptr*/) {
  int slave;
  uint8 currentgroup = 0;

  while (1) {
    if (in_normal_op_mode_ &&
        ((wkc_ < expected_wkc_) || ec_group[currentgroup].docheckstate)) {
      if (needlf_) {
        needlf_ = false;
        printf("\n");
      }
      // one or more slaves are not responding
      ec_group[currentgroup].docheckstate = false;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++) {
        if (ec_slave[slave].name != EXPECTED_SLAVE_NAME)
        {
          continue;
        }
        if ((ec_slave[slave].group == currentgroup) &&
            (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[currentgroup].docheckstate = true;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n",
                   slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n",
                   slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if (ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = false;
              printf("MESSAGE : slave %d reconfigured\n", slave);
            }
          } else if (!ec_slave[slave].islost) {
            // re-check state
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
              ec_slave[slave].islost = true;
              printf("ERROR : slave %d lost\n", slave);
            }
          }
        }
        if (ec_slave[slave].islost) {
          if (ec_slave[slave].state == EC_STATE_NONE) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = false;
              printf("MESSAGE : slave %d recovered\n", slave);
            }
          } else {
            ec_slave[slave].islost = false;
            printf("MESSAGE : slave %d found\n", slave);
          }
        }
      }
      if (!ec_group[currentgroup].docheckstate)
        printf("OK : all slaves resumed OPERATIONAL.\n");
    }
    osal_usleep(10000);
  }
}

void SynapticonSystemInterface::somanetCyclicLoop(
    std::atomic<bool> &in_normal_op_mode) {
  std::vector<bool> first_iteration(num_joints_ , true);

  while (rclcpp::ok() && !eStopEngaged()) {

    {
      std::lock_guard<std::mutex> lock(hw_state_mtx_);
      ec_send_processdata();
      wkc_ = ec_receive_processdata(EC_TIMEOUTRET);

      // This is for COMPENSATE_FOR_ADDED_LOAD mode
      bool need_more_spring_adjust = false;
      if (initial_inertial_actuator_position_) {
        double current_position_rad = input_ticks_to_output_shaft_rad(in_somanet_[INERTIAL_ACTUATOR_IDX]->PositionValue, mechanical_reductions_.at(INERTIAL_ACTUATOR_IDX).load(), encoder_resolutions_[INERTIAL_ACTUATOR_IDX].load());
        need_more_spring_adjust = std::abs(current_position_rad - initial_inertial_actuator_position_.value()) < DYNAMIC_COMP_MOTION_THRESHOLD;
      }

      if (wkc_ >= expected_wkc_) {
        for (size_t joint_idx = 0; joint_idx < num_joints_; ++joint_idx) {
          if (first_iteration.at(joint_idx)) {
            // Default to PROFILE_TORQUE_MODE
            out_somanet_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
            // small offset to account for friction
            if (in_somanet_[joint_idx]->VelocityValue > 0) {
              out_somanet_[joint_idx]->TorqueOffset = TORQUE_FRICTION_OFFSET.at(joint_idx);
            } else {
              out_somanet_[joint_idx]->TorqueOffset = -TORQUE_FRICTION_OFFSET.at(joint_idx);
            }
            out_somanet_[joint_idx]->TargetTorque = 0;
            first_iteration.at(joint_idx) = false;
          }

          // Fault reset: Fault -> Switch on disabled, if the drive is in fault
          // state
          if ((in_somanet_[joint_idx]->Statusword & 0b0000000001001111) ==
              0b0000000000001000) {
            out_somanet_[joint_idx]->Controlword = 0b10000000;
            RCLCPP_ERROR_STREAM(getLogger(), "Ethercat state fault detected on joint " << joint_idx
                              << ", TorqueDemand: " << in_somanet_[joint_idx]->TorqueDemand
                              << ", VelocityDemandValue: " << in_somanet_[joint_idx]->VelocityDemandValue);
          }

          // Shutdown: Switch on disabled -> Ready to switch on
          else if ((in_somanet_[joint_idx]->Statusword &
                    0b0000000001001111) == 0b0000000001000000)
          {
            // If the QUICK_STOP controller is on, don't leave this state
            if ((control_level_[joint_idx] != control_level_t::QUICK_STOP) &&
               (control_level_[joint_idx] != control_level_t::UNDEFINED))
            {
              out_somanet_[joint_idx]->Controlword = 0b00000110;
            }
          }
          // Switch on: Ready to switch on -> Switched on
          else if ((in_somanet_[joint_idx]->Statusword &
                    0b0000000001101111) == 0b0000000000100001)
            out_somanet_[joint_idx]->Controlword = 0b00000111;

          // Enable operation: Switched on -> Operation enabled
          else if ((in_somanet_[joint_idx]->Statusword &
                    0b0000000001101111) == 0b0000000000100011)
            out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;

          // Normal operation
          else if ((in_somanet_[joint_idx]->Statusword &
                    0b0000000001101111) == 0b0000000000100111) {
            in_normal_op_mode = true;

            int32_t spring_pot_position = read_sdo_value(SPRING_ADJUST_IDX + 1, 0x2402, 0x00);

            if (control_level_[joint_idx] == control_level_t::HAND_GUIDED) {
              // Wrist pitch and wrist roll velocity are controlled by dials
              // The other joints are in a zero-torque mode (plus friction offset)
              if (joint_idx == WRIST_PITCH_IDX) {
                int32_t wrist_pitch_dial_value = read_sdo_value(WRIST_ROLL_IDX + 1, 0x2403, 0x00);
                wrist_pitch_dial_value = std::clamp(wrist_pitch_dial_value, WRIST_DIAL_MIN, WRIST_DIAL_MAX);
                // Scale the value from [-1,1]
                double normalized_dial = (wrist_pitch_dial_value - ANALOG_INPUT_MIDPOINT) / (0.5 * (WRIST_DIAL_MAX - WRIST_DIAL_MIN));
                // Deadband
                if (std::abs(normalized_dial) < WRIST_PITCH_DEADBAND) {
                  normalized_dial = 0;
                }
                double velocity = normalized_dial * MYSTERY_VELOCITY_MULTIPLIER * mechanical_reductions_.at(joint_idx).load() * MAX_WRIST_PITCH_VELOCITY;

                out_somanet_[joint_idx]->TargetVelocity = velocity;
                out_somanet_[joint_idx]->OpMode = CYCLIC_VELOCITY_MODE;
                out_somanet_[joint_idx]->VelocityOffset = 0;
                out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
              } else if (joint_idx == WRIST_ROLL_IDX) {
                int32_t wrist_roll_dial_value = read_sdo_value(WRIST_ROLL_IDX + 1, 0x2404, 0x00);
                wrist_roll_dial_value = std::clamp(wrist_roll_dial_value, WRIST_DIAL_MIN, WRIST_DIAL_MAX);
                // Scale the value from [-1,1]
                double normalized_dial = (wrist_roll_dial_value - ANALOG_INPUT_MIDPOINT) / (0.5 * (WRIST_DIAL_MAX - WRIST_DIAL_MIN));
                // Deadband
                if (std::abs(normalized_dial) < WRIST_ROLL_DEADBAND) {
                  normalized_dial = 0;
                }
                double velocity = normalized_dial * MYSTERY_VELOCITY_MULTIPLIER * mechanical_reductions_.at(joint_idx).load() * MAX_WRIST_ROLL_VELOCITY;

                out_somanet_[joint_idx]->TargetVelocity = velocity;
                out_somanet_[joint_idx]->OpMode = CYCLIC_VELOCITY_MODE;
                out_somanet_[joint_idx]->VelocityOffset = 0;
                out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
              } else {
                if (!std::isnan(threadsafe_commands_efforts_[joint_idx])) {
                  out_somanet_[joint_idx]->TargetTorque =
                      threadsafe_commands_efforts_[joint_idx];
                  out_somanet_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
                  // small offset to account for friction
                  if (in_somanet_[joint_idx]->VelocityValue > 0) {
                    out_somanet_[joint_idx]->TorqueOffset = TORQUE_FRICTION_OFFSET.at(joint_idx);
                  } else {
                    out_somanet_[joint_idx]->TorqueOffset = -TORQUE_FRICTION_OFFSET.at(joint_idx);
                  }
                  out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
                }
              }
            } else if (control_level_[joint_idx] == control_level_t::VELOCITY) {
              if (!std::isnan(threadsafe_commands_velocities_[joint_idx])) {
                out_somanet_[joint_idx]->TargetVelocity =
                    threadsafe_commands_velocities_[joint_idx];
                out_somanet_[joint_idx]->OpMode = CYCLIC_VELOCITY_MODE;
                out_somanet_[joint_idx]->VelocityOffset = 0;
                out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
              }
            } else if (control_level_[joint_idx] == control_level_t::POSITION) {
              if (!std::isnan(threadsafe_commands_positions_[joint_idx])) {
                out_somanet_[joint_idx]->TargetPosition = threadsafe_commands_positions_[joint_idx];
                out_somanet_[joint_idx]->OpMode = CYCLIC_POSITION_MODE;
                out_somanet_[joint_idx]->VelocityOffset = 0;
                out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
              }
            } else if (control_level_[joint_idx] == control_level_t::QUICK_STOP)
            {
              // Turn the brake on
              out_somanet_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
              out_somanet_[joint_idx]->TorqueOffset = 0;
              out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_ON;
            }  else if (control_level_[joint_idx] == control_level_t::SPRING_ADJUST)
            {
              // Spring adjust joint: proportional control based on analog input 2 potentiometer
              if (joint_idx == SPRING_ADJUST_IDX) {
                // Ensure a valid command
                if (std::isnan(threadsafe_commands_spring_adjust_[joint_idx])) {
                  out_somanet_[joint_idx]->TargetTorque = 0;
                  out_somanet_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
                  out_somanet_[joint_idx]->TorqueOffset = 0;
                  // This is safe since the joint is non-backdrivable
                  out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
                  continue;
                }

                // Create a local boolean variable since atomics can't be passed by reference
                bool allow_mode_change = allow_mode_change_.load();
                double actuator_torque = spring_adjust_torque_pd(
                  threadsafe_commands_spring_adjust_[joint_idx],
                  spring_pot_position,
                  spring_adjust_state_,
                  allow_mode_change  // Pass the local variable instead of the atomic member
                );
                // Update the atomic member with the new value
                allow_mode_change_.store(allow_mode_change);

                out_somanet_[joint_idx]->TargetTorque = actuator_torque;
                out_somanet_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
                out_somanet_[joint_idx]->TorqueOffset = 0;
                out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
              }
              else {
                RCLCPP_ERROR(getLogger(), "Should never get here since the other joints are in QUICK_STOP mode");
              }
            } else if (control_level_[joint_idx] == control_level_t::COMPENSATE_FOR_REMOVED_LOAD)
            {
              // Spring adjust joint: proportional control based on analog input 2 potentiometer
              if (joint_idx == SPRING_ADJUST_IDX) {

                // Create a local boolean variable since atomics can't be passed by reference
                bool allow_mode_change = allow_mode_change_.load();
                double actuator_torque = spring_adjust_torque_pd(
                  SPRING_POSITION_WITHOUT_PAYLOAD,
                  spring_pot_position,
                  spring_adjust_state_,
                  allow_mode_change  // Pass the local variable instead of the atomic member
                );
                // Update the atomic member with the new value
                allow_mode_change_.store(allow_mode_change);

                out_somanet_[joint_idx]->TargetTorque = actuator_torque;
                out_somanet_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
                out_somanet_[joint_idx]->TorqueOffset = 0;
                out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
              }
              else {
                RCLCPP_ERROR(getLogger(), "Should never get here since the other joints are in QUICK_STOP mode");
              }
            } else if (control_level_[joint_idx] == control_level_t::COMPENSATE_FOR_ADDED_LOAD)
            {
              // All actuators except those in the elevation link have been put in brake mode
              // The trident brake should be released
              // TODO: put all actuators except those in the elevation link in impedance mode so they can allow arcing motion
              // Record the starting position of the inertial actuator
              // Begin moving the spring adjust joint (velocity mode)
              // Monitor for the inertial actuator position to change more than X rad
              // Then stop the spring adjust motion
              // TODO: if spring adjust hits a position limit, stop and go to QUICK_STOP mode

              if (joint_idx == SPRING_ADJUST_IDX) {
                if (need_more_spring_adjust)
                {
                  allow_mode_change_ = false;
                  out_somanet_[joint_idx]->TargetTorque = SPRING_ADJUST_MAX_TORQUE;
                  out_somanet_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
                  out_somanet_[joint_idx]->TorqueOffset = 0;
                  out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
                }
                // else, stop the spring adjust motion
                else
                {
                  allow_mode_change_ = true;
                  out_somanet_[joint_idx]->OpMode = CYCLIC_VELOCITY_MODE;
                  out_somanet_[joint_idx]->TargetVelocity = 0;
                  out_somanet_[joint_idx]->VelocityOffset = 0;
                  out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
                }
              }
              // Inertial actuator joint should be free to move so we can detect when motion is complete
              else if (joint_idx == INERTIAL_ACTUATOR_IDX) {
                if (need_more_spring_adjust) {
                  out_somanet_[joint_idx]->TargetTorque = 0;
                  out_somanet_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
                  out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
                }
                else {
                  out_somanet_[joint_idx]->OpMode = CYCLIC_VELOCITY_MODE;
                  out_somanet_[joint_idx]->TargetVelocity = 0;
                  out_somanet_[joint_idx]->VelocityOffset = 0;
                  out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
                }
              }
              else {
                RCLCPP_ERROR(getLogger(), "Should never get here since the other joints are in QUICK_STOP mode");
              }
            } else if (control_level_[joint_idx] == control_level_t::UNDEFINED) {
              out_somanet_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
              // small offset to account for friction
              if (in_somanet_[joint_idx]->VelocityValue > 0) {
                out_somanet_[joint_idx]->TorqueOffset = TORQUE_FRICTION_OFFSET.at(joint_idx);
              } else {
                out_somanet_[joint_idx]->TorqueOffset = -TORQUE_FRICTION_OFFSET.at(joint_idx);
              }
              out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_OFF;
            }
          } // normal operation

          // Error logging
          // See https://doc.synapticon.com/node/system_integration/status_and_controlword.html?Highlight=status%20word
          if (in_somanet_[joint_idx]->Statusword & (1 << 11)) {
            RCLCPP_WARN_STREAM(getLogger(), "Internal limit is active for joint " << joint_idx
                              << ", TorqueDemand: " << in_somanet_[joint_idx]->TorqueDemand
                              << ", VelocityDemandValue: " << in_somanet_[joint_idx]->VelocityDemandValue);
          }
          int32_t int_wrist_pitch_temperature = read_sdo_value(WRIST_PITCH_IDX + 1, 0x2038, 0x01);
          float wrist_pitch_thermistor_temperature;
          std::memcpy(&wrist_pitch_thermistor_temperature, &int_wrist_pitch_temperature, sizeof(float));
          if (wrist_pitch_thermistor_temperature > 50) {
            RCLCPP_WARN_STREAM(getLogger(), "High temperature on wrist pitch thermistor [C]: " << wrist_pitch_thermistor_temperature);
          }
        }

        // printf("Processdata cycle %4d , WKC %d ,", i, wkc);
        // printf(" Statusword: %X ,", in_somanet_1->Statusword);
        // printf(" Op Mode Display: %d ,", in_somanet_1->OpModeDisplay);
        // printf(" ActualPos: %" PRId32 " ,\n", in_somanet_[0]->PositionValue);
        // printf(" DemandPos: %" PRId32 " ,",
        // in_somanet_[0]->PositionDemandInternalValue); printf(" ActualVel:
        // %" PRId32 " ,", in_somanet_[0]->VelocityValue);
        // printf(" DemandVel: %" PRId32 " ,", in_somanet_[0]->VelocityDemandValue);
        // printf("ActualTorque: %" PRId32 " ,", in_somanet_[0]->TorqueValue);
        // printf(" DemandTorque: %" PRId32 " ,", in_somanet_[0]->TorqueDemand);
        // printf("\n");

        // printf(" T:%" PRId64 "\r", ec_DCtime);
        needlf_ = true;
      }
    } // scope of in_somanet_ mutex lock
    osal_usleep(5000);
  }

  // Before exiting, set all motors to safe state with brakes on
  std::lock_guard<std::mutex> lock(hw_state_mtx_);
  for (size_t joint_idx = 0; joint_idx < num_joints_; ++joint_idx) {
    out_somanet_[joint_idx]->OpMode = PROFILE_TORQUE_MODE;
    out_somanet_[joint_idx]->TorqueOffset = 0;
    out_somanet_[joint_idx]->TargetTorque = 0;
    out_somanet_[joint_idx]->Controlword = NORMAL_OPERATION_BRAKES_ON;
  }
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  // Signal shutdown to somanet control thread
  in_normal_op_mode = false;
  ec_close();
  // Exit the thread
  return;
}

bool SynapticonSystemInterface::eStopEngaged() {
    // First ensure process data communication is working
    ec_send_processdata();
    int wkc = ec_receive_processdata(EC_TIMEOUTRET);

    if (wkc < expected_wkc_) {
        RCLCPP_ERROR(getLogger(), "Process data communication failed");
        return true; // Force e-stop on communication failure
    }

    uint16_t slave_number = 1;
    uint16_t index = 0x6621;
    uint8_t subindex = 0x01;
    bool operate_all_subindices = FALSE;
    bool value_holder;
    int object_size = sizeof(value_holder);

    int result = ec_SDOread(slave_number, index, subindex, operate_all_subindices, &object_size, &value_holder, EC_TIMEOUTRXM);

    if (result <= 0) {
        RCLCPP_ERROR(getLogger(), "Failed to read emergency stop status from slave %d", slave_number);
        return true; // Force e-stop on read failure
    }
    return value_holder;
}

} // namespace synapticon_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(synapticon_ros2_control::SynapticonSystemInterface,
                       hardware_interface::SystemInterface)
