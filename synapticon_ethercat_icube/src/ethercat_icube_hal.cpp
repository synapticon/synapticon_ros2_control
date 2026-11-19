// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief EthercatIcubeHal lifecycle: configuring and activating the bus, reading the states, the link health.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "ethercat_driver/ethercat_bus_manager.hpp"

#include "ethercat_icube_common.hpp"
#include "slave_health.hpp"
#include "synapticon_hardware/ethercat/pdo_channels.hpp"
#include "synapticon_ethercat_icube/ethercat_icube_hal.hpp"

namespace synapticon_ethercat_icube
{

namespace
{
/// Cycles in a row that a resumed exchange must look operational before start() trusts it. The master refreshes the
/// slave states only every few cycles, so a shorter run could still show the state from before the stop().
constexpr int OP_CONFIRM_CYCLES = 50;

rclcpp::Logger logger()
{
  return rclcpp::get_logger(LOGGER_NAME);
}

/// True when the slave_config of `joint` sets `auto_state_transitions: false`. The EtherCAT driver otherwise walks the
/// drive through the CiA 402 states by itself, up to Operation Enabled. Logged when not.
bool leaves_power_to_hal(const EthercatJointConfig& joint)
{
  try
  {
    const YAML::Node flag = YAML::LoadFile(joint.slave_config_file)["auto_state_transitions"];
    if (flag && !flag.as<bool>())
    {
      return true;
    }
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(logger(), "Joint '%s': slave_config %s: %s", joint.name.c_str(), joint.slave_config_file.c_str(),
                 e.what());
    return false;
  }
  RCLCPP_ERROR(logger(),
               "Joint '%s': its slave_config %s needs the line 'auto_state_transitions: false', so that the EtherCAT "
               "driver never powers the drive by itself.",
               joint.name.c_str(), joint.slave_config_file.c_str());
  return false;
}

/// The EtherCAT driver's description of the drive of `joint`: the slave plugin with its parameters, and the buffers it
/// exchanges. The plugin finds each channel in the buffers by the slot given in its `command_interface/<channel>` or
/// `state_interface/<channel>` parameter. A failed setup is logged as "drive module <ring position>".
ethercat_driver::ConfiguredEcModule drive_module(const EthercatJointConfig& joint, std::vector<double>& inputs,
                                                 std::vector<double>& outputs)
{
  ethercat_driver::ConfiguredEcModule module{ .parameters = {},
                                              .input_values = &inputs,
                                              .output_values = &outputs,
                                              .component_name = joint.name,
                                              .module_type = "drive",
                                              .module_number = joint.position };
  auto& parameters = module.parameters;
  parameters.emplace("plugin", joint.plugin);
  parameters.emplace("slave_config", joint.slave_config_file);
  parameters.emplace("alias", std::to_string(joint.alias));
  parameters.emplace("position", std::to_string(joint.position));
  parameters.emplace("mode_of_operation", std::to_string(joint.mode_of_operation));
  const auto add_slot = [&parameters](const char* direction) {
    return [&parameters, direction](std::string_view channel, std::size_t slot) {
      parameters.emplace(std::string(direction) + "/" + std::string(channel), std::to_string(slot));
    };
  };
  for_each_output_slot(add_slot("command_interface"));
  for_each_input_slot(add_slot("state_interface"));
  return module;
}
}  // namespace

EthercatIcubeHal::EthercatIcubeHal() noexcept : EthercatPdoHal(LOGGER_NAME)
{
}

EthercatIcubeHal::~EthercatIcubeHal()
{
  disconnect();
}

bool EthercatIcubeHal::connect()
{
  if (bus_configured_)
  {
    return true;  // connected already
  }
  if (!set_up_)
  {
    RCLCPP_ERROR(logger(), "connect() refused: setup() has not run yet.");
    return false;
  }
  if (!std::all_of(joints_.begin(), joints_.end(), leaves_power_to_hal))
  {
    return false;
  }
  bus_ = std::make_unique<ethercat_driver::EthercatBusManager>();
  // Sized here and never resized, because the slave plugins point into these buffers.
  if (!set_up_drives(drives(), config_.quick_stop_hold, config_.control_frequency) || !configure_bus() ||
      !check_drives(drives(), { config_.sdo_read_attempts, config_.sdo_retry_wait_s }))
  {
    bus_.reset();
    return false;
  }
  if (!config_.quick_stop_hold)
  {
    RCLCPP_WARN(logger(), "Quick stop hold is off: a quick-stopped drive in impedance mode is not held at the position "
                          "where the quick stop began.");
  }
  bus_configured_ = true;
  RCLCPP_INFO(logger(), "Bus ready in PRE-OP with %zu joint(s).", joints_.size());
  return true;
}

bool EthercatIcubeHal::start()
{
  if (streaming_)
  {
    return true;  // streaming already
  }
  if (!bus_configured_)
  {
    RCLCPP_ERROR(logger(), "start() refused: connect() has not run yet.");
    return false;
  }
  // The master is activated once per connection. Activating it again would hand out a new process data buffer that
  // the frames never reach, so a later start() resumes the exchange and waits for the drives to return to OP.
  if (!bus_active_)
  {
    if (!activate_bus())
    {
      // The master may be half activated, so the bus is released, and only a new connect() can try again.
      RCLCPP_ERROR(logger(), "The EtherCAT driver could not bring the bus to OP within %.1fs. The bus is released.",
                   config_.readiness_timeout_s);
      disconnect();
      return false;
    }
    bus_active_ = true;
  }
  else if (!wait_for_op())
  {
    return false;
  }
  streaming_ = true;
  if (!wait_for_streaming(drives(), config_.control_frequency, config_.readiness_timeout_s))
  {
    end_exchange();  // no drive has power yet
    return false;
  }
  // Not counted before: on the way to OP the working counter is short by design.
  bad_cycles_.start(config_.max_bad_cycles);
  RCLCPP_INFO(logger(), "Bus in OP: %zu drive(s) exchange valid data, all unpowered.", joints_.size());
  return true;
}

bool EthercatIcubeHal::read(std::vector<DriveState>& states)
{
  if (!streaming_)
  {
    RCLCPP_ERROR(logger(), "read() refused: start() has not run yet.");
    return false;
  }
  if (!exchange_read())
  {
    return false;
  }
  unpack_states(states);
  return true;
}

bool EthercatIcubeHal::stop()
{
  if (!streaming_)
  {
    return true;
  }
  // Switch the drives off under control before the exchange ends. A drive action may have powered one without enable().
  if (!disable())
  {
    RCLCPP_WARN(logger(), "stop(): not every drive switched off under control. Without process data they leave OP, "
                          "which takes their power.");
  }
  end_exchange();
  return true;
}

bool EthercatIcubeHal::disconnect()
{
  stop();
  if (bus_ && bus_active_)
  {
    deactivate_bus();
  }
  bus_active_ = false;
  bus_.reset();
  bus_configured_ = false;
  enabled_ = false;
  return true;
}

bool EthercatIcubeHal::is_connected() const
{
  return bus_configured_;
}

std::size_t EthercatIcubeHal::joint_count() const
{
  return joints_.size();
}

void EthercatIcubeHal::link_health(LinkHealth& health) const
{
  health.drives.assign(joints_.size(), EthercatSlaveHealth{});
  health.bus = EthercatBusHealth{};
  if (!bus_)
  {
    return;
  }
  // The master lists its slaves by bus address, each joint finds its own among them. The bus-wide AL state covers only
  // these: the master never configures another slave, which then stays in PRE-OP.
  const uint8_t al_states = fill_joint_slave_health(bus_->slaveStates(), joints_, health.drives);
  const ec_master_state_t& master = bus_->masterState();
  const ec_domain_state_t& domain = bus_->domainState();
  health.bus = EthercatBusHealth{ master.link_up != 0, master.slaves_responding, al_states, domain.working_counter,
                                  static_cast<uint8_t>(domain.wc_state) };
}

bool EthercatIcubeHal::configure_bus()
{
  std::vector<ethercat_driver::ConfiguredEcModule> drive_modules;
  drive_modules.reserve(joints_.size());
  for (std::size_t j = 0; j < joints_.size(); ++j)
  {
    drive_modules.push_back(drive_module(joints_[j], inputs_[j], outputs_[j]));
  }
  // A plain bus of drives: no transfers between slaves and no FSoE.
  const ethercat_driver::EthercatBusConfig master{ .master_id = config_.master_id,
                                                   .control_frequency = config_.control_frequency,
                                                   .transfer_config = {},
                                                   .fsoe_config = {} };
  if (!bus_->configureModules(master, drive_modules))
  {
    RCLCPP_ERROR(logger(), "The EtherCAT driver could not set up the drive modules.");
    return false;
  }
  // The master stays inactive after this: no process data, so the bring-up checks may read SDOs and wait.
  if (!bus_->configureBus())
  {
    RCLCPP_ERROR(logger(), "The EtherCAT driver could not configure the bus.");
    return false;
  }
  return true;
}

int EthercatIcubeHal::upload(uint16_t position, uint16_t index, uint8_t sub_index, std::vector<uint8_t>& bytes,
                             std::size_t& size, uint32_t& abort_code) const
{
  if (bus_active_)
  {
    RCLCPP_ERROR(logger(), "SDO read of 0x%04X:%u refused: the bus is active.", index, sub_index);
    return -1;
  }
  return master_upload(position, index, sub_index, bytes, size, abort_code);
}

int EthercatIcubeHal::download(uint16_t position, uint16_t index, uint8_t sub_index, const std::vector<uint8_t>& bytes,
                               uint32_t& abort_code) const
{
  if (bus_active_)
  {
    RCLCPP_ERROR(logger(), "SDO write of 0x%04X:%u refused: the bus is active.", index, sub_index);
    return -1;
  }
  return master_download(position, index, sub_index, bytes, abort_code);
}

int EthercatIcubeHal::master_upload(uint16_t position, uint16_t index, uint8_t sub_index, std::vector<uint8_t>& bytes,
                                    std::size_t& size, uint32_t& abort_code) const
{
  if (!bus_)
  {
    return -1;
  }
  return bus_->readSlaveSdo(position, index, sub_index, bytes.data(), bytes.size(), &size, &abort_code);
}

int EthercatIcubeHal::master_download(uint16_t position, uint16_t index, uint8_t sub_index,
                                      const std::vector<uint8_t>& bytes, uint32_t& abort_code) const
{
  if (!bus_)
  {
    return -1;
  }
  return bus_->writeSlaveSdo(position, index, sub_index, bytes.data(), bytes.size(), &abort_code);
}

bool EthercatIcubeHal::activate_bus()
{
  const auto limit = std::chrono::duration<double>(config_.readiness_timeout_s);
  return bus_->activateBus(std::chrono::duration_cast<std::chrono::nanoseconds>(limit));
}

bool EthercatIcubeHal::drives_operational() const
{
  LinkHealth health;
  link_health(health);
  return health.bus.domain_wc_state == synapticon_hardware::wc_state::COMPLETE &&
         std::all_of(health.drives.begin(), health.drives.end(), synapticon_hardware::is_operational);
}

ethercat_driver::EthercatCycleResult EthercatIcubeHal::bus_read()
{
  return bus_->read();
}

ethercat_driver::EthercatCycleResult EthercatIcubeHal::bus_write()
{
  return bus_->write();
}

bool EthercatIcubeHal::process_data_complete() const
{
  return bus_->domainState().wc_state == EC_WC_COMPLETE;
}

bool EthercatIcubeHal::exchange_read()
{
  const bool sent = bad_cycles_.frame_in_flight();
  const auto result = bus_read();
  if (result == ethercat_driver::EthercatCycleResult::kError)
  {
    return false;
  }
  if (!sent)
  {
    return bad_cycles_.usable();  // nothing was sent, so there is nothing to count
  }
  // A skipped receive brings no data back, so it counts as incomplete.
  return count_bad_cycles(result == ethercat_driver::EthercatCycleResult::kCompleted && process_data_complete());
}

bool EthercatIcubeHal::exchange_write()
{
  const auto result = bus_write();
  if (result == ethercat_driver::EthercatCycleResult::kCompleted)
  {
    bad_cycles_.sent();
  }
  return result != ethercat_driver::EthercatCycleResult::kError;
}

bool EthercatIcubeHal::count_bad_cycles(bool complete)
{
  using Change = synapticon_hardware::ethercat::BadCycleCount::Change;
  switch (bad_cycles_.received(complete))
  {
    case Change::LIMIT_REACHED:
      RCLCPP_ERROR(logger(),
                   "No complete process data for %d cycles in a row. read() fails until it is complete again.",
                   bad_cycles_.limit());
      break;
    case Change::COMPLETE_AGAIN:
      RCLCPP_INFO(logger(), "Process data complete again, read() delivers fresh data.");
      break;
    case Change::NONE:
      break;
  }
  return bad_cycles_.usable();
}

void EthercatIcubeHal::deactivate_bus()
{
  bus_->deactivateBus();
}

std::vector<JointConfig> EthercatIcubeHal::drives() const
{
  return { joints_.begin(), joints_.end() };
}

void EthercatIcubeHal::end_exchange()
{
  streaming_ = false;
  bad_cycles_.stop();
  enabled_ = false;  // without process data the drives leave OP, which de-energises them
  RCLCPP_INFO(logger(), "Exchange stopped, the master stays active until disconnect().");
}

bool EthercatIcubeHal::wait_for_op()
{
  const auto period = std::chrono::duration<double>(1.0 / config_.control_frequency);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                             std::chrono::duration<double>(config_.readiness_timeout_s));
  int confirmed = 0;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (!exchange_read() || !exchange_write())
    {
      RCLCPP_ERROR(logger(), "start(): the bus exchange failed while waiting for the drives to return to OP.");
      return false;
    }
    confirmed = drives_operational() ? confirmed + 1 : 0;
    if (confirmed >= OP_CONFIRM_CYCLES)
    {
      return true;
    }
    std::this_thread::sleep_for(period);
  }
  RCLCPP_ERROR(logger(), "start(): not every drive returned to OP within %.1fs.", config_.readiness_timeout_s);
  return false;
}

}  // namespace synapticon_ethercat_icube

PLUGINLIB_EXPORT_CLASS(synapticon_ethercat_icube::EthercatIcubeHal, synapticon_hardware::SynapticonHalI)
