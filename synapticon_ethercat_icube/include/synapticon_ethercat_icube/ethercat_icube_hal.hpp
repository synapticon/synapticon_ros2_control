// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief SynapticonHalI implementation for an EtherCAT bus, built on ethercat_driver_ros2.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "synapticon_hardware/hal_config.hpp"
#include "synapticon_hardware/ethercat_health.hpp"
#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/cia402/state_machine.hpp"
#include "synapticon_hardware/ethercat/bus_config.hpp"
#include "synapticon_hardware/ethercat/ethercat_pdo_hal.hpp"
#include "synapticon_hardware/ethercat/joint_config.hpp"

namespace ethercat_driver
{
class EthercatBusManager;
}  // namespace ethercat_driver

namespace synapticon_ethercat_icube
{

using synapticon_hardware::DriveAction;
using synapticon_hardware::DriveCommand;
using synapticon_hardware::DriveState;
using synapticon_hardware::EthercatBusHealth;
using synapticon_hardware::EthercatSlaveHealth;
using synapticon_hardware::HalConfig;
using synapticon_hardware::LinkHealth;

/// @brief The bus settings of the EtherCAT ICube HAL: the shared ones, plus the IgH master to use.
struct EthercatConfig : synapticon_hardware::ethercat::BusConfig
{
  unsigned int master_id{ 0 };  ///< Which IgH master runs the bus, 0 for the first.
};

/// @brief Per-joint EtherCAT configuration: the drive on the bus, and the slave plugin the EtherCAT driver loads.
struct EthercatJointConfig : synapticon_hardware::ethercat::JointConfig
{
  /// Slave plugin to load.
  std::string plugin{ "synapticon_ethercat_icube/SynapticonDrive" };
};

/// @brief The EtherCAT HAL on the IgH master, through ICube's ethercat_driver_ros2.
///
/// Runs ethercat_driver::EthercatBusManager with one SynapticonDrive slave plugin per joint, laid out by the joint's
/// slave_config. start() activates the bus and exchanges frames until every drive streams a valid statusword, within
/// readiness_timeout_s. stop() switches the drives off and ends the exchange: the master stays active until
/// disconnect(), and a later start() resumes the exchange and waits for the drives to return to OP. The PDO buffers
/// and what works on them come from EthercatPdoHal, the CiA 402 walks, the drive actions and the setpoint forwarding
/// from Cia402Hal. This class provides the bus.
class EthercatIcubeHal : public synapticon_hardware::ethercat::EthercatPdoHal
{
public:
  EthercatIcubeHal() noexcept;
  ~EthercatIcubeHal() override;

  /// Typed configuration, for direct use. Call once before connect(). The HalConfig overload ends here.
  bool setup(EthercatConfig config, std::vector<EthercatJointConfig> joints);

  /// Configuration from the description's parameters. The hardware parameter `robot_config` names a robot definition
  /// YAML whose `bus` and `joints` fill in what the description leaves out. The hardware parameters
  /// `default_slave_config` and `mode_of_operation` apply to joints without their own.
  bool setup(const HalConfig& config) override;

  bool connect() override;
  bool start() override;
  bool read(std::vector<DriveState>& states) override;
  /// Switch the drives off with disable() first, then end the exchange. The master stays active until disconnect().
  bool stop() override;
  bool disconnect() override;
  bool is_connected() const override;
  /// The joints of the last successful setup(), also before connect().
  std::size_t joint_count() const override;

  /// Each joint's slave and the bus as the master sees them. Without a bus, every entry reads offline.
  void link_health(LinkHealth& health) const override;

protected:
  /// Configure the slaves through the master, ready for SDO reads but not active. Virtual so a test can stand in for
  /// the master at bring-up.
  virtual bool configure_bus();
  /// One SDO upload through the master, only while the bus is configured and not active. A test can answer for the
  /// drive instead.
  int upload(uint16_t position, uint16_t index, uint8_t sub_index, std::vector<uint8_t>& bytes, std::size_t& size,
             uint32_t& abort_code) const override;
  /// Bring the configured slaves to OP and start the cyclic exchange. Virtual so a test can stand in for the master.
  virtual bool activate_bus();
  /// Whether the master reports a complete working counter and every joint's drive online and in OP. Virtual so a test
  /// can stand in for the master.
  virtual bool drives_operational() const;
  bool exchange_read() override;
  bool exchange_write() override;
  /// Stop the cyclic exchange. Virtual for the same reason as activate_bus().
  virtual void deactivate_bus();

private:
  /// Refuse a setup while the HAL is brought up, otherwise forget the configuration of the setup before. False, logged,
  /// when refused.
  bool begin_setup();
  /// The joints' drives, without the ICube slave plugin.
  std::vector<synapticon_hardware::ethercat::JointConfig> drives() const;
  /// End the exchange without switching the drives off first. The master stays active until disconnect().
  void end_exchange();
  /// Exchange process data until drives_operational() holds for a run of cycles, at most readiness_timeout_s. False,
  /// logged, otherwise.
  bool wait_for_op();

  // Fixed by setup().
  EthercatConfig config_{};
  std::vector<EthercatJointConfig> joints_{};
  bool set_up_{ false };

  std::unique_ptr<ethercat_driver::EthercatBusManager> bus_;
  bool bus_configured_{ false };
  bool bus_active_{ false };  ///< the master was activated on this connection
};

}  // namespace synapticon_ethercat_icube
