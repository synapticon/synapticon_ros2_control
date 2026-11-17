// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief SynapticonHalI for an EtherCAT bus driven by the SOEM master library, in user space.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "synapticon_hardware/ethercat/bus_config.hpp"
#include "synapticon_hardware/ethercat/ethercat_pdo_hal.hpp"
#include "synapticon_hardware/ethercat/joint_config.hpp"
#include "synapticon_hardware/hal_config.hpp"
#include "synapticon_hardware/link_health.hpp"

namespace synapticon_ethercat_soem
{

using synapticon_hardware::DriveState;
using synapticon_hardware::HalConfig;
using synapticon_hardware::LinkHealth;
using synapticon_hardware::ethercat::JointConfig;

struct StartupSdo;

/// @brief Bus-wide settings for a Synapticon drive segment on a SOEM master.
struct SoemConfig : synapticon_hardware::ethercat::BusConfig
{
  /// The network interface the drives are wired to, for example `eth0`.
  std::string network_interface;
  /// Max time [s] of each wait for every drive to reach a state: PRE-OP in connect() and stop(), SAFE-OP and then OP
  /// in start().
  double state_timeout_s{ 5.0 };
  /// Cycles in a row without a complete exchange (no frame back, or a short working counter) after which read()
  /// fails, until a complete exchange comes back. Fewer are bridged with the last data received. Only a cycle whose
  /// frame was sent counts.
  int max_bad_cycles{ 10 };
};

/// @brief SynapticonHalI over a direct EtherCAT bus, with the SOEM master library and no kernel module.
///
/// connect() opens a raw socket on the network interface, finds the drives, writes each drive's startup SDOs and
/// runs the bring-up checks. start() writes the PDO mapping of each drive's slave_config, maps the process data,
/// starts the distributed clocks and brings the drives to OP. The cyclic exchange runs only in exchange_read() and
/// exchange_write(), without a thread of its own. Each exchange_write() also writes the host time into the reference
/// clock, as the IgH master does, so that the drives' SYNC0 follows the host loop instead of drifting against it. The
/// PDO buffers and what works on them come from EthercatPdoHal, the CiA 402 walks, the drive actions and the setpoint
/// forwarding from Cia402Hal.
///
/// The raw socket needs CAP_NET_RAW. Only one EtherCAT master may use the network interface at a time.
class EthercatSoemHal : public synapticon_hardware::ethercat::EthercatPdoHal
{
public:
  EthercatSoemHal();
  ~EthercatSoemHal() override;

  /// Typed configuration, for direct use. Call once before connect(). The HalConfig overload ends here.
  bool setup(SoemConfig config, std::vector<JointConfig> joints);

  /// Configuration from the description's parameters. The hardware parameter `network_interface` names the network
  /// interface the drives are wired to.
  bool setup(const HalConfig& config) override;

  bool connect() override;
  bool start() override;
  bool read(std::vector<DriveState>& states) override;
  /// Switch the drives off with disable() first, then bring them back to PRE-OP.
  bool stop() override;
  bool disconnect() override;
  bool is_connected() const override;
  /// The joints of the last successful setup(), also before connect().
  std::size_t joint_count() const override;
  void link_health(LinkHealth& health) const override;

protected:
  bool exchange_read() override;
  bool exchange_write() override;
  /// One SDO upload through SOEM's mailbox, while the drives are in PRE-OP.
  int upload(uint16_t position, uint16_t index, uint8_t sub_index, std::vector<uint8_t>& bytes, std::size_t& size,
             uint32_t& abort_code) const override;
  /// One SDO download through SOEM's mailbox, while the drives are in PRE-OP.
  int download(uint16_t position, uint16_t index, uint8_t sub_index, const std::vector<uint8_t>& bytes,
               uint32_t& abort_code) const override;

private:
  /// The SOEM context with its buffers, the process data and each drive's layout in it.
  struct Bus;

  /// Refuse a setup while the HAL is brought up, otherwise forget the configuration of the setup before. False, logged,
  /// when refused.
  bool begin_setup();
  /// Find each joint's drive and check its identity. False, logged, when one is missing or another device.
  bool find_drives();
  /// Write each drive's startup SDOs. False, logged, at the first one that fails.
  bool write_startup_sdos();
  /// Write one startup SDO of `joint` and read it back unless it has verify false, with one more attempt when it is
  /// refused or reads back another value. False, logged, when the second attempt fails as well.
  bool write_startup_sdo(std::size_t joint, const StartupSdo& sdo);
  /// Request `state` of every drive and wait for it, exchanging process data while waiting when `cycling`. The
  /// request is repeated until it takes.
  bool reach_state(uint16_t state, bool cycling);
  /// Count the cycle just received while streaming: one more without a complete exchange, or a complete one. False
  /// once max_bad_cycles incomplete ones came in a row.
  bool count_bad_cycles();
  /// Exchange process data at the control frequency for `duration`.
  void exchange_for(std::chrono::milliseconds duration);
  /// End the exchange without switching the drives off first, and bring them back to PRE-OP.
  void leave_op();
  /// Bring the drives back to PRE-OP and switch their SYNC0 off. False, logged, when a drive did not reach PRE-OP.
  bool back_to_pre_op();

  // Fixed by setup().
  SoemConfig config_{};
  std::vector<JointConfig> joints_{};
  bool set_up_{ false };

  std::unique_ptr<Bus> bus_;
};

}  // namespace synapticon_ethercat_soem
