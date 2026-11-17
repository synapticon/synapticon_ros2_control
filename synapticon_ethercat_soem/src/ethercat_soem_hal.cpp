// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief EthercatSoemHal: the SOEM master, its bring-up, the PDO mapping and the cyclic exchange.

#include "synapticon_ethercat_soem/ethercat_soem_hal.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <linux/if_ether.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

extern "C" {
#include "ethercat.h"
}

#include "bad_cycle_count.hpp"
#include "coe_encoding.hpp"
#include "error_list.hpp"
#include "process_data_mapping.hpp"
#include "process_image.hpp"
#include "slave_config.hpp"
#include "synapticon_hardware/ethercat/robot_config.hpp"
#include "synapticon_hardware/hal_config.hpp"

namespace synapticon_ethercat_soem
{

namespace
{
namespace ec = synapticon_hardware::ethercat;

constexpr const char* LOGGER_NAME = "EthercatSoemHal";
constexpr uint16_t SM_RXPDO_ASSIGN = 0x1C12;
constexpr uint16_t SM_TXPDO_ASSIGN = 0x1C13;
/// While the working counter is short, the slave states are read back every this many cycles.
constexpr unsigned STATE_REFRESH_CYCLES = 100;
/// A state request that has not taken is sent again after this time.
constexpr std::chrono::milliseconds STATE_REQUEST_INTERVAL{ 100 };
/// Process data exchanged in SAFE-OP before OP is requested. Covers the start delay of SYNC0 in ecx_dcsync0().
constexpr std::chrono::milliseconds SAFE_OP_EXCHANGE{ 200 };
/// Process data bytes reserved for each slave on the bus, beyond the joints' own, for slaves without a joint.
constexpr std::size_t IO_MAP_RESERVE_PER_SLAVE = 512;
/// Buffer size for reading a startup SDO back: the widest value type, 64 bits.
constexpr std::size_t SDO_READ_BACK_BYTES = 8;
/// Nanoseconds from the Unix epoch to 2000-01-01, the EtherCAT epoch.
constexpr int64_t EPOCH_OFFSET_NS = 946684800LL * 1000000000LL;

rclcpp::Logger logger()
{
  return rclcpp::get_logger(LOGGER_NAME);
}

/// The time of `clock` in nanoseconds.
int64_t clock_ns(clockid_t clock)
{
  timespec now{};
  clock_gettime(clock, &now);
  return static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
}

/// Why a raw socket on `interface` cannot be opened, for the log: SOEM's own error is lost by the time it returns.
std::string raw_socket_problem(const std::string& interface)
{
  if (if_nametoindex(interface.c_str()) == 0)
  {
    return "no network interface of that name exists (if_nametoindex)";
  }
  const int probe = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
  if (probe < 0)
  {
    const int error = errno;
    return std::string("socket(PF_PACKET, SOCK_RAW) failed: ") + std::strerror(error) +
           (error == EPERM ? ", the process lacks the capability CAP_NET_RAW" : "");
  }
  close(probe);
  return "SOEM could not bind to it, is another EtherCAT master using it?";
}

/// Whether the network interface `interface` is up, or nothing when its flags cannot be read.
std::optional<bool> interface_up(const std::string& interface)
{
  const int probe = socket(AF_INET, SOCK_DGRAM, 0);
  if (probe < 0)
  {
    return std::nullopt;
  }
  ifreq request{};
  std::strncpy(request.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  const bool read = ioctl(probe, SIOCGIFFLAGS, &request) == 0;
  close(probe);
  if (!read)
  {
    return std::nullopt;
  }
  return (request.ifr_flags & IFF_UP) != 0;
}

/// The name of an AL state for the logs.
const char* state_name(uint16_t state)
{
  switch (state & 0x0F)
  {
    case EC_STATE_INIT:
      return "INIT";
    case EC_STATE_PRE_OP:
      return "PRE-OP";
    case EC_STATE_BOOT:
      return "BOOT";
    case EC_STATE_SAFE_OP:
      return "SAFE-OP";
    case EC_STATE_OPERATIONAL:
      return "OP";
    default:
      return "NONE";
  }
}
}  // namespace

struct EthercatSoemHal::Bus
{
  Bus()
  {
    context.port = &port;
    context.slavelist = slaves.data();
    context.slavecount = &slave_count;
    context.maxslave = EC_MAXSLAVE;
    context.grouplist = groups.data();
    context.maxgroup = EC_MAXGROUP;
    context.esibuf = esi_buffer.data();
    context.esimap = esi_map.data();
    context.esislave = 0;
    context.elist = &errors;
    context.idxstack = &index_stack;
    context.ecaterror = &error_flag;
    context.DCtime = &dc_time;
    context.SMcommtype = sm_comm_types.data();
    context.PDOassign = pdo_assigns.data();
    context.PDOdesc = pdo_descriptions.data();
    context.eepSM = &eeprom_sm;
    context.eepFMMU = &eeprom_fmmu;
    context.FOEhook = nullptr;
    context.EOEhook = nullptr;
    context.manualstatechange = 1;  // every state request comes from reach_state(), so SAFE-OP follows the DC setup
    context.userdata = this;        // the PDO mapping hook finds the bus through it
  }

  /// The joint whose drive is the SOEM slave `slave`, if any.
  std::optional<std::size_t> joint_of(uint16_t slave) const
  {
    const auto found = std::find(slave_of_joint.begin(), slave_of_joint.end(), slave);
    if (found == slave_of_joint.end())
    {
      return std::nullopt;
    }
    return static_cast<std::size_t>(found - slave_of_joint.begin());
  }

  /// One SDO download, logged when it fails. True when the drive took it.
  bool download(uint16_t slave, uint16_t index, uint8_t sub_index, std::vector<uint8_t> bytes)
  {
    drain_error_list(context);
    const int wkc = ecx_SDOwrite(&context, slave, index, sub_index, FALSE, static_cast<int>(bytes.size()), bytes.data(),
                                 EC_TIMEOUTRXM);
    if (wkc > 0)
    {
      return true;
    }
    const uint32_t abort_code = drain_error_list(context, SdoAddress{ slave, index, sub_index });
    RCLCPP_ERROR(logger(), "SDO download 0x%04X:%u to slave %u failed (abort=0x%08X).", index, sub_index, slave,
                 abort_code);
    return false;
  }

  /// One SDO upload into `bytes`, resized to what the drive sent. Logged when it fails. True when it answered.
  bool upload(uint16_t slave, uint16_t index, uint8_t sub_index, std::vector<uint8_t>& bytes)
  {
    bytes.assign(SDO_READ_BACK_BYTES, 0);
    int size = static_cast<int>(bytes.size());
    drain_error_list(context);
    const int wkc = ecx_SDOread(&context, slave, index, sub_index, FALSE, &size, bytes.data(), EC_TIMEOUTRXM);
    if (wkc > 0)
    {
      bytes.resize(static_cast<std::size_t>(size));
      return true;
    }
    const uint32_t abort_code = drain_error_list(context, SdoAddress{ slave, index, sub_index });
    RCLCPP_ERROR(logger(), "SDO upload 0x%04X:%u from slave %u failed (abort=0x%08X).", index, sub_index, slave,
                 abort_code);
    return false;
  }

  /// Write the PDO mapping of `pdos` and assign them to the sync manager behind `assign_object`.
  bool map_pdos(uint16_t slave, uint16_t assign_object, const std::vector<Pdo>& pdos)
  {
    bool ok = download(slave, assign_object, 0, { 0 });
    for (std::size_t k = 0; k < pdos.size(); ++k)
    {
      const Pdo& pdo = pdos[k];
      ok = download(slave, pdo.index, 0, { 0 }) && ok;
      for (std::size_t i = 0; i < pdo.channels.size(); ++i)
      {
        ok = download(slave, pdo.index, static_cast<uint8_t>(i + 1), pdo_entry_bytes(pdo.channels[i])) && ok;
      }
      ok = download(slave, pdo.index, 0, { static_cast<uint8_t>(pdo.channels.size()) }) && ok;
      ok = download(slave, assign_object, static_cast<uint8_t>(k + 1),
                    { static_cast<uint8_t>(pdo.index), static_cast<uint8_t>(pdo.index >> 8) }) &&
           ok;
    }
    return download(slave, assign_object, 0, { static_cast<uint8_t>(pdos.size()) }) && ok;
  }

  /// Write the host time into the system time of the reference clock, so that the drives' clocks, and with them their
  /// SYNC0, follow the host loop. The datagram goes out in a frame of its own, without waiting for its return.
  void sync_reference_clock()
  {
    release_clock_sync();
    if (!slaves[0].hasdc)
    {
      return;
    }
    // ecx_configdc() set the clocks to the host's real time since 2000-01-01, the EtherCAT epoch. The monotonic
    // clock carries that time on, so a step of the real time does not reach the drives. The register takes the low
    // 32 bits.
    const auto time = static_cast<uint64_t>(clock_ns(CLOCK_MONOTONIC) + realtime_offset_ns - EPOCH_OFFSET_NS);
    uint32 low_bits = htoel(static_cast<uint32>(time));
    const uint8 index = ecx_getindex(&port);
    ecx_setupdatagram(&port, &port.txbuf[index], EC_CMD_FPWR, index, slaves[slaves[0].DCnext].configadr,
                      ECT_REG_DCSYSTIME, sizeof(low_bits), &low_bits);
    ecx_outframe_red(&port, index);
    clock_sync_index = index;
  }

  /// Note how far the real time is ahead of the monotonic clock. Call it when ecx_configdc() sets the clocks.
  void capture_realtime_offset()
  {
    realtime_offset_ns = clock_ns(CLOCK_REALTIME) - clock_ns(CLOCK_MONOTONIC);
  }

  /// Free the buffer of the last reference clock write. A frame that returns later is dropped by SOEM.
  void release_clock_sync()
  {
    if (clock_sync_index)
    {
      ecx_setbufstat(&port, *clock_sync_index, EC_BUF_EMPTY);
      clock_sync_index.reset();
    }
  }

  /// SOEM's hook for the PRE-OP to SAFE-OP transition: the PDO mapping of the drive's slave_config.
  static int write_pdo_mapping(ecx_contextt* context, uint16 slave)
  {
    auto* bus = static_cast<Bus*>(context->userdata);
    const auto joint = bus->joint_of(slave);
    if (!joint)
    {
      return 0;
    }
    const SlaveConfig& config = bus->slave_configs[*joint];
    const bool rx = bus->map_pdos(slave, SM_RXPDO_ASSIGN, config.rpdo);
    const bool tx = bus->map_pdos(slave, SM_TXPDO_ASSIGN, config.tpdo);
    bus->mapping_failed = bus->mapping_failed || !rx || !tx;
    return 1;
  }

  // The SOEM context and the buffers it works in.
  ecx_portt port{};
  std::vector<ec_slavet> slaves = std::vector<ec_slavet>(EC_MAXSLAVE);
  int slave_count{ 0 };
  std::vector<ec_groupt> groups = std::vector<ec_groupt>(EC_MAXGROUP);
  std::vector<uint8> esi_buffer = std::vector<uint8>(EC_MAXEEPBUF);
  std::vector<uint32> esi_map = std::vector<uint32>(EC_MAXEEPBITMAP);
  ec_eringt errors{};
  ec_idxstackT index_stack{};
  boolean error_flag{ FALSE };
  int64 dc_time{ 0 };
  std::vector<ec_SMcommtypet> sm_comm_types = std::vector<ec_SMcommtypet>(EC_MAX_MAPT);
  std::vector<ec_PDOassignt> pdo_assigns = std::vector<ec_PDOassignt>(EC_MAX_MAPT);
  std::vector<ec_PDOdesct> pdo_descriptions = std::vector<ec_PDOdesct>(EC_MAX_MAPT);
  ec_eepromSMt eeprom_sm{};
  ec_eepromFMMUt eeprom_fmmu{};
  ecx_contextt context{};
  bool open{ false };

  // Per joint: its slave_config, its SOEM slave number and its process image.
  std::vector<SlaveConfig> slave_configs;
  std::vector<uint16_t> slave_of_joint;
  std::vector<ProcessImage> images;
  bool mapping_failed{ false };

  // The process data.
  std::vector<uint8_t> io_map;
  int expected_wkc{ 0 };
  int wkc{ EC_NOFRAME };
  unsigned cycles{ 0 };
  BadCycleCount bad_cycles;               ///< the frame in flight, and cycles in a row without a complete exchange
  bool states_unknown{ false };           ///< no frame came back since the slave states were last read
  std::optional<uint8> clock_sync_index;  ///< the frame buffer of the last reference clock write
  int64_t realtime_offset_ns{ 0 };        ///< real time minus monotonic time when the clocks were set up
};

EthercatSoemHal::EthercatSoemHal() : EthercatPdoHal(LOGGER_NAME)
{
}

EthercatSoemHal::~EthercatSoemHal()
{
  disconnect();
}

// --- Setup ---------------------------------------------------------------------------------------

bool EthercatSoemHal::begin_setup()
{
  if (bus_)
  {
    RCLCPP_ERROR(logger(), "setup() refused: the HAL is brought up, disconnect() first.");
    return false;
  }
  set_up_ = false;
  config_ = SoemConfig{};
  joints_.clear();
  return true;
}

bool EthercatSoemHal::setup(SoemConfig config, std::vector<JointConfig> joints)
{
  if (!begin_setup())
  {
    return false;
  }
  if (joints.empty() || config.network_interface.empty())
  {
    RCLCPP_ERROR(logger(), "setup(): %s",
                 joints.empty() ? "the configuration has no joints." : "the 'network_interface' parameter is missing.");
    return false;
  }
  if (const auto shared = ec::shared_drive_error(joints))
  {
    RCLCPP_ERROR(logger(), "setup(): %s", shared->c_str());
    return false;
  }
  config_ = std::move(config);
  joints_ = std::move(joints);
  set_up_ = true;
  return true;
}

namespace
{
/// The whole number in `text`, when it is at least 1. Throws std::invalid_argument otherwise.
int positive_count(const std::string& text)
{
  std::size_t used = 0;
  const int number = std::stoi(text, &used);
  if (used != text.size() || number < 1)
  {
    throw std::invalid_argument(text + " is not a whole number of at least 1");
  }
  return number;
}

/// The bus settings of the SOEM HAL from the hardware parameters. Logs and returns nothing for a rejected value.
std::optional<SoemConfig> soem_bus_from(const synapticon_hardware::ParameterMap& hardware)
{
  try
  {
    SoemConfig bus;
    static_cast<ec::BusConfig&>(bus) = ec::bus_from(hardware);
    bus.network_interface = synapticon_hardware::param_or(hardware, "network_interface");
    if (const auto timeout = hardware.find("state_timeout_s"); timeout != hardware.end())
    {
      bus.state_timeout_s = ec::positive(timeout->second);
    }
    if (const auto bad_cycles = hardware.find("max_bad_cycles"); bad_cycles != hardware.end())
    {
      bus.max_bad_cycles = positive_count(bad_cycles->second);
    }
    return bus;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger(), "setup(): hardware parameter not usable: %s", e.what());
    return std::nullopt;
  }
}

/// The drive of every joint. Logs the first joint whose parameters are rejected and returns nothing then.
std::optional<std::vector<JointConfig>> drives_from(const std::vector<synapticon_hardware::JointSpec>& specs)
{
  std::vector<JointConfig> drives;
  drives.reserve(specs.size());
  for (const auto& spec : specs)
  {
    try
    {
      drives.push_back(ec::joint_from(spec.name, spec.parameters));
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger(), "setup(): joint '%s' not usable: %s", spec.name.c_str(), e.what());
      return std::nullopt;
    }
  }
  return drives;
}
}  // namespace

bool EthercatSoemHal::setup(const HalConfig& description)
{
  if (!begin_setup())
  {
    return false;
  }
  HalConfig config = description;  // a robot definition file may fill in joints and defaults
  if (std::string error; !ec::apply_robot_config(config, error))
  {
    RCLCPP_ERROR(logger(), "setup(): %s", error.c_str());
    return false;
  }
  auto bus = soem_bus_from(config.hardware_parameters);
  auto drives = bus ? drives_from(config.joints) : std::nullopt;
  return drives && setup(std::move(*bus), std::move(*drives));
}

// --- Lifecycle -----------------------------------------------------------------------------------

bool EthercatSoemHal::connect()
{
  if (bus_)
  {
    return true;  // connected already
  }
  if (!set_up_)
  {
    RCLCPP_ERROR(logger(), "connect(): no configuration, setup() has to come first.");
    return false;
  }
  bus_ = std::make_unique<Bus>();
  for (const auto& joint : joints_)
  {
    std::string error;
    const auto config = load_slave_config(joint.slave_config_file, error);
    if (!config)
    {
      RCLCPP_ERROR(logger(), "Joint '%s': %s", joint.name.c_str(), error.c_str());
      bus_.reset();
      return false;
    }
    bus_->slave_configs.push_back(*config);
    bus_->images.emplace_back(*config, joint.mode_of_operation);
  }
  bus_->slave_of_joint.assign(joints_.size(), 0);
  if (!set_up_drives(joints_, config_.quick_stop_hold, config_.control_frequency))
  {
    bus_.reset();
    return false;
  }

  if (ecx_init(&bus_->context, config_.network_interface.c_str()) <= 0)
  {
    RCLCPP_ERROR(logger(), "Cannot open the network interface '%s' (ecx_init): %s.", config_.network_interface.c_str(),
                 raw_socket_problem(config_.network_interface).c_str());
    bus_.reset();
    return false;
  }
  bus_->open = true;
  if (ecx_config_init(&bus_->context, FALSE) <= 0)
  {
    if (interface_up(config_.network_interface) == false)
    {
      RCLCPP_ERROR(logger(), "No EtherCAT slave answered on '%s': the interface is down (ip link set %s up).",
                   config_.network_interface.c_str(), config_.network_interface.c_str());
    }
    else
    {
      RCLCPP_ERROR(logger(), "No EtherCAT slave answered on '%s'.", config_.network_interface.c_str());
    }
    disconnect();
    return false;
  }
  if (!find_drives())
  {
    disconnect();
    return false;
  }
  if (!reach_state(EC_STATE_PRE_OP, false))
  {
    disconnect();
    return false;
  }
  // The startup SDOs go before the checks, which read some of them back.
  if (!write_startup_sdos())
  {
    disconnect();
    return false;
  }
  for (const uint16_t slave : bus_->slave_of_joint)
  {
    bus_->slaves[slave].PO2SOconfigx = &Bus::write_pdo_mapping;
  }
  if (!check_drives(joints_, { config_.sdo_read_attempts, config_.sdo_retry_wait_s }))
  {
    disconnect();
    return false;
  }
  if (!config_.quick_stop_hold)
  {
    RCLCPP_WARN(logger(), "Quick stop hold is off: a quick-stopped drive in impedance mode is not held at the position "
                          "where the quick stop began.");
  }
  RCLCPP_INFO(logger(), "Bus ready in PRE-OP with %zu joint(s) on '%s'.", joints_.size(),
              config_.network_interface.c_str());
  return true;
}

bool EthercatSoemHal::start()
{
  if (streaming_)
  {
    return true;  // streaming already
  }
  if (!bus_)
  {
    RCLCPP_ERROR(logger(), "start() refused: connect() has not run yet.");
    return false;
  }
  ecx_contextt& context = bus_->context;

  // The PDO mapping is written by the hook in the transition to SAFE-OP, then read back to lay out the process data.
  // SOEM maps every slave on the bus, also those without a joint, so the buffer leaves room for them. Mapping only
  // places the slaves in the buffer, the exchange then uses the whole mapped size, which therefore has to fit. Every
  // start() maps from scratch, also after a stop() or a failed start().
  forget_process_data_mapping(context);
  std::size_t bytes = 0;
  for (const auto& image : bus_->images)
  {
    bytes += image.output_bytes() + image.input_bytes();
  }
  bus_->io_map.assign(bytes + IO_MAP_RESERVE_PER_SLAVE * static_cast<std::size_t>(bus_->slave_count), 0);
  bus_->mapping_failed = false;
  // Each failure from here on goes back to PRE-OP, where the PDO mapping can be written again.
  const int mapped = ecx_config_map_group(&context, bus_->io_map.data(), 0);
  if (bus_->mapping_failed)
  {
    RCLCPP_ERROR(logger(), "start(): writing the PDO mapping of the slave_config failed.");
    back_to_pre_op();
    return false;
  }
  if (mapped < 0 || static_cast<std::size_t>(mapped) > bus_->io_map.size())
  {
    RCLCPP_ERROR(logger(),
                 "start(): the %d slave(s) on the bus map %d bytes of process data, more than the %zu reserved.",
                 bus_->slave_count, mapped, bus_->io_map.size());
    back_to_pre_op();
    return false;
  }
  for (std::size_t j = 0; j < joints_.size(); ++j)
  {
    const ec_slavet& slave = bus_->slaves[bus_->slave_of_joint[j]];
    const ProcessImage& image = bus_->images[j];
    if (slave.Obytes != image.output_bytes() || slave.Ibytes != image.input_bytes())
    {
      RCLCPP_ERROR(logger(),
                   "Joint '%s': the drive maps %u output and %u input bytes, its slave_config %zu and %zu. Check the "
                   "slave_config against the drive's ESI.",
                   joints_[j].name.c_str(), slave.Obytes, slave.Ibytes, image.output_bytes(), image.input_bytes());
      back_to_pre_op();
      return false;
    }
    if (slave.Istartbit != 0 || slave.Ostartbit != 0)
    {
      RCLCPP_ERROR(logger(),
                   "Joint '%s': the drive's process data starts at bit %u (outputs) and %u (inputs) of a byte. The HAL "
                   "takes it only at the start of a byte.",
                   joints_[j].name.c_str(), slave.Ostartbit, slave.Istartbit);
      back_to_pre_op();
      return false;
    }
  }

  // SYNC0 at the control frequency, for every drive whose slave_config sets assign_activate. The drives check it in
  // the transition to SAFE-OP, which is requested only afterwards.
  bus_->capture_realtime_offset();
  ecx_configdc(&context);
  const auto cycle_ns = static_cast<uint32>(1e9 / config_.control_frequency);
  for (std::size_t j = 0; j < joints_.size(); ++j)
  {
    if (bus_->slave_configs[j].assign_activate != 0)
    {
      ecx_dcsync0(&context, bus_->slave_of_joint[j], TRUE, cycle_ns, 0);
    }
  }
  if (!reach_state(EC_STATE_SAFE_OP, false))
  {
    back_to_pre_op();
    return false;
  }
  bus_->expected_wkc = bus_->groups[0].outputsWKC * 2 + bus_->groups[0].inputsWKC;
  // OP is requested once SYNC0 runs and the drives have valid outputs.
  exchange_for(SAFE_OP_EXCHANGE);
  if (!reach_state(EC_STATE_OPERATIONAL, true))
  {
    back_to_pre_op();
    return false;
  }
  bus_->bad_cycles.start(config_.max_bad_cycles);
  streaming_ = true;
  if (!wait_for_streaming(joints_, config_.control_frequency, config_.readiness_timeout_s))
  {
    leave_op();  // no drive has power yet
    return false;
  }
  RCLCPP_INFO(logger(), "Bus in OP: %zu drive(s) exchange valid data, all unpowered.", joints_.size());
  return true;
}

bool EthercatSoemHal::read(std::vector<DriveState>& states)
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

bool EthercatSoemHal::stop()
{
  if (!streaming_)
  {
    return true;
  }
  // Switch the drives off under control before they leave OP. A drive action may have powered one without enable().
  if (!disable())
  {
    RCLCPP_WARN(logger(), "stop(): not every drive switched off under control. Leaving OP takes their power.");
  }
  leave_op();
  return true;
}

bool EthercatSoemHal::disconnect()
{
  stop();
  if (bus_ && bus_->open)
  {
    bus_->context.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&bus_->context, 0);
    ecx_close(&bus_->context);
  }
  bus_.reset();
  enabled_ = false;
  return true;
}

// --- Status --------------------------------------------------------------------------------------

bool EthercatSoemHal::is_connected() const
{
  return bus_ && bus_->open;
}

std::size_t EthercatSoemHal::joint_count() const
{
  return joints_.size();
}

void EthercatSoemHal::link_health(LinkHealth& health) const
{
  health.drives.resize(joints_.size());
  health.bus = {};
  if (!bus_ || !bus_->open)
  {
    return;
  }
  uint8_t al_states = 0;
  uint32_t responding = 0;
  for (std::size_t j = 0; j < joints_.size(); ++j)
  {
    const auto state = static_cast<uint8_t>(bus_->slaves[bus_->slave_of_joint[j]].state & 0x0F);
    health.drives[j].al_state = state;
    health.drives[j].online = state != EC_STATE_NONE;
    al_states = static_cast<uint8_t>(al_states | state);
    responding += state != EC_STATE_NONE ? 1 : 0;
  }
  const bool complete = streaming_ && bus_->wkc == bus_->expected_wkc;
  health.bus.link_up = bus_->wkc != EC_NOFRAME || !streaming_;
  health.bus.slaves_responding = responding;
  health.bus.al_states = al_states;
  health.bus.domain_working_counter = bus_->wkc > 0 ? static_cast<uint32_t>(bus_->wkc) : 0;
  namespace wc = synapticon_hardware::wc_state;
  health.bus.domain_wc_state = complete ? wc::COMPLETE : (bus_->wkc > 0 ? wc::PARTIAL : wc::NONE);
}

// --- Bus hooks -----------------------------------------------------------------------------------

bool EthercatSoemHal::exchange_read()
{
  if (!bus_->bad_cycles.frame_in_flight())
  {
    return bus_->bad_cycles.usable();  // nothing was sent, so there is nothing to receive or count
  }
  // Half a cycle at most, so that a late or lost frame never delays the next cycle.
  bus_->wkc = ecx_receive_processdata(&bus_->context, static_cast<int>(0.5e6 / config_.control_frequency));
  bus_->release_clock_sync();
  for (std::size_t j = 0; j < joints_.size(); ++j)
  {
    bus_->images[j].read_inputs(bus_->slaves[bus_->slave_of_joint[j]].inputs, inputs_[j]);
  }
  if (bus_->wkc == EC_NOFRAME)
  {
    // No frame came back, so no drive answers. Reading their states would only block.
    for (const uint16_t slave : bus_->slave_of_joint)
    {
      bus_->slaves[slave].state = EC_STATE_NONE;
    }
    bus_->states_unknown = true;
  }
  else if (bus_->states_unknown || (bus_->wkc != bus_->expected_wkc && ++bus_->cycles % STATE_REFRESH_CYCLES == 0))
  {
    // Frames are back after a gap, or a drive left OP or dropped off: read the states back. Only now and then while
    // the working counter stays short, since the read blocks.
    ecx_readstate(&bus_->context);
    bus_->states_unknown = false;
  }
  return count_bad_cycles();
}

bool EthercatSoemHal::exchange_write()
{
  for (std::size_t j = 0; j < joints_.size(); ++j)
  {
    bus_->images[j].write_outputs(outputs_[j], bus_->slaves[bus_->slave_of_joint[j]].outputs);
  }
  bus_->sync_reference_clock();
  if (ecx_send_processdata(&bus_->context) <= 0)
  {
    return false;
  }
  bus_->bad_cycles.sent();
  return true;
}

int EthercatSoemHal::upload(uint16_t position, uint16_t index, uint8_t sub_index, std::vector<uint8_t>& bytes,
                            std::size_t& size, uint32_t& abort_code) const
{
  if (!bus_ || !bus_->open)
  {
    return -1;
  }
  const auto slave = static_cast<uint16_t>(position + 1);
  int length = static_cast<int>(bytes.size());
  drain_error_list(bus_->context);
  const int wkc = ecx_SDOread(&bus_->context, slave, index, sub_index, FALSE, &length, bytes.data(), EC_TIMEOUTRXM);
  if (wkc > 0)
  {
    size = static_cast<std::size_t>(length);
    return 0;
  }
  abort_code = drain_error_list(bus_->context, SdoAddress{ slave, index, sub_index });
  return -1;
}

int EthercatSoemHal::download(uint16_t position, uint16_t index, uint8_t sub_index, const std::vector<uint8_t>& bytes,
                              uint32_t& abort_code) const
{
  if (!bus_ || !bus_->open)
  {
    return -1;
  }
  const auto slave = static_cast<uint16_t>(position + 1);
  std::vector<uint8_t> data(bytes);  // SOEM takes a mutable buffer
  drain_error_list(bus_->context);
  const int wkc = ecx_SDOwrite(&bus_->context, slave, index, sub_index, FALSE, static_cast<int>(data.size()),
                               data.data(), EC_TIMEOUTRXM);
  if (wkc > 0)
  {
    return 0;
  }
  abort_code = drain_error_list(bus_->context, SdoAddress{ slave, index, sub_index });
  return -1;
}

// --- Internal ------------------------------------------------------------------------------------

bool EthercatSoemHal::find_drives()
{
  for (std::size_t j = 0; j < joints_.size(); ++j)
  {
    const JointConfig& joint = joints_[j];
    const SlaveConfig& expected = bus_->slave_configs[j];
    if (joint.alias != 0)
    {
      RCLCPP_ERROR(logger(), "Joint '%s': alias %u given, the SOEM HAL addresses drives by ring position only.",
                   joint.name.c_str(), joint.alias);
      return false;
    }
    const uint16_t slave = static_cast<uint16_t>(joint.position + 1);  // SOEM counts the slaves from 1
    if (slave > bus_->slave_count)
    {
      RCLCPP_ERROR(logger(),
                   "Joint '%s': no slave found at ring position %u (slave_config expects vendor=0x%x, product=0x%x). "
                   "%d slave(s) answered on '%s'.",
                   joint.name.c_str(), joint.position, expected.vendor_id, expected.product_id, bus_->slave_count,
                   config_.network_interface.c_str());
      return false;
    }
    const ec_slavet& found = bus_->slaves[slave];
    if (found.eep_man != expected.vendor_id || found.eep_id != expected.product_id)
    {
      RCLCPP_ERROR(logger(),
                   "Joint '%s': the slave at ring position %u is vendor=0x%x, product=0x%x (%s), but its slave_config "
                   "expects vendor=0x%x, product=0x%x.",
                   joint.name.c_str(), joint.position, found.eep_man, found.eep_id, found.name, expected.vendor_id,
                   expected.product_id);
      return false;
    }
    bus_->slave_of_joint[j] = slave;
  }
  return true;
}

bool EthercatSoemHal::write_startup_sdos()
{
  for (std::size_t j = 0; j < joints_.size(); ++j)
  {
    for (const auto& sdo : bus_->slave_configs[j].sdo)
    {
      if (!write_startup_sdo(j, sdo))
      {
        return false;
      }
    }
  }
  return true;
}

bool EthercatSoemHal::write_startup_sdo(std::size_t joint, const StartupSdo& sdo)
{
  // The value was checked at load. Two attempts, since right after a bus release the drive's mailbox may drop an SDO,
  // or acknowledge it without applying it.
  const uint16_t slave = bus_->slave_of_joint[joint];
  const std::vector<uint8_t> written = sdo_bytes(sdo);
  std::vector<uint8_t> read;
  bool taken = false;
  bool answered = false;
  for (int attempt = 0; attempt < 2; ++attempt)
  {
    taken = bus_->download(slave, sdo.index, sdo.sub_index, written);
    answered = taken && bus_->upload(slave, sdo.index, sdo.sub_index, read);
    if (answered && read == written)
    {
      return true;
    }
  }
  const char* name = joints_[joint].name.c_str();
  const std::string value = coe_value_text(sdo.type, written);
  if (!taken)
  {
    RCLCPP_ERROR(logger(), "Joint '%s': the drive refused the startup SDO 0x%04X:%u = %s of its slave_config.", name,
                 sdo.index, sdo.sub_index, value.c_str());
  }
  else if (!answered)
  {
    RCLCPP_ERROR(logger(), "Joint '%s': the startup SDO 0x%04X:%u = %s of its slave_config cannot be read back.", name,
                 sdo.index, sdo.sub_index, value.c_str());
  }
  else
  {
    RCLCPP_ERROR(logger(),
                 "Joint '%s': the drive keeps 0x%04X:%u at %s after writing the startup SDO %s of its slave_config "
                 "twice.",
                 name, sdo.index, sdo.sub_index, coe_value_text(sdo.type, read).c_str(), value.c_str());
  }
  return false;
}

bool EthercatSoemHal::reach_state(uint16_t state, bool cycling)
{
  ecx_contextt& context = bus_->context;
  const auto period = std::chrono::duration<double>(1.0 / config_.control_frequency);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                             std::chrono::duration<double>(config_.state_timeout_s));
  auto next_request = std::chrono::steady_clock::now();
  ecx_statecheck(&context, 0, state, 0);  // the AL status of the bus, with its error bit, into slavelist[0]
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (std::chrono::steady_clock::now() >= next_request)
    {
      // A drive whose error bit is set follows a state request only with the error acknowledged. ecx_statecheck()
      // overwrites the request with the AL status read back.
      const bool error = (context.slavelist[0].state & EC_STATE_ERROR) != 0;
      context.slavelist[0].state = error ? static_cast<uint16_t>(state | EC_STATE_ACK) : state;
      ecx_writestate(&context, 0);
      next_request += STATE_REQUEST_INTERVAL;
    }
    if (cycling)
    {
      exchange_write();
      exchange_read();
    }
    ecx_statecheck(&context, 0, state, cycling ? 1000 : 50000);
    if ((context.slavelist[0].state & 0x0F) == state)
    {
      ecx_readstate(&context);  // each drive's own state, for the link health
      return true;
    }
    if (cycling)
    {
      std::this_thread::sleep_for(period);
    }
  }
  ecx_readstate(&context);
  for (std::size_t j = 0; j < joints_.size(); ++j)
  {
    const ec_slavet& slave = bus_->slaves[bus_->slave_of_joint[j]];
    RCLCPP_ERROR(logger(), "Joint '%s' did not reach %s within %.1fs: it is in %s%s (AL status 0x%04X, %s).",
                 joints_[j].name.c_str(), state_name(state), config_.state_timeout_s, state_name(slave.state),
                 (slave.state & EC_STATE_ERROR) != 0 ? "+ERROR" : "", slave.ALstatuscode,
                 ec_ALstatuscode2string(slave.ALstatuscode));
  }
  return false;
}

bool EthercatSoemHal::count_bad_cycles()
{
  // Not counted before streaming: in SAFE-OP and on the way to OP the working counter is short by design.
  switch (bus_->bad_cycles.received(bus_->wkc == bus_->expected_wkc))
  {
    case BadCycleCount::Change::LIMIT_REACHED:
      RCLCPP_ERROR(logger(),
                   "No complete process data for %d cycles in a row (working counter %d, expected %d). read() "
                   "fails until it is complete again.",
                   bus_->bad_cycles.limit(), bus_->wkc, bus_->expected_wkc);
      break;
    case BadCycleCount::Change::COMPLETE_AGAIN:
      RCLCPP_INFO(logger(), "Process data complete again, read() delivers fresh data.");
      break;
    case BadCycleCount::Change::NONE:
      break;
  }
  return bus_->bad_cycles.usable();
}

void EthercatSoemHal::exchange_for(std::chrono::milliseconds duration)
{
  const auto period = std::chrono::duration<double>(1.0 / config_.control_frequency);
  const auto end = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < end)
  {
    exchange_write();
    exchange_read();
    std::this_thread::sleep_for(period);
  }
}

void EthercatSoemHal::leave_op()
{
  streaming_ = false;
  bus_->bad_cycles.stop();
  enabled_ = false;  // a drive leaving OP de-energises
  if (back_to_pre_op())
  {
    RCLCPP_INFO(logger(), "Drives back in PRE-OP.");
  }
  else
  {
    RCLCPP_WARN(logger(), "Left OP, but not every drive is back in PRE-OP.");
  }
}

bool EthercatSoemHal::back_to_pre_op()
{
  bus_->release_clock_sync();
  const bool reached = reach_state(EC_STATE_PRE_OP, false);
  for (const uint16_t slave : bus_->slave_of_joint)
  {
    ecx_dcsync0(&bus_->context, slave, FALSE, 0, 0);
  }
  return reached;
}

}  // namespace synapticon_ethercat_soem

PLUGINLIB_EXPORT_CLASS(synapticon_ethercat_soem::EthercatSoemHal, synapticon_hardware::SynapticonHalI)
