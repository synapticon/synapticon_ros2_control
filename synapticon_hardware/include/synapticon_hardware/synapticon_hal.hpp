// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief SynapticonHalI: what the hardware interface calls to reach the drives.

#include <cstddef>
#include <vector>

#include "synapticon_hardware/drive_data.hpp"
#include "synapticon_hardware/hal_config.hpp"
#include "synapticon_hardware/link_health.hpp"

namespace synapticon_hardware
{

/// @brief One way of reaching the drives, for example an EtherCAT master or a simulation.
///
/// The hardware interface owns one HAL and climbs it like a ladder, one rung per call, and back down the same way:
///
/// @code
///   setup()
///   connect()    ...   disconnect()      the link
///   start()      ...   stop()            the cyclic exchange
///   enable()     ...   disable()         the power of the drives
///   read() write()                       every control cycle, while the exchange runs
/// @endcode
///
/// Each rung can be climbed again after it was left. read() and write() run in the control loop, everything else
/// may take seconds. The caller serialises all calls on one instance.
///
/// Every HAL follows the same call rules:
///
/// - An up-call on a rung the HAL is on already succeeds and changes nothing: connect() while connected, start()
///   while streaming. enable() while enabled succeeds too, and brings back every drive with energize that a drive
///   action took down.
/// - A down-call for a rung the HAL is not on succeeds and changes nothing: disable() and stop() while not streaming,
///   disconnect() always.
/// - disable() while streaming always takes every drive down, also when the HAL is not enabled, since a drive
///   action can have powered a drive.
/// - An up-call out of order fails: start() before connect(), enable() before start().
/// - stop() while drives may have power switches them off first, as disable() does.
///
/// What a false return means:
///
/// - setup(): no configuration is kept.
/// - connect(): this instance is spent. Make a new HAL instead of calling connect() again.
/// - disconnect(): nothing to act on. The link is closed in any case.
/// - read(): no fresh data this cycle. How to react is up to the caller.
/// - disable() that ran out of time: the HAL is no longer enabled, while some drives may still have power.
/// - any other call: the HAL stays on the rung it was on.
class SynapticonHalI
{
public:
  SynapticonHalI() = default;
  virtual ~SynapticonHalI() = default;

  // One instance stands for one link, so it cannot be copied. Without a copy there is no implicit move either.
  SynapticonHalI(const SynapticonHalI&) = delete;
  SynapticonHalI& operator=(const SynapticonHalI&) = delete;

  /// The first call, made once: store and validate the configuration. No drive is contacted yet, and keys that
  /// belong to other HALs pass unchecked.
  virtual bool setup(const HalConfig& config) = 0;

  /// Open the link and prepare every drive for the cyclic exchange, after setup().
  virtual bool connect() = 0;

  /// Close the link and free all the HAL holds. Allowed on every rung, destructors included. connect() may follow.
  virtual bool disconnect() = 0;

  /// Start the cyclic exchange, with the drives still unpowered. From now on read() delivers fresh data.
  /// The drives act on write() after enable(). Some HALs already pass the commands on before that.
  virtual bool start() = 0;

  /// Stop the cyclic exchange, and keep the link. read() and write() then wait for the next start().
  /// Drives that may still have power are switched off first, as by disable().
  virtual bool stop() = 0;

  /// Give the drives power, so they follow their commands. Needs start() first. The HAL decides how long this may
  /// take.
  virtual bool enable() = 0;

  /// Take the power off the drives, and keep the exchange and the link, so enable() can follow right away.
  /// Drives that have no power are left alone.
  virtual bool disable() = 0;

  /// One state per joint, in joint order, into `states`, which gets joint_count() entries. Runs in the control loop:
  /// it never blocks, and allocates nothing once the exchange runs.
  virtual bool read(std::vector<DriveState>& states) = 0;

  /// Send one command per joint, in joint order. Runs in the control loop, under the same rules as read(). A NaN
  /// setpoint keeps the value sent last, an infinite one is an error (see DriveCommand). False when the commands did
  /// not go out, or when a setpoint was infinite.
  virtual bool write(const std::vector<DriveCommand>& commands) = 0;

  /// How many joints the HAL serves, as setup() found them.
  virtual std::size_t joint_count() const = 0;

  /// True from a successful connect() until disconnect().
  virtual bool is_connected() const = 0;

  /// True while the cyclic exchange runs, whether the drives have power or not.
  virtual bool is_streaming() const = 0;

  /// True from a successful enable() until disable(), or until a drive loses its power by itself.
  virtual bool is_enabled() const = 0;

  /// True when all three above are. An override may add conditions, and must keep these three.
  virtual bool is_ready() const
  {
    return is_connected() && is_streaming() && is_enabled();
  }

  /// The health of the link to each drive, and of the bus. It writes into `health`, so a caller that keeps it
  /// allocates nothing per cycle. A HAL without such a link keeps this default: all healthy.
  virtual void link_health(LinkHealth& health) const
  {
    fill_healthy_link(joint_count(), health);
  }
};

}  // namespace synapticon_hardware
