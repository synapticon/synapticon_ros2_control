// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The slave plugin that the EtherCAT driver creates for each SOMANET drive.

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

namespace synapticon_ethercat_icube
{

/// @brief A generic CiA 402 drive of the EtherCAT driver that also knows the vendor impedance mode.
///
/// It changes one decision of EcCiA402Drive and inherits the rest.
class SynapticonDrive : public ethercat_generic_plugins::EcCiA402Drive
{
protected:
  /// Whether the commanded target position reaches 0x607A. In impedance mode (-6) that object is the rest position
  /// of the spring, so the command must get through as in CSP, where the generic drive alone would send the actual
  /// position.
  bool targetPositionPassthrough() const override;
};

}  // namespace synapticon_ethercat_icube
