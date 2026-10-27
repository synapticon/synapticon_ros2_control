// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief The SOMANET slave plugin of the EtherCAT driver.

#include "synapticon_ethercat_icube/synapticon_drive.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "synapticon_hardware/ethercat/coe_objects.hpp"

namespace synapticon_ethercat_icube
{

bool SynapticonDrive::targetPositionPassthrough() const
{
  const bool impedance = mode_of_operation_display_ == synapticon_hardware::ethercat::mode::IMPEDANCE;
  return impedance || EcCiA402Drive::targetPositionPassthrough();
}

}  // namespace synapticon_ethercat_icube

PLUGINLIB_EXPORT_CLASS(synapticon_ethercat_icube::SynapticonDrive, ethercat_interface::EcSlave)
