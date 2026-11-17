// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

/// @file
/// @brief Reading SOEM's error list.

#include "error_list.hpp"

#include <rclcpp/rclcpp.hpp>

namespace synapticon_ethercat_soem
{

namespace
{
/// Whether `error` is the SDO abort of `transfer`.
bool is_abort_of(const ec_errort& error, const SdoAddress& transfer)
{
  return error.Etype == EC_ERR_TYPE_SDO_ERROR && error.Slave == transfer.slave && error.Index == transfer.index &&
         error.SubIdx == transfer.sub_index;
}

void log_entry(const ec_errort& error)
{
  const auto logger = rclcpp::get_logger("EthercatSoemHal");
  switch (error.Etype)
  {
    case EC_ERR_TYPE_EMERGENCY:
      RCLCPP_WARN(logger, "Slave %u sent an emergency: error code 0x%04X, error register 0x%02X.", error.Slave,
                  error.ErrorCode, error.ErrorReg);
      break;
    case EC_ERR_TYPE_SDO_ERROR:
    case EC_ERR_TYPE_SDOINFO_ERROR:
      RCLCPP_WARN(logger, "Slave %u aborted an SDO transfer of 0x%04X:%u (abort=0x%08X).", error.Slave, error.Index,
                  error.SubIdx, static_cast<uint32_t>(error.AbortCode));
      break;
    default:
      RCLCPP_WARN(logger, "Slave %u: SOEM error of type %d at 0x%04X:%u (code 0x%04X).", error.Slave,
                  static_cast<int>(error.Etype), error.Index, error.SubIdx, error.ErrorCode);
      break;
  }
}
}  // namespace

uint32_t drain_error_list(ecx_contextt& context, const std::optional<SdoAddress>& transfer)
{
  uint32_t abort_code = 0;
  ec_errort error{};
  while (ecx_poperror(&context, &error))
  {
    if (transfer && is_abort_of(error, *transfer))
    {
      abort_code = static_cast<uint32_t>(error.AbortCode);
    }
    else
    {
      log_entry(error);
    }
  }
  return abort_code;
}

}  // namespace synapticon_ethercat_soem
