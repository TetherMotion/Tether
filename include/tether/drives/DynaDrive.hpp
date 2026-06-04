/**
 * @file DynaDrive.hpp
 * @brief DynaDrive (ANYdrive / rsl_drive_sdk) — aggregate header
 *
 * Vendor/Product from SII dump:
 *   Vendor ID:    0x00414E59  (ASCII "ANY")
 *   Product Code: 0x17010001
 *   Name:         "Synchron"
 */

#pragma once

#include <string>

#include "tether/drives/DynaDrive/DynaDrivePDO.hpp"
#include "tether/drives/DynaDrive/Registers/Common.hpp"
#include "tether/drives/DynaDrive/Registers/Controlword.hpp"
#include "tether/drives/DynaDrive/Registers/ModesOfOperation.hpp"
#include "tether/drives/DynaDrive/Registers/Statusword.hpp"
#include "tether/drives/DynaDrive/Registers/Identity.hpp"
#include "tether/drives/DynaDrive/Registers/PDOAssignment.hpp"

namespace EtherCAT {
namespace Drives {

// Re-export canonical identity constants
constexpr uint32_t kDynaDriveVendorId    = Registers::DynaDrive::Identity::VendorID.default_value;
constexpr uint32_t kDynaDriveProductCode = Registers::DynaDrive::Identity::ProductCode.default_value;

// Re-export runtime statusword helper
using DynaDriveStatusword = Registers::DynaDrive::Status::StatuswordDecoder;

// ============================================================================
// State name helper
// ============================================================================

std::string dynaDriveStateName(Registers::DynaDrive::Status::StateOptions state);

} // namespace Drives
} // namespace EtherCAT
