/**
 * @file NexcobotESC211.hpp
 * @brief Nexcobot ESC211 EtherCAT drive aggregate header
 *
 * This header pulls together the register definitions and PDO mapping
 * constants for the Nexcobot ESC211 servo drive/ESC.
 *
 * ## Register Groups
 *
 * - Identity (0x10F1, 0x10F8) — device-specific non-generic CoE objects
 * - PDO Mapping (0x1600, 0x1601, 0x1610-0x1617, 0x1A01, 0x1A02, 0x1A10-0x1A17)
 *
 * Generic CiA 301 objects (0x1000-0x1018) are available via
 * `tether/profiles/cia301/CiA301Defs.hpp`.
 */

#pragma once

#include <cstdint>

#include "tether/drives/NexcobotESC211/NexcobotESC211Registers.hpp"

namespace EtherCAT {
namespace Drives {

// Re-export canonical identity constants
constexpr uint32_t kNexcobotESC211VendorId    = NexcobotESC211::kVendorId;
constexpr uint32_t kNexcobotESC211ProductCode = NexcobotESC211::kProductCode;

} // namespace Drives
} // namespace EtherCAT
