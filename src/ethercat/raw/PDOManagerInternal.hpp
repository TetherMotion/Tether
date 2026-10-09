/**
 * @file PDOManagerInternal.hpp
 * @brief Shared internals for the PDOManager translation units.
 *
 * @internal Internal header — not installed, not part of the public API.
 * Sync-manager register layout, byte-order helper and log tag shared by
 * PDOManager.cpp and its per-mode TUs after the split.
 */

#pragma once

#include <cstdint>

namespace EtherCAT {

inline constexpr const char* TAG = "ec_pdo_mgr";

// ============================================================================
// SM Register Definitions
// ============================================================================

enum SMRegisters : uint16_t {
    EC_REG_SM0_BASE = 0x0800,
    EC_REG_SM1_BASE = 0x0808,
    EC_REG_SM2_BASE = 0x0810,
    EC_REG_SM3_BASE = 0x0818,
};

enum SMOffsets : uint8_t {
    SM_OFF_PHYS_ADDR = 0x00,
    SM_OFF_LENGTH    = 0x02,
    SM_OFF_CONTROL   = 0x04,
    SM_OFF_STATUS    = 0x05,
    SM_OFF_ACTIVATE  = 0x06,
    SM_OFF_PDI_CTRL  = 0x07,
};

enum SMActivateBits : uint8_t {
    SM_ACT_ENABLE      = 0x01,
    SM_ACT_REPEAT_REQ  = 0x02,
    SM_ACT_DC_EVENT0   = 0x04,
    SM_ACT_DC_EVENT1   = 0x08,
    SM_ACT_LATCH_EVENT = 0x10,
};

inline uint16_t sm_base_address(uint8_t sm_index) {
    return static_cast<uint16_t>(EC_REG_SM0_BASE + (sm_index * 8));
}

// Trivial host↔LE helper (ESP32 is already little-endian)
inline uint16_t host_to_le16(uint16_t v) { return v; }

} // namespace EtherCAT
