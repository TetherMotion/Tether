/**
 * @file RawConstants.hpp
 * @brief EtherCAT raw protocol constants and register address enums
 *
 * Extracted from internal.hpp. Contains:
 * - FIRE_AND_FORGET_IDX constant
 * - EC_REG_* register address enum
 * - SII/EEPROM constants (ECT_SII_*, EC_ECMD_*, EC_ESTAT_*)
 * - SII mailbox address constants
 * - Default SM configuration values
 * - Mailbox protocol type constants
 * - CoE service type and SDO command specifier constants
 * - SM status bit masks
 */

#pragma once

#include <cstdint>
#include <cstddef>

#include "tether/ethercat/SMRegisters.hpp"

namespace EtherCAT {
namespace Raw {

constexpr uint8_t FIRE_AND_FORGET_IDX = 0xFE;

// ============================================================================
// EtherCAT Register Addresses
// ============================================================================

enum : uint16_t {
    EC_REG_AL_CONTROL = 0x0120,
    EC_REG_AL_STATUS = 0x0130,
    EC_REG_AL_STATUS_CODE = 0x0134,

    EC_REG_WD_DIV = 0x0400,
    EC_REG_WD_TIME_PDI = 0x0410,
    EC_REG_WD_TIME_PDATA = 0x0420,
    EC_REG_WD_STATUS = 0x0440,
    EC_REG_WD_CNT_PDI = 0x0442,
    EC_REG_WD_CNT_PDATA = 0x0443,

    EC_REG_EEPCTL = 0x0502,
    EC_REG_EEPSTAT = 0x0502,
    EC_REG_EEPDAT = 0x0508,
    EC_REG_SM0 = 0x0800,
    EC_REG_SM1 = 0x0808,
    EC_REG_SM0STAT = EC_REG_SM0 + 0x05,
    EC_REG_SM1STAT = EC_REG_SM1 + 0x05,
};

// ============================================================================
// SII/EEPROM Constants
// ============================================================================

enum : uint16_t {
    ECT_SII_START = 0x0040,
    ECT_SII_CAT_STRING = 10,
};

enum : uint16_t {
    EC_ECMD_NOP = 0x0000,
    EC_ECMD_READ = 0x0100,
};

enum : uint16_t {
    EC_ESTAT_R64 = 0x0040,
    EC_ESTAT_NACK = 0x2000,
    EC_ESTAT_EMASK = 0x7800,
    EC_ESTAT_BUSY = 0x8000,
};

enum : uint16_t {
    ECT_SII_RXMBXADR = 0x0018,
    ECT_SII_TXMBXADR = 0x001a,
    ECT_SII_MBXPROTO = 0x001c,
};

enum : uint32_t {
    EC_DEFAULTMBXSM0 = 0x00010026,
    EC_DEFAULTMBXSM1 = 0x00010022,
};

// ============================================================================
// Mailbox Protocol Types
// ============================================================================

enum : uint8_t {
    EC_MBXT_ERR = 0x00,
    EC_MBXT_AOE = 0x01,
    EC_MBXT_EOE = 0x02,
    EC_MBXT_COE = 0x03,
    EC_MBXT_FOE = 0x04,
    EC_MBXT_SOE = 0x05,
    EC_MBXT_VOE = 0x0F,
};

enum : uint8_t {
    EC_COES_SDOREQ = 0x02,
    EC_COES_SDORES = 0x03,
};

enum : uint8_t {
    EC_SDO_DOWN_REQ = 0x20,
    EC_SDO_DOWN_SEG_REQ = 0x00,
    EC_SDO_UP_REQ = 0x40,
    EC_SDO_SEG_UP_REQ = 0x60,
    EC_SDO_ABORT = 0x80,
};

// ============================================================================
// Sync Manager Status Bit Masks
// ============================================================================

enum : uint8_t {
    EC_SM_STATUS_WRITE_EVENT      = 0x01,
    EC_SM_STATUS_READ_EVENT       = 0x02,
    EC_SM_STATUS_MBXFULL          = 0x08,
    EC_SM_STATUS_READ_BUFFER_FULL = 0x40,
    EC_SM_STATUS_WRITE_BUFFER_FULL= 0x80,
};

static inline uint16_t sm_status_address(uint8_t sm_index) {
    return static_cast<uint16_t>(EC_REG_SM0 + (sm_index * 8u) + 5u);
}

} // namespace Raw
} // namespace EtherCAT
