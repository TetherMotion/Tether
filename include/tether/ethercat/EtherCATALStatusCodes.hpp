/**
 * @file EtherCATALStatusCodes.hpp
 * @brief EtherCAT types: AL Status Codes
 *
 * Split out of Types.hpp.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

namespace EtherCAT {

// ============================================================================
// AL Status Codes
// ============================================================================

namespace alcode {

constexpr uint16_t NO_ERROR               = 0x0000;
constexpr uint16_t UNSPECIFIED_ERROR      = 0x0001;
constexpr uint16_t NO_MEMORY              = 0x0002;
constexpr uint16_t INVALID_REQ_STATE_CHG  = 0x0011;
constexpr uint16_t UNKNOWN_REQ_STATE      = 0x0012;
constexpr uint16_t BOOTSTRAP_NOT_SUPP     = 0x0013;
constexpr uint16_t NO_VALID_FIRMWARE      = 0x0014;
constexpr uint16_t INVALID_MAILBOX_CFG1   = 0x0016;
constexpr uint16_t INVALID_MAILBOX_CFG2   = 0x0017;
constexpr uint16_t INVALID_SM_CFG         = 0x0018;
constexpr uint16_t NO_VALID_INPUTS        = 0x0019;
constexpr uint16_t NO_VALID_OUTPUTS       = 0x001A;
constexpr uint16_t SYNC_ERROR             = 0x001B;
constexpr uint16_t SM_WATCHDOG            = 0x001C;
constexpr uint16_t INVALID_SM_TYPES       = 0x001D;
constexpr uint16_t INVALID_OUTPUT_CFG     = 0x001E;
constexpr uint16_t INVALID_INPUT_CFG      = 0x001F;
constexpr uint16_t INVALID_WATCHDOG_CFG   = 0x0020;
constexpr uint16_t SLAVE_NEEDS_COLD_START = 0x0021;
constexpr uint16_t SLAVE_NEEDS_INIT       = 0x0022;
constexpr uint16_t SLAVE_NEEDS_PREOP      = 0x0023;
constexpr uint16_t SLAVE_NEEDS_SAFEOP     = 0x0024;
constexpr uint16_t INVALID_INPUT_MAP      = 0x0025;
constexpr uint16_t INVALID_OUTPUT_MAP     = 0x0026;
constexpr uint16_t INCONSISTENT_SETTINGS  = 0x0027;
constexpr uint16_t FREERUN_NOT_SUPPORTED  = 0x0028;
constexpr uint16_t SYNC_NOT_SUPPORTED     = 0x0029;
constexpr uint16_t FREERUN_3BUF_NEEDED    = 0x002A;
constexpr uint16_t BG_WATCHDOG            = 0x002B;
constexpr uint16_t NO_VALID_INPUTS_OUTPUTS= 0x002C;
constexpr uint16_t FATAL_SYNC_ERROR       = 0x002D;
constexpr uint16_t NO_SYNC_ERROR          = 0x002E;  ///< "Err74.1" - No sync
constexpr uint16_t INVALID_DC_SYNC_CFG    = 0x0030;
constexpr uint16_t INVALID_DC_LATCH_CFG   = 0x0031;
constexpr uint16_t PLL_ERROR              = 0x0032;
constexpr uint16_t DC_SYNC_IO_ERROR       = 0x0033;
constexpr uint16_t DC_SYNC_TIMEOUT        = 0x0034;
constexpr uint16_t DC_INVALID_SYNC_CYCLE  = 0x0035;
constexpr uint16_t DC_SYNC0_CYCLE_ERROR   = 0x0036;
constexpr uint16_t DC_SYNC1_CYCLE_ERROR   = 0x0037;
constexpr uint16_t MBX_AOE_ERROR          = 0x0041;
constexpr uint16_t MBX_EOE_ERROR          = 0x0042;
constexpr uint16_t MBX_COE_ERROR          = 0x0043;
constexpr uint16_t MBX_FOE_ERROR          = 0x0044;
constexpr uint16_t MBX_SOE_ERROR          = 0x0045;
constexpr uint16_t MBX_VOE_ERROR          = 0x004F;
constexpr uint16_t EEPROM_NO_ACCESS       = 0x0050;
constexpr uint16_t EEPROM_ERROR           = 0x0051;
constexpr uint16_t SLAVE_RESTARTED        = 0x0060;
constexpr uint16_t DEVICE_ID_UPDATE_ERR   = 0x0061;
constexpr uint16_t APPLICATION_CTRL_ERR   = 0x00F0;

/// Get string description for AL status code
inline const char* alStatusCodeToString(uint16_t code) {
    switch (code) {
        case NO_ERROR: return "No error";
        case UNSPECIFIED_ERROR: return "Unspecified error";
        case NO_MEMORY: return "No memory";
        case INVALID_REQ_STATE_CHG: return "Invalid requested state change";
        case UNKNOWN_REQ_STATE: return "Unknown requested state";
        case BOOTSTRAP_NOT_SUPP: return "Bootstrap not supported";
        case NO_VALID_FIRMWARE: return "No valid firmware";
        case INVALID_MAILBOX_CFG1: return "Invalid mailbox configuration (SM0)";
        case INVALID_MAILBOX_CFG2: return "Invalid mailbox configuration (SM1)";
        case INVALID_SM_CFG: return "Invalid sync manager configuration";
        case NO_VALID_INPUTS: return "No valid inputs available";
        case NO_VALID_OUTPUTS: return "No valid outputs available";
        case SYNC_ERROR: return "Synchronization error";
        case SM_WATCHDOG: return "Sync manager watchdog";
        case INVALID_SM_TYPES: return "Invalid sync manager types";
        case INVALID_OUTPUT_CFG: return "Invalid output configuration";
        case INVALID_INPUT_CFG: return "Invalid input configuration";
        case INVALID_WATCHDOG_CFG: return "Invalid watchdog configuration";
        case SLAVE_NEEDS_COLD_START: return "Slave needs cold start";
        case SLAVE_NEEDS_INIT: return "Slave needs INIT state";
        case SLAVE_NEEDS_PREOP: return "Slave needs PRE-OP state";
        case SLAVE_NEEDS_SAFEOP: return "Slave needs SAFE-OP state";
        case INVALID_INPUT_MAP: return "Invalid input mapping";
        case INVALID_OUTPUT_MAP: return "Invalid output mapping";
        case INCONSISTENT_SETTINGS: return "Inconsistent settings";
        case FREERUN_NOT_SUPPORTED: return "Freerun not supported";
        case SYNC_NOT_SUPPORTED: return "Sync not supported";
        case FREERUN_3BUF_NEEDED: return "Freerun needs 3 buffers";
        case BG_WATCHDOG: return "Background watchdog";
        case NO_VALID_INPUTS_OUTPUTS: return "No valid inputs and outputs";
        case FATAL_SYNC_ERROR: return "Fatal sync error";
        case NO_SYNC_ERROR: return "No sync error (Err74.1)";
        case INVALID_DC_SYNC_CFG: return "Invalid DC sync configuration";
        case INVALID_DC_LATCH_CFG: return "Invalid DC latch configuration";
        case PLL_ERROR: return "PLL error";
        case DC_SYNC_IO_ERROR: return "DC sync I/O error";
        case DC_SYNC_TIMEOUT: return "DC sync timeout";
        case DC_INVALID_SYNC_CYCLE: return "Invalid DC sync cycle time";
        case DC_SYNC0_CYCLE_ERROR: return "DC SYNC0 cycle error";
        case DC_SYNC1_CYCLE_ERROR: return "DC SYNC1 cycle error";
        case MBX_AOE_ERROR: return "Mailbox AoE error";
        case MBX_EOE_ERROR: return "Mailbox EoE error";
        case MBX_COE_ERROR: return "Mailbox CoE error";
        case MBX_FOE_ERROR: return "Mailbox FoE error";
        case MBX_SOE_ERROR: return "Mailbox SoE error";
        case MBX_VOE_ERROR: return "Mailbox VoE error";
        case EEPROM_NO_ACCESS: return "EEPROM no access";
        case EEPROM_ERROR: return "EEPROM error";
        case SLAVE_RESTARTED: return "Slave restarted locally";
        case DEVICE_ID_UPDATE_ERR: return "Device ID update error";
        case APPLICATION_CTRL_ERR: return "Application controller error";
        default: return "Unknown error";
    }
}

} // namespace alcode

} // namespace EtherCAT
