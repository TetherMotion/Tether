// SPDX-License-Identifier: MIT
/**
 * @file CiA402StateUtils.hpp
 * @brief Stateless CiA 402 state machine utility functions
 *
 * @details
 * Provides human-readable state names, statusword diagnostics formatting,
 * and drive state decoding for the CiA 402 profile.  These functions are
 * pure/stateless — they operate only on the values passed to them and
 * have no dependency on the CiA402Drive class instance state.
 *
 * Extracted from CiA402DriveStateMachine.cpp to improve modularity.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "tether/profiles/cia301/CiA402Defs.hpp"

namespace EtherCAT {
class SlaveGroup;
namespace CoE { struct CoETransactionOptions; }

// ============================================================================
// EtherCAT State Machine States
// ============================================================================

enum class ECState : uint8_t {
    Init      = 0x01,
    PreOp     = 0x02,
    Bootstrap = 0x03,
    SafeOp    = 0x04,
    Op        = 0x08,
    Unknown   = 0x00
};

/**
 * @brief Map an EtherCAT state code to a human-readable string.
 * @param state  ECState enumerator.
 * @return static name (e.g. "OP"), or "Unknown".
 */
const char* getECStateName(ECState state);

// ============================================================================
// CiA 402 State Machine States
// ============================================================================

enum class DriveState : uint8_t {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
    Unknown
};

/**
 * @brief Map a CiA 402 drive state to a human-readable string.
 * @param state  DriveState enumerator.
 * @return static state name, or "Unknown".
 */
const char* getDriveStateName(DriveState state);

/**
 * @brief Decode a CiA 402 drive state from a statusword.
 * @param statusword  0x6041 statusword value.
 * @return decoded DriveState (Unknown when the bits match no state).
 */
DriveState decodeDriveState(uint16_t statusword);

/**
 * Format a statusword into a human-readable diagnostic string.
 * Format: "0x1637: OpEn,Volt,QS,Remote,TR,Bit12 | STATE"
 */
const char* formatStatuswordDiagnostics(uint16_t sw, char* buffer, size_t buffer_size);

} // namespace EtherCAT

namespace CiA402 {

/**
 * Map a CiA 402 operating mode code (0x6060) to its standard abbreviation.
 * Returns "PP", "VL", "PV", "PT", "HM", "IP", "CSP", "CSV", "CST", or "Mode N".
 */
/**
 * @param mode  0x6060 modes_of_operation value.
 * @return static abbreviation, or "Mode N" for unlisted codes.
 */
const char* getOperatingModeName(int8_t mode);

/**
 * @brief Set the same operating mode on a whole group and verify it.
 *
 * Writes 0x6060 modes_of_operation on all group members (one queued SDO
 * transaction per member), waits @p settle_ms for the drives to mirror
 * the mode, then verifies 0x6061 modes_of_operation_display on all
 * members.  Logs per-member failures via @p tag.
 *
 * @param group      Drive group to configure.
 * @param mode       OperatingMode to write to 0x6060.
 * @param opts       CoE transaction options (timeout, retries).
 * @param settle_ms  Wait between the write and the display readback.
 * @param tag        Logger tag for per-member failures.
 * @return true when the write succeeded everywhere and every display
 *         readback equals @p mode.
 */
bool setOperatingModeVerified(
    EtherCAT::SlaveGroup& group,
    OperatingMode mode,
    const EtherCAT::CoE::CoETransactionOptions& opts,
    uint32_t settle_ms,
    const char* tag);

} // namespace CiA402
