// SPDX-License-Identifier: MIT
/**
 * @file CiA402StateUtils.cpp
 * @brief Stateless CiA 402 state machine utility functions
 *
 * @details
 * Extracted from CiA402DriveStateMachine.cpp.  These functions are
 * pure/stateless — they operate only on the values passed to them.
 */

#include "tether/profiles/cia402/CiA402StateUtils.hpp"

#include <cstdio>
#include <algorithm>

namespace EtherCAT {

// ============================================================================
// EtherCAT State Name
// ============================================================================

const char* getECStateName(ECState state) {
    switch (state) {
        case ECState::Init:      return "INIT";
        case ECState::PreOp:     return "PRE_OP";
        case ECState::Bootstrap: return "BOOTSTRAP";
        case ECState::SafeOp:    return "SAFE_OP";
        case ECState::Op:        return "OP";
        default:                 return "UNKNOWN";
    }
}

// ============================================================================
// Statusword Diagnostics
// ============================================================================

/**
 * @brief Format statusword diagnostic information
 *
 * Returns a complete bit-by-bit breakdown of the CiA 402 statusword.
 * Format: "0x1637: OpEn,Volt,QS,Remote,TR,Bit12 | STATE"
 */
const char* formatStatuswordDiagnostics(uint16_t sw, char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) return "";

    buffer[0] = '\0';
    size_t pos = 0;

    auto append = [&](const char* fmt, auto... args) {
        if (pos >= buffer_size - 1) return;
        int n;
        if constexpr (sizeof...(args) == 0) {
            // No format arguments: copy the literal directly to avoid
            // -Wformat-security (format string is not a literal here).
            n = snprintf(buffer + pos, buffer_size - pos, "%s", fmt);
        } else {
            n = snprintf(buffer + pos, buffer_size - pos, fmt, args...);
        }
        if (n > 0) pos = std::min(pos + static_cast<size_t>(n), buffer_size - 1);
    };

    // Start with hex value
    append("0x%04X: ", sw);

    // State machine bits (0-2, 5-6)
    bool rtso = sw & (1 << 0);  // ReadyToSwitchOn
    bool so   = sw & (1 << 1);  // SwitchedOn
    bool oe   = sw & (1 << 2);  // OperationEnabled
    bool qs   = sw & (1 << 5);  // QuickStop (1=normal)
    bool sod  = sw & (1 << 6);  // SwitchOnDisabled

    if (oe) append("OpEn,");
    else if (so) append("SwitchOn,");
    else if (rtso) append("ReadyToSO,");
    else if (sod) append("SODisabled,");

    // Fault and warning
    if (sw & (1 << 3)) append("FAULT,");
    if (sw & (1 << 7)) append("WARN,");

    // Voltage enabled
    if (sw & (1 << 4)) append("Volt,");
    else append("NoVolt,");

    // Quick stop (inverted logic - 1 is OK)
    if (!qs) append("QSActive,");

    // Remote
    if (sw & (1 << 9)) append("Remote,");

    // Target reached
    if (sw & (1 << 10)) append("TR,");

    // Internal limit
    if (sw & (1 << 11)) append("Limit,");

    // Bit 12 (mode-specific: HomingAttained or SetPointAck)
    if (sw & (1 << 12)) append("Bit12,");

    // Bit 13 (mode-specific: HomingError or FollowingError)
    if (sw & (1 << 13)) append("Bit13,");

    // Remove trailing comma
    if (pos > 0 && buffer[pos-1] == ',') {
        buffer[pos-1] = '\0';
    }

    return buffer;
}

// ============================================================================
// Drive State Name
// ============================================================================

const char* getDriveStateName(DriveState state) {
    switch (state) {
        case DriveState::NotReadyToSwitchOn: return "Not Ready to Switch On";
        case DriveState::SwitchOnDisabled:   return "Switch On Disabled";
        case DriveState::ReadyToSwitchOn:    return "Ready to Switch On";
        case DriveState::SwitchedOn:         return "Switched On";
        case DriveState::OperationEnabled:   return "Operation Enabled";
        case DriveState::QuickStopActive:    return "Quick Stop Active";
        case DriveState::FaultReactionActive: return "Fault Reaction Active";
        case DriveState::Fault:              return "Fault";
        default:                             return "Unknown";
    }
}

// ============================================================================
// Drive State Decoder
// ============================================================================

DriveState decodeDriveState(uint16_t statusword) {
    // Decode according to CiA 402 state encoding
    // See CiA402Defs.hpp StatuswordBits

    // Check fault first
    if ((statusword & 0x004F) == 0x0008) {
        return DriveState::Fault;
    }
    if ((statusword & 0x004F) == 0x000F) {
        return DriveState::FaultReactionActive;
    }

    // Check other states
    if ((statusword & 0x006F) == 0x0000) {
        return DriveState::NotReadyToSwitchOn;
    }
    if ((statusword & 0x004F) == 0x0040) {
        return DriveState::SwitchOnDisabled;
    }
    if ((statusword & 0x006F) == 0x0021) {
        return DriveState::ReadyToSwitchOn;
    }
    if ((statusword & 0x006F) == 0x0023) {
        return DriveState::SwitchedOn;
    }
    if ((statusword & 0x006F) == 0x0027) {
        return DriveState::OperationEnabled;
    }
    if ((statusword & 0x006F) == 0x0007) {
        return DriveState::QuickStopActive;
    }

    return DriveState::Unknown;
}

} // namespace EtherCAT
