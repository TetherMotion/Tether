/**
 * @file AS715N.cpp
 * @brief ANCTL AS715N Servo Drive Fault Detection and Reset
 *
 * Manual notes (A6-EC series):
 * - 0x203F is UInt32: high 16 bits internal, low 16 bits external.
 * - External code is digit-nibble encoded (e.g., Er74.1 => 0x0741).
 * - Fault reset is performed via 0x2031:01 (F31.00) after switching S-ON off.
 */
#include "tether/drives/AS715N.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/platform/Platform.hpp"
#include <cstdio>
#include <cstring>

static const char* TAG = "AS715N";

namespace EtherCAT {
namespace Drives {

// Error parsing implementation moved to `AS715NErrors.{hpp,cpp}`
// (keeps `AS715NError` declarations available via the new header)

// Register / SDO helpers moved to `AS715NRegisters.{hpp,cpp}`
// (AS715NFaultHandler still declared in AS715N.hpp)

// ============================================================================
// Fault Detection
// ============================================================================

bool AS715NFaultHandler::checkFault(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx, uint16_t* mfr_error, uint16_t* cia402_error) {
    uint16_t statusword = 0;
    auto sw_result = sdo.readU16(0x6041, 0x00, {.timeout_ms = 3000});
    if (!sw_result.has_value()) {
        TETHER_LOGW(TAG, "Failed to read StatusWord (0x6041) from %s (SDO upload failed)", sdo.logPrefix().c_str());

        // Best-effort: still try to read fault codes.
        const auto mfr_ext = readManufacturerFaultExtended(sdo, slave_idx);
        const uint16_t mfr = mfr_ext.external_code;
        const uint16_t cia = readCiA402Error(sdo, slave_idx);
        if (mfr_error) *mfr_error = mfr;
        if (cia402_error) *cia402_error = cia;

        if (mfr != 0 || cia != 0) {
            AS715NError err = AS715NError::parse(mfr);
            TETHER_LOGE(TAG, "┌─── AS715N FAULT REPORT (StatusWord unreadable) ─────────\n│ 0x203F: external=0x%04X internal=0x%04X\n│ Mfr Error: %s (%s)\n│ 0x603F: 0x%04X\n└─────────────────────────────────────────────────────────",
                       mfr_ext.external_code, mfr_ext.internal_code, err.name, err.description, cia);
        } else {
            TETHER_LOGW(TAG, "%s: also failed to read 0x203F/0x603F (mailbox/CoE may be unavailable yet)", sdo.logPrefix().c_str());
        }

        // Conservative: if we can't read status, treat as fault/unknown.
        return true;
    }
    statusword = sw_result.value();

    // CiA 402 StatusWord bit 3 = Fault
    bool has_fault = (statusword & (1u << 3)) != 0;

    if (has_fault) {
        TETHER_LOGW(TAG, "%s: Fault detected! StatusWord=0x%04X", sdo.logPrefix().c_str(), statusword);

        // Read detailed error codes
        const auto mfr_ext = readManufacturerFaultExtended(sdo, slave_idx);
        const uint16_t mfr = mfr_ext.external_code;
        uint16_t cia = readCiA402Error(sdo, slave_idx);

        // Parse and log human-readable error
        AS715NError err = AS715NError::parse(mfr);
        TETHER_LOGE(TAG, "┌─── AS715N FAULT REPORT ─────────────────────────────────\n│ StatusWord:     0x%04X (Fault=%d, Warning=%d)\n│ 0x203F:         external=0x%04X internal=0x%04X\n│ Mfr Error:      %s\n│ Description:    %s\n│ CiA 402 Error:  0x%04X\n│ Recoverable:    %s\n│ DC Sync Error:  %s\n└─────────────────────────────────────────────────────────",
                   statusword,
                   (statusword >> 3) & 1,
                   (statusword >> 7) & 1,
                   mfr_ext.external_code, mfr_ext.internal_code,
                   err.name, err.description,
                   cia,
                   err.is_recoverable ? "YES" : "NO",
                   err.isDCSyncError() ? "YES" : "NO");

        if (mfr_error) *mfr_error = mfr;
        if (cia402_error) *cia402_error = cia;
    } else {
        if (mfr_error) *mfr_error = 0;
        if (cia402_error) *cia402_error = 0;

        // Also check Warning bit (bit 7)
        if (statusword & (1u << 7)) {
            TETHER_LOGW(TAG, "%s: Warning active (StatusWord=0x%04X, no fault)", sdo.logPrefix().c_str(), statusword);
        }
    }

    return has_fault;
}

// ============================================================================
// Fault Reset
// ============================================================================

bool AS715NFaultHandler::resetFault(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx) {
    TETHER_LOGI(TAG, "%s: Attempting fault reset via 0x2031:01 (F31.00)...", sdo.logPrefix().c_str());

    // 0 -> 1 -> 0 sequence
    if (!sdo.writeU16(AS715NDevice::kControlInProgressIndex, AS715NDevice::kFaultResetSubIndex, 0, {.timeout_ms = 3000}).has_value()) {
        TETHER_LOGE(TAG, "%s: Failed to write 0x2031:01=0", sdo.logPrefix().c_str());
        return false;
    }
    Tether::Platform::Clock::instance().delayMilliseconds(50);

    if (!sdo.writeU16(AS715NDevice::kControlInProgressIndex, AS715NDevice::kFaultResetSubIndex, 1, {.timeout_ms = 3000}).has_value()) {
        TETHER_LOGE(TAG, "%s: Failed to write 0x2031:01=1", sdo.logPrefix().c_str());
        return false;
    }
    Tether::Platform::Clock::instance().delayMilliseconds(200);

    (void)sdo.writeU16(AS715NDevice::kControlInProgressIndex, AS715NDevice::kFaultResetSubIndex, 0, {.timeout_ms = 3000});
    Tether::Platform::Clock::instance().delayMilliseconds(50);

    // Verify via StatusWord when possible; otherwise fall back to 0x203F external code cleared.
    uint16_t statusword = 0;
    auto sw_result = sdo.readU16(0x6041, 0x00, {.timeout_ms = 3000});
    if (sw_result.has_value()) {
        statusword = sw_result.value();
        const bool fault_cleared = (statusword & (1u << 3)) == 0;
        if (fault_cleared) {
            TETHER_LOGI(TAG, "%s: Fault CLEARED (StatusWord=0x%04X)", sdo.logPrefix().c_str(), statusword);
        } else {
            TETHER_LOGW(TAG, "%s: Fault NOT cleared (StatusWord=0x%04X)", sdo.logPrefix().c_str(), statusword);
        }
        return fault_cleared;
    }

    const uint16_t mfr_after = readManufacturerFault(sdo, slave_idx);
    const bool cleared = (mfr_after == 0);
    if (cleared) {
        TETHER_LOGI(TAG, "%s: Fault appears CLEARED (0x203F external now 0)", sdo.logPrefix().c_str());
    } else {
        TETHER_LOGW(TAG, "%s: Fault may persist (0x203F external=0x%04X)", sdo.logPrefix().c_str(), mfr_after);
    }
    return cleared;
}

// ============================================================================
// DC Sync Error Recovery
// ============================================================================

bool AS715NFaultHandler::handleNoSyncError(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx, uint8_t max_attempts) {
    // First check if the current error is indeed a DC sync error
    uint16_t mfr = readManufacturerFault(sdo, slave_idx);

    if (mfr == 0) {
        TETHER_LOGI(TAG, "%s: No manufacturer fault present", sdo.logPrefix().c_str());
        return true;  // No fault
    }

    AS715NError err = AS715NError::parse(mfr);
    if (!err.isDCSyncError()) {
        TETHER_LOGW(TAG, "%s: Error %s is not a DC sync error — cannot handle with sync recovery",
                    sdo.logPrefix().c_str(), err.name);
        return false;
    }

    TETHER_LOGI(TAG, "%s: Handling DC sync error %s (%s), max %u attempts",
                sdo.logPrefix().c_str(), err.name, err.description, max_attempts);

    for (uint8_t attempt = 1; attempt <= max_attempts || max_attempts == 0; ++attempt) {
        TETHER_LOGI(TAG, "%s: DC sync error reset attempt %u/%u...",
                    sdo.logPrefix().c_str(), attempt, max_attempts);

        if (resetFault(sdo, slave_idx)) {
            // Verify fault is truly gone by re-reading manufacturer fault
            Tether::Platform::Clock::instance().delayMilliseconds(100);
            uint16_t new_mfr = readManufacturerFault(sdo, slave_idx);
            if (new_mfr == 0) {
                TETHER_LOGI(TAG, "%s: DC sync error cleared on attempt %u", sdo.logPrefix().c_str(), attempt);
                return true;
            }
            TETHER_LOGW(TAG, "%s: Fault reset succeeded but manufacturer error still %u",
                        sdo.logPrefix().c_str(), new_mfr);
        }

        if (max_attempts != 0 && attempt >= max_attempts) {
            break;
        }

        // Wait before next attempt
        Tether::Platform::Clock::instance().delayMilliseconds(500);
    }

    TETHER_LOGE(TAG, "%s: Failed to clear DC sync error after %u attempts — CRITICAL",
                sdo.logPrefix().c_str(), max_attempts);
    return false;
}

} // namespace Drives
} // namespace EtherCAT
