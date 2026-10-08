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
#include "tether/profiles/cia402/CiA402Drive.hpp"
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
        TETHER_LOGW(TAG, "Failed to read StatusWord (0x6041) from {} (SDO upload failed)", sdo.logPrefix().c_str());

        // Best-effort: still try to read fault codes.
        const auto mfr_ext = readManufacturerFaultExtended(sdo, slave_idx);
        const uint16_t mfr = mfr_ext.external_code;
        const uint16_t cia = readCiA402Error(sdo, slave_idx);
        if (mfr_error) *mfr_error = mfr;
        if (cia402_error) *cia402_error = cia;

        if (mfr != 0 || cia != 0) {
            AS715NError err = AS715NError::parse(mfr);
            TETHER_LOGE(TAG, "┌─── AS715N FAULT REPORT (StatusWord unreadable) ─────────\n│ 0x203F: external=0x{:04X} internal=0x{:04X}\n│ Mfr Error: {} ({})\n│ 0x603F: 0x{:04X}\n└─────────────────────────────────────────────────────────",
                       mfr_ext.external_code, mfr_ext.internal_code, err.name, err.description, cia);
        } else {
            TETHER_LOGW(TAG, "{}: also failed to read 0x203F/0x603F (mailbox/CoE may be unavailable yet)", sdo.logPrefix().c_str());
        }

        // Conservative: if we can't read status, treat as fault/unknown.
        return true;
    }
    statusword = sw_result.value();

    // CiA 402 StatusWord bit 3 = Fault
    bool has_fault = (statusword & (1u << 3)) != 0;

    if (has_fault) {
        TETHER_LOGW(TAG, "{}: Fault detected! StatusWord=0x{:04X}", sdo.logPrefix().c_str(), statusword);

        // Read detailed error codes
        const auto mfr_ext = readManufacturerFaultExtended(sdo, slave_idx);
        const uint16_t mfr = mfr_ext.external_code;
        uint16_t cia = readCiA402Error(sdo, slave_idx);

        // Parse and log human-readable error
        AS715NError err = AS715NError::parse(mfr);
        TETHER_LOGE(TAG, "┌─── AS715N FAULT REPORT ─────────────────────────────────\n│ StatusWord:     0x{:04X} (Fault={}, Warning={})\n│ 0x203F:         external=0x{:04X} internal=0x{:04X}\n│ Mfr Error:      {}\n│ Description:    {}\n│ CiA 402 Error:  0x{:04X}\n│ Recoverable:    {}\n│ DC Sync Error:  {}\n└─────────────────────────────────────────────────────────",
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
            TETHER_LOGW(TAG, "{}: Warning active (StatusWord=0x{:04X}, no fault)", sdo.logPrefix().c_str(), statusword);
        }
    }

    return has_fault;
}

// ============================================================================
// Fault Reset
// ============================================================================

bool AS715NFaultHandler::resetFault(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx,
                                    CiA402Drive* drive) {
    // Per the A6-EC manual the F31.00 pulse is only accepted with S-ON
    // (Controlword bit 0) cleared — a drive still switched on keeps the
    // fault latched and the reset silently does nothing.
    if (auto cw = sdo.readU16(0x6040, 0x00, {.timeout_ms = 3000});
        cw.has_value() && (*cw & 0x0001u)) {
        const uint16_t son_off = static_cast<uint16_t>(*cw & ~0x0001u);
        if (!sdo.writeU16(0x6040, 0x00, son_off, {.timeout_ms = 3000})
                 .has_value()) {
            TETHER_LOGE(TAG, "{}: failed to clear S-ON before F31 fault reset",
                        sdo.logPrefix().c_str());
            return false;
        }
        if (drive) drive->setControlword(son_off);
        Tether::Platform::Clock::instance().delayMilliseconds(50);
    }

    TETHER_LOGI(TAG, "{}: Attempting fault reset via 0x2031:01 (F31.00)...", sdo.logPrefix().c_str());

    // 0 -> 1 -> 0 sequence
    if (!sdo.writeU16(AS715NDevice::kControlInProgressIndex, AS715NDevice::kFaultResetSubIndex, 0, {.timeout_ms = 3000}).has_value()) {
        TETHER_LOGE(TAG, "{}: Failed to write 0x2031:01=0", sdo.logPrefix().c_str());
        return false;
    }
    Tether::Platform::Clock::instance().delayMilliseconds(50);

    if (!sdo.writeU16(AS715NDevice::kControlInProgressIndex, AS715NDevice::kFaultResetSubIndex, 1, {.timeout_ms = 3000}).has_value()) {
        TETHER_LOGE(TAG, "{}: Failed to write 0x2031:01=1", sdo.logPrefix().c_str());
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
            TETHER_LOGI(TAG, "{}: Fault CLEARED (StatusWord=0x{:04X})", sdo.logPrefix().c_str(), statusword);
        } else {
            TETHER_LOGW(TAG, "{}: Fault NOT cleared (StatusWord=0x{:04X})", sdo.logPrefix().c_str(), statusword);
        }
        return fault_cleared;
    }

    const uint16_t mfr_after = readManufacturerFault(sdo, slave_idx);
    const bool cleared = (mfr_after == 0);
    if (cleared) {
        TETHER_LOGI(TAG, "{}: Fault appears CLEARED (0x203F external now 0)", sdo.logPrefix().c_str());
    } else {
        TETHER_LOGW(TAG, "{}: Fault may persist (0x203F external=0x{:04X})", sdo.logPrefix().c_str(), mfr_after);
    }
    return cleared;
}

bool AS715NFaultHandler::resetAllFaults(EtherCAT::CoE::CoEManager& sdo,
                                        uint16_t slave_idx) {
    uint16_t mfr_error = 0, cia402_error = 0;
    if (!checkFault(sdo, slave_idx, &mfr_error, &cia402_error)) {
        TETHER_LOGI(TAG, "{}: no fault — nothing to reset",
                    sdo.logPrefix().c_str());
        return true;
    }

    const AS715NError err = AS715NError::parse(mfr_error);
    if (mfr_error != 0 && err.isDCSyncError()) {
        // ErC1.x "Synchronization loss" latches whenever cyclic/DC sync
        // stops (e.g. a previous session ended) and clears on its own once
        // cyclic PDO exchange + DC lock are re-established — the F31.00
        // reset cannot clear it while DC is not running and only wastes
        // seconds of startup.  Defer to the OP-path; callers needing an
        // explicit reset can still use handleNoSyncError().
        TETHER_LOGI(TAG, "{}: DC sync error {} — stale latch, deferring "
                         "(clears once DC lock is re-established)",
                    sdo.logPrefix().c_str(), err.name);
        return true;
    }
    if (mfr_error != 0 && !err.is_recoverable) {
        TETHER_LOGW(TAG, "{}: error {} is marked non-recoverable — "
                         "attempting reset anyway",
                    sdo.logPrefix().c_str(), err.name);
    } else if (mfr_error == 0) {
        TETHER_LOGI(TAG, "{}: fault bit set but no manufacturer error "
                         "(0x603F=0x{:04X}) — plain reset",
                    sdo.logPrefix().c_str(), cia402_error);
    }

    return resetFault(sdo, slave_idx);
}

// ============================================================================
// DC Sync Error Recovery
// ============================================================================

bool AS715NFaultHandler::handleNoSyncError(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx, uint8_t max_attempts) {
    // First check if the current error is indeed a DC sync error
    uint16_t mfr = readManufacturerFault(sdo, slave_idx);

    if (mfr == 0) {
        TETHER_LOGI(TAG, "{}: No manufacturer fault present", sdo.logPrefix().c_str());
        return true;  // No fault
    }

    AS715NError err = AS715NError::parse(mfr);
    if (!err.isDCSyncError()) {
        TETHER_LOGW(TAG, "{}: Error {} is not a DC sync error — cannot handle with sync recovery",
                    sdo.logPrefix().c_str(), err.name);
        return false;
    }

    TETHER_LOGI(TAG, "{}: Handling DC sync error {} ({}), max {} attempts",
                sdo.logPrefix().c_str(), err.name, err.description, max_attempts);

    for (uint8_t attempt = 1; attempt <= max_attempts || max_attempts == 0; ++attempt) {
        TETHER_LOGI(TAG, "{}: DC sync error reset attempt {}/{}...",
                    sdo.logPrefix().c_str(), attempt, max_attempts);

        if (resetFault(sdo, slave_idx)) {
            // Verify fault is truly gone by re-reading manufacturer fault
            Tether::Platform::Clock::instance().delayMilliseconds(100);
            uint16_t new_mfr = readManufacturerFault(sdo, slave_idx);
            if (new_mfr == 0) {
                TETHER_LOGI(TAG, "{}: DC sync error cleared on attempt {}", sdo.logPrefix().c_str(), attempt);
                return true;
            }
            TETHER_LOGW(TAG, "{}: Fault reset succeeded but manufacturer error still {}",
                        sdo.logPrefix().c_str(), new_mfr);
        }

        if (max_attempts != 0 && attempt >= max_attempts) {
            break;
        }

        // Wait before next attempt
        Tether::Platform::Clock::instance().delayMilliseconds(500);
    }

    TETHER_LOGE(TAG, "{}: Failed to clear DC sync error after {} attempts — CRITICAL",
                sdo.logPrefix().c_str(), max_attempts);
    return false;
}

} // namespace Drives
} // namespace EtherCAT
