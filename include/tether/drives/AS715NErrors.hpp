#pragma once

#include <cstddef>
#include <cstdint>

namespace EtherCAT {
namespace Drives {

struct AS715NError {
    uint16_t raw_code = 0;   // external code (0x203F low-16)
    /// class_code = (raw_code >> 4) & 0xFF — the upper two hex nibbles.
    /// E.g. 0x0C11 (ErC1.1) → class_code = 0xC1; 0x0874 (Er87.4) → class_code = 0x87.
    uint8_t class_code = 0;
    /// sub_code = raw_code & 0x0F — the lowest hex nibble.
    uint8_t sub_code = 0;
    bool is_recoverable = false;
    const char* name = "NoError";
    const char* description = "No error";

    static AS715NError parse(uint16_t raw_code);
    /// Format a human-readable error name into @p buffer using the same hex-nibble
    /// scheme as on the drive display: "Er%X.%X" (e.g. "ErC1.1", "Er87.4").
    static void format(char* buffer, size_t size, uint8_t class_code, uint8_t sub_code);

    /// Returns true for all EtherCAT class C communication errors (0x0Cxx, "ErCx.y")
    /// and legacy decimal-format 0x07xx sync errors ("Er74.x").
    bool isDCSyncError() const {
        return (raw_code & 0xFF00u) == 0x0C00u || (raw_code & 0xFF00u) == 0x0700u;
    }
    /// Returns true for the specific "no sync" / "synchronization loss" fault codes
    /// (ErC1.1 / 0x0C11 on current firmware, Er74.1 / 0x0741 on legacy firmware).
    bool isNoSyncError() const {
        return raw_code == 0x0C11u   // ErC1.1 — Synchronization loss (current firmware)
            || raw_code == 0x0741u;  // Er74.1 — No sync (legacy firmware)
    }
};

namespace ErrorCodes {
// ---- Legacy decimal-format codes (older AS715N firmware) ------------------
constexpr uint16_t DCSyncCycleSettingError = 0x0740;  ///< Er74.0 — legacy
constexpr uint16_t NoSync                 = 0x0741;  ///< Er74.1 — legacy
constexpr uint16_t ChipSyncIncompleteInOp  = 0x0742;  ///< Er74.2 — legacy

// ---- EtherCAT Class-C communication errors (ErC1.x / ErC2.x) -------------
/// ErC1.0 — Excessive EtherCAT synchronization period error (resettable)
constexpr uint16_t EthSyncCycleError       = 0x0C10;
/// ErC1.1 — Synchronization loss (resettable)
constexpr uint16_t SyncLoss               = 0x0C11;
/// ErC1.2 — Network status switchover error (resettable)
constexpr uint16_t NetworkStatusSwitchover = 0x0C12;
/// ErC1.4 — Network cable connection unreliable (resettable)
constexpr uint16_t NetworkCableUnreliable  = 0x0C14;
/// ErC1.5 — Data frame loss protection error (resettable)
constexpr uint16_t DataFrameLossProtection = 0x0C15;
/// ErC1.6 — Data frame forwarding error (resettable)
constexpr uint16_t DataFrameForwarding     = 0x0C16;
/// ErC1.7 — Data update timeout (resettable)
constexpr uint16_t DataUpdateTimeout       = 0x0C17;
/// ErC1.8 — Watchdog expired (resettable)
constexpr uint16_t WatchdogExpired         = 0x0C18;
/// ErC2.0 — SYNC signal loss (resettable)
constexpr uint16_t SYNCSIgnalLoss          = 0x0C20;
}  // namespace ErrorCodes

}  // namespace Drives
}  // namespace EtherCAT
